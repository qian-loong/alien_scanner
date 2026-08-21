#include "swarm_data_plane/RuntimeAuthority.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        const RouteDescriptor * find_route(
                const TopologySnapshot & topology,
                const std::string & route_id) noexcept
        {
            const auto found = std::find_if(
                    topology.routes.begin(), topology.routes.end(),
                    [&](const RouteDescriptor & route) {
                        return route.route_id == route_id;
                    });
            return found == topology.routes.end() ? nullptr : &*found;
        }

        bool route_contains_intermediate(
                const TopologySnapshot & topology,
                const RouteDescriptor & route,
                const VehicleIdentity & identity) noexcept
        {
            VehicleIdentity current = route.source;
            for(std::size_t index = 0U; index < route.hops.size(); ++index) {
                const auto & hop = route.hops[index];
                const auto link = std::find_if(
                        topology.links.begin(), topology.links.end(),
                        [&](const LinkDescriptor & value) {
                            return value.link_id == hop.link_id
                                   && value.link_epoch == hop.link_epoch;
                        });
                if(link == topology.links.end() || link->source != current) {
                    return false;
                }
                if(index + 1U < route.hops.size() && link->target == identity) {
                    return true;
                }
                current = link->target;
            }
            return false;
        }

        bool route_reaches_relay(
                const TopologySnapshot & topology,
                const RouteDescriptor & route,
                const VehicleIdentity & identity) noexcept
        {
            return route.target == identity
                   || route_contains_intermediate(topology, route, identity);
        }

        bool contains_assignment(
                const std::vector<RoleAssignment> & assignments,
                const VehicleIdentity & identity) noexcept
        {
            return std::any_of(
                    assignments.begin(), assignments.end(),
                    [&](const RoleAssignment & assignment) {
                        return assignment.identity == identity;
                    });
        }

    }// namespace

    RuntimeAuthority::RuntimeAuthority(
            TopologyState topology,
            RoleState roles,
            RuntimeAuthorityConfig config)
            : topology_(std::move(topology))
            , roles_(std::move(roles))
            , config_(std::move(config))
    {
        const auto & snapshot = topology_.snapshot();
        const auto * active_member = topology_.find_member(config_.active_relay);
        const auto * standby_member = topology_.find_member(config_.standby_relay);
        const auto * active_evidence = roles_.find_capability_evidence(
                config_.active_relay);
        const auto * standby_evidence = roles_.find_capability_evidence(
                config_.standby_relay);
        if(config_.active_relay == config_.standby_relay
           || config_.heartbeat_timeout_ns == 0U
           || config_.initial_time_ns == 0U
           || config_.route_failovers.empty()
           || !validate_topology_snapshot(snapshot)
           || roles_.snapshot().fleet_id != snapshot.fleet_id
           || roles_.snapshot().topology_epoch != snapshot.topology_epoch
           || active_member == nullptr || standby_member == nullptr
           || active_member->state != MembershipState::Ready
           || standby_member->state != MembershipState::Ready
           || active_evidence == nullptr || standby_evidence == nullptr
           || config_.failover_assignment.identity != config_.standby_relay
           || config_.failover_assignment.primary_role != PrimaryRole::Relay
           || config_.failover_assignment.lifecycle != RoleLifecycle::Active
           || config_.failover_assignment.services.size() != 1U
           || config_.failover_assignment.services.front().service
                      != ServiceKind::Relay
           || config_.failover_assignment.services.front().lifecycle
                      != ServiceLifecycle::Active) {
            throw std::invalid_argument("runtime authority configuration is invalid");
        }
        for(std::size_t index = 0U; index < config_.route_failovers.size(); ++index) {
            const auto & binding = config_.route_failovers[index];
            const auto * active_route = find_route(snapshot, binding.route_id);
            const bool duplicate = std::any_of(
                    config_.route_failovers.begin(),
                    config_.route_failovers.begin()
                            + static_cast<std::ptrdiff_t>(index),
                    [&](const RuntimeRouteFailover & previous) {
                        return previous.route_id == binding.route_id;
                    });
            if(duplicate
               || !PerceptionMapUpdate::CanonicalCodec::validate_string(
                       binding.route_id, 128U, "authority route identity", false)
               || active_route == nullptr
               || active_route->graph != LogicalGraphKind::Map
               || !route_reaches_relay(
                       snapshot, *active_route, config_.active_relay)
               || binding.failover_route.route_id != binding.route_id
               || binding.failover_route.graph != LogicalGraphKind::Map
               || !route_reaches_relay(
                       snapshot, binding.failover_route, config_.standby_relay)
               || binding.failover_route.source != active_route->source
               || binding.failover_route.route_epoch <= active_route->route_epoch) {
                throw std::invalid_argument(
                        "runtime authority route failover configuration is invalid");
            }
        }

        const bool active_has_relay_route = std::any_of(
                snapshot.routes.begin(), snapshot.routes.end(),
                [&](const RouteDescriptor & route) {
                    return route_contains_intermediate(
                            snapshot, route, config_.active_relay);
                });
        const bool standby_has_relay_route = std::any_of(
                snapshot.routes.begin(), snapshot.routes.end(),
                [&](const RouteDescriptor & route) {
                    return route_contains_intermediate(
                            snapshot, route, config_.standby_relay);
                });
        if(!active_has_relay_route || !standby_has_relay_route
           || !contains_assignment(
                   roles_.snapshot().assignments, config_.active_relay)
           || contains_assignment(
                   roles_.snapshot().assignments, config_.standby_relay)) {
            throw std::invalid_argument(
                    "runtime authority relay role or route prerequisites are invalid");
        }

        receipts_.reserve(snapshot.members.size());
        receipts_ = {
                {config_.active_relay, active_evidence->evidence_revision,
                 config_.initial_time_ns},
                {config_.standby_relay, standby_evidence->evidence_revision,
                 config_.initial_time_ns}};
    }

    RuntimeAuthorityResult RuntimeAuthority::observe_evidence(
            CapabilityEvidence evidence,
            std::uint64_t receive_time_ns)
    {
        auto * receipt = find_receipt(evidence.identity);
        if(receipt == nullptr && topology_.find_member(evidence.identity) == nullptr) {
            return {RuntimeAuthorityStatus::RejectedEvidence, false,
                    "capability evidence is not for a current fleet member"};
        }
        if(receipt != nullptr && receive_time_ns < receipt->receive_time_ns) {
            return {RuntimeAuthorityStatus::RejectedClock, false,
                    "capability evidence receipt time regressed"};
        }
        const auto identity = evidence.identity;
        const auto result = roles_.update_capability_evidence(
                std::move(evidence), topology_.snapshot());
        if(!result) {
            return {RuntimeAuthorityStatus::RejectedEvidence, false,
                    result.diagnostic};
        }
        const auto * accepted = roles_.find_capability_evidence(identity);
        if(result.state_changed && accepted != nullptr) {
            if(receipt == nullptr) {
                receipts_.push_back({identity, accepted->evidence_revision, receive_time_ns});
                return {RuntimeAuthorityStatus::EvidenceApplied, true, {}};
            }
            receipt->evidence_revision = accepted->evidence_revision;
            receipt->receive_time_ns = receive_time_ns;
            return {RuntimeAuthorityStatus::EvidenceApplied, true, {}};
        }
        return {RuntimeAuthorityStatus::EvidenceIgnored, false, {}};
    }

    RuntimeAuthorityResult RuntimeAuthority::tick(std::uint64_t now_ns)
    {
        if(failed_over_) {
            return {RuntimeAuthorityStatus::NoChange, false, {}};
        }
        const auto * receipt = find_receipt(config_.active_relay);
        if(receipt == nullptr || now_ns < receipt->receive_time_ns) {
            return {RuntimeAuthorityStatus::RejectedClock, false,
                    "authority tick time is before the active Relay receipt"};
        }
        if(now_ns - receipt->receive_time_ns <= config_.heartbeat_timeout_ns) {
            return {RuntimeAuthorityStatus::NoChange, false, {}};
        }
        return commit_failover();
    }

    RoleResult RuntimeAuthority::begin_role_transition(
            std::string transition_id,
            RoleCandidate candidate)
    {
        return roles_.begin_transition(
                std::move(transition_id), std::move(candidate), topology_.snapshot());
    }

    RoleResult RuntimeAuthority::acknowledge_role_transition(
            const std::string & transition_id,
            const VehicleIdentity & identity,
            RoleTransitionAckKind kind)
    {
        return roles_.acknowledge_transition(
                transition_id, identity, kind, topology_.snapshot());
    }

    RoleResult RuntimeAuthority::commit_role_transition(
            const std::string & transition_id)
    {
        return roles_.commit_transition(transition_id, topology_.snapshot());
    }

    RoleResult RuntimeAuthority::rollback_role_transition(
            const std::string & transition_id)
    {
        return roles_.rollback_transition(transition_id);
    }

    const TopologySnapshot & RuntimeAuthority::topology() const noexcept
    {
        return topology_.snapshot();
    }

    const RoleSnapshot & RuntimeAuthority::roles() const noexcept
    {
        return roles_.snapshot();
    }

    const RoleTransition * RuntimeAuthority::active_transition() const noexcept
    {
        return roles_.active_transition();
    }

    const RoleTransition * RuntimeAuthority::last_transition() const noexcept
    {
        return roles_.last_transition();
    }

    bool RuntimeAuthority::failed_over() const noexcept
    {
        return failed_over_;
    }

    RuntimeAuthority::EvidenceReceipt * RuntimeAuthority::find_receipt(
            const VehicleIdentity & identity) noexcept
    {
        const auto found = std::find_if(
                receipts_.begin(), receipts_.end(),
                [&](const EvidenceReceipt & receipt) {
                    return receipt.identity == identity;
                });
        return found == receipts_.end() ? nullptr : &*found;
    }

    const RuntimeAuthority::EvidenceReceipt * RuntimeAuthority::find_receipt(
            const VehicleIdentity & identity) const noexcept
    {
        const auto found = std::find_if(
                receipts_.begin(), receipts_.end(),
                [&](const EvidenceReceipt & receipt) {
                    return receipt.identity == identity;
                });
        return found == receipts_.end() ? nullptr : &*found;
    }

    RuntimeAuthorityResult RuntimeAuthority::commit_failover()
    {
        auto next_topology = topology_;
        auto next_roles = roles_;

        const auto lost = next_topology.transition_member(
                config_.active_relay, MembershipState::Lost);
        if(!lost) {
            return {RuntimeAuthorityStatus::RejectedFailover, false,
                    "failed to mark the active Relay Lost: " + lost.diagnostic};
        }

        TopologyCandidate candidate;
        candidate.base_topology_epoch = next_topology.snapshot().topology_epoch;
        candidate.links = next_topology.snapshot().links;
        candidate.edges = next_topology.snapshot().edges;
        candidate.routes = next_topology.snapshot().routes;
        candidate.routes.erase(
                std::remove_if(
                        candidate.routes.begin(), candidate.routes.end(),
                        [&](const RouteDescriptor & route) {
                            return std::any_of(
                                    config_.route_failovers.begin(),
                                    config_.route_failovers.end(),
                                    [&](const RuntimeRouteFailover & binding) {
                                        return route.route_id == binding.route_id;
                                    });
                        }),
                candidate.routes.end());
        for(const auto & binding : config_.route_failovers) {
            auto failover_route = binding.failover_route;
            failover_route.topology_epoch = candidate.base_topology_epoch;
            candidate.routes.push_back(std::move(failover_route));
        }
        const auto rerouted = next_topology.replace_topology(std::move(candidate));
        if(!rerouted) {
            return {RuntimeAuthorityStatus::RejectedFailover, false,
                    "failed to commit the standby route: " + rerouted.diagnostic};
        }

        RoleCandidate role_candidate;
        role_candidate.base_role_epoch = next_roles.snapshot().role_epoch;
        role_candidate.topology_epoch = next_topology.snapshot().topology_epoch;
        role_candidate.assignments = next_roles.snapshot().assignments;
        role_candidate.assignments.erase(
                std::remove_if(
                        role_candidate.assignments.begin(),
                        role_candidate.assignments.end(),
                        [&](const RoleAssignment & assignment) {
                            return assignment.identity == config_.active_relay
                                   || assignment.identity == config_.standby_relay;
                        }),
                role_candidate.assignments.end());
        auto failover_assignment = config_.failover_assignment;
        const auto * standby_evidence = next_roles.find_capability_evidence(
                config_.standby_relay);
        if(standby_evidence == nullptr) {
            return {RuntimeAuthorityStatus::RejectedFailover, false,
                    "standby Relay capability evidence is missing"};
        }
        failover_assignment.capability_revision =
                standby_evidence->evidence_revision;
        role_candidate.assignments.push_back(std::move(failover_assignment));

        const std::string transition_id =
                "relay-failover-" + std::to_string(next_topology.snapshot().topology_epoch);
        const auto prepared = next_roles.begin_transition(
                transition_id, std::move(role_candidate), next_topology.snapshot());
        if(!prepared) {
            return {RuntimeAuthorityStatus::RejectedFailover, false,
                    "failed to prepare the standby Relay role: "
                            + prepared.diagnostic};
        }
        const auto committed = next_roles.commit_transition(
                transition_id, next_topology.snapshot());
        if(!committed) {
            return {RuntimeAuthorityStatus::RejectedFailover, false,
                    "failed to commit the standby Relay role: "
                            + committed.diagnostic};
        }

        topology_ = std::move(next_topology);
        roles_ = std::move(next_roles);
        failed_over_ = true;
        return {RuntimeAuthorityStatus::FailoverCommitted, true, {}};
    }

}// namespace SwarmDataPlane
