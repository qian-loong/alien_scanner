#include "swarm_data_plane/TopologyState.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        bool valid_text(const std::string & value, const TopologyLimits & limits)
        {
            return static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                    value, limits.max_identity_bytes, "topology identity", false));
        }

        bool nonzero_hash(const PerceptionMapUpdate::Hash256 & hash) noexcept
        {
            return std::any_of(hash.begin(), hash.end(), [](std::uint8_t byte) {
                return byte != 0U;
            });
        }

        bool transition_allowed(MembershipState from, MembershipState to) noexcept
        {
            switch(from) {
                case MembershipState::Joining:
                    return to == MembershipState::Resyncing
                           || to == MembershipState::Lost
                           || to == MembershipState::Quarantined;
                case MembershipState::Resyncing:
                    return to == MembershipState::Ready || to == MembershipState::Lost
                           || to == MembershipState::Quarantined;
                case MembershipState::Ready:
                    return to == MembershipState::Draining || to == MembershipState::Lost
                           || to == MembershipState::Quarantined;
                case MembershipState::Draining:
                case MembershipState::Lost:
                case MembershipState::Quarantined:
                    return to == MembershipState::Absent;
                case MembershipState::Absent:
                    return false;
            }
            return false;
        }

        bool can_replace_session(MembershipState state) noexcept
        {
            return state == MembershipState::Absent || state == MembershipState::Lost
                   || state == MembershipState::Quarantined;
        }

        bool same_component_inventory(
                const std::vector<ComponentRegistration> & lhs,
                const std::vector<ComponentRegistration> & rhs) noexcept
        {
            if(lhs.size() != rhs.size()) {
                return false;
            }
            for(std::size_t index = 0U; index < lhs.size(); ++index) {
                if(lhs[index].component_id != rhs[index].component_id
                   || rhs[index].session < lhs[index].session) {
                    return false;
                }
            }
            return true;
        }

        bool same_route_metadata_except_topology_epoch(
                const RouteDescriptor & lhs,
                const RouteDescriptor & rhs) noexcept
        {
            return lhs.route_id == rhs.route_id && lhs.graph == rhs.graph
                   && lhs.source == rhs.source && lhs.target == rhs.target
                   && lhs.route_epoch == rhs.route_epoch
                   && lhs.ttl_hops == rhs.ttl_hops
                   && lhs.validity_budget_ns == rhs.validity_budget_ns
                   && lhs.hops == rhs.hops;
        }

        void canonicalize(VehicleRegistration & registration)
        {
            std::sort(registration.components.begin(), registration.components.end());
            std::sort(registration.sensors.begin(), registration.sensors.end());
        }

    }// namespace

    TopologyState::TopologyState(
            std::string fleet_id,
            std::vector<WhitelistEntry> whitelist,
            TopologyLimits limits)
            : limits_(std::move(limits)), whitelist_(std::move(whitelist))
    {
        if(limits_.max_identity_bytes == 0U || limits_.max_members == 0U
           || limits_.max_components_per_member == 0U
           || limits_.max_sensors_per_member == 0U || limits_.max_links == 0U
           || limits_.max_edges == 0U || limits_.max_routes == 0U
           || limits_.max_link_epoch_history < limits_.max_links
           || limits_.max_route_epoch_history < limits_.max_routes
           || limits_.max_route_hops == 0U || limits_.max_ttl_hops == 0U
           || limits_.max_validity_budget_ns == 0U || !valid_text(fleet_id, limits_)
           || whitelist_.empty() || whitelist_.size() > limits_.max_members) {
            throw std::invalid_argument("topology limits, fleet id, or whitelist are invalid");
        }
        std::sort(whitelist_.begin(), whitelist_.end());
        for(std::size_t index = 0U; index < whitelist_.size(); ++index) {
            const auto & entry = whitelist_[index];
            if(!valid_text(entry.vehicle_id, limits_) || entry.max_components == 0U
               || entry.max_components > limits_.max_components_per_member
               || entry.max_sensors == 0U
               || entry.max_sensors > limits_.max_sensors_per_member
               || (index > 0U
                   && whitelist_[index - 1U].vehicle_id == entry.vehicle_id)) {
                throw std::invalid_argument("topology whitelist entry is invalid");
            }
        }
        snapshot_.fleet_id = std::move(fleet_id);
        snapshot_.topology_epoch = 1U;
    }

    bool TopologyState::is_retired(const VehicleIdentity & identity) const noexcept
    {
        return std::find(retired_sessions_.begin(), retired_sessions_.end(), identity)
               != retired_sessions_.end();
    }

    const MemberRecord * TopologyState::find_member(
            const VehicleIdentity & identity) const noexcept
    {
        const auto found = std::find_if(
                snapshot_.members.begin(), snapshot_.members.end(),
                [&](const MemberRecord & member) {
                    return member.registration.identity == identity;
                });
        return found == snapshot_.members.end() ? nullptr : &*found;
    }

    const MemberRecord * TopologyState::find_member(
            const std::string & vehicle_id) const noexcept
    {
        const auto found = std::find_if(
                snapshot_.members.begin(), snapshot_.members.end(),
                [&](const MemberRecord & member) {
                    return member.registration.identity.vehicle_id == vehicle_id;
                });
        return found == snapshot_.members.end() ? nullptr : &*found;
    }

    TopologyResult TopologyState::register_vehicle(VehicleRegistration registration)
    {
        canonicalize(registration);
        return commit_registration(std::move(registration));
    }

    TopologyResult TopologyState::commit_registration(VehicleRegistration registration)
    {
        const auto whitelist = std::find_if(
                whitelist_.begin(), whitelist_.end(), [&](const WhitelistEntry & entry) {
                    return entry.vehicle_id == registration.identity.vehicle_id;
                });
        if(registration.identity.fleet_id != snapshot_.fleet_id
           || !valid_text(registration.identity.fleet_id, limits_)
           || !valid_text(registration.identity.vehicle_id, limits_)
           || registration.identity.session.boot_time_ns == 0U
           || registration.registration_generation == 0U
           || whitelist == whitelist_.end()) {
            return {TopologyStatus::RejectedAdmission, false,
                    "vehicle identity, session, generation, or whitelist admission failed"};
        }
        if(registration.components.size() > whitelist->max_components
           || registration.sensors.size() > whitelist->max_sensors) {
            return {TopologyStatus::RejectedResourceLimit, false,
                    "vehicle inventory exceeds its whitelist budget"};
        }
        for(std::size_t index = 0U; index < registration.components.size(); ++index) {
            const auto & component = registration.components[index];
            if(!valid_text(component.component_id, limits_)
               || component.session.boot_time_ns == 0U
               || (index > 0U
                   && registration.components[index - 1U].component_id
                              == component.component_id)) {
                return {TopologyStatus::RejectedInvalid, false,
                        "component inventory contains an invalid or duplicate identity"};
            }
        }
        for(std::size_t index = 0U; index < registration.sensors.size(); ++index) {
            const auto & sensor = registration.sensors[index];
            if(!valid_text(sensor.sensor_id, limits_)
               || !nonzero_hash(sensor.descriptor_hash)
               || (index > 0U
                   && registration.sensors[index - 1U].sensor_id == sensor.sensor_id)) {
                return {TopologyStatus::RejectedInvalid, false,
                        "sensor inventory contains an invalid or duplicate descriptor"};
            }
        }
        if(is_retired(registration.identity)) {
            return {TopologyStatus::RejectedStale, false,
                    "vehicle session has already been retired"};
        }

        auto candidate = snapshot_;
        auto existing = std::find_if(
                candidate.members.begin(), candidate.members.end(),
                [&](const MemberRecord & member) {
                    return member.registration.identity.vehicle_id
                           == registration.identity.vehicle_id;
                });
        if(existing == candidate.members.end()) {
            if(candidate.members.size() >= limits_.max_members) {
                return {TopologyStatus::RejectedResourceLimit, false,
                        "active member limit is exhausted"};
            }
            candidate.members.push_back({
                    std::move(registration), MembershipState::Joining,
                    MemberAvailability::Removed, {}});
            return advance_epoch(std::move(candidate));
        }

        const auto previous_identity = existing->registration.identity;
        if(previous_identity == registration.identity) {
            if(registration.registration_generation
               < existing->registration.registration_generation) {
                return {TopologyStatus::RejectedStale, false,
                        "registration generation is stale"};
            }
            if(registration == existing->registration) {
                return {TopologyStatus::IgnoredDuplicate, false, {}};
            }
            if(registration.registration_generation
               == existing->registration.registration_generation) {
                return {TopologyStatus::RejectedConflict, false,
                        "registration generation was reused for different content"};
            }
            if(registration.sensors != existing->registration.sensors
               || !same_component_inventory(
                       existing->registration.components, registration.components)) {
                return {TopologyStatus::RejectedConflict, false,
                        "descriptor inventory changed within one vehicle session"};
            }
            existing->registration = std::move(registration);
            return advance_epoch(std::move(candidate));
        }

        if(!can_replace_session(existing->state)
           || !(previous_identity.session < registration.identity.session)) {
            return {TopologyStatus::RejectedAdmission, false,
                    "a newer vehicle session cannot replace the current active session"};
        }
        if(retired_sessions_.size() >= limits_.max_retired_sessions) {
            return {TopologyStatus::RejectedResourceLimit, false,
                    "retired vehicle session fence is full"};
        }
        prune_identity(candidate, previous_identity);
        existing = std::find_if(
                candidate.members.begin(), candidate.members.end(),
                [&](const MemberRecord & member) {
                    return member.registration.identity.vehicle_id
                           == registration.identity.vehicle_id;
                });
        existing->registration = std::move(registration);
        existing->state = MembershipState::Joining;
        existing->availability = MemberAvailability::Removed;
        existing->prerequisites = {};
        const auto committed = advance_epoch(std::move(candidate));
        if(committed) {
            retired_sessions_.push_back(previous_identity);
        }
        return committed;
    }

    TopologyResult TopologyState::set_prerequisites(
            const VehicleIdentity & identity,
            ResyncPrerequisites prerequisites)
    {
        auto candidate = snapshot_;
        auto member = std::find_if(
                candidate.members.begin(), candidate.members.end(),
                [&](const MemberRecord & value) {
                    return value.registration.identity == identity;
                });
        if(member == candidate.members.end() || is_retired(identity)) {
            return {TopologyStatus::RejectedAdmission, false,
                    "resync prerequisites target an unknown or retired session"};
        }
        if(member->state != MembershipState::Joining
           && member->state != MembershipState::Resyncing) {
            return {TopologyStatus::RejectedTransition, false,
                    "resync prerequisites can only update a pending member"};
        }
        if(member->prerequisites == prerequisites) {
            return {TopologyStatus::IgnoredDuplicate, false, {}};
        }
        member->prerequisites = prerequisites;
        return advance_epoch(std::move(candidate));
    }

    TopologyResult TopologyState::transition_member(
            const VehicleIdentity & identity,
            MembershipState target)
    {
        auto candidate = snapshot_;
        auto member = std::find_if(
                candidate.members.begin(), candidate.members.end(),
                [&](const MemberRecord & value) {
                    return value.registration.identity == identity;
                });
        if(member == candidate.members.end() || is_retired(identity)) {
            return {TopologyStatus::RejectedAdmission, false,
                    "membership transition targets an unknown or retired session"};
        }
        if(member->state == target) {
            return {TopologyStatus::IgnoredDuplicate, false, {}};
        }
        if(!transition_allowed(member->state, target)) {
            return {TopologyStatus::RejectedTransition, false,
                    "membership transition is not allowed"};
        }
        if(target == MembershipState::Ready && !member->prerequisites.ready()) {
            return {TopologyStatus::RejectedPrerequisite, false,
                    "member cannot become Ready before all resync prerequisites"};
        }
        if(member->state == MembershipState::Ready) {
            prune_identity(candidate, identity);
            member = std::find_if(
                    candidate.members.begin(), candidate.members.end(),
                    [&](const MemberRecord & value) {
                        return value.registration.identity == identity;
                    });
        }
        member->state = target;
        member->availability = availability_for(target);
        return advance_epoch(std::move(candidate));
    }

    TopologyResult TopologyState::replace_topology(TopologyCandidate topology)
    {
        if(topology.base_topology_epoch != snapshot_.topology_epoch) {
            return {TopologyStatus::RejectedStale, false,
                    "topology candidate is based on a stale snapshot"};
        }
        if(topology.links.size() > limits_.max_links
           || topology.edges.size() > limits_.max_edges
           || topology.routes.size() > limits_.max_routes) {
            return {TopologyStatus::RejectedResourceLimit, false,
                    "topology candidate exceeds configured collection limits"};
        }
        auto next_link_history = link_epoch_history_;
        for(const auto & link : topology.links) {
            auto previous = std::find_if(
                    next_link_history.begin(), next_link_history.end(),
                    [&](const LinkDescriptor & value) {
                        return value.link_id == link.link_id;
                    });
            if(previous == next_link_history.end()) {
                if(next_link_history.size() >= limits_.max_link_epoch_history) {
                    return {TopologyStatus::RejectedResourceLimit, false,
                            "link epoch history limit is exhausted"};
                }
                next_link_history.push_back(link);
                continue;
            }
            if(link.link_epoch < previous->link_epoch) {
                return {TopologyStatus::RejectedStale, false,
                        "link epoch is stale for the stable link identity"};
            }
            if(link.link_epoch == previous->link_epoch) {
                if(!(link == *previous)) {
                    return {TopologyStatus::RejectedConflict, false,
                            "link epoch was reused for different content"};
                }
                const auto active = std::find_if(
                        snapshot_.links.begin(), snapshot_.links.end(),
                        [&](const LinkDescriptor & value) {
                            return value.link_id == link.link_id;
                        });
                if(active == snapshot_.links.end()) {
                    return {TopologyStatus::RejectedStale, false,
                            "a retired link epoch cannot be replayed"};
                }
                continue;
            }
            *previous = link;
        }
        auto next_route_history = route_epoch_history_;
        for(const auto & route : topology.routes) {
            auto previous = std::find_if(
                    next_route_history.begin(), next_route_history.end(),
                    [&](const RouteDescriptor & value) {
                        return value.route_id == route.route_id;
                    });
            if(previous == next_route_history.end()) {
                if(next_route_history.size() >= limits_.max_route_epoch_history) {
                    return {TopologyStatus::RejectedResourceLimit, false,
                            "route epoch history limit is exhausted"};
                }
                next_route_history.push_back(route);
                continue;
            }
            if(route.route_epoch < previous->route_epoch) {
                return {TopologyStatus::RejectedStale, false,
                        "route epoch is stale for the stable route identity"};
            }
            if(route.route_epoch == previous->route_epoch) {
                if(!same_route_metadata_except_topology_epoch(route, *previous)) {
                    return {TopologyStatus::RejectedConflict, false,
                            "route epoch was reused for different metadata"};
                }
                const auto active = std::find_if(
                        snapshot_.routes.begin(), snapshot_.routes.end(),
                        [&](const RouteDescriptor & value) {
                            return value.route_id == route.route_id;
                        });
                if(active == snapshot_.routes.end()) {
                    return {TopologyStatus::RejectedStale, false,
                            "a retired route epoch cannot be replayed"};
                }
                continue;
            }
            *previous = route;
        }
        auto candidate = snapshot_;
        candidate.links = std::move(topology.links);
        candidate.edges = std::move(topology.edges);
        candidate.routes = std::move(topology.routes);
        const auto committed = advance_epoch(std::move(candidate));
        if(committed) {
            link_epoch_history_ = std::move(next_link_history);
            route_epoch_history_ = std::move(next_route_history);
        }
        return committed;
    }

    TopologyResult TopologyState::validate_route(
            const RouteDescriptor & route,
            std::uint16_t current_hop,
            std::uint64_t accumulated_forwarding_ns) const
    {
        if(route.topology_epoch != snapshot_.topology_epoch) {
            return {TopologyStatus::RejectedStale, false,
                    "route belongs to a stale topology epoch"};
        }
        const auto found = std::find_if(
                snapshot_.routes.begin(), snapshot_.routes.end(),
                [&](const RouteDescriptor & current) {
                    return current.route_id == route.route_id;
                });
        if(found == snapshot_.routes.end()) {
            return {TopologyStatus::RejectedRoute, false,
                    "route is not part of the committed topology"};
        }
        if(!(*found == route)) {
            return {TopologyStatus::RejectedConflict, false,
                    "route identity was reused for different metadata"};
        }
        if(current_hop >= route.hops.size() || current_hop >= route.ttl_hops) {
            return {TopologyStatus::RejectedRoute, false,
                    "route hop TTL is exhausted"};
        }
        if(accumulated_forwarding_ns >= route.validity_budget_ns) {
            return {TopologyStatus::RejectedRoute, false,
                    "route validity budget is exhausted"};
        }
        return {TopologyStatus::Applied, false, {}};
    }

    TopologyResult TopologyState::advance_epoch(TopologySnapshot candidate)
    {
        if(snapshot_.topology_epoch == std::numeric_limits<std::uint64_t>::max()) {
            return {TopologyStatus::RejectedResourceLimit, false,
                    "topology epoch cannot advance without overflow"};
        }
        candidate.topology_epoch = snapshot_.topology_epoch + 1U;
        for(auto & route : candidate.routes) {
            route.topology_epoch = candidate.topology_epoch;
        }
        std::sort(candidate.members.begin(), candidate.members.end());
        std::sort(candidate.links.begin(), candidate.links.end());
        std::sort(candidate.edges.begin(), candidate.edges.end());
        std::sort(candidate.routes.begin(), candidate.routes.end());
        const auto valid = validate_topology_snapshot(candidate, limits_);
        if(!valid) {
            return valid;
        }
        snapshot_ = std::move(candidate);
        return {TopologyStatus::Applied, true, {}};
    }

    void TopologyState::prune_identity(
            TopologySnapshot & candidate,
            const VehicleIdentity & identity) const
    {
        std::vector<RouteHop> removed_hops;
        removed_hops.reserve(candidate.links.size());
        for(const auto & link : candidate.links) {
            if(link.source == identity || link.target == identity) {
                removed_hops.push_back({link.link_id, link.link_epoch});
            }
        }
        candidate.routes.erase(
                std::remove_if(
                        candidate.routes.begin(), candidate.routes.end(),
                        [&](const RouteDescriptor & route) {
                            return route.source == identity || route.target == identity
                                   || std::any_of(
                                           route.hops.begin(), route.hops.end(),
                                           [&](const RouteHop & hop) {
                                               return std::find(
                                                              removed_hops.begin(),
                                                              removed_hops.end(), hop)
                                                      != removed_hops.end();
                                           });
                        }),
                candidate.routes.end());
        candidate.links.erase(
                std::remove_if(
                        candidate.links.begin(), candidate.links.end(),
                        [&](const LinkDescriptor & link) {
                            return link.source == identity || link.target == identity;
                        }),
                candidate.links.end());
        candidate.edges.erase(
                std::remove_if(
                        candidate.edges.begin(), candidate.edges.end(),
                        [&](const GraphEdge & edge) {
                            return edge.source == identity || edge.target == identity;
                        }),
                candidate.edges.end());
    }

    const TopologySnapshot & TopologyState::snapshot() const noexcept
    {
        return snapshot_;
    }

}// namespace SwarmDataPlane
