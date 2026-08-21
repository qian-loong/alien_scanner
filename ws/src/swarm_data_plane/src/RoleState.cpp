#include "swarm_data_plane/RoleState.hpp"

#include "swarm_data_plane/RoleRuntimePolicy.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        bool valid_text(
                const std::string & value,
                std::size_t max_bytes,
                const char * field)
        {
            return static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                    value, max_bytes, field, false));
        }

        const MemberRecord * find_member(
                const TopologySnapshot & topology,
                const VehicleIdentity & identity) noexcept
        {
            const auto found = std::find_if(
                    topology.members.begin(), topology.members.end(),
                    [&](const MemberRecord & member) {
                        return member.registration.identity == identity;
                    });
            return found == topology.members.end() ? nullptr : &*found;
        }

        bool capability_registration_state(MembershipState state) noexcept
        {
            return state == MembershipState::Joining
                   || state == MembershipState::Resyncing
                   || state == MembershipState::Ready;
        }

        void canonicalize(CapabilityRegistration & registration)
        {
            std::sort(
                    registration.declared_capabilities.begin(),
                    registration.declared_capabilities.end());
        }

        void canonicalize(CapabilityEvidence & evidence)
        {
            std::sort(
                    evidence.effective_capabilities.begin(),
                    evidence.effective_capabilities.end());
            std::sort(evidence.service_health.begin(), evidence.service_health.end());
        }

        void canonicalize(RoleCandidate & candidate)
        {
            for(auto & assignment : candidate.assignments) {
                std::sort(assignment.services.begin(), assignment.services.end());
            }
            std::sort(candidate.assignments.begin(), candidate.assignments.end());
        }

        bool contains_capability(
                const std::vector<CapabilityKind> & capabilities,
                CapabilityKind capability) noexcept
        {
            return std::binary_search(
                    capabilities.begin(), capabilities.end(), capability);
        }

        const ServiceHealthEvidence * find_service_health(
                const CapabilityEvidence & evidence,
                ServiceKind service) noexcept
        {
            const auto found = std::find_if(
                    evidence.service_health.begin(), evidence.service_health.end(),
                    [&](const ServiceHealthEvidence & value) {
                        return value.service == service;
                    });
            return found == evidence.service_health.end() ? nullptr : &*found;
        }

        const LinkDescriptor * find_link(
                const TopologySnapshot & topology,
                const std::string & link_id,
                std::uint64_t link_epoch) noexcept
        {
            const auto found = std::find_if(
                    topology.links.begin(), topology.links.end(),
                    [&](const LinkDescriptor & link) {
                        return link.link_id == link_id
                               && link.link_epoch == link_epoch;
                    });
            return found == topology.links.end() ? nullptr : &*found;
        }

        bool healthy_link(const LinkDescriptor * link) noexcept
        {
            return link != nullptr
                   && (link->health == LinkHealth::Up
                       || link->health == LinkHealth::Degraded);
        }

        bool participates_in_communication_graph(
                const TopologySnapshot & topology,
                const VehicleIdentity & identity) noexcept
        {
            return std::any_of(
                    topology.edges.begin(), topology.edges.end(),
                    [&](const GraphEdge & edge) {
                        return edge.graph == LogicalGraphKind::Communication
                               && (edge.source == identity || edge.target == identity)
                               && healthy_link(find_link(
                                       topology, edge.link_id, edge.link_epoch));
                    });
        }

        bool is_intermediate_route_member(
                const TopologySnapshot & topology,
                const VehicleIdentity & identity) noexcept
        {
            for(const auto & route : topology.routes) {
                if(route.hops.size() < 2U) {
                    continue;
                }
                VehicleIdentity current = route.source;
                for(std::size_t index = 0U; index < route.hops.size(); ++index) {
                    const auto & hop = route.hops[index];
                    const auto * link = find_link(topology, hop.link_id, hop.link_epoch);
                    if(!healthy_link(link) || link->source != current) {
                        break;
                    }
                    if(index + 1U < route.hops.size() && link->target == identity) {
                        return true;
                    }
                    current = link->target;
                }
            }
            return false;
        }

        bool is_map_route_target(
                const TopologySnapshot & topology,
                const VehicleIdentity & identity) noexcept
        {
            return std::any_of(
                    topology.routes.begin(), topology.routes.end(),
                    [&](const RouteDescriptor & route) {
                        return route.graph == LogicalGraphKind::Map
                               && route.target == identity;
                    });
        }

        bool assignment_health_allows_new_work(
                const CapabilityEvidence & evidence) noexcept
        {
            const bool vehicle_allowed = evidence.vehicle_health == VehicleHealth::Healthy
                                         || evidence.vehicle_health
                                                    == VehicleHealth::Degraded;
            const bool resource_allowed = evidence.resource_health
                                                  == ResourceHealth::Healthy
                                          || evidence.resource_health
                                                     == ResourceHealth::LinkDegraded;
            return vehicle_allowed && resource_allowed;
        }

        bool service_health_matches_lifecycle(
                ResourceHealth health,
                ServiceLifecycle lifecycle) noexcept
        {
            switch(lifecycle) {
                case ServiceLifecycle::Active:
                    return health == ResourceHealth::Healthy;
                case ServiceLifecycle::Degraded:
                    return health == ResourceHealth::ComputeOverBudget
                           || health == ResourceHealth::MemoryOverBudget
                           || health == ResourceHealth::LinkDegraded;
                case ServiceLifecycle::Draining:
                    return true;
            }
            return false;
        }

        const RoleAssignment * find_assignment(
                const std::vector<RoleAssignment> & assignments,
                const VehicleIdentity & identity) noexcept
        {
            const auto found = std::find_if(
                    assignments.begin(), assignments.end(),
                    [&](const RoleAssignment & assignment) {
                        return assignment.identity == identity;
                    });
            return found == assignments.end() ? nullptr : &*found;
        }

    }// namespace

    RoleState::RoleState(
            const TopologySnapshot & topology,
            RoleLimits limits)
            : limits_(std::move(limits))
    {
        if(limits_.max_identity_bytes == 0U
           || limits_.max_transition_id_bytes == 0U
           || limits_.max_capabilities == 0U
           || limits_.max_capability_registrations == 0U
           || limits_.max_assignments == 0U
           || limits_.max_services_per_assignment == 0U
           || limits_.max_transition_members == 0U
           || limits_.max_acknowledgements == 0U
           || limits_.max_transition_history == 0U
           || limits_.max_queue_bytes == 0U || limits_.max_memory_bytes == 0U
           || limits_.max_network_bits_per_second == 0U
           || limits_.max_contributors == 0U
           || limits_.max_parallel_work == 0U
           || !validate_topology_snapshot(topology)) {
            throw std::invalid_argument("role limits or initial topology are invalid");
        }
        snapshot_.fleet_id = topology.fleet_id;
        snapshot_.topology_epoch = topology.topology_epoch;
        snapshot_.role_epoch = 1U;
    }

    const CapabilityRegistration * RoleState::find_capability_registration(
            const VehicleIdentity & identity) const noexcept
    {
        const auto found = std::find_if(
                capabilities_.begin(), capabilities_.end(),
                [&](const CapabilityRecord & record) {
                    return record.registration.identity == identity;
                });
        return found == capabilities_.end() ? nullptr : &found->registration;
    }

    const CapabilityEvidence * RoleState::find_capability_evidence(
            const VehicleIdentity & identity) const noexcept
    {
        const auto found = std::find_if(
                capabilities_.begin(), capabilities_.end(),
                [&](const CapabilityRecord & record) {
                    return record.registration.identity == identity;
                });
        return found == capabilities_.end() || !found->evidence.has_value()
                       ? nullptr
                       : &*found->evidence;
    }

    RoleResult RoleState::register_capabilities(
            CapabilityRegistration registration,
            const TopologySnapshot & topology)
    {
        canonicalize(registration);
        const auto structure = validate_capability_registration(registration, limits_);
        if(!structure) {
            return structure;
        }
        if(topology.fleet_id != snapshot_.fleet_id
           || !validate_topology_snapshot(topology)) {
            return {RoleStatus::RejectedInvalid, false,
                    "capability registration topology is invalid"};
        }
        const auto * member = find_member(topology, registration.identity);
        if(member == nullptr || !capability_registration_state(member->state)) {
            return {RoleStatus::RejectedAdmission, false,
                    "capability registration targets an unknown or ineligible member"};
        }

        auto exact = std::find_if(
                capabilities_.begin(), capabilities_.end(),
                [&](const CapabilityRecord & record) {
                    return record.registration.identity == registration.identity;
                });
        if(exact != capabilities_.end()) {
            if(registration.registration_generation
               < exact->registration.registration_generation) {
                return {RoleStatus::RejectedStale, false,
                        "capability registration generation is stale"};
            }
            if(registration == exact->registration) {
                return {RoleStatus::IgnoredDuplicate, false, {}};
            }
            if(registration.registration_generation
               == exact->registration.registration_generation) {
                return {RoleStatus::RejectedConflict, false,
                        "capability generation was reused for different content"};
            }
            if(registration.declared_capabilities
               != exact->registration.declared_capabilities) {
                return {RoleStatus::RejectedConflict, false,
                        "declared capabilities changed within one vehicle session"};
            }
            exact->registration = std::move(registration);
            return {RoleStatus::Applied, true, {}};
        }

        const auto same_vehicle = std::find_if(
                capabilities_.begin(), capabilities_.end(),
                [&](const CapabilityRecord & record) {
                    return record.registration.identity.vehicle_id
                           == registration.identity.vehicle_id;
                });
        if(same_vehicle != capabilities_.end()) {
            capabilities_.erase(
                    std::remove_if(
                            capabilities_.begin(), capabilities_.end(),
                            [&](const CapabilityRecord & record) {
                                return record.registration.identity.vehicle_id
                                       == registration.identity.vehicle_id;
                            }),
                    capabilities_.end());
        } else if(capabilities_.size() >= limits_.max_capability_registrations) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "capability registration limit is exhausted"};
        }

        capabilities_.push_back({std::move(registration), std::nullopt});
        std::sort(
                capabilities_.begin(), capabilities_.end(),
                [](const CapabilityRecord & lhs, const CapabilityRecord & rhs) {
                    return lhs.registration.identity < rhs.registration.identity;
                });
        return {RoleStatus::Applied, true, {}};
    }

    RoleResult RoleState::update_capability_evidence(
            CapabilityEvidence evidence,
            const TopologySnapshot & topology)
    {
        canonicalize(evidence);
        const auto structure = validate_capability_evidence(evidence, limits_);
        if(!structure) {
            return structure;
        }
        if(topology.fleet_id != snapshot_.fleet_id
           || !validate_topology_snapshot(topology)) {
            return {RoleStatus::RejectedInvalid, false,
                    "capability evidence topology is invalid"};
        }
        const auto * member = find_member(topology, evidence.identity);
        if(member == nullptr || !capability_registration_state(member->state)) {
            return {RoleStatus::RejectedAdmission, false,
                    "capability evidence targets an unknown or ineligible member"};
        }
        auto record = std::find_if(
                capabilities_.begin(), capabilities_.end(),
                [&](const CapabilityRecord & value) {
                    return value.registration.identity == evidence.identity;
                });
        if(record == capabilities_.end()) {
            return {RoleStatus::RejectedAdmission, false,
                    "capability evidence has no declared registration"};
        }
        if(!std::includes(
                   record->registration.declared_capabilities.begin(),
                   record->registration.declared_capabilities.end(),
                   evidence.effective_capabilities.begin(),
                   evidence.effective_capabilities.end())) {
            return {RoleStatus::RejectedCapability, false,
                    "effective capabilities are not a declared subset"};
        }
        for(const auto & service : evidence.service_health) {
            if(!contains_capability(
                       record->registration.declared_capabilities,
                       required_capability(service.service))) {
                return {RoleStatus::RejectedCapability, false,
                        "service health references an undeclared capability"};
            }
        }
        if(record->evidence.has_value()) {
            if(evidence.evidence_revision < record->evidence->evidence_revision) {
                return {RoleStatus::RejectedStale, false,
                        "capability evidence revision is stale"};
            }
            if(evidence == *record->evidence) {
                return {RoleStatus::IgnoredDuplicate, false, {}};
            }
            if(evidence.evidence_revision == record->evidence->evidence_revision) {
                return {RoleStatus::RejectedConflict, false,
                        "capability evidence revision was reused"};
            }
        }
        record->evidence = std::move(evidence);
        return {RoleStatus::Applied, true, {}};
    }

    RoleResult RoleState::validate_assignment(
            const RoleAssignment & assignment,
            const TopologySnapshot & topology,
            bool require_current_capability_revision) const
    {
        const auto * member = find_member(topology, assignment.identity);
        if(member == nullptr || member->state != MembershipState::Ready) {
            return {RoleStatus::RejectedAdmission, false,
                    "role assignment requires an exact Ready member"};
        }
        const auto registration = std::find_if(
                capabilities_.begin(), capabilities_.end(),
                [&](const CapabilityRecord & record) {
                    return record.registration.identity == assignment.identity;
                });
        if(registration == capabilities_.end() || !registration->evidence.has_value()) {
            return {RoleStatus::RejectedCapability, false,
                    "role assignment lacks capability registration or evidence"};
        }
        const auto & evidence = *registration->evidence;
        if(assignment.capability_revision > evidence.evidence_revision
           || (require_current_capability_revision
               && assignment.capability_revision != evidence.evidence_revision)) {
            return {RoleStatus::RejectedStale, false,
                    "role assignment references stale capability evidence"};
        }

        const auto primary_capability = required_capability(assignment.primary_role);
        if(primary_capability.has_value()
           && !contains_capability(
                   registration->registration.declared_capabilities,
                   *primary_capability)) {
            return {RoleStatus::RejectedCapability, false,
                    "primary role capability was not declared"};
        }
        if(assignment.lifecycle == RoleLifecycle::Active) {
            if(!assignment_health_allows_new_work(evidence)) {
                return {RoleStatus::RejectedHealth, false,
                        "vehicle or resource health rejects an active role"};
            }
            if(primary_capability.has_value()
               && !contains_capability(
                       evidence.effective_capabilities, *primary_capability)) {
                return {RoleStatus::RejectedCapability, false,
                        "primary role capability is not currently effective"};
            }
        }

        for(const auto & service : assignment.services) {
            const auto capability = required_capability(service.service);
            if(!contains_capability(
                       registration->registration.declared_capabilities, capability)) {
                return {RoleStatus::RejectedCapability, false,
                        "service capability was not declared"};
            }
            if(service.lifecycle != ServiceLifecycle::Draining
               && !contains_capability(evidence.effective_capabilities, capability)) {
                return {RoleStatus::RejectedCapability, false,
                        "service capability is not currently effective"};
            }
            const auto * health = find_service_health(evidence, service.service);
            if(service.lifecycle != ServiceLifecycle::Draining
               && (health == nullptr
                   || !service_health_matches_lifecycle(
                           health->health, service.lifecycle))) {
                return {RoleStatus::RejectedHealth, false,
                        "service health does not match its lifecycle"};
            }
            if(service.lifecycle == ServiceLifecycle::Draining) {
                continue;
            }
            if(service.service == ServiceKind::Relay
               && (!participates_in_communication_graph(topology, assignment.identity)
                   || !is_intermediate_route_member(topology, assignment.identity))) {
                return {RoleStatus::RejectedPrerequisite, false,
                        "RelayService requires healthy communication and routed participation"};
            }
            if(service.service == ServiceKind::Aggregation
               && !is_map_route_target(topology, assignment.identity)) {
                return {RoleStatus::RejectedPrerequisite, false,
                        "AggregationService requires a committed map route target"};
            }
        }
        return {RoleStatus::Applied, false, {}};
    }

    RoleResult RoleState::validate_candidate(
            RoleCandidate & candidate,
            const TopologySnapshot & topology) const
    {
        canonicalize(candidate);
        if(candidate.base_role_epoch != snapshot_.role_epoch) {
            return {RoleStatus::RejectedStale, false,
                    "role candidate is based on a stale role epoch"};
        }
        if(candidate.topology_epoch < snapshot_.topology_epoch
           || candidate.topology_epoch != topology.topology_epoch
           || topology.fleet_id != snapshot_.fleet_id) {
            return {RoleStatus::RejectedStale, false,
                    "role candidate is based on a stale topology epoch"};
        }
        if(!validate_topology_snapshot(topology)) {
            return {RoleStatus::RejectedInvalid, false,
                    "role candidate topology is invalid"};
        }
        if(snapshot_.role_epoch == std::numeric_limits<std::uint64_t>::max()) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "role epoch cannot advance without overflow"};
        }

        RoleSnapshot target;
        target.fleet_id = snapshot_.fleet_id;
        target.topology_epoch = topology.topology_epoch;
        target.role_epoch = snapshot_.role_epoch + 1U;
        target.assignments = candidate.assignments;
        const auto structure = validate_role_snapshot(target, limits_);
        if(!structure) {
            return structure;
        }
        for(const auto & assignment : candidate.assignments) {
            const auto * committed = find_assignment(
                    snapshot_.assignments, assignment.identity);
            const auto valid = validate_assignment(
                    assignment, topology,
                    committed == nullptr || !(*committed == assignment));
            if(!valid) {
                return valid;
            }
        }
        return {RoleStatus::Applied, false, {}};
    }

    std::vector<VehicleIdentity> RoleState::changed_members(
            const std::vector<RoleAssignment> & target) const
    {
        std::vector<VehicleIdentity> changed;
        for(const auto & current : snapshot_.assignments) {
            const auto * replacement = find_assignment(target, current.identity);
            if(replacement == nullptr || !(*replacement == current)) {
                changed.push_back(current.identity);
            }
        }
        for(const auto & replacement : target) {
            const auto * current = find_assignment(snapshot_.assignments, replacement.identity);
            if(current == nullptr) {
                changed.push_back(replacement.identity);
            }
        }
        std::sort(changed.begin(), changed.end());
        changed.erase(std::unique(changed.begin(), changed.end()), changed.end());
        return changed;
    }

    std::vector<VehicleIdentity> RoleState::required_ack_members(
            const RoleTransition & transition,
            const TopologySnapshot & topology) const
    {
        std::vector<VehicleIdentity> required;
        for(const auto & current : snapshot_.assignments) {
            const auto * member = find_member(topology, current.identity);
            if(member != nullptr && member->state == MembershipState::Ready
               && std::binary_search(
                       transition.changed_members.begin(),
                       transition.changed_members.end(), current.identity)) {
                required.push_back(current.identity);
            }
        }
        return required;
    }

    bool RoleState::has_ack(
            const RoleTransition & transition,
            const VehicleIdentity & identity,
            RoleTransitionAckKind kind) const noexcept
    {
        return std::binary_search(
                transition.acknowledgements.begin(),
                transition.acknowledgements.end(),
                RoleTransitionAck {identity, kind});
    }

    bool RoleState::transition_id_was_used(
            const std::string & transition_id) const noexcept
    {
        return std::find(
                       transition_id_history_.begin(), transition_id_history_.end(),
                       transition_id)
               != transition_id_history_.end();
    }

    void RoleState::remember_transition_id(std::string transition_id)
    {
        if(transition_id_history_.size() >= limits_.max_transition_history) {
            transition_id_history_.erase(transition_id_history_.begin());
        }
        transition_id_history_.push_back(std::move(transition_id));
    }

    RoleResult RoleState::begin_transition(
            std::string transition_id,
            RoleCandidate candidate,
            const TopologySnapshot & topology)
    {
        if(active_transition_.has_value()) {
            return {RoleStatus::RejectedTransition, false,
                    "another role transition is already active"};
        }
        if(!valid_text(
                   transition_id, limits_.max_transition_id_bytes,
                   "role transition identity")) {
            return {RoleStatus::RejectedInvalid, false,
                    "role transition identity is invalid"};
        }
        if(transition_id_was_used(transition_id)) {
            return {RoleStatus::RejectedConflict, false,
                    "role transition identity has already been used"};
        }
        const auto valid = validate_candidate(candidate, topology);
        if(!valid) {
            return valid;
        }
        auto changed = changed_members(candidate.assignments);
        if(changed.empty()) {
            return {RoleStatus::IgnoredDuplicate, false, {}};
        }
        if(changed.size() > limits_.max_transition_members) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "role transition exceeds its member limit"};
        }

        RoleTransition transition;
        transition.transition_id = std::move(transition_id);
        transition.state = RoleTransitionState::Prepared;
        transition.base_role_epoch = candidate.base_role_epoch;
        transition.topology_epoch = candidate.topology_epoch;
        transition.target_assignments = std::move(candidate.assignments);
        transition.changed_members = std::move(changed);
        const auto structure = validate_role_transition(
                transition, snapshot_.fleet_id, limits_);
        if(!structure) {
            return structure;
        }
        active_transition_ = std::move(transition);
        return {RoleStatus::Applied, true, {}};
    }

    RoleResult RoleState::acknowledge_transition(
            const std::string & transition_id,
            const VehicleIdentity & identity,
            RoleTransitionAckKind kind,
            const TopologySnapshot & topology)
    {
        if(!active_transition_.has_value()
           || active_transition_->transition_id != transition_id) {
            return {RoleStatus::RejectedTransition, false,
                    "role transition acknowledgement targets an unknown transition"};
        }
        if(kind != RoleTransitionAckKind::Quiesced
           && kind != RoleTransitionAckKind::HandoffReady) {
            return {RoleStatus::RejectedInvalid, false,
                    "role transition acknowledgement kind is invalid"};
        }
        if(topology.topology_epoch != active_transition_->topology_epoch
           || topology.fleet_id != snapshot_.fleet_id
           || !validate_topology_snapshot(topology)) {
            return {RoleStatus::RejectedStale, false,
                    "role transition acknowledgement uses a stale topology"};
        }
        const auto required = required_ack_members(*active_transition_, topology);
        if(!std::binary_search(required.begin(), required.end(), identity)) {
            return {RoleStatus::RejectedAdmission, false,
                    "role transition acknowledgement targets an unrelated member"};
        }
        if(kind == RoleTransitionAckKind::HandoffReady
           && !has_ack(
                   *active_transition_, identity,
                   RoleTransitionAckKind::Quiesced)) {
            return {RoleStatus::RejectedPrerequisite, false,
                    "handoff acknowledgement requires quiesce first"};
        }
        if(has_ack(*active_transition_, identity, kind)) {
            return {RoleStatus::IgnoredDuplicate, false, {}};
        }
        if(active_transition_->acknowledgements.size()
           >= limits_.max_acknowledgements) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "role transition acknowledgement limit is exhausted"};
        }
        active_transition_->acknowledgements.push_back({identity, kind});
        std::sort(
                active_transition_->acknowledgements.begin(),
                active_transition_->acknowledgements.end());
        const bool all_quiesced = std::all_of(
                required.begin(), required.end(), [&](const VehicleIdentity & member) {
                    return has_ack(
                            *active_transition_, member,
                            RoleTransitionAckKind::Quiesced);
                });
        const bool all_handoff = std::all_of(
                required.begin(), required.end(), [&](const VehicleIdentity & member) {
                    return has_ack(
                            *active_transition_, member,
                            RoleTransitionAckKind::HandoffReady);
                });
        active_transition_->state = all_quiesced && all_handoff
                                            ? RoleTransitionState::HandoffReady
                                            : RoleTransitionState::Quiescing;
        return {RoleStatus::Applied, true, {}};
    }

    RoleResult RoleState::commit_transition(
            const std::string & transition_id,
            const TopologySnapshot & topology)
    {
        if(!active_transition_.has_value()
           || active_transition_->transition_id != transition_id) {
            return {RoleStatus::RejectedTransition, false,
                    "role commit targets an unknown transition"};
        }
        if(topology.topology_epoch != active_transition_->topology_epoch
           || topology.fleet_id != snapshot_.fleet_id) {
            return {RoleStatus::RejectedStale, false,
                    "role transition became stale before commit"};
        }
        const auto required = required_ack_members(*active_transition_, topology);
        const bool ready = std::all_of(
                required.begin(), required.end(), [&](const VehicleIdentity & member) {
                    return has_ack(
                                   *active_transition_, member,
                                   RoleTransitionAckKind::Quiesced)
                           && has_ack(
                                   *active_transition_, member,
                                   RoleTransitionAckKind::HandoffReady);
                });
        if(!ready) {
            return {RoleStatus::RejectedPrerequisite, false,
                    "role commit is waiting for quiesce and handoff acknowledgements"};
        }

        RoleCandidate candidate;
        candidate.base_role_epoch = active_transition_->base_role_epoch;
        candidate.topology_epoch = active_transition_->topology_epoch;
        candidate.assignments = active_transition_->target_assignments;
        const auto valid = validate_candidate(candidate, topology);
        if(!valid) {
            return valid;
        }

        snapshot_.topology_epoch = topology.topology_epoch;
        ++snapshot_.role_epoch;
        snapshot_.assignments = std::move(candidate.assignments);
        active_transition_->state = RoleTransitionState::Committed;
        active_transition_->target_assignments = snapshot_.assignments;
        remember_transition_id(active_transition_->transition_id);
        last_transition_ = std::move(active_transition_);
        active_transition_.reset();
        return {RoleStatus::Applied, true, {}};
    }

    RoleResult RoleState::rollback_transition(const std::string & transition_id)
    {
        if(!active_transition_.has_value()
           || active_transition_->transition_id != transition_id) {
            return {RoleStatus::RejectedTransition, false,
                    "role rollback targets an unknown transition"};
        }
        active_transition_->state = RoleTransitionState::RolledBack;
        remember_transition_id(active_transition_->transition_id);
        last_transition_ = std::move(active_transition_);
        active_transition_.reset();
        return {RoleStatus::Applied, true, "role transition rolled back"};
    }

    bool RoleState::can_accept_new_work(
            const VehicleIdentity & identity) const noexcept
    {
        return static_cast<bool>(evaluate_role_work_admission(
                snapshot_, find_capability_evidence(identity), active_transition(),
                identity));
    }

    bool RoleState::can_service_accept_new_work(
            const VehicleIdentity & identity,
            ServiceKind service) const noexcept
    {
        return static_cast<bool>(evaluate_service_work_admission(
                snapshot_, find_capability_evidence(identity), active_transition(),
                identity, service));
    }

    const RoleSnapshot & RoleState::snapshot() const noexcept
    {
        return snapshot_;
    }

    const RoleTransition * RoleState::active_transition() const noexcept
    {
        return active_transition_.has_value() ? &*active_transition_ : nullptr;
    }

    const RoleTransition * RoleState::last_transition() const noexcept
    {
        return last_transition_.has_value() ? &*last_transition_ : nullptr;
    }

}// namespace SwarmDataPlane
