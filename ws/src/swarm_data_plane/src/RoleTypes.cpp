#include "swarm_data_plane/RoleTypes.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <tuple>

namespace SwarmDataPlane {

    namespace {

        template<typename Container, typename Compare>
        bool strictly_sorted(const Container & values, Compare compare)
        {
            for(std::size_t index = 1U; index < values.size(); ++index) {
                if(!compare(values[index - 1U], values[index])) {
                    return false;
                }
            }
            return true;
        }

        bool valid_text(
                const std::string & value,
                std::size_t max_bytes,
                const char * field)
        {
            return static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                    value, max_bytes, field, false));
        }

        bool valid_identity(
                const VehicleIdentity & identity,
                const std::string & fleet_id,
                const RoleLimits & limits)
        {
            return identity.fleet_id == fleet_id
                   && valid_text(
                           identity.fleet_id, limits.max_identity_bytes,
                           "role fleet identity")
                   && valid_text(
                           identity.vehicle_id, limits.max_identity_bytes,
                           "role vehicle identity")
                   && identity.session.boot_time_ns != 0U;
        }

        bool valid_capability(CapabilityKind capability) noexcept
        {
            switch(capability) {
                case CapabilityKind::Exploration:
                case CapabilityKind::RelayForwarding:
                case CapabilityKind::MapAggregation:
                    return true;
            }
            return false;
        }

        bool valid_vehicle_health(VehicleHealth health) noexcept
        {
            switch(health) {
                case VehicleHealth::Unknown:
                case VehicleHealth::Healthy:
                case VehicleHealth::Degraded:
                case VehicleHealth::LowPower:
                case VehicleHealth::Failsafe:
                    return true;
            }
            return false;
        }

        bool valid_resource_health(ResourceHealth health) noexcept
        {
            switch(health) {
                case ResourceHealth::Unknown:
                case ResourceHealth::Healthy:
                case ResourceHealth::ComputeOverBudget:
                case ResourceHealth::MemoryOverBudget:
                case ResourceHealth::LinkDegraded:
                    return true;
            }
            return false;
        }

        bool valid_primary_role(PrimaryRole role) noexcept
        {
            switch(role) {
                case PrimaryRole::Explorer:
                case PrimaryRole::Relay:
                case PrimaryRole::EdgeAggregator:
                case PrimaryRole::Reserve:
                    return true;
            }
            return false;
        }

        bool valid_role_lifecycle(RoleLifecycle lifecycle) noexcept
        {
            switch(lifecycle) {
                case RoleLifecycle::Active:
                case RoleLifecycle::Draining:
                    return true;
            }
            return false;
        }

        bool valid_service(ServiceKind service) noexcept
        {
            switch(service) {
                case ServiceKind::Relay:
                case ServiceKind::Aggregation:
                    return true;
            }
            return false;
        }

        bool valid_service_lifecycle(ServiceLifecycle lifecycle) noexcept
        {
            switch(lifecycle) {
                case ServiceLifecycle::Active:
                case ServiceLifecycle::Degraded:
                case ServiceLifecycle::Draining:
                    return true;
            }
            return false;
        }

        bool valid_transition_state(RoleTransitionState state) noexcept
        {
            switch(state) {
                case RoleTransitionState::Prepared:
                case RoleTransitionState::Quiescing:
                case RoleTransitionState::HandoffReady:
                case RoleTransitionState::Committed:
                case RoleTransitionState::RolledBack:
                    return true;
            }
            return false;
        }

        bool valid_ack_kind(RoleTransitionAckKind kind) noexcept
        {
            switch(kind) {
                case RoleTransitionAckKind::Quiesced:
                case RoleTransitionAckKind::HandoffReady:
                    return true;
            }
            return false;
        }

        RoleResult validate_budget(
                const ServiceAssignment & service,
                const RoleLimits & limits)
        {
            const auto & budget = service.budget;
            if(budget.queue_bytes == 0U || budget.memory_bytes == 0U
               || budget.network_bits_per_second == 0U
               || budget.max_parallel_work == 0U
               || budget.queue_bytes > limits.max_queue_bytes
               || budget.memory_bytes > limits.max_memory_bytes
               || budget.network_bits_per_second
                          > limits.max_network_bits_per_second
               || budget.max_contributors > limits.max_contributors
               || budget.max_parallel_work > limits.max_parallel_work) {
                return {RoleStatus::RejectedResourceLimit, false,
                        "service budget is zero or exceeds configured limits"};
            }
            if((service.service == ServiceKind::Relay
                && budget.max_contributors != 0U)
               || (service.service == ServiceKind::Aggregation
                   && budget.max_contributors == 0U)) {
                return {RoleStatus::RejectedCombination, false,
                        "service budget fields do not match the service kind"};
            }
            return {RoleStatus::Applied, false, {}};
        }

        RoleResult validate_assignment(
                const RoleAssignment & assignment,
                const std::string & fleet_id,
                const RoleLimits & limits)
        {
            if(!valid_identity(assignment.identity, fleet_id, limits)
               || assignment.capability_revision == 0U
               || !valid_primary_role(assignment.primary_role)
               || !valid_role_lifecycle(assignment.lifecycle)) {
                return {RoleStatus::RejectedInvalid, false,
                        "role assignment identity, revision, role, or lifecycle is invalid"};
            }
            if(assignment.services.size() > limits.max_services_per_assignment) {
                return {RoleStatus::RejectedResourceLimit, false,
                        "role assignment exceeds the service limit"};
            }
            if(assignment.services.size() > 1U
               && !strictly_sorted(
                       assignment.services,
                       [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
                return {RoleStatus::RejectedInvalid, false,
                        "service assignments are not in canonical order"};
            }
            std::set<ServiceKind> service_kinds;
            for(const auto & service : assignment.services) {
                if(!valid_service(service.service)
                   || !valid_service_lifecycle(service.lifecycle)
                   || !service_kinds.insert(service.service).second) {
                    return {RoleStatus::RejectedInvalid, false,
                            "service kind, lifecycle, or uniqueness is invalid"};
                }
                const auto budget = validate_budget(service, limits);
                if(!budget) {
                    return budget;
                }
            }

            const auto has_service = [&](ServiceKind kind) {
                return std::any_of(
                        assignment.services.begin(), assignment.services.end(),
                        [&](const ServiceAssignment & value) {
                            return value.service == kind;
                        });
            };
            switch(assignment.primary_role) {
                case PrimaryRole::Explorer:
                    if(has_service(ServiceKind::Aggregation)) {
                        return {RoleStatus::RejectedCombination, false,
                                "Explorer cannot hold AggregationService"};
                    }
                    break;
                case PrimaryRole::Relay:
                    if(assignment.services.size() != 1U
                       || !has_service(ServiceKind::Relay)) {
                        return {RoleStatus::RejectedCombination, false,
                                "Relay must hold exactly RelayService"};
                    }
                    break;
                case PrimaryRole::EdgeAggregator:
                    if(assignment.services.size() != 1U
                       || !has_service(ServiceKind::Aggregation)) {
                        return {RoleStatus::RejectedCombination, false,
                                "EdgeAggregator must hold exactly AggregationService"};
                    }
                    break;
                case PrimaryRole::Reserve:
                    if(!assignment.services.empty()) {
                        return {RoleStatus::RejectedCombination, false,
                                "Reserve cannot hold a service assignment"};
                    }
                    break;
            }
            return {RoleStatus::Applied, false, {}};
        }

    }// namespace

    bool CapabilityRegistration::operator==(
            const CapabilityRegistration & other) const noexcept
    {
        return identity == other.identity
               && registration_generation == other.registration_generation
               && declared_capabilities == other.declared_capabilities;
    }

    bool ServiceHealthEvidence::operator==(
            const ServiceHealthEvidence & other) const noexcept
    {
        return service == other.service && health == other.health;
    }

    bool ServiceHealthEvidence::operator<(
            const ServiceHealthEvidence & other) const noexcept
    {
        return std::tie(service, health) < std::tie(other.service, other.health);
    }

    bool CapabilityEvidence::operator==(
            const CapabilityEvidence & other) const noexcept
    {
        return identity == other.identity
               && evidence_revision == other.evidence_revision
               && effective_capabilities == other.effective_capabilities
               && vehicle_health == other.vehicle_health
               && resource_health == other.resource_health
               && service_health == other.service_health;
    }

    bool ServiceBudget::operator==(const ServiceBudget & other) const noexcept
    {
        return queue_bytes == other.queue_bytes && memory_bytes == other.memory_bytes
               && network_bits_per_second == other.network_bits_per_second
               && max_contributors == other.max_contributors
               && max_parallel_work == other.max_parallel_work;
    }

    bool ServiceAssignment::operator==(
            const ServiceAssignment & other) const noexcept
    {
        return service == other.service && lifecycle == other.lifecycle
               && budget == other.budget;
    }

    bool ServiceAssignment::operator<(
            const ServiceAssignment & other) const noexcept
    {
        return std::tie(service, lifecycle)
               < std::tie(other.service, other.lifecycle);
    }

    bool RoleAssignment::operator==(const RoleAssignment & other) const noexcept
    {
        return identity == other.identity
               && capability_revision == other.capability_revision
               && primary_role == other.primary_role
               && lifecycle == other.lifecycle && services == other.services;
    }

    bool RoleAssignment::operator<(const RoleAssignment & other) const noexcept
    {
        return identity < other.identity;
    }

    bool RoleSnapshot::operator==(const RoleSnapshot & other) const noexcept
    {
        return protocol_version == other.protocol_version && fleet_id == other.fleet_id
               && topology_epoch == other.topology_epoch
               && role_epoch == other.role_epoch
               && assignments == other.assignments;
    }

    bool RoleTransitionAck::operator==(
            const RoleTransitionAck & other) const noexcept
    {
        return identity == other.identity && kind == other.kind;
    }

    bool RoleTransitionAck::operator<(
            const RoleTransitionAck & other) const noexcept
    {
        return std::tie(identity, kind) < std::tie(other.identity, other.kind);
    }

    bool RoleTransition::operator==(const RoleTransition & other) const noexcept
    {
        return transition_id == other.transition_id && state == other.state
               && base_role_epoch == other.base_role_epoch
               && topology_epoch == other.topology_epoch
               && target_assignments == other.target_assignments
               && changed_members == other.changed_members
               && acknowledgements == other.acknowledgements;
    }

    std::optional<CapabilityKind> required_capability(PrimaryRole role) noexcept
    {
        switch(role) {
            case PrimaryRole::Explorer:
                return CapabilityKind::Exploration;
            case PrimaryRole::Relay:
                return CapabilityKind::RelayForwarding;
            case PrimaryRole::EdgeAggregator:
                return CapabilityKind::MapAggregation;
            case PrimaryRole::Reserve:
                return std::nullopt;
        }
        return std::nullopt;
    }

    CapabilityKind required_capability(ServiceKind service) noexcept
    {
        switch(service) {
            case ServiceKind::Relay:
                return CapabilityKind::RelayForwarding;
            case ServiceKind::Aggregation:
                return CapabilityKind::MapAggregation;
        }
        return CapabilityKind::Exploration;
    }

    RoleResult validate_capability_registration(
            const CapabilityRegistration & registration,
            const RoleLimits & limits)
    {
        if(!valid_identity(registration.identity, registration.identity.fleet_id, limits)
           || registration.registration_generation == 0U) {
            return {RoleStatus::RejectedInvalid, false,
                    "capability registration identity or generation is invalid"};
        }
        if(registration.declared_capabilities.size() > limits.max_capabilities) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "declared capability set exceeds its limit"};
        }
        if(registration.declared_capabilities.size() > 1U
           && !strictly_sorted(
                   registration.declared_capabilities,
                   [](CapabilityKind lhs, CapabilityKind rhs) { return lhs < rhs; })) {
            return {RoleStatus::RejectedInvalid, false,
                    "declared capabilities are not in canonical order"};
        }
        if(std::any_of(
                   registration.declared_capabilities.begin(),
                   registration.declared_capabilities.end(),
                   [](CapabilityKind capability) {
                       return !valid_capability(capability);
                   })) {
            return {RoleStatus::RejectedInvalid, false,
                    "declared capability set contains an unknown value"};
        }
        return {RoleStatus::Applied, false, {}};
    }

    RoleResult validate_capability_evidence(
            const CapabilityEvidence & evidence,
            const RoleLimits & limits)
    {
        if(!valid_identity(evidence.identity, evidence.identity.fleet_id, limits)
           || evidence.evidence_revision == 0U
           || !valid_vehicle_health(evidence.vehicle_health)
           || !valid_resource_health(evidence.resource_health)) {
            return {RoleStatus::RejectedInvalid, false,
                    "capability evidence identity, revision, or health is invalid"};
        }
        if(evidence.effective_capabilities.size() > limits.max_capabilities
           || evidence.service_health.size() > limits.max_services_per_assignment) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "capability evidence exceeds configured limits"};
        }
        if(evidence.effective_capabilities.size() > 1U
           && !strictly_sorted(
                   evidence.effective_capabilities,
                   [](CapabilityKind lhs, CapabilityKind rhs) { return lhs < rhs; })) {
            return {RoleStatus::RejectedInvalid, false,
                    "effective capabilities are not in canonical order"};
        }
        if(std::any_of(
                   evidence.effective_capabilities.begin(),
                   evidence.effective_capabilities.end(),
                   [](CapabilityKind capability) {
                       return !valid_capability(capability);
                   })) {
            return {RoleStatus::RejectedInvalid, false,
                    "effective capability set contains an unknown value"};
        }
        if(evidence.service_health.size() > 1U
           && !strictly_sorted(
                   evidence.service_health,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {RoleStatus::RejectedInvalid, false,
                    "service health evidence is not in canonical order"};
        }
        std::set<ServiceKind> service_kinds;
        for(const auto & service : evidence.service_health) {
            if(!valid_service(service.service)
               || !valid_resource_health(service.health)
               || !service_kinds.insert(service.service).second) {
                return {RoleStatus::RejectedInvalid, false,
                        "service health evidence contains an unknown or duplicate value"};
            }
        }
        return {RoleStatus::Applied, false, {}};
    }

    RoleResult validate_role_snapshot(
            const RoleSnapshot & snapshot,
            const RoleLimits & limits)
    {
        if(snapshot.protocol_version != kRoleProtocolVersion
           || !valid_text(
                   snapshot.fleet_id, limits.max_identity_bytes,
                   "role snapshot fleet identity")
           || snapshot.topology_epoch == 0U || snapshot.role_epoch == 0U) {
            return {RoleStatus::RejectedInvalid, false,
                    "role protocol, fleet identity, or epoch is invalid"};
        }
        if(snapshot.assignments.size() > limits.max_assignments) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "role snapshot exceeds the assignment limit"};
        }
        if(snapshot.assignments.size() > 1U
           && !strictly_sorted(
                   snapshot.assignments,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {RoleStatus::RejectedInvalid, false,
                    "role assignments are not in canonical order"};
        }
        std::set<std::string> vehicle_ids;
        for(const auto & assignment : snapshot.assignments) {
            const auto valid = validate_assignment(
                    assignment, snapshot.fleet_id, limits);
            if(!valid) {
                return valid;
            }
            if(!vehicle_ids.insert(assignment.identity.vehicle_id).second) {
                return {RoleStatus::RejectedConflict, false,
                        "role snapshot contains multiple sessions for one vehicle"};
            }
        }
        return {RoleStatus::Applied, false, {}};
    }

    RoleResult validate_role_transition(
            const RoleTransition & transition,
            const std::string & fleet_id,
            const RoleLimits & limits)
    {
        if(!valid_text(
                   transition.transition_id, limits.max_transition_id_bytes,
                   "role transition identity")
           || !valid_transition_state(transition.state)
           || transition.base_role_epoch == 0U
           || transition.base_role_epoch == std::numeric_limits<std::uint64_t>::max()
           || transition.topology_epoch == 0U) {
            return {RoleStatus::RejectedInvalid, false,
                    "role transition identity, state, or epoch is invalid"};
        }
        if(transition.changed_members.empty()
           || transition.changed_members.size() > limits.max_transition_members
           || transition.acknowledgements.size() > limits.max_acknowledgements) {
            return {RoleStatus::RejectedResourceLimit, false,
                    "role transition member or acknowledgement limit is invalid"};
        }
        if(transition.changed_members.size() > 1U
           && !strictly_sorted(
                   transition.changed_members,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {RoleStatus::RejectedInvalid, false,
                    "changed members are not in canonical order"};
        }
        for(const auto & identity : transition.changed_members) {
            if(!valid_identity(identity, fleet_id, limits)) {
                return {RoleStatus::RejectedInvalid, false,
                        "changed member identity is invalid"};
            }
        }
        if(transition.acknowledgements.size() > 1U
           && !strictly_sorted(
                   transition.acknowledgements,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {RoleStatus::RejectedInvalid, false,
                    "transition acknowledgements are not in canonical order"};
        }
        for(const auto & acknowledgement : transition.acknowledgements) {
            if(!valid_identity(acknowledgement.identity, fleet_id, limits)
               || !valid_ack_kind(acknowledgement.kind)
               || !std::binary_search(
                       transition.changed_members.begin(),
                       transition.changed_members.end(), acknowledgement.identity)) {
                return {RoleStatus::RejectedInvalid, false,
                        "transition acknowledgement is invalid or unrelated"};
            }
        }

        RoleSnapshot target;
        target.fleet_id = fleet_id;
        target.topology_epoch = transition.topology_epoch;
        target.role_epoch = transition.base_role_epoch + 1U;
        target.assignments = transition.target_assignments;
        return validate_role_snapshot(target, limits);
    }

}// namespace SwarmDataPlane
