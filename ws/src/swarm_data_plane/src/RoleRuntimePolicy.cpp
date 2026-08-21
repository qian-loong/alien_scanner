#include "swarm_data_plane/RoleRuntimePolicy.hpp"

#include <algorithm>

namespace SwarmDataPlane {

    namespace {

        const RoleAssignment * find_assignment(
                const RoleSnapshot & snapshot,
                const VehicleIdentity & identity) noexcept
        {
            const auto found = std::find_if(
                    snapshot.assignments.begin(), snapshot.assignments.end(),
                    [&](const RoleAssignment & assignment) {
                        return assignment.identity == identity;
                    });
            return found == snapshot.assignments.end() ? nullptr : &*found;
        }

        bool contains_capability(
                const CapabilityEvidence & evidence,
                CapabilityKind capability) noexcept
        {
            return std::binary_search(
                    evidence.effective_capabilities.begin(),
                    evidence.effective_capabilities.end(), capability);
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

        bool contains_transition_member(
                const RoleTransition & transition,
                const VehicleIdentity & identity) noexcept
        {
            return std::binary_search(
                    transition.changed_members.begin(),
                    transition.changed_members.end(), identity);
        }

        bool contains_quiesced_ack(
                const RoleTransition & transition,
                const VehicleIdentity & identity) noexcept
        {
            return std::binary_search(
                    transition.acknowledgements.begin(),
                    transition.acknowledgements.end(),
                    RoleTransitionAck {identity, RoleTransitionAckKind::Quiesced});
        }

        bool transition_blocks_new_work(
                const RoleTransition * transition,
                const VehicleIdentity & identity,
                TransitionAdmissionMode mode) noexcept
        {
            if(transition == nullptr
               || transition->state == RoleTransitionState::Committed
               || transition->state == RoleTransitionState::RolledBack) {
                return false;
            }
            if(mode == TransitionAdmissionMode::ChangedMember) {
                return contains_transition_member(*transition, identity);
            }
            return contains_quiesced_ack(*transition, identity);
        }

    }// namespace

    WorkAdmissionResult evaluate_role_work_admission(
            const RoleSnapshot & snapshot,
            const CapabilityEvidence * evidence,
            const RoleTransition * transition,
            const VehicleIdentity & identity,
            std::optional<PrimaryRole> required_role,
            TransitionAdmissionMode transition_mode) noexcept
    {
        const auto * assignment = find_assignment(snapshot, identity);
        if(assignment == nullptr) {
            return {WorkAdmissionStatus::NoAssignment};
        }
        if(required_role.has_value()
           && assignment->primary_role != *required_role) {
            return {WorkAdmissionStatus::WrongPrimaryRole};
        }
        if(assignment->lifecycle != RoleLifecycle::Active) {
            return {WorkAdmissionStatus::RoleInactive};
        }
        if(evidence == nullptr || evidence->identity != identity) {
            return {WorkAdmissionStatus::MissingEvidence};
        }
        if(!assignment_health_allows_new_work(*evidence)) {
            return {WorkAdmissionStatus::HealthBlocked};
        }
        const auto capability = required_capability(assignment->primary_role);
        if(capability.has_value() && !contains_capability(*evidence, *capability)) {
            return {WorkAdmissionStatus::CapabilityMissing};
        }
        if(transition_blocks_new_work(transition, identity, transition_mode)) {
            return {WorkAdmissionStatus::TransitionBlocked};
        }
        return {WorkAdmissionStatus::Allowed};
    }

    WorkAdmissionResult evaluate_service_work_admission(
            const RoleSnapshot & snapshot,
            const CapabilityEvidence * evidence,
            const RoleTransition * transition,
            const VehicleIdentity & identity,
            ServiceKind service,
            TransitionAdmissionMode transition_mode) noexcept
    {
        const auto role = evaluate_role_work_admission(
                snapshot, evidence, transition, identity, std::nullopt,
                transition_mode);
        if(!role) {
            return role;
        }
        const auto * assignment = find_assignment(snapshot, identity);
        const auto selected = std::find_if(
                assignment->services.begin(), assignment->services.end(),
                [&](const ServiceAssignment & value) {
                    return value.service == service;
                });
        if(selected == assignment->services.end()) {
            return {WorkAdmissionStatus::ServiceMissing};
        }
        if(selected->lifecycle != ServiceLifecycle::Active) {
            return {WorkAdmissionStatus::ServiceInactive};
        }
        if(!contains_capability(*evidence, required_capability(service))) {
            return {WorkAdmissionStatus::CapabilityMissing};
        }
        const auto * health = find_service_health(*evidence, service);
        if(health == nullptr || health->health != ResourceHealth::Healthy) {
            return {WorkAdmissionStatus::ServiceHealthBlocked};
        }
        return {WorkAdmissionStatus::Allowed};
    }

    const char * work_admission_status_name(WorkAdmissionStatus status) noexcept
    {
        switch(status) {
            case WorkAdmissionStatus::Allowed:
                return "Allowed";
            case WorkAdmissionStatus::NoAssignment:
                return "NoAssignment";
            case WorkAdmissionStatus::WrongPrimaryRole:
                return "WrongPrimaryRole";
            case WorkAdmissionStatus::RoleInactive:
                return "RoleInactive";
            case WorkAdmissionStatus::MissingEvidence:
                return "MissingEvidence";
            case WorkAdmissionStatus::HealthBlocked:
                return "HealthBlocked";
            case WorkAdmissionStatus::CapabilityMissing:
                return "CapabilityMissing";
            case WorkAdmissionStatus::TransitionBlocked:
                return "TransitionBlocked";
            case WorkAdmissionStatus::ServiceMissing:
                return "ServiceMissing";
            case WorkAdmissionStatus::ServiceInactive:
                return "ServiceInactive";
            case WorkAdmissionStatus::ServiceHealthBlocked:
                return "ServiceHealthBlocked";
        }
        return "Unknown";
    }

}// namespace SwarmDataPlane
