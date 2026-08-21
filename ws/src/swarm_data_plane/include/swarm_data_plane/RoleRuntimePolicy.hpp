#ifndef SWARM_DATA_PLANE_ROLE_RUNTIME_POLICY_HPP
#define SWARM_DATA_PLANE_ROLE_RUNTIME_POLICY_HPP

#include "swarm_data_plane/RoleTypes.hpp"

#include <optional>

namespace SwarmDataPlane {

    enum class WorkAdmissionStatus : std::uint8_t
    {
        Allowed,
        NoAssignment,
        WrongPrimaryRole,
        RoleInactive,
        MissingEvidence,
        HealthBlocked,
        CapabilityMissing,
        TransitionBlocked,
        ServiceMissing,
        ServiceInactive,
        ServiceHealthBlocked
    };

    enum class TransitionAdmissionMode : std::uint8_t
    {
        AfterQuiescedAck,
        ChangedMember
    };

    struct WorkAdmissionResult {
        WorkAdmissionStatus status = WorkAdmissionStatus::NoAssignment;

        explicit operator bool() const noexcept
        {
            return status == WorkAdmissionStatus::Allowed;
        }
    };

    WorkAdmissionResult evaluate_role_work_admission(
            const RoleSnapshot & snapshot,
            const CapabilityEvidence * evidence,
            const RoleTransition * transition,
            const VehicleIdentity & identity,
            std::optional<PrimaryRole> required_role = std::nullopt,
            TransitionAdmissionMode transition_mode =
                    TransitionAdmissionMode::AfterQuiescedAck) noexcept;

    WorkAdmissionResult evaluate_service_work_admission(
            const RoleSnapshot & snapshot,
            const CapabilityEvidence * evidence,
            const RoleTransition * transition,
            const VehicleIdentity & identity,
            ServiceKind service,
            TransitionAdmissionMode transition_mode =
                    TransitionAdmissionMode::AfterQuiescedAck) noexcept;

    const char * work_admission_status_name(WorkAdmissionStatus status) noexcept;

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_ROLE_RUNTIME_POLICY_HPP
