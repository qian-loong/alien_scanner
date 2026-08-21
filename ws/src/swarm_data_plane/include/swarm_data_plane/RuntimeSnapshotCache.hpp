#ifndef SWARM_DATA_PLANE_RUNTIME_SNAPSHOT_CACHE_HPP
#define SWARM_DATA_PLANE_RUNTIME_SNAPSHOT_CACHE_HPP

#include "swarm_data_plane/RoleRuntimePolicy.hpp"

#include <optional>
#include <string>

namespace SwarmDataPlane {

    enum class RuntimeSnapshotStatus : std::uint8_t
    {
        Applied,
        IgnoredDuplicate,
        RejectedInvalid,
        RejectedIdentity,
        RejectedStale,
        RejectedConflict
    };

    struct RuntimeSnapshotResult {
        RuntimeSnapshotStatus status = RuntimeSnapshotStatus::RejectedInvalid;
        bool state_changed = false;
        std::string diagnostic;

        explicit operator bool() const noexcept
        {
            return status == RuntimeSnapshotStatus::Applied
                   || status == RuntimeSnapshotStatus::IgnoredDuplicate;
        }
    };

    class RuntimeSnapshotCache
    {
    public:
        explicit RuntimeSnapshotCache(
                VehicleIdentity identity,
                TopologyLimits topology_limits = {},
                RoleLimits role_limits = {});

        RuntimeSnapshotResult apply_topology(TopologySnapshot snapshot);
        RuntimeSnapshotResult apply_role(RoleSnapshot snapshot);
        RuntimeSnapshotResult apply_evidence(CapabilityEvidence evidence);
        RuntimeSnapshotResult apply_transition(RoleTransition transition);

        bool aligned() const noexcept;
        const VehicleIdentity & identity() const noexcept;
        const TopologySnapshot * topology() const noexcept;
        const RoleSnapshot * role() const noexcept;
        const CapabilityEvidence * evidence() const noexcept;
        const RoleTransition * transition() const noexcept;

        WorkAdmissionResult role_admission(
                std::optional<PrimaryRole> required_role = std::nullopt,
                TransitionAdmissionMode transition_mode =
                        TransitionAdmissionMode::ChangedMember) const noexcept;
        WorkAdmissionResult service_admission(
                ServiceKind service,
                TransitionAdmissionMode transition_mode =
                        TransitionAdmissionMode::ChangedMember) const noexcept;

    private:
        VehicleIdentity identity_;
        TopologyLimits topology_limits_;
        RoleLimits role_limits_;
        std::optional<TopologySnapshot> topology_;
        std::optional<RoleSnapshot> role_;
        std::optional<CapabilityEvidence> evidence_;
        std::optional<RoleTransition> transition_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_RUNTIME_SNAPSHOT_CACHE_HPP
