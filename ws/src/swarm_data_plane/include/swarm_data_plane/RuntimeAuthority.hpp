#ifndef SWARM_DATA_PLANE_RUNTIME_AUTHORITY_HPP
#define SWARM_DATA_PLANE_RUNTIME_AUTHORITY_HPP

#include "swarm_data_plane/RoleState.hpp"
#include "swarm_data_plane/TopologyState.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace SwarmDataPlane {

    struct RuntimeRouteFailover {
        std::string route_id;
        RouteDescriptor failover_route;
    };

    struct RuntimeAuthorityConfig {
        VehicleIdentity active_relay;
        VehicleIdentity standby_relay;
        std::vector<RuntimeRouteFailover> route_failovers;
        RoleAssignment failover_assignment;
        std::uint64_t heartbeat_timeout_ns = 0U;
        std::uint64_t initial_time_ns = 0U;
    };

    enum class RuntimeAuthorityStatus : std::uint8_t
    {
        EvidenceApplied,
        EvidenceIgnored,
        FailoverCommitted,
        NoChange,
        RejectedEvidence,
        RejectedClock,
        RejectedFailover
    };

    struct RuntimeAuthorityResult {
        RuntimeAuthorityStatus status = RuntimeAuthorityStatus::NoChange;
        bool state_changed = false;
        std::string diagnostic;

        explicit operator bool() const noexcept
        {
            return status == RuntimeAuthorityStatus::EvidenceApplied
                   || status == RuntimeAuthorityStatus::EvidenceIgnored
                   || status == RuntimeAuthorityStatus::FailoverCommitted
                   || status == RuntimeAuthorityStatus::NoChange;
        }
    };

    class RuntimeAuthority
    {
    public:
        RuntimeAuthority(
                TopologyState topology,
                RoleState roles,
                RuntimeAuthorityConfig config);

        RuntimeAuthorityResult observe_evidence(
                CapabilityEvidence evidence,
                std::uint64_t receive_time_ns);
        RuntimeAuthorityResult tick(std::uint64_t now_ns);
        RoleResult begin_role_transition(
                std::string transition_id,
                RoleCandidate candidate);
        RoleResult acknowledge_role_transition(
                const std::string & transition_id,
                const VehicleIdentity & identity,
                RoleTransitionAckKind kind);
        RoleResult commit_role_transition(const std::string & transition_id);
        RoleResult rollback_role_transition(const std::string & transition_id);

        const TopologySnapshot & topology() const noexcept;
        const RoleSnapshot & roles() const noexcept;
        const RoleTransition * active_transition() const noexcept;
        const RoleTransition * last_transition() const noexcept;
        bool failed_over() const noexcept;

    private:
        struct EvidenceReceipt {
            VehicleIdentity identity;
            std::uint64_t evidence_revision = 0U;
            std::uint64_t receive_time_ns = 0U;
        };

        EvidenceReceipt * find_receipt(const VehicleIdentity & identity) noexcept;
        const EvidenceReceipt * find_receipt(
                const VehicleIdentity & identity) const noexcept;
        RuntimeAuthorityResult commit_failover();

        TopologyState topology_;
        RoleState roles_;
        RuntimeAuthorityConfig config_;
        std::vector<EvidenceReceipt> receipts_;
        bool failed_over_ = false;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_RUNTIME_AUTHORITY_HPP
