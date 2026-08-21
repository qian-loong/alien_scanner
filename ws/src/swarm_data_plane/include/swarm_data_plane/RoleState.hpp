#ifndef SWARM_DATA_PLANE_ROLE_STATE_HPP
#define SWARM_DATA_PLANE_ROLE_STATE_HPP

#include "swarm_data_plane/RoleTypes.hpp"

#include <optional>

namespace SwarmDataPlane {

    class RoleState
    {
    public:
        explicit RoleState(
                const TopologySnapshot & topology,
                RoleLimits limits = {});

        RoleResult register_capabilities(
                CapabilityRegistration registration,
                const TopologySnapshot & topology);
        RoleResult update_capability_evidence(
                CapabilityEvidence evidence,
                const TopologySnapshot & topology);
        RoleResult begin_transition(
                std::string transition_id,
                RoleCandidate candidate,
                const TopologySnapshot & topology);
        RoleResult acknowledge_transition(
                const std::string & transition_id,
                const VehicleIdentity & identity,
                RoleTransitionAckKind kind,
                const TopologySnapshot & topology);
        RoleResult commit_transition(
                const std::string & transition_id,
                const TopologySnapshot & topology);
        RoleResult rollback_transition(const std::string & transition_id);

        bool can_accept_new_work(const VehicleIdentity & identity) const noexcept;
        bool can_service_accept_new_work(
                const VehicleIdentity & identity,
                ServiceKind service) const noexcept;

        const RoleSnapshot & snapshot() const noexcept;
        const RoleTransition * active_transition() const noexcept;
        const RoleTransition * last_transition() const noexcept;
        const CapabilityRegistration * find_capability_registration(
                const VehicleIdentity & identity) const noexcept;
        const CapabilityEvidence * find_capability_evidence(
                const VehicleIdentity & identity) const noexcept;

    private:
        struct CapabilityRecord {
            CapabilityRegistration registration;
            std::optional<CapabilityEvidence> evidence;
        };

        RoleResult validate_candidate(
                RoleCandidate & candidate,
                const TopologySnapshot & topology) const;
        RoleResult validate_assignment(
                const RoleAssignment & assignment,
                const TopologySnapshot & topology,
                bool require_current_capability_revision) const;
        std::vector<VehicleIdentity> changed_members(
                const std::vector<RoleAssignment> & target) const;
        std::vector<VehicleIdentity> required_ack_members(
                const RoleTransition & transition,
                const TopologySnapshot & topology) const;
        bool has_ack(
                const RoleTransition & transition,
                const VehicleIdentity & identity,
                RoleTransitionAckKind kind) const noexcept;
        bool transition_id_was_used(const std::string & transition_id) const noexcept;
        void remember_transition_id(std::string transition_id);

        RoleLimits limits_;
        std::vector<CapabilityRecord> capabilities_;
        RoleSnapshot snapshot_;
        std::optional<RoleTransition> active_transition_;
        std::optional<RoleTransition> last_transition_;
        std::vector<std::string> transition_id_history_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_ROLE_STATE_HPP
