#ifndef SWARM_DATA_PLANE_ROS_ROLE_CONVERSIONS_HPP
#define SWARM_DATA_PLANE_ROS_ROLE_CONVERSIONS_HPP

#include "swarm_data_interfaces/msg/capability_evidence.hpp"
#include "swarm_data_interfaces/msg/capability_registration.hpp"
#include "swarm_data_interfaces/msg/role_snapshot.hpp"
#include "swarm_data_interfaces/msg/role_transition_descriptor.hpp"
#include "swarm_data_plane/RoleTypes.hpp"

#include <optional>
#include <string>

namespace SwarmDataPlane::Ros {

    struct DecodeCapabilityRegistrationResult {
        bool success = false;
        std::optional<CapabilityRegistration> registration;
        std::string diagnostic;
    };

    struct DecodeCapabilityEvidenceResult {
        bool success = false;
        std::optional<CapabilityEvidence> evidence;
        std::string diagnostic;
    };

    struct DecodeRoleSnapshotResult {
        bool success = false;
        std::optional<RoleSnapshot> snapshot;
        std::string diagnostic;
    };

    struct DecodeRoleTransitionResult {
        bool success = false;
        std::optional<RoleTransition> transition;
        std::string diagnostic;
    };

    bool encode_capability_registration(
            const CapabilityRegistration & registration,
            swarm_data_interfaces::msg::CapabilityRegistration & message,
            std::string & diagnostic,
            const RoleLimits & limits = {});

    DecodeCapabilityRegistrationResult decode_capability_registration(
            const swarm_data_interfaces::msg::CapabilityRegistration & message,
            const RoleLimits & limits = {});

    bool encode_capability_evidence(
            const CapabilityEvidence & evidence,
            swarm_data_interfaces::msg::CapabilityEvidence & message,
            std::string & diagnostic,
            const RoleLimits & limits = {});

    DecodeCapabilityEvidenceResult decode_capability_evidence(
            const swarm_data_interfaces::msg::CapabilityEvidence & message,
            const RoleLimits & limits = {});

    bool encode_role_snapshot(
            const RoleSnapshot & snapshot,
            swarm_data_interfaces::msg::RoleSnapshot & message,
            std::string & diagnostic,
            const RoleLimits & limits = {});

    DecodeRoleSnapshotResult decode_role_snapshot(
            const swarm_data_interfaces::msg::RoleSnapshot & message,
            const RoleLimits & limits = {});

    bool encode_role_transition(
            const RoleTransition & transition,
            swarm_data_interfaces::msg::RoleTransitionDescriptor & message,
            std::string & diagnostic,
            const RoleLimits & limits = {});

    DecodeRoleTransitionResult decode_role_transition(
            const swarm_data_interfaces::msg::RoleTransitionDescriptor & message,
            const RoleLimits & limits = {});

}// namespace SwarmDataPlane::Ros

#endif// SWARM_DATA_PLANE_ROS_ROLE_CONVERSIONS_HPP
