#include "swarm_data_plane/ros/RoleConversions.hpp"

#include "swarm_data_plane/ros/VehicleIdentityConversions.hpp"

#include <algorithm>
#include <utility>

namespace SwarmDataPlane::Ros {

    namespace {

        constexpr std::size_t kRosMaxCapabilities = 8U;
        constexpr std::size_t kRosMaxAssignments = 64U;
        constexpr std::size_t kRosMaxServicesPerAssignment = 2U;
        constexpr std::size_t kRosMaxTransitionMembers = 64U;
        constexpr std::size_t kRosMaxAcknowledgements = 128U;
        constexpr std::size_t kRosMaxIdentityBytes = 128U;
        constexpr std::size_t kRosMaxTransitionIdBytes = 128U;

        RoleLimits bounded_ros_limits(const RoleLimits & limits)
        {
            RoleLimits bounded = limits;
            bounded.max_identity_bytes = std::min(
                    bounded.max_identity_bytes, kRosMaxIdentityBytes);
            bounded.max_transition_id_bytes = std::min(
                    bounded.max_transition_id_bytes, kRosMaxTransitionIdBytes);
            bounded.max_capabilities = std::min(
                    bounded.max_capabilities, kRosMaxCapabilities);
            bounded.max_assignments = std::min(
                    bounded.max_assignments, kRosMaxAssignments);
            bounded.max_services_per_assignment = std::min(
                    bounded.max_services_per_assignment,
                    kRosMaxServicesPerAssignment);
            bounded.max_transition_members = std::min(
                    bounded.max_transition_members, kRosMaxTransitionMembers);
            bounded.max_acknowledgements = std::min(
                    bounded.max_acknowledgements, kRosMaxAcknowledgements);
            return bounded;
        }

        bool identity_size_within_limit(
                const swarm_data_interfaces::msg::VehicleIdentity & identity,
                std::size_t max_bytes) noexcept
        {
            return identity.fleet_id.size() <= max_bytes
                   && identity.vehicle_id.size() <= max_bytes;
        }

        void encode_service_budget(
                const ServiceBudget & budget,
                swarm_data_interfaces::msg::ServiceBudget & message)
        {
            message.queue_bytes = budget.queue_bytes;
            message.memory_bytes = budget.memory_bytes;
            message.network_bits_per_second = budget.network_bits_per_second;
            message.max_contributors = budget.max_contributors;
            message.max_parallel_work = budget.max_parallel_work;
        }

        ServiceBudget decode_service_budget(
                const swarm_data_interfaces::msg::ServiceBudget & message) noexcept
        {
            return {
                    message.queue_bytes,
                    message.memory_bytes,
                    message.network_bits_per_second,
                    message.max_contributors,
                    message.max_parallel_work};
        }

        void encode_service_assignment(
                const ServiceAssignment & assignment,
                swarm_data_interfaces::msg::ServiceAssignment & message)
        {
            message.service = static_cast<std::uint8_t>(assignment.service);
            message.lifecycle = static_cast<std::uint8_t>(assignment.lifecycle);
            encode_service_budget(assignment.budget, message.budget);
        }

        ServiceAssignment decode_service_assignment(
                const swarm_data_interfaces::msg::ServiceAssignment & message)
        {
            return {
                    static_cast<ServiceKind>(message.service),
                    static_cast<ServiceLifecycle>(message.lifecycle),
                    decode_service_budget(message.budget)};
        }

        void encode_role_assignment(
                const RoleAssignment & assignment,
                swarm_data_interfaces::msg::RoleAssignment & message)
        {
            Detail::encode_vehicle_identity(assignment.identity, message.identity);
            message.capability_revision = assignment.capability_revision;
            message.primary_role = static_cast<std::uint8_t>(assignment.primary_role);
            message.lifecycle = static_cast<std::uint8_t>(assignment.lifecycle);
            message.services.clear();
            message.services.reserve(assignment.services.size());
            for(const auto & service : assignment.services) {
                swarm_data_interfaces::msg::ServiceAssignment converted;
                encode_service_assignment(service, converted);
                message.services.push_back(std::move(converted));
            }
        }

        RoleAssignment decode_role_assignment(
                const swarm_data_interfaces::msg::RoleAssignment & message)
        {
            RoleAssignment assignment;
            assignment.identity = Detail::decode_vehicle_identity(message.identity);
            assignment.capability_revision = message.capability_revision;
            assignment.primary_role = static_cast<PrimaryRole>(message.primary_role);
            assignment.lifecycle = static_cast<RoleLifecycle>(message.lifecycle);
            assignment.services.reserve(message.services.size());
            for(const auto & service : message.services) {
                assignment.services.push_back(decode_service_assignment(service));
            }
            return assignment;
        }

        bool assignment_sizes_within_limits(
                const swarm_data_interfaces::msg::RoleAssignment & assignment,
                const RoleLimits & limits,
                std::string & diagnostic)
        {
            if(!identity_size_within_limit(
                       assignment.identity, limits.max_identity_bytes)) {
                diagnostic = "role assignment identity exceeds configured limits";
                return false;
            }
            if(assignment.services.size() > limits.max_services_per_assignment) {
                diagnostic = "role assignment service collection exceeds configured limits";
                return false;
            }
            return true;
        }

        bool registration_sizes_within_limits(
                const swarm_data_interfaces::msg::CapabilityRegistration & message,
                const RoleLimits & limits,
                std::string & diagnostic)
        {
            if(!identity_size_within_limit(message.identity, limits.max_identity_bytes)) {
                diagnostic = "capability registration identity exceeds configured limits";
                return false;
            }
            if(message.declared_capabilities.size() > limits.max_capabilities) {
                diagnostic = "declared capability collection exceeds configured limits";
                return false;
            }
            return true;
        }

        bool evidence_sizes_within_limits(
                const swarm_data_interfaces::msg::CapabilityEvidence & message,
                const RoleLimits & limits,
                std::string & diagnostic)
        {
            if(!identity_size_within_limit(message.identity, limits.max_identity_bytes)) {
                diagnostic = "capability evidence identity exceeds configured limits";
                return false;
            }
            if(message.effective_capabilities.size() > limits.max_capabilities
               || message.service_health.size()
                          > limits.max_services_per_assignment) {
                diagnostic = "capability evidence collection exceeds configured limits";
                return false;
            }
            return true;
        }

        bool snapshot_sizes_within_limits(
                const swarm_data_interfaces::msg::RoleSnapshot & message,
                const RoleLimits & limits,
                std::string & diagnostic)
        {
            if(message.fleet_id.size() > limits.max_identity_bytes) {
                diagnostic = "role snapshot fleet identity exceeds configured limits";
                return false;
            }
            if(message.assignments.size() > limits.max_assignments) {
                diagnostic = "role snapshot assignment collection exceeds configured limits";
                return false;
            }
            for(const auto & assignment : message.assignments) {
                if(!assignment_sizes_within_limits(
                           assignment, limits, diagnostic)) {
                    return false;
                }
            }
            return true;
        }

        bool transition_sizes_within_limits(
                const swarm_data_interfaces::msg::RoleTransitionDescriptor & message,
                const RoleLimits & limits,
                std::string & diagnostic)
        {
            if(message.transition_id.size() > limits.max_transition_id_bytes) {
                diagnostic = "role transition identity exceeds configured limits";
                return false;
            }
            if(message.target_assignments.size() > limits.max_assignments
               || message.changed_members.size() > limits.max_transition_members
               || message.acknowledgements.size() > limits.max_acknowledgements) {
                diagnostic = "role transition collection exceeds configured limits";
                return false;
            }
            for(const auto & assignment : message.target_assignments) {
                if(!assignment_sizes_within_limits(
                           assignment, limits, diagnostic)) {
                    return false;
                }
            }
            for(const auto & identity : message.changed_members) {
                if(!identity_size_within_limit(identity, limits.max_identity_bytes)) {
                    diagnostic = "changed member identity exceeds configured limits";
                    return false;
                }
            }
            for(const auto & acknowledgement : message.acknowledgements) {
                if(!identity_size_within_limit(
                           acknowledgement.identity, limits.max_identity_bytes)) {
                    diagnostic = "transition acknowledgement identity exceeds configured limits";
                    return false;
                }
            }
            return true;
        }

    }// namespace

    bool encode_capability_registration(
            const CapabilityRegistration & registration,
            swarm_data_interfaces::msg::CapabilityRegistration & message,
            std::string & diagnostic,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        const auto validation = validate_capability_registration(
                registration, bounded);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }
        message = swarm_data_interfaces::msg::CapabilityRegistration {};
        message.protocol_version = kRoleProtocolVersion;
        Detail::encode_vehicle_identity(registration.identity, message.identity);
        message.registration_generation = registration.registration_generation;
        message.declared_capabilities.reserve(
                registration.declared_capabilities.size());
        for(const auto capability : registration.declared_capabilities) {
            message.declared_capabilities.push_back(
                    static_cast<std::uint8_t>(capability));
        }
        diagnostic.clear();
        return true;
    }

    DecodeCapabilityRegistrationResult decode_capability_registration(
            const swarm_data_interfaces::msg::CapabilityRegistration & message,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        std::string diagnostic;
        if(message.protocol_version != kRoleProtocolVersion) {
            return {false, std::nullopt,
                    "capability registration protocol version is unsupported"};
        }
        if(!registration_sizes_within_limits(message, bounded, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }
        CapabilityRegistration decoded;
        decoded.identity = Detail::decode_vehicle_identity(message.identity);
        decoded.registration_generation = message.registration_generation;
        decoded.declared_capabilities.reserve(message.declared_capabilities.size());
        for(const auto capability : message.declared_capabilities) {
            decoded.declared_capabilities.push_back(
                    static_cast<CapabilityKind>(capability));
        }
        const auto validation = validate_capability_registration(decoded, bounded);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

    bool encode_capability_evidence(
            const CapabilityEvidence & evidence,
            swarm_data_interfaces::msg::CapabilityEvidence & message,
            std::string & diagnostic,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        const auto validation = validate_capability_evidence(evidence, bounded);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }
        message = swarm_data_interfaces::msg::CapabilityEvidence {};
        message.protocol_version = kRoleProtocolVersion;
        Detail::encode_vehicle_identity(evidence.identity, message.identity);
        message.evidence_revision = evidence.evidence_revision;
        message.vehicle_health = static_cast<std::uint8_t>(evidence.vehicle_health);
        message.resource_health = static_cast<std::uint8_t>(evidence.resource_health);
        message.effective_capabilities.reserve(
                evidence.effective_capabilities.size());
        for(const auto capability : evidence.effective_capabilities) {
            message.effective_capabilities.push_back(
                    static_cast<std::uint8_t>(capability));
        }
        message.service_health.reserve(evidence.service_health.size());
        for(const auto & service : evidence.service_health) {
            swarm_data_interfaces::msg::ServiceHealthEvidence converted;
            converted.service = static_cast<std::uint8_t>(service.service);
            converted.health = static_cast<std::uint8_t>(service.health);
            message.service_health.push_back(std::move(converted));
        }
        diagnostic.clear();
        return true;
    }

    DecodeCapabilityEvidenceResult decode_capability_evidence(
            const swarm_data_interfaces::msg::CapabilityEvidence & message,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        std::string diagnostic;
        if(message.protocol_version != kRoleProtocolVersion) {
            return {false, std::nullopt,
                    "capability evidence protocol version is unsupported"};
        }
        if(!evidence_sizes_within_limits(message, bounded, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }
        CapabilityEvidence decoded;
        decoded.identity = Detail::decode_vehicle_identity(message.identity);
        decoded.evidence_revision = message.evidence_revision;
        decoded.vehicle_health = static_cast<VehicleHealth>(message.vehicle_health);
        decoded.resource_health = static_cast<ResourceHealth>(message.resource_health);
        decoded.effective_capabilities.reserve(
                message.effective_capabilities.size());
        for(const auto capability : message.effective_capabilities) {
            decoded.effective_capabilities.push_back(
                    static_cast<CapabilityKind>(capability));
        }
        decoded.service_health.reserve(message.service_health.size());
        for(const auto & service : message.service_health) {
            decoded.service_health.push_back({
                    static_cast<ServiceKind>(service.service),
                    static_cast<ResourceHealth>(service.health)});
        }
        const auto validation = validate_capability_evidence(decoded, bounded);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

    bool encode_role_snapshot(
            const RoleSnapshot & snapshot,
            swarm_data_interfaces::msg::RoleSnapshot & message,
            std::string & diagnostic,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        const auto validation = validate_role_snapshot(snapshot, bounded);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }
        message = swarm_data_interfaces::msg::RoleSnapshot {};
        message.protocol_version = snapshot.protocol_version;
        message.fleet_id = snapshot.fleet_id;
        message.topology_epoch = snapshot.topology_epoch;
        message.role_epoch = snapshot.role_epoch;
        message.assignments.reserve(snapshot.assignments.size());
        for(const auto & assignment : snapshot.assignments) {
            swarm_data_interfaces::msg::RoleAssignment converted;
            encode_role_assignment(assignment, converted);
            message.assignments.push_back(std::move(converted));
        }
        diagnostic.clear();
        return true;
    }

    DecodeRoleSnapshotResult decode_role_snapshot(
            const swarm_data_interfaces::msg::RoleSnapshot & message,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        std::string diagnostic;
        if(!snapshot_sizes_within_limits(message, bounded, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }
        RoleSnapshot decoded;
        decoded.protocol_version = message.protocol_version;
        decoded.fleet_id = message.fleet_id;
        decoded.topology_epoch = message.topology_epoch;
        decoded.role_epoch = message.role_epoch;
        decoded.assignments.reserve(message.assignments.size());
        for(const auto & assignment : message.assignments) {
            decoded.assignments.push_back(decode_role_assignment(assignment));
        }
        const auto validation = validate_role_snapshot(decoded, bounded);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

    bool encode_role_transition(
            const RoleTransition & transition,
            swarm_data_interfaces::msg::RoleTransitionDescriptor & message,
            std::string & diagnostic,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        const auto fleet_id = transition.changed_members.empty()
                                      ? std::string {}
                                      : transition.changed_members.front().fleet_id;
        const auto validation = validate_role_transition(
                transition, fleet_id, bounded);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }
        message = swarm_data_interfaces::msg::RoleTransitionDescriptor {};
        message.protocol_version = kRoleProtocolVersion;
        message.transition_id = transition.transition_id;
        message.state = static_cast<std::uint8_t>(transition.state);
        message.base_role_epoch = transition.base_role_epoch;
        message.topology_epoch = transition.topology_epoch;
        message.target_assignments.reserve(transition.target_assignments.size());
        for(const auto & assignment : transition.target_assignments) {
            swarm_data_interfaces::msg::RoleAssignment converted;
            encode_role_assignment(assignment, converted);
            message.target_assignments.push_back(std::move(converted));
        }
        message.changed_members.reserve(transition.changed_members.size());
        for(const auto & identity : transition.changed_members) {
            swarm_data_interfaces::msg::VehicleIdentity converted;
            Detail::encode_vehicle_identity(identity, converted);
            message.changed_members.push_back(std::move(converted));
        }
        message.acknowledgements.reserve(transition.acknowledgements.size());
        for(const auto & acknowledgement : transition.acknowledgements) {
            swarm_data_interfaces::msg::RoleTransitionAck converted;
            Detail::encode_vehicle_identity(
                    acknowledgement.identity, converted.identity);
            converted.kind = static_cast<std::uint8_t>(acknowledgement.kind);
            message.acknowledgements.push_back(std::move(converted));
        }
        diagnostic.clear();
        return true;
    }

    DecodeRoleTransitionResult decode_role_transition(
            const swarm_data_interfaces::msg::RoleTransitionDescriptor & message,
            const RoleLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        std::string diagnostic;
        if(message.protocol_version != kRoleProtocolVersion) {
            return {false, std::nullopt,
                    "role transition protocol version is unsupported"};
        }
        if(!transition_sizes_within_limits(message, bounded, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }
        RoleTransition decoded;
        decoded.transition_id = message.transition_id;
        decoded.state = static_cast<RoleTransitionState>(message.state);
        decoded.base_role_epoch = message.base_role_epoch;
        decoded.topology_epoch = message.topology_epoch;
        decoded.target_assignments.reserve(message.target_assignments.size());
        for(const auto & assignment : message.target_assignments) {
            decoded.target_assignments.push_back(
                    decode_role_assignment(assignment));
        }
        decoded.changed_members.reserve(message.changed_members.size());
        for(const auto & identity : message.changed_members) {
            decoded.changed_members.push_back(
                    Detail::decode_vehicle_identity(identity));
        }
        decoded.acknowledgements.reserve(message.acknowledgements.size());
        for(const auto & acknowledgement : message.acknowledgements) {
            decoded.acknowledgements.push_back({
                    Detail::decode_vehicle_identity(acknowledgement.identity),
                    static_cast<RoleTransitionAckKind>(acknowledgement.kind)});
        }
        const auto fleet_id = decoded.changed_members.empty()
                                      ? std::string {}
                                      : decoded.changed_members.front().fleet_id;
        const auto validation = validate_role_transition(
                decoded, fleet_id, bounded);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

}// namespace SwarmDataPlane::Ros
