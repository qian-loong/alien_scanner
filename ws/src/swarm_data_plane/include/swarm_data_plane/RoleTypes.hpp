#ifndef SWARM_DATA_PLANE_ROLE_TYPES_HPP
#define SWARM_DATA_PLANE_ROLE_TYPES_HPP

#include "swarm_data_plane/TopologyTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace SwarmDataPlane {

    constexpr std::uint16_t kRoleProtocolVersion = 1U;

    enum class CapabilityKind : std::uint8_t
    {
        Exploration     = 1,
        RelayForwarding = 2,
        MapAggregation  = 3
    };

    enum class VehicleHealth : std::uint8_t
    {
        Unknown  = 1,
        Healthy  = 2,
        Degraded = 3,
        LowPower = 4,
        Failsafe = 5
    };

    enum class ResourceHealth : std::uint8_t
    {
        Unknown           = 1,
        Healthy           = 2,
        ComputeOverBudget = 3,
        MemoryOverBudget  = 4,
        LinkDegraded      = 5
    };

    enum class PrimaryRole : std::uint8_t
    {
        Explorer       = 1,
        Relay          = 2,
        EdgeAggregator = 3,
        Reserve        = 4
    };

    enum class RoleLifecycle : std::uint8_t
    {
        Active   = 1,
        Draining = 2
    };

    enum class ServiceKind : std::uint8_t
    {
        Relay       = 1,
        Aggregation = 2
    };

    enum class ServiceLifecycle : std::uint8_t
    {
        Active   = 1,
        Degraded = 2,
        Draining = 3
    };

    enum class RoleTransitionState : std::uint8_t
    {
        Prepared     = 1,
        Quiescing    = 2,
        HandoffReady = 3,
        Committed    = 4,
        RolledBack   = 5
    };

    enum class RoleTransitionAckKind : std::uint8_t
    {
        Quiesced    = 1,
        HandoffReady = 2
    };

    struct CapabilityRegistration {
        VehicleIdentity             identity;
        std::uint64_t               registration_generation = 0U;
        std::vector<CapabilityKind> declared_capabilities;

        bool operator==(const CapabilityRegistration & other) const noexcept;
    };

    struct ServiceHealthEvidence {
        ServiceKind    service = ServiceKind::Relay;
        ResourceHealth health = ResourceHealth::Unknown;

        bool operator==(const ServiceHealthEvidence & other) const noexcept;
        bool operator<(const ServiceHealthEvidence & other) const noexcept;
    };

    struct CapabilityEvidence {
        VehicleIdentity                    identity;
        std::uint64_t                      evidence_revision = 0U;
        std::vector<CapabilityKind>        effective_capabilities;
        VehicleHealth                     vehicle_health = VehicleHealth::Unknown;
        ResourceHealth                    resource_health = ResourceHealth::Unknown;
        std::vector<ServiceHealthEvidence> service_health;

        bool operator==(const CapabilityEvidence & other) const noexcept;
    };

    struct ServiceBudget {
        std::uint64_t queue_bytes = 0U;
        std::uint64_t memory_bytes = 0U;
        std::uint64_t network_bits_per_second = 0U;
        std::uint32_t max_contributors = 0U;
        std::uint32_t max_parallel_work = 0U;

        bool operator==(const ServiceBudget & other) const noexcept;
    };

    struct ServiceAssignment {
        ServiceKind      service = ServiceKind::Relay;
        ServiceLifecycle lifecycle = ServiceLifecycle::Active;
        ServiceBudget    budget;

        bool operator==(const ServiceAssignment & other) const noexcept;
        bool operator<(const ServiceAssignment & other) const noexcept;
    };

    struct RoleAssignment {
        VehicleIdentity               identity;
        std::uint64_t                 capability_revision = 0U;
        PrimaryRole                  primary_role = PrimaryRole::Reserve;
        RoleLifecycle                lifecycle = RoleLifecycle::Active;
        std::vector<ServiceAssignment> services;

        bool operator==(const RoleAssignment & other) const noexcept;
        bool operator<(const RoleAssignment & other) const noexcept;
    };

    struct RoleSnapshot {
        std::uint16_t               protocol_version = kRoleProtocolVersion;
        std::string                 fleet_id;
        std::uint64_t               topology_epoch = 0U;
        std::uint64_t               role_epoch = 0U;
        std::vector<RoleAssignment> assignments;

        bool operator==(const RoleSnapshot & other) const noexcept;
    };

    struct RoleCandidate {
        std::uint64_t               base_role_epoch = 0U;
        std::uint64_t               topology_epoch = 0U;
        std::vector<RoleAssignment> assignments;
    };

    struct RoleTransitionAck {
        VehicleIdentity       identity;
        RoleTransitionAckKind kind = RoleTransitionAckKind::Quiesced;

        bool operator==(const RoleTransitionAck & other) const noexcept;
        bool operator<(const RoleTransitionAck & other) const noexcept;
    };

    struct RoleTransition {
        std::string                  transition_id;
        RoleTransitionState         state = RoleTransitionState::Prepared;
        std::uint64_t                base_role_epoch = 0U;
        std::uint64_t                topology_epoch = 0U;
        std::vector<RoleAssignment> target_assignments;
        std::vector<VehicleIdentity> changed_members;
        std::vector<RoleTransitionAck> acknowledgements;

        bool operator==(const RoleTransition & other) const noexcept;
    };

    struct RoleLimits {
        std::size_t max_identity_bytes = 128U;
        std::size_t max_transition_id_bytes = 128U;
        std::size_t max_capabilities = 8U;
        std::size_t max_capability_registrations = 64U;
        std::size_t max_assignments = 64U;
        std::size_t max_services_per_assignment = 2U;
        std::size_t max_transition_members = 64U;
        std::size_t max_acknowledgements = 128U;
        std::size_t max_transition_history = 128U;
        std::uint64_t max_queue_bytes = 1ULL << 34U;
        std::uint64_t max_memory_bytes = 1ULL << 40U;
        std::uint64_t max_network_bits_per_second = 1'000'000'000'000ULL;
        std::uint32_t max_contributors = 4096U;
        std::uint32_t max_parallel_work = 4096U;
    };

    enum class RoleStatus : std::uint8_t
    {
        Applied,
        IgnoredDuplicate,
        RejectedInvalid,
        RejectedAdmission,
        RejectedStale,
        RejectedConflict,
        RejectedCombination,
        RejectedCapability,
        RejectedHealth,
        RejectedResourceLimit,
        RejectedTransition,
        RejectedPrerequisite
    };

    struct RoleResult {
        RoleStatus  status = RoleStatus::RejectedInvalid;
        bool        state_changed = false;
        std::string diagnostic;

        explicit operator bool() const noexcept
        {
            return status == RoleStatus::Applied
                   || status == RoleStatus::IgnoredDuplicate;
        }
    };

    std::optional<CapabilityKind> required_capability(PrimaryRole role) noexcept;
    CapabilityKind required_capability(ServiceKind service) noexcept;

    RoleResult validate_capability_registration(
            const CapabilityRegistration & registration,
            const RoleLimits & limits = {});
    RoleResult validate_capability_evidence(
            const CapabilityEvidence & evidence,
            const RoleLimits & limits = {});
    RoleResult validate_role_snapshot(
            const RoleSnapshot & snapshot,
            const RoleLimits & limits = {});
    RoleResult validate_role_transition(
            const RoleTransition & transition,
            const std::string & fleet_id,
            const RoleLimits & limits = {});

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_ROLE_TYPES_HPP
