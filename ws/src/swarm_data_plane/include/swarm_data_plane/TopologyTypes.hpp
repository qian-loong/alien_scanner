#ifndef SWARM_DATA_PLANE_TOPOLOGY_TYPES_HPP
#define SWARM_DATA_PLANE_TOPOLOGY_TYPES_HPP

#include "perception_core/types/identity.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace SwarmDataPlane {

    constexpr std::uint16_t kTopologyProtocolVersion = 1U;

    enum class MembershipState : std::uint8_t
    {
        Absent      = 1,
        Joining     = 2,
        Resyncing   = 3,
        Ready       = 4,
        Draining    = 5,
        Lost        = 6,
        Quarantined = 7
    };

    enum class MemberAvailability : std::uint8_t
    {
        Removed = 1,
        Live    = 2,
        Frozen  = 3
    };

    enum class LogicalGraphKind : std::uint8_t
    {
        Communication = 1,
        Control       = 2,
        Map           = 3
    };

    enum class LinkHealth : std::uint8_t
    {
        Unknown  = 1,
        Up       = 2,
        Degraded = 3,
        Down     = 4
    };

    struct VehicleIdentity {
        std::string           fleet_id;
        std::string           vehicle_id;
        Perception::SessionID session {0U, 0U};

        bool operator==(const VehicleIdentity & other) const noexcept;
        bool operator!=(const VehicleIdentity & other) const noexcept;
        bool operator<(const VehicleIdentity & other) const noexcept;
    };

    struct ComponentRegistration {
        std::string           component_id;
        Perception::SessionID session {0U, 0U};

        bool operator==(const ComponentRegistration & other) const noexcept;
        bool operator<(const ComponentRegistration & other) const noexcept;
    };

    struct SensorDescriptorIdentity {
        std::string                      sensor_id;
        PerceptionMapUpdate::Hash256     descriptor_hash {};

        bool operator==(const SensorDescriptorIdentity & other) const noexcept;
        bool operator<(const SensorDescriptorIdentity & other) const noexcept;
    };

    struct VehicleRegistration {
        VehicleIdentity                       identity;
        std::uint64_t                         registration_generation = 0U;
        std::vector<ComponentRegistration>    components;
        std::vector<SensorDescriptorIdentity> sensors;

        bool operator==(const VehicleRegistration & other) const noexcept;
    };

    struct ResyncPrerequisites {
        bool link_ready      = false;
        bool clock_ready     = false;
        bool alignment_ready = false;
        bool map_ready       = false;

        bool ready() const noexcept;
        bool operator==(const ResyncPrerequisites & other) const noexcept;
    };

    struct MemberRecord {
        VehicleRegistration registration;
        MembershipState     state = MembershipState::Absent;
        MemberAvailability  availability = MemberAvailability::Removed;
        ResyncPrerequisites prerequisites;

        bool operator==(const MemberRecord & other) const noexcept;
        bool operator<(const MemberRecord & other) const noexcept;
    };

    struct WhitelistEntry {
        std::string vehicle_id;
        std::size_t max_components = 32U;
        std::size_t max_sensors = 32U;

        bool operator==(const WhitelistEntry & other) const noexcept;
        bool operator<(const WhitelistEntry & other) const noexcept;
    };

    struct LinkDescriptor {
        std::string      link_id;
        std::uint64_t    link_epoch = 0U;
        VehicleIdentity  source;
        VehicleIdentity  target;
        LinkHealth       health = LinkHealth::Unknown;
        std::uint64_t    latency_budget_ns = 0U;
        std::uint64_t    bandwidth_budget_bps = 0U;
        std::uint32_t    loss_budget_ppm = 0U;

        bool operator==(const LinkDescriptor & other) const noexcept;
        bool operator<(const LinkDescriptor & other) const noexcept;
    };

    struct GraphEdge {
        LogicalGraphKind graph = LogicalGraphKind::Communication;
        std::string      link_id;
        std::uint64_t    link_epoch = 0U;
        VehicleIdentity  source;
        VehicleIdentity  target;

        bool operator==(const GraphEdge & other) const noexcept;
        bool operator<(const GraphEdge & other) const noexcept;
    };

    struct RouteHop {
        std::string   link_id;
        std::uint64_t link_epoch = 0U;

        bool operator==(const RouteHop & other) const noexcept;
        bool operator<(const RouteHop & other) const noexcept;
    };

    struct RouteDescriptor {
        std::string           route_id;
        LogicalGraphKind      graph = LogicalGraphKind::Control;
        VehicleIdentity       source;
        VehicleIdentity       target;
        std::uint64_t         topology_epoch = 0U;
        std::uint64_t         route_epoch = 0U;
        std::uint16_t         ttl_hops = 0U;
        std::uint64_t         validity_budget_ns = 0U;
        std::vector<RouteHop> hops;

        bool operator==(const RouteDescriptor & other) const noexcept;
        bool operator<(const RouteDescriptor & other) const noexcept;
    };

    struct TopologySnapshot {
        std::uint16_t              protocol_version = kTopologyProtocolVersion;
        std::string                fleet_id;
        std::uint64_t              topology_epoch = 0U;
        std::vector<MemberRecord>  members;
        std::vector<LinkDescriptor> links;
        std::vector<GraphEdge>      edges;
        std::vector<RouteDescriptor> routes;

        bool operator==(const TopologySnapshot & other) const noexcept;
    };

    struct TopologyCandidate {
        std::uint64_t               base_topology_epoch = 0U;
        std::vector<LinkDescriptor> links;
        std::vector<GraphEdge>      edges;
        std::vector<RouteDescriptor> routes;
    };

    struct TopologyLimits {
        std::size_t max_identity_bytes = 128U;
        std::size_t max_members = 64U;
        std::size_t max_retired_sessions = 128U;
        std::size_t max_components_per_member = 32U;
        std::size_t max_sensors_per_member = 32U;
        std::size_t max_links = 256U;
        std::size_t max_edges = 512U;
        std::size_t max_routes = 256U;
        std::size_t max_link_epoch_history = 512U;
        std::size_t max_route_epoch_history = 512U;
        std::size_t max_route_hops = 64U;
        std::uint16_t max_ttl_hops = 64U;
        std::uint64_t max_validity_budget_ns = 60'000'000'000U;
    };

    enum class TopologyStatus : std::uint8_t
    {
        Applied,
        IgnoredDuplicate,
        RejectedInvalid,
        RejectedAdmission,
        RejectedStale,
        RejectedConflict,
        RejectedTransition,
        RejectedPrerequisite,
        RejectedResourceLimit,
        RejectedRoute
    };

    struct TopologyResult {
        TopologyStatus status = TopologyStatus::RejectedInvalid;
        bool           state_changed = false;
        std::string    diagnostic;

        explicit operator bool() const noexcept
        {
            return status == TopologyStatus::Applied
                   || status == TopologyStatus::IgnoredDuplicate;
        }
    };

    MemberAvailability availability_for(MembershipState state) noexcept;
    TopologyResult validate_topology_snapshot(
            const TopologySnapshot & snapshot,
            const TopologyLimits & limits = {});

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_TOPOLOGY_TYPES_HPP
