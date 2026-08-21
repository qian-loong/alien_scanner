#ifndef SWARM_DATA_PLANE_TOPOLOGY_STATE_HPP
#define SWARM_DATA_PLANE_TOPOLOGY_STATE_HPP

#include "swarm_data_plane/TopologyTypes.hpp"

#include <optional>

namespace SwarmDataPlane {

    class TopologyState
    {
    public:
        TopologyState(
                std::string fleet_id,
                std::vector<WhitelistEntry> whitelist,
                TopologyLimits limits = {});

        TopologyResult register_vehicle(VehicleRegistration registration);
        TopologyResult set_prerequisites(
                const VehicleIdentity & identity,
                ResyncPrerequisites prerequisites);
        TopologyResult transition_member(
                const VehicleIdentity & identity,
                MembershipState target);
        TopologyResult replace_topology(TopologyCandidate candidate);
        TopologyResult validate_route(
                const RouteDescriptor & route,
                std::uint16_t current_hop,
                std::uint64_t accumulated_forwarding_ns) const;

        const TopologySnapshot & snapshot() const noexcept;
        const MemberRecord * find_member(const VehicleIdentity & identity) const noexcept;
        const MemberRecord * find_member(const std::string & vehicle_id) const noexcept;

    private:
        TopologyResult commit_registration(VehicleRegistration registration);
        TopologyResult advance_epoch(TopologySnapshot candidate);
        void prune_identity(TopologySnapshot & candidate, const VehicleIdentity & identity) const;
        bool is_retired(const VehicleIdentity & identity) const noexcept;

        TopologyLimits limits_;
        std::vector<WhitelistEntry> whitelist_;
        std::vector<VehicleIdentity> retired_sessions_;
        std::vector<LinkDescriptor> link_epoch_history_;
        std::vector<RouteDescriptor> route_epoch_history_;
        TopologySnapshot snapshot_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_TOPOLOGY_STATE_HPP
