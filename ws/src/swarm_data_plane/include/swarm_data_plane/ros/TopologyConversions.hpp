#ifndef SWARM_DATA_PLANE_ROS_TOPOLOGY_CONVERSIONS_HPP
#define SWARM_DATA_PLANE_ROS_TOPOLOGY_CONVERSIONS_HPP

#include "swarm_data_interfaces/msg/topology_snapshot.hpp"
#include "swarm_data_plane/TopologyTypes.hpp"

#include <optional>
#include <string>

namespace SwarmDataPlane::Ros {

    struct DecodeTopologySnapshotResult {
        bool success = false;
        std::optional<TopologySnapshot> snapshot;
        std::string diagnostic;
    };

    bool encode_topology_snapshot(
            const TopologySnapshot & snapshot,
            swarm_data_interfaces::msg::TopologySnapshot & message,
            std::string & diagnostic,
            const TopologyLimits & limits = {});

    DecodeTopologySnapshotResult decode_topology_snapshot(
            const swarm_data_interfaces::msg::TopologySnapshot & message,
            const TopologyLimits & limits = {});

}// namespace SwarmDataPlane::Ros

#endif// SWARM_DATA_PLANE_ROS_TOPOLOGY_CONVERSIONS_HPP
