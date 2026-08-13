#ifndef SWARM_DATA_PLANE_ROS_ROUTED_MAP_CONVERSIONS_HPP
#define SWARM_DATA_PLANE_ROS_ROUTED_MAP_CONVERSIONS_HPP

#include "perception_map_update/MapUpdateLimits.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"
#include "swarm_data_interfaces/msg/routed_map_update.hpp"

#include <optional>
#include <string>

namespace SwarmDataPlane::Ros {

    struct DecodeRoutedMapResult {
        bool success = false;
        std::optional<RoutedMapUpdate> message;
        std::string diagnostic;
    };

    bool encode_routed_map_update(
            const RoutedMapUpdate & message,
            swarm_data_interfaces::msg::RoutedMapUpdate & output,
            std::string & diagnostic,
            const DataPlaneLimits & limits = {});

    DecodeRoutedMapResult decode_routed_map_update(
            const swarm_data_interfaces::msg::RoutedMapUpdate & message,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});

}// namespace SwarmDataPlane::Ros

#endif// SWARM_DATA_PLANE_ROS_ROUTED_MAP_CONVERSIONS_HPP
