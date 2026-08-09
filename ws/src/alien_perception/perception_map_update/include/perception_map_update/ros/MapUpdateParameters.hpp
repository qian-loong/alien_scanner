#ifndef PERCEPTION_MAP_UPDATE_ROS_MAP_UPDATE_PARAMETERS_HPP
#define PERCEPTION_MAP_UPDATE_ROS_MAP_UPDATE_PARAMETERS_HPP

#include "perception_map_update/MapUpdateLimits.hpp"

#include "rclcpp/node.hpp"

#include <string>

namespace PerceptionMapUpdate::Ros {

    MapUpdateLimits declare_map_update_limits(
            rclcpp::Node & node,
            const std::string & prefix = "map_update");

}// namespace PerceptionMapUpdate::Ros

#endif// PERCEPTION_MAP_UPDATE_ROS_MAP_UPDATE_PARAMETERS_HPP
