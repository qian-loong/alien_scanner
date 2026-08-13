#ifndef SWARM_DATA_PLANE_ROS_QOS_PROFILES_HPP
#define SWARM_DATA_PLANE_ROS_QOS_PROFILES_HPP

#include <rclcpp/qos.hpp>

#include <cstddef>
#include <cstdint>

namespace SwarmDataPlane::Ros {

    rclcpp::QoS map_update_qos(std::size_t depth = 4U);
    rclcpp::QoS state_health_qos(
            std::uint64_t deadline_ns,
            std::uint64_t lifespan_ns);
    rclcpp::QoS diagnostic_qos(std::size_t depth = 16U);

}// namespace SwarmDataPlane::Ros

#endif// SWARM_DATA_PLANE_ROS_QOS_PROFILES_HPP
