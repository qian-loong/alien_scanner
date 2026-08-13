#include "swarm_data_plane/ros/QosProfiles.hpp"

#include <rmw/types.h>

#include <stdexcept>

namespace SwarmDataPlane::Ros {

    namespace {

        rmw_time_t to_rmw_time(std::uint64_t nanoseconds)
        {
            return {nanoseconds / 1'000'000'000U, nanoseconds % 1'000'000'000U};
        }

        void require_positive_depth(std::size_t depth)
        {
            if(depth == 0U) {
                throw std::invalid_argument("QoS history depth must be positive");
            }
        }

    }// namespace

    rclcpp::QoS map_update_qos(std::size_t depth)
    {
        require_positive_depth(depth);
        return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
    }

    rclcpp::QoS state_health_qos(
            std::uint64_t deadline_ns,
            std::uint64_t lifespan_ns)
    {
        if(deadline_ns == 0U || lifespan_ns == 0U) {
            throw std::invalid_argument("state QoS deadline and lifespan must be positive");
        }
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
        qos.deadline(to_rmw_time(deadline_ns));
        qos.lifespan(to_rmw_time(lifespan_ns));
        return qos;
    }

    rclcpp::QoS diagnostic_qos(std::size_t depth)
    {
        require_positive_depth(depth);
        return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
    }

}// namespace SwarmDataPlane::Ros
