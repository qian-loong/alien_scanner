#ifndef PERCEPTION_PROFILING_PROFILE_ROS_CONVERSIONS_HPP
#define PERCEPTION_PROFILING_PROFILE_ROS_CONVERSIONS_HPP

#include "perception_profiling/ProfileDataSink.hpp"
#include "perception_profiling/ProfileScenario.hpp"

#include "perception_interfaces/msg/health_state.hpp"
#include "perception_interfaces/msg/lidar_observation.hpp"
#include "perception_interfaces/msg/local_map_state.hpp"
#include "perception_interfaces/msg/pose_estimate.hpp"

#include "builtin_interfaces/msg/time.hpp"

namespace PerceptionProfiling::Ros {

    builtin_interfaces::msg::Time to_time(Perception::Timestamp stamp);
    Perception::Timestamp from_time(const builtin_interfaces::msg::Time & stamp);

    perception_interfaces::msg::LidarObservation to_message(
            const Perception::LidarObservation & observation);
    perception_interfaces::msg::PoseEstimate to_message(
            const Perception::PoseEstimate & pose);
    perception_interfaces::msg::HealthState make_health_message(
            const ProfileScenario & scenario,
            const builtin_interfaces::msg::Time & publication_stamp);
    Perception::LidarObservation from_message(
            const perception_interfaces::msg::LidarObservation & message);
    ProductionStateProjection to_projection(
            const perception_interfaces::msg::LocalMapState & message,
            std::int64_t receipt_monotonic_ns);
    HealthProjection to_projection(
            const perception_interfaces::msg::HealthState & message,
            std::int64_t receipt_monotonic_ns);

}// namespace PerceptionProfiling::Ros

#endif// PERCEPTION_PROFILING_PROFILE_ROS_CONVERSIONS_HPP
