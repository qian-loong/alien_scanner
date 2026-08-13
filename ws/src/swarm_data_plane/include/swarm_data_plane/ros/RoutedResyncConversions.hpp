#ifndef SWARM_DATA_PLANE_ROS_ROUTED_RESYNC_CONVERSIONS_HPP
#define SWARM_DATA_PLANE_ROS_ROUTED_RESYNC_CONVERSIONS_HPP

#include "swarm_data_plane/RoutedResync.hpp"
#include "swarm_data_interfaces/msg/resync_ack.hpp"
#include "swarm_data_interfaces/msg/resync_intent.hpp"

#include <optional>
#include <string>

namespace SwarmDataPlane::Ros {

    struct DecodeResyncIntentResult {
        bool success = false;
        std::optional<RoutedResyncIntent> intent;
        std::string diagnostic;
    };

    struct DecodeResyncAckResult {
        bool success = false;
        std::optional<RoutedResyncAck> ack;
        std::string diagnostic;
    };

    bool encode_resync_intent(
            const RoutedResyncIntent & intent,
            swarm_data_interfaces::msg::ResyncIntent & message,
            std::string & diagnostic,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});
    DecodeResyncIntentResult decode_resync_intent(
            const swarm_data_interfaces::msg::ResyncIntent & message,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});

    bool encode_resync_ack(
            const RoutedResyncAck & ack,
            swarm_data_interfaces::msg::ResyncAck & message,
            std::string & diagnostic,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});
    DecodeResyncAckResult decode_resync_ack(
            const swarm_data_interfaces::msg::ResyncAck & message,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});

}// namespace SwarmDataPlane::Ros

#endif// SWARM_DATA_PLANE_ROS_ROUTED_RESYNC_CONVERSIONS_HPP
