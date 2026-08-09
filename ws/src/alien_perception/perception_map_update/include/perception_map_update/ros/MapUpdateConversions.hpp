#ifndef PERCEPTION_MAP_UPDATE_ROS_MAP_UPDATE_CONVERSIONS_HPP
#define PERCEPTION_MAP_UPDATE_ROS_MAP_UPDATE_CONVERSIONS_HPP

#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"
#include "perception_map_update/ResyncStateMachine.hpp"
#include "perception_interfaces/msg/map_update.hpp"
#include "perception_interfaces/srv/request_map_resync.hpp"

#include <optional>
#include <string>

namespace PerceptionMapUpdate::Ros {

    struct DecodeMapUpdateResult {
        bool                     success = false;
        std::optional<MapUpdate> update;
        std::string              diagnostic;
    };

    bool encode_map_update(
            const MapUpdate & update,
            perception_interfaces::msg::MapUpdate & message,
            std::string & diagnostic);
    DecodeMapUpdateResult decode_map_update(
            const perception_interfaces::msg::MapUpdate & message,
            const MapUpdateLimits & limits = {});

    bool decode_resync_request(
            const perception_interfaces::srv::RequestMapResync::Request & message,
            ResyncRequest & request,
            const MapUpdateLimits & limits,
            std::string & diagnostic);
    void encode_resync_response(
            const ResyncResponse & response,
            perception_interfaces::srv::RequestMapResync::Response & message);

}// namespace PerceptionMapUpdate::Ros

#endif// PERCEPTION_MAP_UPDATE_ROS_MAP_UPDATE_CONVERSIONS_HPP
