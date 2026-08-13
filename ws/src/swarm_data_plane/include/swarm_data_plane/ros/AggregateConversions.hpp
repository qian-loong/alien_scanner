#ifndef SWARM_DATA_PLANE_ROS_AGGREGATE_CONVERSIONS_HPP
#define SWARM_DATA_PLANE_ROS_AGGREGATE_CONVERSIONS_HPP

#include "swarm_data_plane/AggregateContract.hpp"
#include "swarm_data_interfaces/msg/aggregate_map_update.hpp"

#include <optional>
#include <string>

namespace SwarmDataPlane::Ros {

    struct DecodeAggregateResult {
        bool success = false;
        std::optional<AggregateMapUpdate> update;
        std::string diagnostic;
    };

    bool encode_aggregate_map_update(
            const AggregateMapUpdate & update,
            swarm_data_interfaces::msg::AggregateMapUpdate & message,
            std::string & diagnostic,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});
    DecodeAggregateResult decode_aggregate_map_update(
            const swarm_data_interfaces::msg::AggregateMapUpdate & message,
            const DataPlaneLimits & data_plane_limits = {},
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits = {});

}// namespace SwarmDataPlane::Ros

#endif// SWARM_DATA_PLANE_ROS_AGGREGATE_CONVERSIONS_HPP
