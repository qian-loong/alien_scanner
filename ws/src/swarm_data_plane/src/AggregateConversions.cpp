#include "swarm_data_plane/ros/AggregateConversions.hpp"

#include "swarm_data_plane/ros/RoutedMapConversions.hpp"

#include <algorithm>
#include <utility>

namespace SwarmDataPlane::Ros {

    bool encode_aggregate_map_update(
            const AggregateMapUpdate & update,
            swarm_data_interfaces::msg::AggregateMapUpdate & message,
            std::string & diagnostic,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits &)
    {
        const auto validation = validate_aggregate_map_update(update, data_plane_limits);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }
        if(!encode_routed_map_update(
                   update.aggregate_update,
                   message.aggregate_update,
                   diagnostic,
                   data_plane_limits)) {
            return false;
        }

        message.manifest.protocol_version = update.manifest.protocol_version;
        message.manifest.aggregate_id = update.manifest.aggregate.producer_id;
        message.manifest.aggregate_session_boot_time_ns =
                update.manifest.aggregate.session.boot_time_ns;
        message.manifest.aggregate_session_random_suffix =
                update.manifest.aggregate.session.random_suffix;
        message.manifest.aggregate_revision = update.manifest.aggregate_revision;
        message.manifest.contributors.clear();
        message.manifest.contributors.reserve(update.manifest.contributors.size());
        for(const auto & contributor : update.manifest.contributors) {
            swarm_data_interfaces::msg::ContributorRevision converted;
            converted.vehicle_id = contributor.source.vehicle_id;
            converted.mapper_session_boot_time_ns =
                    contributor.source.mapper_session.boot_time_ns;
            converted.mapper_session_random_suffix =
                    contributor.source.mapper_session.random_suffix;
            converted.map_epoch = contributor.source.map_epoch;
            converted.revision = contributor.revision;
            std::copy(
                    contributor.content_hash.begin(),
                    contributor.content_hash.end(),
                    converted.content_hash.begin());
            converted.active = contributor.active;
            message.manifest.contributors.push_back(std::move(converted));
        }
        std::copy(
                update.manifest.manifest_hash.begin(),
                update.manifest.manifest_hash.end(),
                message.manifest.manifest_hash.begin());
        diagnostic.clear();
        return true;
    }

    DecodeAggregateResult decode_aggregate_map_update(
            const swarm_data_interfaces::msg::AggregateMapUpdate & message,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits)
    {
        auto routed = decode_routed_map_update(
                message.aggregate_update, data_plane_limits, map_update_limits);
        if(!routed.success || !routed.message.has_value()) {
            return {false, std::nullopt, routed.diagnostic};
        }

        AggregateMapUpdate decoded;
        decoded.aggregate_update = std::move(*routed.message);
        decoded.manifest.protocol_version = message.manifest.protocol_version;
        decoded.manifest.aggregate = {
                message.manifest.aggregate_id,
                {message.manifest.aggregate_session_boot_time_ns,
                 message.manifest.aggregate_session_random_suffix}};
        decoded.manifest.aggregate_revision = message.manifest.aggregate_revision;
        decoded.manifest.contributors.reserve(message.manifest.contributors.size());
        for(const auto & contributor : message.manifest.contributors) {
            ContributorRevision converted;
            converted.source = {
                    contributor.vehicle_id,
                    {contributor.mapper_session_boot_time_ns,
                     contributor.mapper_session_random_suffix},
                    contributor.map_epoch};
            converted.revision = contributor.revision;
            std::copy(
                    contributor.content_hash.begin(),
                    contributor.content_hash.end(),
                    converted.content_hash.begin());
            converted.active = contributor.active;
            decoded.manifest.contributors.push_back(std::move(converted));
        }
        std::copy(
                message.manifest.manifest_hash.begin(),
                message.manifest.manifest_hash.end(),
                decoded.manifest.manifest_hash.begin());

        const auto validation = validate_aggregate_map_update(decoded, data_plane_limits);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

}// namespace SwarmDataPlane::Ros
