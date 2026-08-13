#include "swarm_data_plane/ros/RoutedMapConversions.hpp"

#include "perception_map_update/ros/MapUpdateConversions.hpp"
#include "swarm_data_plane/RoutedMapValidator.hpp"

#include <algorithm>
#include <memory>
#include <utility>

namespace SwarmDataPlane::Ros {

    bool encode_routed_map_update(
            const RoutedMapUpdate & message,
            swarm_data_interfaces::msg::RoutedMapUpdate & output,
            std::string & diagnostic,
            const DataPlaneLimits & limits)
    {
        const auto validation = validate_routed_map_update(message, limits);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }
        if(!PerceptionMapUpdate::Ros::encode_map_update(
                   *message.update, output.map_update, diagnostic)) {
            return false;
        }

        output.protocol_version = message.protocol_version;
        output.message_id = message.message_id;
        output.producer_id = message.producer.producer_id;
        output.producer_session_boot_time_ns = message.producer.session.boot_time_ns;
        output.producer_session_random_suffix = message.producer.session.random_suffix;
        output.sequence = message.sequence;
        output.correlation_id = message.correlation_id;
        output.priority = static_cast<std::uint8_t>(message.priority);
        output.origin_clock_domain = message.origin.domain;
        output.origin_clock_session_boot_time_ns = message.origin.session.boot_time_ns;
        output.origin_clock_session_random_suffix = message.origin.session.random_suffix;
        output.origin_time_ns = message.origin.time_ns;
        output.validity_budget_ns = message.validity_budget_ns;
        output.accumulated_forwarding_ns = message.accumulated_forwarding_ns;
        output.route_epoch = message.route.route_epoch;
        output.hop_count = message.route.hop_count;
        output.ttl_hops = message.route.ttl_hops;
        output.payload_bytes = message.payload_bytes;
        std::copy(
                message.payload_hash.begin(),
                message.payload_hash.end(),
                output.payload_hash.begin());
        diagnostic.clear();
        return true;
    }

    DecodeRoutedMapResult decode_routed_map_update(
            const swarm_data_interfaces::msg::RoutedMapUpdate & message,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits)
    {
        auto decoded_map = PerceptionMapUpdate::Ros::decode_map_update(
                message.map_update, map_update_limits);
        if(!decoded_map.success || !decoded_map.update.has_value()) {
            return {false, std::nullopt, decoded_map.diagnostic};
        }

        RoutedMapUpdate decoded;
        decoded.protocol_version = message.protocol_version;
        decoded.message_id = message.message_id;
        decoded.producer = {
                message.producer_id,
                {message.producer_session_boot_time_ns,
                 message.producer_session_random_suffix}};
        decoded.sequence = message.sequence;
        decoded.correlation_id = message.correlation_id;
        decoded.priority = static_cast<LogicalPriority>(message.priority);
        decoded.origin = {
                message.origin_clock_domain,
                {message.origin_clock_session_boot_time_ns,
                 message.origin_clock_session_random_suffix},
                message.origin_time_ns};
        decoded.validity_budget_ns = message.validity_budget_ns;
        decoded.accumulated_forwarding_ns = message.accumulated_forwarding_ns;
        decoded.route = {message.route_epoch, message.hop_count, message.ttl_hops};
        decoded.payload_bytes = message.payload_bytes;
        std::copy(
                message.payload_hash.begin(),
                message.payload_hash.end(),
                decoded.payload_hash.begin());
        decoded.update = std::make_shared<const PerceptionMapUpdate::MapUpdate>(
                std::move(*decoded_map.update));

        const auto validation = validate_routed_map_update(decoded, data_plane_limits);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

}// namespace SwarmDataPlane::Ros
