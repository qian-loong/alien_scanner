#include "swarm_data_plane/ros/RoutedResyncConversions.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ros/MapUpdateConversions.hpp"

#include <algorithm>
#include <utility>

namespace SwarmDataPlane::Ros {

    namespace {

        bool valid_producer(
                const ProducerIdentity & producer,
                const DataPlaneLimits & limits)
        {
            return producer.session.boot_time_ns != 0U
                   && static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                           producer.producer_id,
                           limits.max_producer_id_bytes,
                           "target_producer_id",
                           false));
        }

        bool valid_ack(
                const RoutedResyncAck & ack,
                const DataPlaneLimits & data_plane_limits,
                const PerceptionMapUpdate::MapUpdateLimits & map_update_limits,
                std::string & diagnostic)
        {
            if(ack.protocol_version != kProtocolVersion
               || !valid_producer(ack.target_producer, data_plane_limits)) {
                diagnostic = "resync ack protocol or target producer is invalid";
                return false;
            }
            const auto source = PerceptionMapUpdate::CanonicalCodec::validate_identity(
                    ack.current_source, map_update_limits);
            const auto correlation = PerceptionMapUpdate::CanonicalCodec::validate_string(
                    ack.correlation_id,
                    data_plane_limits.max_correlation_id_bytes,
                    "correlation_id",
                    !ack.accepted);
            const auto detail = PerceptionMapUpdate::CanonicalCodec::validate_string(
                    ack.diagnostic, 256U, "diagnostic", true);
            if(!source || !correlation || !detail) {
                diagnostic = "resync ack source, correlation, or diagnostic is invalid";
                return false;
            }
            diagnostic.clear();
            return true;
        }

    }// namespace

    bool encode_resync_intent(
            const RoutedResyncIntent & intent,
            swarm_data_interfaces::msg::ResyncIntent & message,
            std::string & diagnostic,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits)
    {
        if(intent.protocol_version != kProtocolVersion || intent.route_epoch == 0U
           || !valid_producer(intent.target_producer, data_plane_limits)) {
            diagnostic = "resync intent protocol, target producer, or route epoch is invalid";
            return false;
        }

        perception_interfaces::srv::RequestMapResync::Request c3_message;
        c3_message.requester_id = intent.request.requester.requester_id;
        c3_message.requester_session_boot_time_ns =
                intent.request.requester.requester_session.boot_time_ns;
        c3_message.requester_session_random_suffix =
                intent.request.requester.requester_session.random_suffix;
        c3_message.client_request_id = intent.request.client_request_id;
        c3_message.bootstrap_latest = !intent.request.expected_source.has_value();
        if(intent.request.expected_source.has_value()) {
            c3_message.expected_vehicle_id = intent.request.expected_source->vehicle_id;
            c3_message.expected_mapper_session_boot_time_ns =
                    intent.request.expected_source->mapper_session.boot_time_ns;
            c3_message.expected_mapper_session_random_suffix =
                    intent.request.expected_source->mapper_session.random_suffix;
            c3_message.expected_map_epoch = intent.request.expected_source->map_epoch;
        }
        c3_message.receiver_revision = intent.request.receiver_revision;
        std::copy(
                intent.request.receiver_content_hash.begin(),
                intent.request.receiver_content_hash.end(),
                c3_message.receiver_content_hash.begin());
        c3_message.reason = static_cast<std::uint8_t>(intent.request.reason);

        PerceptionMapUpdate::ResyncRequest validated;
        if(!PerceptionMapUpdate::Ros::decode_resync_request(
                   c3_message, validated, map_update_limits, diagnostic)) {
            return false;
        }

        message.protocol_version = intent.protocol_version;
        message.requester_id = c3_message.requester_id;
        message.requester_session_boot_time_ns =
                c3_message.requester_session_boot_time_ns;
        message.requester_session_random_suffix = c3_message.requester_session_random_suffix;
        message.client_request_id = c3_message.client_request_id;
        message.target_producer_id = intent.target_producer.producer_id;
        message.target_producer_session_boot_time_ns =
                intent.target_producer.session.boot_time_ns;
        message.target_producer_session_random_suffix =
                intent.target_producer.session.random_suffix;
        message.route_epoch = intent.route_epoch;
        message.bootstrap_latest = c3_message.bootstrap_latest;
        message.expected_vehicle_id = c3_message.expected_vehicle_id;
        message.expected_mapper_session_boot_time_ns =
                c3_message.expected_mapper_session_boot_time_ns;
        message.expected_mapper_session_random_suffix =
                c3_message.expected_mapper_session_random_suffix;
        message.expected_map_epoch = c3_message.expected_map_epoch;
        message.receiver_revision = c3_message.receiver_revision;
        std::copy(
                c3_message.receiver_content_hash.begin(),
                c3_message.receiver_content_hash.end(),
                message.receiver_content_hash.begin());
        message.reason = c3_message.reason;
        diagnostic.clear();
        return true;
    }

    DecodeResyncIntentResult decode_resync_intent(
            const swarm_data_interfaces::msg::ResyncIntent & message,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits)
    {
        RoutedResyncIntent intent;
        intent.protocol_version = message.protocol_version;
        intent.target_producer = {
                message.target_producer_id,
                {message.target_producer_session_boot_time_ns,
                 message.target_producer_session_random_suffix}};
        intent.route_epoch = message.route_epoch;
        if(intent.protocol_version != kProtocolVersion || intent.route_epoch == 0U
           || !valid_producer(intent.target_producer, data_plane_limits)) {
            return {false, std::nullopt,
                    "resync intent protocol, target producer, or route epoch is invalid"};
        }

        perception_interfaces::srv::RequestMapResync::Request c3_message;
        c3_message.requester_id = message.requester_id;
        c3_message.requester_session_boot_time_ns = message.requester_session_boot_time_ns;
        c3_message.requester_session_random_suffix = message.requester_session_random_suffix;
        c3_message.client_request_id = message.client_request_id;
        c3_message.bootstrap_latest = message.bootstrap_latest;
        c3_message.expected_vehicle_id = message.expected_vehicle_id;
        c3_message.expected_mapper_session_boot_time_ns =
                message.expected_mapper_session_boot_time_ns;
        c3_message.expected_mapper_session_random_suffix =
                message.expected_mapper_session_random_suffix;
        c3_message.expected_map_epoch = message.expected_map_epoch;
        c3_message.receiver_revision = message.receiver_revision;
        std::copy(
                message.receiver_content_hash.begin(),
                message.receiver_content_hash.end(),
                c3_message.receiver_content_hash.begin());
        c3_message.reason = message.reason;

        std::string diagnostic;
        if(!PerceptionMapUpdate::Ros::decode_resync_request(
                   c3_message, intent.request, map_update_limits, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }
        return {true, std::move(intent), {}};
    }

    bool encode_resync_ack(
            const RoutedResyncAck & ack,
            swarm_data_interfaces::msg::ResyncAck & message,
            std::string & diagnostic,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits)
    {
        if(!valid_ack(ack, data_plane_limits, map_update_limits, diagnostic)) {
            return false;
        }
        message.protocol_version = ack.protocol_version;
        message.accepted = ack.accepted;
        message.correlation_id = ack.correlation_id;
        message.target_producer_id = ack.target_producer.producer_id;
        message.target_producer_session_boot_time_ns =
                ack.target_producer.session.boot_time_ns;
        message.target_producer_session_random_suffix =
                ack.target_producer.session.random_suffix;
        message.current_vehicle_id = ack.current_source.vehicle_id;
        message.current_mapper_session_boot_time_ns =
                ack.current_source.mapper_session.boot_time_ns;
        message.current_mapper_session_random_suffix =
                ack.current_source.mapper_session.random_suffix;
        message.current_map_epoch = ack.current_source.map_epoch;
        message.current_revision = ack.current_revision;
        message.diagnostic = ack.diagnostic;
        diagnostic.clear();
        return true;
    }

    DecodeResyncAckResult decode_resync_ack(
            const swarm_data_interfaces::msg::ResyncAck & message,
            const DataPlaneLimits & data_plane_limits,
            const PerceptionMapUpdate::MapUpdateLimits & map_update_limits)
    {
        RoutedResyncAck ack;
        ack.protocol_version = message.protocol_version;
        ack.accepted = message.accepted;
        ack.correlation_id = message.correlation_id;
        ack.target_producer = {
                message.target_producer_id,
                {message.target_producer_session_boot_time_ns,
                 message.target_producer_session_random_suffix}};
        ack.current_source = {
                message.current_vehicle_id,
                {message.current_mapper_session_boot_time_ns,
                 message.current_mapper_session_random_suffix},
                message.current_map_epoch};
        ack.current_revision = message.current_revision;
        ack.diagnostic = message.diagnostic;
        std::string diagnostic;
        if(!valid_ack(ack, data_plane_limits, map_update_limits, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }
        return {true, std::move(ack), {}};
    }

}// namespace SwarmDataPlane::Ros
