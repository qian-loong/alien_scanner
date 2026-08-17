#include "perception_map_update/ros/MapUpdateConversions.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace PerceptionMapUpdate::Ros {

    namespace {

        bool to_builtin_time(
                Perception::Timestamp timestamp,
                builtin_interfaces::msg::Time & message)
        {
            if(timestamp.nanoseconds < 0) {
                return false;
            }
            const auto seconds = timestamp.nanoseconds / 1'000'000'000LL;
            if(seconds > std::numeric_limits<std::int32_t>::max()) {
                return false;
            }
            message.sec = static_cast<std::int32_t>(seconds);
            message.nanosec = static_cast<std::uint32_t>(
                    timestamp.nanoseconds % 1'000'000'000LL);
            return true;
        }

        bool from_builtin_time(
                const builtin_interfaces::msg::Time & message,
                Perception::Timestamp & timestamp)
        {
            if(message.sec < 0 || message.nanosec >= 1'000'000'000U) {
                return false;
            }
            const auto seconds = static_cast<std::int64_t>(message.sec);
            timestamp.nanoseconds = seconds * 1'000'000'000LL + message.nanosec;
            return true;
        }

        bool bounded_string(
                const std::string & value,
                std::size_t limit,
                bool allow_empty)
        {
            return static_cast<bool>(CanonicalCodec::validate_string(
                    value, limit, "ROS string", allow_empty));
        }

        bool valid_reason(std::uint8_t value)
        {
            return value >= static_cast<std::uint8_t>(ResyncReason::InitialBaseline)
                   && value <= static_cast<std::uint8_t>(ResyncReason::LocalStateInvalid);
        }

        ContentIdentityDescriptor decode_content_identity(
                const perception_interfaces::msg::ContentIdentityDescriptor & message)
        {
            return {
                    static_cast<ContentIdentityScheme>(message.scheme),
                    message.chunk_edge,
                    message.coordinate_key_version,
                    message.node_encoding_version};
        }

        void encode_content_identity(
                const ContentIdentityDescriptor & descriptor,
                perception_interfaces::msg::ContentIdentityDescriptor & message)
        {
            message.scheme = static_cast<std::uint16_t>(descriptor.scheme);
            message.chunk_edge = descriptor.chunk_edge;
            message.coordinate_key_version = descriptor.coordinate_key_version;
            message.node_encoding_version = descriptor.node_encoding_version;
        }

    }// namespace

    bool encode_map_update(
            const MapUpdate & update,
            perception_interfaces::msg::MapUpdate & message,
            std::string & diagnostic)
    {
        builtin_interfaces::msg::Time observation_stamp;
        if(update.protocol_version != kProtocolVersion
           || update.canonical_encoding_version != kCanonicalEncodingVersion
           || update.hash_algorithm != HashAlgorithm::Sha256
           || !update.content_identity.valid()) {
            diagnostic = "map update protocol or content identity is invalid";
            return false;
        }
        if(!to_builtin_time(update.latest_commit.observation_stamp, observation_stamp)) {
            diagnostic = "latest observation stamp is outside builtin_interfaces/Time range";
            return false;
        }
        message.header.frame_id = update.geometry.frame_id;
        message.protocol_version = update.protocol_version;
        message.canonical_encoding_version = update.canonical_encoding_version;
        message.hash_algorithm = static_cast<std::uint8_t>(update.hash_algorithm);
        encode_content_identity(update.content_identity, message.content_identity);
        message.update_kind = static_cast<std::uint8_t>(update.kind);
        message.vehicle_id = update.source.vehicle_id;
        message.mapper_session_boot_time_ns = update.source.mapper_session.boot_time_ns;
        message.mapper_session_random_suffix = update.source.mapper_session.random_suffix;
        message.map_epoch = update.source.map_epoch;
        message.source_map_frame = update.geometry.frame_id;
        message.resolution_m = update.geometry.resolution_m;
        message.lattice_origin.x = update.geometry.lattice_origin.x;
        message.lattice_origin.y = update.geometry.lattice_origin.y;
        message.lattice_origin.z = update.geometry.lattice_origin.z;
        message.base_revision = update.base_revision;
        message.new_revision = update.new_revision;
        message.revision_span = update.revision_span;
        message.observed_coalesced_receipt_count = update.observed_coalesced_receipt_count;
        std::copy(
                update.geometry_fingerprint.begin(),
                update.geometry_fingerprint.end(),
                message.geometry_fingerprint.begin());
        std::copy(
                update.base_content_hash.begin(),
                update.base_content_hash.end(),
                message.base_content_hash.begin());
        std::copy(
                update.content_hash.begin(), update.content_hash.end(), message.content_hash.begin());
        std::copy(
                update.update_hash.begin(), update.update_hash.end(), message.update_hash.begin());
        message.latest_sensor_id = update.latest_commit.sensor_id;
        message.latest_sensor_session_boot_time_ns =
                update.latest_commit.sensor_session.boot_time_ns;
        message.latest_sensor_session_random_suffix =
                update.latest_commit.sensor_session.random_suffix;
        message.latest_observation_stamp = observation_stamp;
        message.latest_observation_clock_domain = update.latest_commit.clock_domain;
        message.latest_commit_changed_cell_count = update.latest_commit.changed_cell_count;
        message.known_cell_count = update.known_cell_count;
        message.operation_count = update.operation_count;
        message.canonical_payload_bytes = update.canonical_payload_bytes;
        message.correlation_id = update.correlation_id;
        message.payload = update.payload;
        diagnostic.clear();
        return true;
    }

    DecodeMapUpdateResult decode_map_update(
            const perception_interfaces::msg::MapUpdate & message,
            const MapUpdateLimits & limits)
    {
        if(message.protocol_version != kProtocolVersion
           || message.canonical_encoding_version != kCanonicalEncodingVersion
           || message.hash_algorithm
                      != static_cast<std::uint8_t>(HashAlgorithm::Sha256)) {
            return {false, std::nullopt,
                    "map update protocol, canonical encoding, or hash algorithm is invalid"};
        }
        if(message.header.frame_id != message.source_map_frame) {
            return {false, std::nullopt, "header frame and source map frame differ"};
        }
        if(!bounded_string(
                   message.vehicle_id, limits.max_identity_string_bytes, false)
           || !bounded_string(message.source_map_frame, limits.max_frame_id_bytes, false)
           || !bounded_string(message.latest_sensor_id, limits.max_sensor_id_bytes, false)
           || !bounded_string(
                   message.latest_observation_clock_domain,
                   limits.max_clock_domain_bytes,
                   false)
           || !bounded_string(
                   message.correlation_id, limits.max_correlation_id_bytes, true)) {
            return {false, std::nullopt, "map update contains an invalid or oversized string"};
        }
        const bool keyframe = message.update_kind
                              == perception_interfaces::msg::MapUpdate::KIND_KEYFRAME;
        const std::size_t payload_limit = keyframe ? limits.max_keyframe_payload_bytes
                                                   : limits.max_delta_payload_bytes;
        if(message.payload.size() > payload_limit
           || message.canonical_payload_bytes != message.payload.size()) {
            return {false, std::nullopt, "map update payload exceeds limit or length mismatches"};
        }
        Perception::Timestamp observation_stamp {0};
        if(!from_builtin_time(message.latest_observation_stamp, observation_stamp)) {
            return {false, std::nullopt, "latest observation stamp is invalid"};
        }

        MapUpdate update;
        update.protocol_version = message.protocol_version;
        update.canonical_encoding_version = message.canonical_encoding_version;
        update.hash_algorithm = static_cast<HashAlgorithm>(message.hash_algorithm);
        update.content_identity = decode_content_identity(message.content_identity);
        if(!update.content_identity.valid()) {
            return {false, std::nullopt, "map update content identity descriptor is invalid"};
        }
        update.kind = static_cast<UpdateKind>(message.update_kind);
        update.source = {
                message.vehicle_id,
                {message.mapper_session_boot_time_ns, message.mapper_session_random_suffix},
                message.map_epoch};
        update.geometry = {
                message.resolution_m,
                {message.lattice_origin.x,
                 message.lattice_origin.y,
                 message.lattice_origin.z},
                message.source_map_frame};
        update.base_revision = message.base_revision;
        update.new_revision = message.new_revision;
        update.revision_span = message.revision_span;
        update.observed_coalesced_receipt_count =
                message.observed_coalesced_receipt_count;
        std::copy(
                message.geometry_fingerprint.begin(),
                message.geometry_fingerprint.end(),
                update.geometry_fingerprint.begin());
        std::copy(
                message.base_content_hash.begin(),
                message.base_content_hash.end(),
                update.base_content_hash.begin());
        std::copy(
                message.content_hash.begin(),
                message.content_hash.end(),
                update.content_hash.begin());
        std::copy(
                message.update_hash.begin(), message.update_hash.end(), update.update_hash.begin());
        update.latest_commit = {
                message.latest_sensor_id,
                {message.latest_sensor_session_boot_time_ns,
                 message.latest_sensor_session_random_suffix},
                observation_stamp,
                message.latest_observation_clock_domain,
                message.latest_commit_changed_cell_count};
        update.known_cell_count = message.known_cell_count;
        update.operation_count = message.operation_count;
        update.canonical_payload_bytes = message.canonical_payload_bytes;
        update.correlation_id = message.correlation_id;
        update.payload = message.payload;
        return {true, std::move(update), {}};
    }

    bool decode_resync_request(
            const perception_interfaces::srv::RequestMapResync::Request & message,
            ResyncRequest & request,
            const MapUpdateLimits & limits,
            std::string & diagnostic)
    {
        request = {};
        if(!bounded_string(
                   message.requester_id, limits.max_identity_string_bytes, false)
           || !bounded_string(
                   message.client_request_id, limits.max_correlation_id_bytes, false)
           || message.requester_session_boot_time_ns == 0U || !valid_reason(message.reason)) {
            diagnostic = "resync requester, client request id, session, or reason is invalid";
            return false;
        }
        const bool empty_expected = message.expected_vehicle_id.empty()
                                    && message.expected_mapper_session_boot_time_ns == 0U
                                    && message.expected_mapper_session_random_suffix == 0U
                                    && message.expected_map_epoch == 0U;
        if(message.bootstrap_latest) {
            if(!empty_expected
               || message.reason
                          != perception_interfaces::srv::RequestMapResync::Request::
                                  REASON_INITIAL_BASELINE) {
                diagnostic = "bootstrap resync must omit expected source and use initial reason";
                return false;
            }
        } else {
            SourceIdentity expected {
                    message.expected_vehicle_id,
                    {message.expected_mapper_session_boot_time_ns,
                     message.expected_mapper_session_random_suffix},
                    message.expected_map_epoch};
            const auto validation = CanonicalCodec::validate_identity(expected, limits);
            if(!validation) {
                diagnostic = validation.diagnostic;
                return false;
            }
            request.expected_source = std::move(expected);
        }
        request.requester = {
                message.requester_id,
                {message.requester_session_boot_time_ns,
                 message.requester_session_random_suffix}};
        request.client_request_id = message.client_request_id;
        request.receiver_revision = message.receiver_revision;
        request.receiver_content_identity.descriptor =
                decode_content_identity(message.receiver_content_identity);
        if(!request.receiver_content_identity.descriptor.valid()) {
            diagnostic = "receiver content identity descriptor is invalid";
            return false;
        }
        std::copy(
                message.receiver_content_hash.begin(),
                message.receiver_content_hash.end(),
                request.receiver_content_identity.digest.begin());
        if(message.bootstrap_latest
           && std::any_of(
                   request.receiver_content_identity.digest.begin(),
                   request.receiver_content_identity.digest.end(),
                   [](std::uint8_t value) { return value != 0U; })) {
            diagnostic = "bootstrap resync receiver digest must be zero";
            return false;
        }
        request.reason = static_cast<ResyncReason>(message.reason);
        diagnostic.clear();
        return true;
    }

    void encode_resync_response(
            const ResyncResponse & response,
            perception_interfaces::srv::RequestMapResync::Response & message)
    {
        message.accepted = response.accepted;
        message.correlation_id = response.correlation_id;
        message.current_vehicle_id = response.current_source.vehicle_id;
        message.current_mapper_session_boot_time_ns =
                response.current_source.mapper_session.boot_time_ns;
        message.current_mapper_session_random_suffix =
                response.current_source.mapper_session.random_suffix;
        message.current_map_epoch = response.current_source.map_epoch;
        message.current_revision = response.current_revision;
        encode_content_identity(
                response.current_content_identity.descriptor,
                message.current_content_identity);
        std::copy(
                response.current_content_identity.digest.begin(),
                response.current_content_identity.digest.end(),
                message.current_content_hash.begin());
        message.diagnostic = response.diagnostic;
    }

}// namespace PerceptionMapUpdate::Ros
