#include "perception_map_update/ros/MapUpdateParameters.hpp"

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace PerceptionMapUpdate::Ros {

    namespace {

        std::string parameter_name(const std::string & prefix, const char * leaf)
        {
            return prefix.empty() ? std::string(leaf) : prefix + '.' + leaf;
        }

        std::size_t positive_size_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::size_t default_value)
        {
            if(default_value > static_cast<std::size_t>(
                                       std::numeric_limits<std::int64_t>::max())) {
                throw std::invalid_argument(name + " default exceeds ROS integer range");
            }
            const auto value = node.declare_parameter<std::int64_t>(
                    name, static_cast<std::int64_t>(default_value));
            if(value <= 0) {
                throw std::invalid_argument(name + " must be positive");
            }
            return static_cast<std::size_t>(value);
        }

        std::uint64_t nonnegative_u64_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::uint64_t default_value)
        {
            if(default_value > static_cast<std::uint64_t>(
                                       std::numeric_limits<std::int64_t>::max())) {
                throw std::invalid_argument(name + " default exceeds ROS integer range");
            }
            const auto value = node.declare_parameter<std::int64_t>(
                    name, static_cast<std::int64_t>(default_value));
            if(value < 0) {
                throw std::invalid_argument(name + " must be non-negative");
            }
            return static_cast<std::uint64_t>(value);
        }

    }// namespace

    MapUpdateLimits declare_map_update_limits(
            rclcpp::Node & node,
            const std::string & prefix)
    {
        MapUpdateLimits limits;
        limits.max_identity_string_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_identity_string_bytes"),
                limits.max_identity_string_bytes);
        limits.max_frame_id_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_frame_id_bytes"),
                limits.max_frame_id_bytes);
        limits.max_sensor_id_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_sensor_id_bytes"),
                limits.max_sensor_id_bytes);
        limits.max_clock_domain_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_clock_domain_bytes"),
                limits.max_clock_domain_bytes);
        limits.max_correlation_id_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_correlation_id_bytes"),
                limits.max_correlation_id_bytes);
        limits.max_known_cells = positive_size_parameter(
                node, parameter_name(prefix, "max_known_cells"), limits.max_known_cells);
        limits.max_delta_operations = positive_size_parameter(
                node,
                parameter_name(prefix, "max_delta_operations"),
                limits.max_delta_operations);
        limits.max_keyframe_payload_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_keyframe_payload_bytes"),
                limits.max_keyframe_payload_bytes);
        limits.max_delta_payload_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_delta_payload_bytes"),
                limits.max_delta_payload_bytes);
        limits.max_receiver_cells = positive_size_parameter(
                node,
                parameter_name(prefix, "max_receiver_cells"),
                limits.max_receiver_cells);
        limits.max_live_chunks = positive_size_parameter(
                node,
                parameter_name(prefix, "max_live_chunks"),
                limits.max_live_chunks);
        limits.max_merkle_nodes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_merkle_nodes"),
                limits.max_merkle_nodes);
        limits.max_merkle_owned_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_merkle_owned_bytes"),
                limits.max_merkle_owned_bytes);
        limits.max_candidate_merkle_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_candidate_merkle_bytes"),
                limits.max_candidate_merkle_bytes);
        limits.max_peak_apply_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_peak_apply_bytes"),
                limits.max_peak_apply_bytes);
        limits.max_retained_snapshot_bytes = positive_size_parameter(
                node,
                parameter_name(prefix, "max_retained_snapshot_bytes"),
                limits.max_retained_snapshot_bytes);
        limits.max_recent_resync_requests = positive_size_parameter(
                node,
                parameter_name(prefix, "max_recent_resync_requests"),
                limits.max_recent_resync_requests);
        limits.max_delta_chain_length = nonnegative_u64_parameter(
                node,
                parameter_name(prefix, "max_delta_chain_length"),
                limits.max_delta_chain_length);
        limits.max_revision_span = nonnegative_u64_parameter(
                node,
                parameter_name(prefix, "max_revision_span"),
                limits.max_revision_span);
        if(limits.max_revision_span == 0U) {
            throw std::invalid_argument(
                    parameter_name(prefix, "max_revision_span") + " must be positive");
        }
        limits.periodic_keyframe_revision_interval = nonnegative_u64_parameter(
                node,
                parameter_name(prefix, "periodic_keyframe_revision_interval"),
                limits.periodic_keyframe_revision_interval);
        limits.delta_enabled = node.declare_parameter<bool>(
                parameter_name(prefix, "delta_enabled"), limits.delta_enabled);
        return limits;
    }

}// namespace PerceptionMapUpdate::Ros
