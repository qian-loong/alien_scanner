#include "ProfileRosConversions.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace PerceptionProfiling::Ros {

    builtin_interfaces::msg::Time to_time(Perception::Timestamp stamp)
    {
        if(stamp.nanoseconds < 0) {
            throw std::invalid_argument("ROS timestamp must be non-negative");
        }
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<std::int32_t>(stamp.nanoseconds / 1'000'000'000LL);
        result.nanosec = static_cast<std::uint32_t>(stamp.nanoseconds % 1'000'000'000LL);
        return result;
    }

    Perception::Timestamp from_time(const builtin_interfaces::msg::Time & stamp)
    {
        return Perception::Timestamp::from_nanoseconds(
                static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL
                + static_cast<std::int64_t>(stamp.nanosec));
    }

    perception_interfaces::msg::LidarObservation to_message(
            const Perception::LidarObservation & observation)
    {
        perception_interfaces::msg::LidarObservation message;
        message.header.stamp = to_time(observation.origin_stamp);
        message.header.frame_id = observation.frame_id;
        message.sensor_id = observation.sensor_id.value;
        message.session_boot_time_ns = observation.session_id.boot_time_ns;
        message.session_random_suffix = observation.session_id.random_suffix;
        message.clock_domain = observation.clock_domain;
        message.ray_evidence = static_cast<std::uint8_t>(observation.ray_evidence);
        if(observation.is_2d()) {
            const auto & scan = observation.as_scan_2d();
            message.data_type = perception_interfaces::msg::LidarObservation::DATA_TYPE_SCAN_2D;
            message.angle_min = scan.angle_min_rad;
            message.angle_max = scan.angle_max_rad;
            message.angle_increment = scan.angle_increment_rad;
            message.range_min = scan.range_min_m;
            message.range_max = scan.range_max_m;
            message.ranges = scan.ranges;
            message.intensities = scan.intensities;
        }
        else {
            const auto & cloud = observation.as_cloud_3d();
            message.data_type = perception_interfaces::msg::LidarObservation::DATA_TYPE_CLOUD_3D;
            message.points.reserve(cloud.points.size());
            message.point_intensities.reserve(cloud.points.size());
            for(const auto & point : cloud.points) {
                geometry_msgs::msg::Point ros_point;
                ros_point.x = point.x;
                ros_point.y = point.y;
                ros_point.z = point.z;
                message.points.push_back(ros_point);
                message.point_intensities.push_back(point.intensity);
            }
        }
        return message;
    }

    perception_interfaces::msg::PoseEstimate to_message(
            const Perception::PoseEstimate & pose)
    {
        perception_interfaces::msg::PoseEstimate message;
        message.header.stamp = to_time(pose.stamp);
        message.header.frame_id = pose.frame_id;
        message.source_id = pose.source_id.value;
        message.session_boot_time_ns = pose.session_id.boot_time_ns;
        message.session_random_suffix = pose.session_id.random_suffix;
        message.clock_domain = pose.clock_domain;
        message.pose.position.x = pose.position.x();
        message.pose.position.y = pose.position.y();
        message.pose.position.z = pose.position.z();
        message.pose.orientation.w = pose.orientation.w();
        message.pose.orientation.x = pose.orientation.x();
        message.pose.orientation.y = pose.orientation.y();
        message.pose.orientation.z = pose.orientation.z();
        std::fill(message.covariance.begin(), message.covariance.end(), 0.0);
        if(pose.covariance.has_value()) {
            for(int row = 0; row < 6; ++row) {
                for(int column = 0; column < 6; ++column) {
                    message.covariance[static_cast<std::size_t>(row * 6 + column)] =
                            (*pose.covariance)(row, column);
                }
            }
        }
        message.quality = pose.quality;
        message.freshness_ns = pose.freshness.nanoseconds;
        message.reset_epoch = pose.reset_epoch;
        return message;
    }

    perception_interfaces::msg::HealthState make_health_message(
            const ProfileScenario & scenario,
            const builtin_interfaces::msg::Time & publication_stamp)
    {
        perception_interfaces::msg::HealthState message;
        message.header.stamp = publication_stamp;
        message.header.frame_id = scenario.config().map_frame;
        message.producer_source_id = scenario.config().producer_source_id;
        message.producer_session_boot_time_ns =
                scenario.config().producer_session.boot_time_ns;
        message.producer_session_random_suffix =
                scenario.config().producer_session.random_suffix;
        const auto fingerprint = scenario.contract_fingerprint();
        message.mapper_contract_schema_version = fingerprint.schema_version;
        message.mapper_contract_fingerprint = fingerprint.hex_digest;
        message.state = perception_interfaces::msg::HealthState::STATE_HEALTHY;
        message.has_2d_lidar = true;
        message.has_3d_lidar = false;
        message.has_fresh_pose = true;
        message.has_free_space_hit_rays = true;
        message.has_full_no_return_rays = true;
        message.active_sensor_count = 1U;
        return message;
    }

    Perception::LidarObservation from_message(
            const perception_interfaces::msg::LidarObservation & message)
    {
        if(message.sensor_id.empty() || message.header.frame_id.empty()
           || message.clock_domain.empty() || message.session_boot_time_ns == 0U) {
            throw std::invalid_argument("observation provenance is incomplete");
        }
        Perception::LidarObservation result;
        result.sensor_id = Perception::SensorID {message.sensor_id};
        result.session_id = {message.session_boot_time_ns, message.session_random_suffix};
        result.frame_id = message.header.frame_id;
        result.clock_domain = message.clock_domain;
        result.origin_stamp = from_time(message.header.stamp);
        switch(message.ray_evidence) {
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_HIT_ONLY:
                result.ray_evidence = Perception::RayEvidenceCapability::HitOnly;
                break;
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_HIT_RAY:
                result.ray_evidence = Perception::RayEvidenceCapability::HitRay;
                break;
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_FULL_RAY:
                result.ray_evidence = Perception::RayEvidenceCapability::FullRay;
                break;
            default:
                throw std::invalid_argument("observation has unknown ray evidence");
        }
        if(message.data_type == perception_interfaces::msg::LidarObservation::DATA_TYPE_SCAN_2D) {
            if(!message.points.empty() || !message.point_intensities.empty()) {
                throw std::invalid_argument("Scan2D contains Cloud3D residue");
            }
            result.data = Perception::Scan2D {
                    message.angle_min,
                    message.angle_max,
                    message.angle_increment,
                    message.range_min,
                    message.range_max,
                    message.ranges,
                    message.intensities};
        }
        else if(message.data_type
                == perception_interfaces::msg::LidarObservation::DATA_TYPE_CLOUD_3D) {
            throw std::invalid_argument("profile sink accepts only Scan2D observations");
        }
        else {
            throw std::invalid_argument("observation has unknown data type");
        }
        return result;
    }

    ProductionStateProjection to_projection(
            const perception_interfaces::msg::LocalMapState & message,
            std::int64_t receipt_monotonic_ns)
    {
        ProductionStateProjection result;
        result.receipt_monotonic_ns = receipt_monotonic_ns;
        result.state_sequence = message.state_sequence;
        result.map_epoch = message.map_epoch;
        result.revision = message.revision;
        result.observation_stamp_ns = from_time(message.last_observation_stamp).nanoseconds;
        result.contract_fingerprint = message.mapper_contract_fingerprint;
        result.changed_cell_count = message.last_commit_changed_cell_count;
        if(message.has_known_bounds) {
            result.bounds = PerceptionLocalMap::AxisAlignedBounds {
                    {message.known_bounds_min.x,
                     message.known_bounds_min.y,
                     message.known_bounds_min.z},
                    {message.known_bounds_max.x,
                     message.known_bounds_max.y,
                     message.known_bounds_max.z}};
        }
        return result;
    }

    HealthProjection to_projection(
            const perception_interfaces::msg::HealthState & message,
            std::int64_t receipt_monotonic_ns)
    {
        return {
                receipt_monotonic_ns,
                from_time(message.header.stamp).nanoseconds,
                message.state,
                message.producer_source_id,
                {message.producer_session_boot_time_ns,
                 message.producer_session_random_suffix},
                message.mapper_contract_fingerprint,
                message.has_full_no_return_rays,
                message.active_sensor_count};
    }

    MapUpdateProjection to_projection(
            const perception_interfaces::msg::MapUpdate & message,
            std::int64_t receipt_monotonic_ns)
    {
        MapUpdateProjection result;
        result.receipt_monotonic_ns = receipt_monotonic_ns;
        result.stamp_ns = from_time(message.header.stamp).nanoseconds;
        result.protocol_version = message.protocol_version;
        result.canonical_encoding_version = message.canonical_encoding_version;
        result.hash_algorithm = message.hash_algorithm;
        result.content_identity_scheme = message.content_identity.scheme;
        result.content_identity_chunk_edge = message.content_identity.chunk_edge;
        result.content_identity_coordinate_key_version =
                message.content_identity.coordinate_key_version;
        result.content_identity_node_encoding_version =
                message.content_identity.node_encoding_version;
        result.update_kind = message.update_kind;
        result.vehicle_id = message.vehicle_id;
        result.mapper_session = {
                message.mapper_session_boot_time_ns,
                message.mapper_session_random_suffix};
        result.map_epoch = message.map_epoch;
        result.base_revision = message.base_revision;
        result.new_revision = message.new_revision;
        result.revision_span = message.revision_span;
        result.observed_coalesced_receipt_count =
                message.observed_coalesced_receipt_count;
        result.known_cell_count = message.known_cell_count;
        result.operation_count = message.operation_count;
        result.canonical_payload_bytes = message.canonical_payload_bytes;
        result.base_content_hash = message.base_content_hash;
        result.content_hash = message.content_hash;
        result.update_hash = message.update_hash;
        return result;
    }

}// namespace PerceptionProfiling::Ros
