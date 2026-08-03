#include "OctoMapSnapshotBridge.hpp"

#ifdef PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS
#include "StageLatencyInstrumentation.hpp"
#endif

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "perception_core/health/MapperContractFingerprint.hpp"
#include "perception_core/observation/lidar_observation.hpp"
#include "perception_local_map/LocalObservationMapper.hpp"
#include "perception_local_map/OctoMapBackend.hpp"
#include "perception_interfaces/msg/health_state.hpp"
#include "perception_interfaces/msg/lidar_observation.hpp"
#include "perception_interfaces/msg/local_map_state.hpp"
#include "perception_interfaces/msg/pose_estimate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

    using Perception::Duration;
    using Perception::HealthState;
    using Perception::LidarObservation;
    using Perception::MapperInputContract;
    using Perception::PoseEstimate;
    using Perception::RayEvidenceCapability;
    using Perception::SensorDescriptor;
    using Perception::SensorID;
    using Perception::SensorRequirement;
    using Perception::SensorType;
    using Perception::SessionID;
    using Perception::Timestamp;
    using namespace PerceptionLocalMap;

    SessionID generate_session_id()
    {
        const auto boot = std::chrono::steady_clock::now().time_since_epoch();
        std::random_device random;
        return {
                static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(boot).count()),
                random()};
    }

    std::int64_t monotonic_now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
    }

    SensorType parse_sensor_type(const std::string & value)
    {
        if(value == "2d" || value == "LIDAR_2D") {
            return SensorType::LIDAR_2D;
        }
        if(value == "3d" || value == "LIDAR_3D") {
            return SensorType::LIDAR_3D;
        }
        throw std::invalid_argument("sensor type must be 2d or 3d");
    }

    RayEvidenceCapability parse_ray_evidence(const std::string & value)
    {
        if(value == "hit_only") {
            return RayEvidenceCapability::HitOnly;
        }
        if(value == "hit_ray") {
            return RayEvidenceCapability::HitRay;
        }
        if(value == "full_ray") {
            return RayEvidenceCapability::FullRay;
        }
        throw std::invalid_argument("ray evidence must be hit_only, hit_ray, or full_ray");
    }

    RayEvidenceCapability decode_ray_evidence(std::uint8_t value)
    {
        switch(value) {
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_HIT_ONLY:
                return RayEvidenceCapability::HitOnly;
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_HIT_RAY:
                return RayEvidenceCapability::HitRay;
            case perception_interfaces::msg::LidarObservation::RAY_EVIDENCE_FULL_RAY:
                return RayEvidenceCapability::FullRay;
            default:
                throw std::invalid_argument("unknown wire ray evidence");
        }
    }

    HealthState decode_health(std::uint8_t value)
    {
        switch(value) {
            case perception_interfaces::msg::HealthState::STATE_HEALTHY:
                return HealthState::Healthy;
            case perception_interfaces::msg::HealthState::STATE_DEGRADED:
                return HealthState::Degraded;
            case perception_interfaces::msg::HealthState::STATE_UNAVAILABLE:
                return HealthState::Unavailable;
            default:
                throw std::invalid_argument("unknown wire health state");
        }
    }

    std::uint8_t encode_health(HealthState value)
    {
        switch(value) {
            case HealthState::Healthy:
                return perception_interfaces::msg::LocalMapState::STATE_HEALTHY;
            case HealthState::Degraded:
                return perception_interfaces::msg::LocalMapState::STATE_DEGRADED;
            case HealthState::Unavailable:
                return perception_interfaces::msg::LocalMapState::STATE_UNAVAILABLE;
        }
        return perception_interfaces::msg::LocalMapState::STATE_UNAVAILABLE;
    }

    std::uint8_t encode_ray_evidence(RayEvidenceCapability value)
    {
        switch(value) {
            case RayEvidenceCapability::HitOnly:
                return perception_interfaces::msg::LocalMapState::RAY_EVIDENCE_HIT_ONLY;
            case RayEvidenceCapability::HitRay:
                return perception_interfaces::msg::LocalMapState::RAY_EVIDENCE_HIT_RAY;
            case RayEvidenceCapability::FullRay:
                return perception_interfaces::msg::LocalMapState::RAY_EVIDENCE_FULL_RAY;
        }
        return perception_interfaces::msg::LocalMapState::RAY_EVIDENCE_HIT_ONLY;
    }

    builtin_interfaces::msg::Time to_time(Timestamp value)
    {
        builtin_interfaces::msg::Time result;
        result.sec = static_cast<std::int32_t>(value.nanoseconds / 1'000'000'000LL);
        result.nanosec = static_cast<std::uint32_t>(value.nanoseconds % 1'000'000'000LL);
        return result;
    }

    bool is_zero_scan_residue(const perception_interfaces::msg::LidarObservation & message)
    {
        return message.angle_min == 0.0 && message.angle_max == 0.0
               && message.angle_increment == 0.0 && message.range_min == 0.0
               && message.range_max == 0.0 && message.ranges.empty()
               && message.intensities.empty();
    }

    LidarObservation decode_observation(
            const perception_interfaces::msg::LidarObservation & message)
    {
        if(message.sensor_id.empty() || message.header.frame_id.empty()
           || message.clock_domain.empty() || message.session_boot_time_ns == 0U) {
            throw std::invalid_argument("observation provenance is incomplete");
        }
        LidarObservation result;
        result.sensor_id = SensorID {message.sensor_id};
        result.session_id = {message.session_boot_time_ns, message.session_random_suffix};
        result.frame_id = message.header.frame_id;
        result.clock_domain = message.clock_domain;
        result.origin_stamp = Timestamp::from_nanoseconds(rclcpp::Time(message.header.stamp).nanoseconds());
        result.ray_evidence = decode_ray_evidence(message.ray_evidence);

        switch(message.data_type) {
            case perception_interfaces::msg::LidarObservation::DATA_TYPE_SCAN_2D:
                if(!message.points.empty() || !message.point_intensities.empty()) {
                    throw std::invalid_argument("Scan2D contains Cloud3D residue");
                }
                result.data = Perception::Scan2D {
                        message.angle_min, message.angle_max, message.angle_increment,
                        message.range_min, message.range_max, message.ranges,
                        message.intensities};
                break;
            case perception_interfaces::msg::LidarObservation::DATA_TYPE_CLOUD_3D: {
                if(!is_zero_scan_residue(message)
                   || result.ray_evidence != RayEvidenceCapability::HitOnly
                   || (!message.point_intensities.empty()
                       && message.point_intensities.size() != message.points.size())) {
                    throw std::invalid_argument("Cloud3D schema/capability is invalid");
                }
                Perception::Cloud3D cloud;
                cloud.points.reserve(message.points.size());
                for(std::size_t index = 0; index < message.points.size(); ++index) {
                    cloud.points.emplace_back(
                            static_cast<float>(message.points[index].x),
                            static_cast<float>(message.points[index].y),
                            static_cast<float>(message.points[index].z),
                            message.point_intensities.empty()
                                    ? 0.0F
                                    : message.point_intensities[index]);
                }
                result.data = std::move(cloud);
                break;
            }
            default:
                throw std::invalid_argument("unknown wire observation data type");
        }
        return result;
    }

    PoseEstimate decode_pose(const perception_interfaces::msg::PoseEstimate & message)
    {
        PoseEstimate result {
                Perception::SourceID {message.source_id},
                {message.session_boot_time_ns, message.session_random_suffix},
                message.header.frame_id, message.clock_domain,
                Timestamp::from_nanoseconds(rclcpp::Time(message.header.stamp).nanoseconds()),
                Eigen::Vector3d(
                        message.pose.position.x, message.pose.position.y,
                        message.pose.position.z),
                Eigen::Quaterniond(
                        message.pose.orientation.w, message.pose.orientation.x,
                        message.pose.orientation.y, message.pose.orientation.z),
                PoseEstimate::Matrix6d::Zero(), message.quality,
                Duration::from_nanoseconds(message.freshness_ns), message.reset_epoch};
        for(int row = 0; row < 6; ++row) {
            for(int column = 0; column < 6; ++column) {
                (*result.covariance)(row, column) =
                        message.covariance[static_cast<std::size_t>(row * 6 + column)];
            }
        }
        return result;
    }

}// namespace

class PerceptionLocalMapNode final : public rclcpp::Node
{
public:
    PerceptionLocalMapNode()
        : Node("perception_local_map_node")
        , tf_buffer_(get_clock())
        , tf_listener_(tf_buffer_)
    {
        observation_topic_ = declare_parameter<std::string>(
                "observation_topic", "perception/observations");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "perception/pose");
        health_topic_ = declare_parameter<std::string>("health_topic", "perception/health");
        state_topic_ = declare_parameter<std::string>("state_topic", "local_map/state");
        octomap_topic_ = declare_parameter<std::string>("octomap_topic", "local_map/octomap");
        body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
        health_validity_ns_ = seconds_to_ns(
                declare_parameter<double>("health_timeout_s", 1.0), "health_timeout_s");
        const double heartbeat_period = declare_parameter<double>(
                "state_heartbeat_period_s", 0.25);
        const auto heartbeat_period_ns = seconds_to_ns(
                heartbeat_period, "state_heartbeat_period_s");
        if(heartbeat_period_ns > std::numeric_limits<std::int64_t>::max() / 2) {
            throw std::invalid_argument("state_heartbeat_period_s is too large");
        }

        MapperConfig config = load_config();
        const auto backend_type = declare_parameter<std::string>("backend_type", "octomap");
        if(backend_type != "octomap") {
            throw std::invalid_argument("backend_type must be octomap");
        }
        mapper_ = std::make_unique<LocalObservationMapper>(
                std::move(config),
                [](const MapGeometry & geometry) {
                    return std::make_unique<OctoMapBackend>(geometry);
                });

        rclcpp::QoS state_qos(rclcpp::KeepLast(1));
        state_qos.reliable().durability_volatile();
        state_qos.lifespan(rclcpp::Duration::from_nanoseconds(heartbeat_period_ns));
        state_qos.deadline(rclcpp::Duration::from_nanoseconds(heartbeat_period_ns * 2));
        state_publisher_ = create_publisher<perception_interfaces::msg::LocalMapState>(
                state_topic_, state_qos);
        octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
                octomap_topic_, rclcpp::QoS(1).reliable().transient_local());
        diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", 10);

        observation_subscription_ = create_subscription<perception_interfaces::msg::LidarObservation>(
                observation_topic_, rclcpp::SensorDataQoS(),
                [this](const perception_interfaces::msg::LidarObservation::SharedPtr message) {
                    on_observation(*message);
                });
        pose_subscription_ = create_subscription<perception_interfaces::msg::PoseEstimate>(
                pose_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
                [this](const perception_interfaces::msg::PoseEstimate::SharedPtr message) {
                    on_pose(*message);
                });
        health_subscription_ = create_subscription<perception_interfaces::msg::HealthState>(
                health_topic_, rclcpp::QoS(10).reliable().durability_volatile(),
                [this](const perception_interfaces::msg::HealthState::SharedPtr message) {
                    on_health(*message);
                });

        heartbeat_timer_ = create_wall_timer(
                std::chrono::nanoseconds(heartbeat_period_ns),
                [this]() {
                    const auto now = monotonic_now_ns();
                    mapper_->tick(now);
                    publish_state(now);
                });
        publish_state(monotonic_now_ns());
    }

private:
    std::unique_ptr<LocalObservationMapper> mapper_;
    tf2_ros::Buffer                         tf_buffer_;
    tf2_ros::TransformListener              tf_listener_;

    std::string observation_topic_;
    std::string pose_topic_;
    std::string health_topic_;
    std::string state_topic_;
    std::string octomap_topic_;
    std::string body_frame_;
    std::int64_t health_validity_ns_ = 0;
    std::atomic<std::uint64_t> state_sequence_ {0};

    rclcpp::Subscription<perception_interfaces::msg::LidarObservation>::SharedPtr observation_subscription_;
    rclcpp::Subscription<perception_interfaces::msg::PoseEstimate>::SharedPtr pose_subscription_;
    rclcpp::Subscription<perception_interfaces::msg::HealthState>::SharedPtr health_subscription_;
    rclcpp::Publisher<perception_interfaces::msg::LocalMapState>::SharedPtr state_publisher_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    static std::int64_t seconds_to_ns(double value, const char * name)
    {
        if(!std::isfinite(value) || value <= 0.0
           || value > static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9) {
            throw std::invalid_argument(std::string(name) + " must be finite and positive");
        }
        return static_cast<std::int64_t>(value * 1e9);
    }

    SensorDescriptor load_descriptor(const std::string & sensor_id)
    {
        const auto prefix = "sensor." + sensor_id + ".";
        const auto type = parse_sensor_type(
                declare_parameter<std::string>(prefix + "type", "2d"));
        const auto vertical_min = type == SensorType::LIDAR_3D ? -1.57 : 0.0;
        const auto vertical_max = type == SensorType::LIDAR_3D ? 1.57 : 0.0;
        return {
                SensorID {sensor_id}, type,
                declare_parameter<std::string>(prefix + "frame_id", sensor_id + "_link"),
                Eigen::Vector3d(
                        declare_parameter<double>(prefix + "mounting_x", 0.0),
                        declare_parameter<double>(prefix + "mounting_y", 0.0),
                        declare_parameter<double>(prefix + "mounting_z", 0.0)),
                Eigen::Quaterniond(
                        declare_parameter<double>(prefix + "mounting_qw", 1.0),
                        declare_parameter<double>(prefix + "mounting_qx", 0.0),
                        declare_parameter<double>(prefix + "mounting_qy", 0.0),
                        declare_parameter<double>(prefix + "mounting_qz", 0.0)),
                Perception::FieldOfView {
                        declare_parameter<double>(prefix + "fov_horizontal_min_rad", -M_PI),
                        declare_parameter<double>(prefix + "fov_horizontal_max_rad", M_PI),
                        declare_parameter<double>(prefix + "fov_vertical_min_rad", vertical_min),
                        declare_parameter<double>(prefix + "fov_vertical_max_rad", vertical_max)},
                declare_parameter<double>(prefix + "angular_resolution_rad", 0.01),
                declare_parameter<double>(prefix + "range_min_m", 0.1),
                declare_parameter<double>(prefix + "range_max_m", 30.0),
                parse_ray_evidence(declare_parameter<std::string>(
                        prefix + "ray_evidence", "hit_only"))};
    }

    MapperConfig load_config()
    {
        MapperConfig config;
        config.vehicle_id = declare_parameter<std::string>("vehicle_id", "vehicle-1");
        config.mapper_session = generate_session_id();
        config.geometry = {
                declare_parameter<double>("resolution_m", 0.2),
                {declare_parameter<double>("lattice_origin_x", 0.0),
                 declare_parameter<double>("lattice_origin_y", 0.0),
                 declare_parameter<double>("lattice_origin_z", 0.0)},
                declare_parameter<std::string>("source_local_map_frame", "map")};
        config.body_frame = body_frame_;

        const auto sensor_ids = declare_parameter<std::vector<std::string>>(
                "sensor_ids", std::vector<std::string> {});
        for(const auto & sensor_id : sensor_ids) {
            config.sensor_descriptors.push_back(load_descriptor(sensor_id));
        }

        const auto minimum_type = parse_sensor_type(
                declare_parameter<std::string>("minimum_lidar_type", "2d"));
        const auto degraded_type = parse_sensor_type(
                declare_parameter<std::string>("degraded_lidar_type", "2d"));
        const auto minimum_count = declare_parameter<std::int64_t>("minimum_lidar_count", 1);
        const auto degraded_count = declare_parameter<std::int64_t>("degraded_lidar_count", 1);
        if(minimum_count <= 0 || degraded_count <= 0) {
            throw std::invalid_argument("lidar requirement counts must be positive");
        }
        config.input_contract.minimum_viable = {
                SensorRequirement {
                        minimum_type,
                        static_cast<std::size_t>(minimum_count), {},
                        parse_ray_evidence(declare_parameter<std::string>(
                                "minimum_lidar_ray_evidence", "hit_only"))}};
        config.input_contract.degraded_combinations = {
                Perception::DegradedCombination {
                        {SensorRequirement {
                                degraded_type,
                                static_cast<std::size_t>(degraded_count), {},
                                parse_ray_evidence(declare_parameter<std::string>(
                                        "degraded_lidar_ray_evidence", "hit_only"))}},
                        "Configured degraded lidar combination"}};
        config.input_contract.requires_pose = declare_parameter<bool>("requires_pose", true);
        config.input_contract.pose_freshness_threshold = Duration::from_nanoseconds(
                seconds_to_ns(declare_parameter<double>("pose_timeout_s", 1.0), "pose_timeout_s"));
        const auto expected_pose_frame = declare_parameter<std::string>(
                "expected_pose_frame", config.geometry.frame_id);
        if(!expected_pose_frame.empty()) {
            config.input_contract.expected_pose_frame = expected_pose_frame;
        }
        config.input_contract.minimum_pose_quality = declare_parameter<double>(
                "minimum_pose_quality", 0.0);
        if(!std::isfinite(config.input_contract.minimum_pose_quality)
           || config.input_contract.minimum_pose_quality < 0.0
           || config.input_contract.minimum_pose_quality > 1.0) {
            throw std::invalid_argument("minimum_pose_quality must be in [0, 1]");
        }
        const auto recovery_samples = declare_parameter<std::int64_t>(
                "recovery_stability_samples", 3);
        if(recovery_samples <= 0) {
            throw std::invalid_argument("recovery_stability_samples must be positive");
        }
        config.input_contract.recovery_stability_samples =
                static_cast<std::size_t>(recovery_samples);
        config.sensor_validity_ns = seconds_to_ns(
                declare_parameter<double>("sensor_timeout_s", 1.0), "sensor_timeout_s");
        config.health_validity_ns = health_validity_ns_;
        config.pose_receive_validity_ns = seconds_to_ns(
                declare_parameter<double>("pose_receive_timeout_s", 1.0),
                "pose_receive_timeout_s");
        config.map_freshness_ns = seconds_to_ns(
                declare_parameter<double>("map_freshness_s", 2.0), "map_freshness_s");
        const auto pose_history_limit = declare_parameter<std::int64_t>(
                "pose_history_limit", 128);
        if(pose_history_limit <= 0) {
            throw std::invalid_argument("pose_history_limit must be positive");
        }
        config.pose_history_limit = static_cast<std::size_t>(pose_history_limit);
        config.position_jump_threshold_m = declare_parameter<double>(
                "position_jump_threshold_m", 2.0);
        config.orientation_jump_threshold_rad = declare_parameter<double>(
                "orientation_jump_threshold_rad", 0.7853981633974483);
        config.extrinsic_position_tolerance_m = declare_parameter<double>(
                "extrinsic_position_tolerance_m", 1e-4);
        config.extrinsic_orientation_tolerance_rad = declare_parameter<double>(
                "extrinsic_orientation_tolerance_rad", 1e-4);
        return config;
    }

    void publish_diagnostic(std::uint8_t level, const std::string & message)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = get_clock()->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name() + std::string(": mapper_input");
        status.hardware_id = "vehicle-local";
        status.level = level;
        status.message = message;
        array.status.push_back(std::move(status));
        diagnostics_publisher_->publish(array);
    }

    SensorExtrinsicSample lookup_extrinsic(const LidarObservation & observation)
    {
        const auto transform = tf_buffer_.lookupTransform(
                body_frame_, observation.frame_id,
                rclcpp::Time(observation.origin_stamp.nanoseconds),
                rclcpp::Duration::from_seconds(0.1));
        return {
                observation.sensor_id, observation.session_id, observation.frame_id,
                Eigen::Vector3d(
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z),
                Eigen::Quaterniond(
                        transform.transform.rotation.w, transform.transform.rotation.x,
                        transform.transform.rotation.y, transform.transform.rotation.z)};
    }

    void on_health(const perception_interfaces::msg::HealthState & message)
    {
        try {
            UpstreamHealth health {
                    message.producer_source_id,
                    {message.producer_session_boot_time_ns,
                     message.producer_session_random_suffix},
                    {message.mapper_contract_schema_version,
                     message.mapper_contract_fingerprint},
                    decode_health(message.state), monotonic_now_ns(), health_validity_ns_};
            if(!mapper_->submit_health(health)) {
                publish_diagnostic(
                        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                        "rejected upstream health contract/session");
            }
        }
        catch(const std::exception & error) {
            publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
        }
        publish_state(monotonic_now_ns());
    }

    void on_pose(const perception_interfaces::msg::PoseEstimate & message)
    {
        try {
            const auto result = mapper_->submit_pose(decode_pose(message), monotonic_now_ns());
            if(result.status != InputStatus::Accepted) {
                publish_diagnostic(
                        diagnostic_msgs::msg::DiagnosticStatus::WARN, result.diagnostic);
            }
        }
        catch(const std::exception & error) {
            publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
        }
        publish_state(monotonic_now_ns());
    }

    void on_observation(const perception_interfaces::msg::LidarObservation & message)
    {
#ifdef PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS
        StageLatency::CallbackScope stage_latency_callback;
#endif
        std::optional<Perception::SensorID> decoded_sensor_id;
        const std::int64_t receive_time = monotonic_now_ns();
        try {
            const auto observation = decode_observation(message);
            decoded_sensor_id = observation.sensor_id;
            const auto now = receive_time;
            const auto result = mapper_->submit_observation(
                    observation, lookup_extrinsic(observation), now);
            if(result.status == InputStatus::Applied && result.receipt.has_value()) {
#ifdef PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS
                stage_latency_callback.mark_applied(result.receipt->revision);
#endif
                publish_octomap(result.receipt.value());
            }
            else if(result.status != InputStatus::NoEvidence) {
                publish_diagnostic(
                        result.status == InputStatus::BackendFault
                                ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
                                : diagnostic_msgs::msg::DiagnosticStatus::WARN,
                        result.diagnostic);
            }
            publish_state(now);
        }
        catch(const tf2::TransformException & error) {
            if(decoded_sensor_id.has_value()) {
                mapper_->mark_sensor_unavailable(
                        decoded_sensor_id.value(), receive_time);
            }
            publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::WARN, error.what());
            publish_state(monotonic_now_ns());
        }
        catch(const std::exception & error) {
            publish_diagnostic(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error.what());
            publish_state(monotonic_now_ns());
        }
    }

    void publish_octomap(const CommitReceipt & receipt)
    {
#ifdef PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS
        StageLatency::StageScope stage_latency_scope(
                StageLatency::Stage::SnapshotTotal, receipt.revision);
#endif
        auto acquired = mapper_->acquire_read_transaction(receipt);
        if(acquired.status == AcquireStatus::Superseded) {
            const auto latest = mapper_->state(monotonic_now_ns()).identity;
            if(latest.revision == 0U) {
                return;
            }
            acquired = mapper_->acquire_read_transaction(
                    {latest.mapper_session, latest.map_epoch, latest.revision});
        }
        if(acquired.status != AcquireStatus::Ready || !acquired.transaction.has_value()) {
            return;
        }
        const auto metadata = acquired.transaction->metadata();
        if(!metadata.has_value()) {
            return;
        }
        octomap_msgs::msg::Octomap message;
        if(!OctoMapSnapshotBridge::materialize(acquired.transaction.value(), message)) {
            return;
        }
        acquired.transaction->close();
        message.header.stamp = get_clock()->now();
        message.header.frame_id = metadata->geometry.frame_id;
        octomap_publisher_->publish(message);
    }

    void publish_state(std::int64_t monotonic_now)
    {
#ifdef PERCEPTION_LOCAL_MAP_ENABLE_STAGE_LATENCY_TRACEPOINTS
        StageLatency::StageScope stage_latency_scope(
                StageLatency::Stage::StatePublication);
#endif
        const auto state = mapper_->state(monotonic_now);
        auto published_health = state.health;
        auto published_capabilities = state.effective_capabilities;
        if(!published_capabilities.is_consistent()
           || published_health == HealthState::Unavailable) {
            published_health = HealthState::Unavailable;
            published_capabilities = {};
        }
        perception_interfaces::msg::LocalMapState message;
        message.header.stamp = get_clock()->now();
        message.header.frame_id = state.geometry.frame_id;
        message.vehicle_id = state.identity.vehicle_id;
        message.mapper_session_boot_time_ns = state.identity.mapper_session.boot_time_ns;
        message.mapper_session_random_suffix = state.identity.mapper_session.random_suffix;
        message.state_sequence = state_sequence_.fetch_add(1) + 1U;
        message.map_epoch = state.identity.map_epoch;
        message.revision = state.identity.revision;
        message.health = encode_health(published_health);
        message.map_fresh = published_health != HealthState::Unavailable && state.map_fresh;
        message.validity_remaining_ns = published_health == HealthState::Unavailable
                                                ? 0
                                                : std::max<std::int64_t>(
                                                          0, state.validity_remaining_ns);
        message.resolution_m = state.geometry.resolution_m;
        message.lattice_origin.x = state.geometry.lattice_origin.x;
        message.lattice_origin.y = state.geometry.lattice_origin.y;
        message.lattice_origin.z = state.geometry.lattice_origin.z;
        message.has_known_bounds = state.known_bounds.has_value();
        if(state.known_bounds.has_value()) {
            message.known_bounds_min.x = state.known_bounds->minimum.x;
            message.known_bounds_min.y = state.known_bounds->minimum.y;
            message.known_bounds_min.z = state.known_bounds->minimum.z;
            message.known_bounds_max.x = state.known_bounds->maximum.x;
            message.known_bounds_max.y = state.known_bounds->maximum.y;
            message.known_bounds_max.z = state.known_bounds->maximum.z;
        }
        message.mapper_contract_schema_version = state.contract_fingerprint.schema_version;
        message.mapper_contract_fingerprint = state.contract_fingerprint.hex_digest;
        message.satisfied_combination_id = published_capabilities.satisfied_combination_id;
        message.has_active_ray_evidence = published_capabilities.has_active_ray_evidence;
        message.maximum_active_ray_evidence = encode_ray_evidence(
                published_capabilities.maximum_active_ray_evidence);
        message.active_sensor_count = published_capabilities.active_sensor_count;
        message.active_2d_hit_only_sensor_count =
                published_capabilities.active_2d_hit_only_sensor_count;
        message.active_2d_hit_ray_sensor_count =
                published_capabilities.active_2d_hit_ray_sensor_count;
        message.active_2d_full_ray_sensor_count =
                published_capabilities.active_2d_full_ray_sensor_count;
        message.active_3d_hit_only_sensor_count =
                published_capabilities.active_3d_hit_only_sensor_count;
        message.active_3d_hit_ray_sensor_count =
                published_capabilities.active_3d_hit_ray_sensor_count;
        message.active_3d_full_ray_sensor_count =
                published_capabilities.active_3d_full_ray_sensor_count;
        if(state.last_commit.has_value()) {
            message.last_sensor_id = state.last_commit->sensor_id.value;
            message.last_sensor_session_boot_time_ns = state.last_commit->sensor_session.boot_time_ns;
            message.last_sensor_session_random_suffix = state.last_commit->sensor_session.random_suffix;
            message.last_observation_stamp = to_time(state.last_commit->observation_stamp);
            message.last_observation_clock_domain = state.last_commit->clock_domain;
            message.last_commit_changed_cell_count = state.last_commit->changed_cell_count;
        }
        message.supports_point_query = state.backend_capabilities.supports_point_query;
        message.supports_bounded_region_query = state.backend_capabilities.supports_bounded_region_query;
        message.supports_hierarchy = state.backend_capabilities.supports_hierarchy;
        message.supports_dirty_region = state.backend_capabilities.supports_dirty_region;
        message.supports_serialization = state.backend_capabilities.supports_serialization;
        message.has_committed_alignment = state.alignment.has_value()
                                          && state.alignment->status == AlignmentStatus::Committed;
        if(state.alignment.has_value()) {
            message.alignment_epoch = state.alignment->alignment_epoch;
            message.alignment_revision = state.alignment->alignment_revision;
        }
        state_publisher_->publish(message);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<PerceptionLocalMapNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("perception_local_map_node"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
