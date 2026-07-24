#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "perception_adapters/laser_scan_adapter.hpp"
#include "perception_adapters/odometry_adapter.hpp"
#include "perception_adapters/point_cloud2_adapter.hpp"
#include "perception_core/health/health_state.hpp"
#include "perception_core/health/mapper_health_gate.hpp"
#include "perception_core/health/mapper_input_contract.hpp"
#include "perception_core/observation/lidar_observation.hpp"
#include "perception_core/observation/pose_estimate.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"
#include "perception_input_node/SensorSessionManager.hpp"
#include "perception_interfaces/msg/health_state.hpp"
#include "perception_interfaces/msg/lidar_observation.hpp"
#include "perception_interfaces/msg/pose_estimate.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include <algorithm>
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
    using Perception::MapperHealthGate;
    using Perception::MapperInputContract;
    using Perception::PoseEstimate;
    using Perception::SensorDescriptor;
    using Perception::SensorHealth;
    using Perception::SensorHealthStatus;
    using Perception::SensorID;
    using Perception::SensorRequirement;
    using Perception::SensorType;
    using Perception::SessionID;
    using Perception::SourceID;
    using Perception::Timestamp;

    SessionID generate_session_id()
    {
        const auto         boot_time = std::chrono::steady_clock::now().time_since_epoch();
        std::random_device random_device;
        return SessionID {
                static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(boot_time).count()),
                random_device()};
    }

    rclcpp::Time to_rcl_time(const Timestamp & timestamp)
    {
        return rclcpp::Time(timestamp.nanoseconds);
    }

    uint8_t to_ros_health_state(HealthState state)
    {
        switch(state) {
            case HealthState::Healthy:
                return perception_interfaces::msg::HealthState::STATE_HEALTHY;
            case HealthState::Degraded:
                return perception_interfaces::msg::HealthState::STATE_DEGRADED;
            case HealthState::Unavailable:
                return perception_interfaces::msg::HealthState::STATE_UNAVAILABLE;
        }
        return perception_interfaces::msg::HealthState::STATE_UNAVAILABLE;
    }

}// namespace

class PerceptionInputNode final : public rclcpp::Node
{
public:
    PerceptionInputNode()
        : Node("perception_input_node")
        , session_id_(generate_session_id())
        , session_manager_(session_id_)
        , source_id_ {declare_parameter<std::string>("pose_source_id", "odom")}
        , pose_input_type_(declare_parameter<std::string>("pose_input_type", "odometry"))
        , expected_pose_frame_(declare_parameter<std::string>("expected_pose_frame", ""))
        , clock_domain_(declare_parameter<std::string>("clock_domain", "vehicle_steady_clock"))
        , minimum_pose_quality_(declare_parameter<double>("minimum_pose_quality", 0.0))
        , sensor_timeout_(declare_parameter<double>("sensor_timeout_s", 1.0))
        , pose_timeout_(declare_parameter<double>("pose_timeout_s", 1.0))
        , health_period_(declare_parameter<double>("health_period_s", 1.0))
        , observation_topic_(declare_parameter<std::string>("observation_topic", "perception/observations"))
        , pose_topic_(declare_parameter<std::string>("pose_topic", "perception/pose"))
        , health_topic_(declare_parameter<std::string>("health_topic", "perception/health"))
        , odom_topic_(declare_parameter<std::string>("odom_topic", "odom"))
        , tf_topic_(declare_parameter<std::string>("tf_topic", "/tf"))
        , tf_child_frame_(declare_parameter<std::string>("tf_child_frame", "base_link"))
    {
        load_contract();
        load_sensor_descriptors();

        observation_publisher_ = create_publisher<perception_interfaces::msg::LidarObservation>(
                observation_topic_, rclcpp::SensorDataQoS());
        pose_publisher_ = create_publisher<perception_interfaces::msg::PoseEstimate>(
                pose_topic_, rclcpp::QoS(10));
        health_publisher_ = create_publisher<perception_interfaces::msg::HealthState>(
                health_topic_, rclcpp::QoS(10));

        create_pose_subscription();

        health_timer_ = create_wall_timer(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::duration<double>(std::max(0.1, health_period_))),
                [this]() { update_health(); });

        evaluate_and_publish_health(true);
        RCLCPP_INFO(
                get_logger(), "Perception input node started with session %llu/%u, %zu sensors and %s pose input",
                static_cast<unsigned long long>(session_id_.boot_time_ns), session_id_.random_suffix,
                descriptors_.size(), pose_input_type_.c_str());
    }

private:
    struct SensorSubscriptions {
        SensorDescriptor descriptor;
        std::string      topic;
    };

    SessionID                               session_id_;
    Perception::Input::SensorSessionManager session_manager_;
    std::vector<SensorDescriptor>           descriptors_;
    std::vector<SensorHealth>               sensor_health_;
    std::vector<SensorSubscriptions>        sensor_subscriptions_;

    SourceID    source_id_;
    std::string pose_input_type_;
    std::string expected_pose_frame_;
    std::string clock_domain_;
    double      minimum_pose_quality_;
    double      sensor_timeout_;
    double      pose_timeout_;
    double      health_period_;
    std::string observation_topic_;
    std::string pose_topic_;
    std::string health_topic_;
    std::string odom_topic_;
    std::string tf_topic_;
    std::string tf_child_frame_;

    std::unique_ptr<MapperHealthGate> health_gate_;
    std::optional<PoseEstimate>       latest_pose_;

    HealthState last_published_state_ = HealthState::Unavailable;
    std::string last_published_reason_;
    bool        has_published_health_ = false;

    Perception::Adapters::LaserScanAdapter   scan_adapter_;
    Perception::Adapters::PointCloud2Adapter cloud_adapter_;
    Perception::Adapters::OdometryAdapter    odom_adapter_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr>   scan_subscriptions_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscriptions_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                   odom_subscription_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr                  tf_subscription_;
    rclcpp::Publisher<perception_interfaces::msg::LidarObservation>::SharedPtr observation_publisher_;
    rclcpp::Publisher<perception_interfaces::msg::PoseEstimate>::SharedPtr     pose_publisher_;
    rclcpp::Publisher<perception_interfaces::msg::HealthState>::SharedPtr      health_publisher_;
    rclcpp::TimerBase::SharedPtr                                               health_timer_;

    void create_pose_subscription()
    {
        if(pose_input_type_ == "odometry") {
            odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                    odom_topic_, rclcpp::SensorDataQoS(),
                    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odometry(msg); });
            return;
        }

        if(pose_input_type_ == "tf") {
            if(tf_child_frame_.empty()) {
                throw std::runtime_error("tf_child_frame must be set when pose_input_type=tf");
            }
            tf_subscription_ = create_subscription<tf2_msgs::msg::TFMessage>(
                    tf_topic_, rclcpp::QoS(rclcpp::KeepLast(100)).reliable().durability_volatile(),
                    [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { on_tf(msg); });
            return;
        }

        throw std::runtime_error(
                "pose_input_type must be either 'odometry' or 'tf', got '" + pose_input_type_ + "'");
    }

    void load_contract()
    {
        MapperInputContract contract;
        const auto          minimum_type   = declare_parameter<std::string>("minimum_lidar_type", "3d");
        const auto          degraded_type  = declare_parameter<std::string>("degraded_lidar_type", "2d");
        const auto          minimum_count  = declare_parameter<int64_t>("minimum_lidar_count", 1);
        const auto          degraded_count = declare_parameter<int64_t>("degraded_lidar_count", 1);
        contract.requires_pose             = declare_parameter<bool>("requires_pose", true);
        contract.pose_freshness_threshold  = Duration::from_seconds(pose_timeout_);
        if(!std::isfinite(minimum_pose_quality_) || minimum_pose_quality_ < 0.0
           || minimum_pose_quality_ > 1.0) {
            throw std::runtime_error("minimum_pose_quality must be in [0, 1]");
        }
        contract.minimum_pose_quality = minimum_pose_quality_;
        if(!expected_pose_frame_.empty()) {
            contract.expected_pose_frame = expected_pose_frame_;
        }
        contract.recovery_stability_samples = static_cast<std::size_t>(std::max<int64_t>(
                1, declare_parameter<int64_t>("recovery_stability_samples", 3)));

        contract.minimum_viable.push_back(
                SensorRequirement {parse_sensor_type(minimum_type),
                                   static_cast<std::size_t>(std::max<int64_t>(1, minimum_count)),
                                   {}});
        contract.degraded_combinations.push_back(
                Perception::DegradedCombination {
                        {SensorRequirement {parse_sensor_type(degraded_type),
                                            static_cast<std::size_t>(std::max<int64_t>(1, degraded_count)),
                                            {}}},
                        "Configured degraded lidar combination"});
        health_gate_ = std::make_unique<MapperHealthGate>(std::move(contract));
    }

    void load_sensor_descriptors()
    {
        const auto sensor_ids = declare_parameter<std::vector<std::string>>(
                "sensor_ids", std::vector<std::string> {});
        for(const auto & sensor_id : sensor_ids) {
            const auto descriptor = descriptor_from_parameters(sensor_id);
            const auto result     = session_manager_.register_sensor(descriptor);
            if(!result.accepted) {
                throw std::runtime_error(result.diagnostic);
            }

            descriptors_.push_back(descriptor);
            sensor_health_.push_back(SensorHealth {descriptor.sensor_id, SensorHealthStatus::Lost, Timestamp::from_nanoseconds(0)});
            sensor_subscriptions_.push_back(SensorSubscriptions {descriptor, declare_parameter<std::string>(
                                                                                     "sensor." + sensor_id + ".topic",
                                                                                     "sensors/" + sensor_id)});

            if(descriptor.type == SensorType::LIDAR_2D) {
                scan_subscriptions_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
                        sensor_subscriptions_.back().topic, rclcpp::SensorDataQoS(),
                        [this, id = descriptor.sensor_id](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                            on_laser_scan(msg, id);
                        }));
            }
            else {
                cloud_subscriptions_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
                        sensor_subscriptions_.back().topic, rclcpp::SensorDataQoS(),
                        [this, id = descriptor.sensor_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                            on_point_cloud(msg, id);
                        }));
            }
        }
    }

    SensorDescriptor descriptor_from_parameters(const std::string & sensor_id)
    {
        const auto prefix               = "sensor." + sensor_id + ".";
        const auto type_name            = declare_parameter<std::string>(prefix + "type", "2d");
        const auto type                 = parse_sensor_type(type_name);
        const auto default_vertical_min = type == SensorType::LIDAR_3D ? -1.57 : 0.0;
        const auto default_vertical_max = type == SensorType::LIDAR_3D ? 1.57 : 0.0;

        return SensorDescriptor {
                SensorID {sensor_id},
                type,
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
                        declare_parameter<double>(prefix + "fov_horizontal_min_rad", -3.141592653589793),
                        declare_parameter<double>(prefix + "fov_horizontal_max_rad", 3.141592653589793),
                        declare_parameter<double>(prefix + "fov_vertical_min_rad", default_vertical_min),
                        declare_parameter<double>(prefix + "fov_vertical_max_rad", default_vertical_max)},
                declare_parameter<double>(prefix + "angular_resolution_rad", 0.01),
                declare_parameter<double>(prefix + "range_min_m", 0.1),
                declare_parameter<double>(prefix + "range_max_m", 30.0)};
    }

    static SensorType parse_sensor_type(const std::string & value)
    {
        if(value == "2d" || value == "LIDAR_2D") {
            return SensorType::LIDAR_2D;
        }
        if(value == "3d" || value == "LIDAR_3D") {
            return SensorType::LIDAR_3D;
        }
        throw std::invalid_argument("Unsupported lidar type '" + value + "'; expected 2d or 3d");
    }

    void on_laser_scan(
            const sensor_msgs::msg::LaserScan::SharedPtr & msg,
            const SensorID &                               sensor_id)
    {
        const auto * descriptor = session_manager_.find_descriptor(sensor_id);
        if(descriptor == nullptr) {
            RCLCPP_ERROR(get_logger(), "Received LaserScan for unknown sensor '%s'", sensor_id.value.c_str());
            return;
        }

        const auto validation = scan_adapter_.validate(*msg, *descriptor);
        if(!validation.valid) {
            RCLCPP_WARN(
                    get_logger(), "Rejecting LaserScan from '%s': %s", sensor_id.value.c_str(),
                    validation.error_message.c_str());
            return;
        }

        publish_observation(scan_adapter_.convert(*msg, *descriptor, session_id_, clock_domain_));
        mark_sensor_active(sensor_id);
        if(!session_manager_.is_frozen()) {
            session_manager_.freeze();
        }
        evaluate_and_publish_health(false);
    }

    void on_point_cloud(
            const sensor_msgs::msg::PointCloud2::SharedPtr & msg,
            const SensorID &                                 sensor_id)
    {
        const auto * descriptor = session_manager_.find_descriptor(sensor_id);
        if(descriptor == nullptr) {
            RCLCPP_ERROR(get_logger(), "Received PointCloud2 for unknown sensor '%s'", sensor_id.value.c_str());
            return;
        }

        const auto validation = cloud_adapter_.validate(*msg, *descriptor);
        if(!validation.valid) {
            RCLCPP_WARN(
                    get_logger(), "Rejecting PointCloud2 from '%s': %s", sensor_id.value.c_str(),
                    validation.error_message.c_str());
            return;
        }

        publish_observation(cloud_adapter_.convert(*msg, *descriptor, session_id_, clock_domain_));
        mark_sensor_active(sensor_id);
        if(!session_manager_.is_frozen()) {
            session_manager_.freeze();
        }
        evaluate_and_publish_health(false);
    }

    void on_odometry(const nav_msgs::msg::Odometry::SharedPtr & msg)
    {
        accept_pose(odom_adapter_.convert(*msg, source_id_, session_id_, clock_domain_));
    }

    void on_tf(const tf2_msgs::msg::TFMessage::SharedPtr & msg)
    {
        const auto transform = std::find_if(
                msg->transforms.begin(), msg->transforms.end(),
                [&](const geometry_msgs::msg::TransformStamped & candidate) {
                    return candidate.child_frame_id == tf_child_frame_;
                });
        if(transform == msg->transforms.end()) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "TF input contains no transform for child frame '%s'", tf_child_frame_.c_str());
            return;
        }

        accept_pose(odom_adapter_.from_tf(*transform, source_id_, session_id_, clock_domain_));
    }

    void accept_pose(PoseEstimate pose)
    {
        publish_pose(pose);
        latest_pose_ = std::move(pose);
        evaluate_and_publish_health(false);
    }

    void mark_sensor_active(const SensorID & sensor_id)
    {
        const auto it = std::find_if(
                sensor_health_.begin(), sensor_health_.end(),
                [&](const SensorHealth & health) { return health.sensor_id == sensor_id; });
        if(it == sensor_health_.end()) {
            return;
        }
        it->status      = SensorHealthStatus::Active;
        it->last_update = Timestamp::from_nanoseconds(get_clock()->now().nanoseconds());
    }

    void update_health()
    {
        const auto now_ns = get_clock()->now().nanoseconds();
        for(auto & health : sensor_health_) {
            if(health.status == SensorHealthStatus::Active && now_ns - health.last_update.nanoseconds > static_cast<int64_t>(sensor_timeout_ * 1e9)) {
                health.status = SensorHealthStatus::Stale;
            }
        }

        if(latest_pose_.has_value()) {
            const auto freshness_ns = std::max<int64_t>(0, now_ns - latest_pose_->stamp.nanoseconds);
            latest_pose_->freshness = Duration::from_nanoseconds(freshness_ns);
        }
        evaluate_and_publish_health(true);
    }

    void evaluate_and_publish_health(bool force_publish)
    {
        const auto state  = health_gate_->evaluate(descriptors_, sensor_health_, latest_pose_);
        const auto reason = health_gate_->degradation_reason();
        if(force_publish || !has_published_health_ || state != last_published_state_ || reason != last_published_reason_) {
            publish_health(state, reason);
            last_published_state_  = state;
            last_published_reason_ = reason;
            has_published_health_  = true;
        }
    }

    void publish_observation(const LidarObservation & observation)
    {
        perception_interfaces::msg::LidarObservation message;
        message.header.stamp = static_cast<builtin_interfaces::msg::Time>(
                to_rcl_time(observation.origin_stamp));
        message.header.frame_id       = observation.frame_id;
        message.sensor_id             = observation.sensor_id.value;
        message.session_boot_time_ns  = observation.session_id.boot_time_ns;
        message.session_random_suffix = observation.session_id.random_suffix;
        message.clock_domain          = observation.clock_domain;

        if(observation.is_2d()) {
            const auto & scan       = observation.as_scan_2d();
            message.data_type       = perception_interfaces::msg::LidarObservation::DATA_TYPE_SCAN_2D;
            message.angle_min       = scan.angle_min_rad;
            message.angle_max       = scan.angle_max_rad;
            message.angle_increment = scan.angle_increment_rad;
            message.range_min       = scan.range_min_m;
            message.range_max       = scan.range_max_m;
            message.ranges          = scan.ranges;
            message.intensities     = scan.intensities;
        }
        else {
            const auto & cloud = observation.as_cloud_3d();
            message.data_type  = perception_interfaces::msg::LidarObservation::DATA_TYPE_CLOUD_3D;
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
        observation_publisher_->publish(message);
    }

    void publish_pose(const PoseEstimate & pose)
    {
        perception_interfaces::msg::PoseEstimate message;
        message.header.stamp          = static_cast<builtin_interfaces::msg::Time>(to_rcl_time(pose.stamp));
        message.header.frame_id       = pose.frame_id;
        message.source_id             = pose.source_id.value;
        message.session_boot_time_ns  = pose.session_id.boot_time_ns;
        message.session_random_suffix = pose.session_id.random_suffix;
        message.clock_domain          = pose.clock_domain;
        message.pose.position.x       = pose.position.x();
        message.pose.position.y       = pose.position.y();
        message.pose.position.z       = pose.position.z();
        message.pose.orientation.w    = pose.orientation.w();
        message.pose.orientation.x    = pose.orientation.x();
        message.pose.orientation.y    = pose.orientation.y();
        message.pose.orientation.z    = pose.orientation.z();
        std::fill(message.covariance.begin(), message.covariance.end(), 0.0);
        if(pose.covariance.has_value()) {
            for(int row = 0; row < 6; ++row) {
                for(int column = 0; column < 6; ++column) {
                    message.covariance[static_cast<std::size_t>(row * 6 + column)] =
                            (*pose.covariance)(row, column);
                }
            }
        }
        message.quality      = pose.quality;
        message.freshness_ns = pose.freshness.nanoseconds;
        message.reset_epoch  = pose.reset_epoch;
        pose_publisher_->publish(message);
    }

    void publish_health(HealthState state, const std::string & reason)
    {
        const auto                              capabilities = health_gate_->current_capability();
        perception_interfaces::msg::HealthState message;
        message.header.stamp        = static_cast<builtin_interfaces::msg::Time>(get_clock()->now());
        message.state               = to_ros_health_state(state);
        message.degradation_reason  = reason;
        message.has_2d_lidar        = capabilities.has_2d_lidar;
        message.has_3d_lidar        = capabilities.has_3d_lidar;
        message.has_fresh_pose      = capabilities.has_fresh_pose;
        message.active_sensor_count = static_cast<uint32_t>(std::min<std::size_t>(
                capabilities.active_sensor_count, std::numeric_limits<uint32_t>::max()));
        health_publisher_->publish(message);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PerceptionInputNode>());
    rclcpp::shutdown();
    return 0;
}
