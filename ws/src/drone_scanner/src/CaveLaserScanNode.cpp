#include "drone_scanner/CaveLaserScanNode.hpp"

#include "drone_scanner/CaveFieldFromParameters.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace DroneScanner {

    namespace {

        constexpr double kMotionEpsilon = 1.0e-6;
        constexpr double kAttitudeToleranceRad = 1.0e-6;

        bool is_finite(const geometry_msgs::msg::Quaternion & value)
        {
            return std::isfinite(value.x) && std::isfinite(value.y)
                   && std::isfinite(value.z) && std::isfinite(value.w);
        }

    }// namespace

    CaveLaserScanNode::CaveLaserScanNode()
        : rclcpp::Node("cave_laser_scan")
    {
        declareCaveFieldParameters(*this);
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        scan_frame_ = declare_parameter<std::string>("scan_frame", "scan_link");
        const auto odom_topic = declare_parameter<std::string>("odom_topic", "odom");
        const auto scan_topic = declare_parameter<std::string>("scan_topic", "raw_scan");
        const auto beam_count_parameter = declare_parameter<std::int64_t>("beam_count", 360);
        const auto range_min_m = declare_parameter<double>("range_min_m", 0.1);
        const auto range_max_m = declare_parameter<double>("range_max_m", 30.0);
        const auto scan_rate_hz = declare_parameter<double>("scan_rate_hz", 10.0);
        const auto range_noise_std = declare_parameter<double>("range_noise_std", 0.0);
        const auto noise_seed = declare_parameter<std::int64_t>("noise_seed", 0);

        if(map_frame_.empty() || base_frame_.empty() || scan_frame_.empty()
           || odom_topic.empty() || scan_topic.empty()) {
            throw std::invalid_argument("CaveLaserScanNode frame and topic names must not be empty");
        }
        if(beam_count_parameter < 2
           || beam_count_parameter > static_cast<std::int64_t>(std::numeric_limits<int>::max())
           || noise_seed < 0
           || noise_seed > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())
           || !std::isfinite(range_noise_std) || range_noise_std < 0.0) {
            throw std::invalid_argument("CaveLaserScanNode scan parameters are invalid");
        }

        const auto beam_count = static_cast<std::size_t>(beam_count_parameter);
        projection_ = std::make_unique<LaserScanProjection>(LaserScanProjectionConfig {
                beam_count, static_cast<float>(range_min_m),
                static_cast<float>(range_max_m), scan_rate_hz});
        scan_period_ns_ = static_cast<std::int64_t>(std::llround(1.0e9 / scan_rate_hz));
        if(scan_period_ns_ <= 0) {
            throw std::invalid_argument("CaveLaserScanNode scan period is invalid");
        }

        field_ = createCaveFieldFromParameters(*this);
        lidar_ = std::make_unique<FakeLidar>(field_, FakeLidarConfig {
                static_cast<int>(beam_count), static_cast<float>(range_max_m),
                static_cast<float>(range_noise_std), static_cast<std::uint32_t>(noise_seed),
                0.0F});

        scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
                scan_topic, rclcpp::SensorDataQoS());
        diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", rclcpp::QoS(10));
        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, rclcpp::SensorDataQoS(),
                [this](const nav_msgs::msg::Odometry::SharedPtr message) { onOdom(message); });

        RCLCPP_INFO(
                get_logger(), "cave laser scan: %zu beams, %.1f Hz, range [%.2f, %.2f] m",
                beam_count, scan_rate_hz, range_min_m, range_max_m);
    }

    bool CaveLaserScanNode::poseFromOdom(
            const nav_msgs::msg::Odometry & message,
            Pose3D & pose) const
    {
        const auto & position = message.pose.pose.position;
        const auto & orientation = message.pose.pose.orientation;
        if(message.header.frame_id != map_frame_ || message.child_frame_id != base_frame_
           || !std::isfinite(position.x) || !std::isfinite(position.y)
           || !std::isfinite(position.z) || !is_finite(orientation)) {
            return false;
        }

        const double norm = std::sqrt(
                orientation.x * orientation.x + orientation.y * orientation.y
                + orientation.z * orientation.z + orientation.w * orientation.w);
        if(norm <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
        const double x = orientation.x / norm;
        const double y = orientation.y / norm;
        const double z = orientation.z / norm;
        const double w = orientation.w / norm;
        const double sin_pitch = std::clamp(2.0 * (w * y - z * x), -1.0, 1.0);
        const double roll = std::atan2(
                2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        const double pitch = std::asin(sin_pitch);
        if(std::abs(roll) > kAttitudeToleranceRad
           || std::abs(pitch) > kAttitudeToleranceRad) {
            return false;
        }

        pose.x = static_cast<float>(position.x);
        pose.y = static_cast<float>(position.y);
        pose.z = static_cast<float>(position.z);
        pose.yaw = static_cast<float>(std::atan2(
                2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
        return std::isfinite(pose.yaw);
    }

    void CaveLaserScanNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr message)
    {
        if(scanning_stopped_) {
            return;
        }
        const std::int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
        if(stamp_ns <= 0 || (last_odom_stamp_ns_ != 0 && stamp_ns <= last_odom_stamp_ns_)) {
            publishDiagnostic(
                    diagnostic_msgs::msg::DiagnosticStatus::WARN,
                    "Rejected non-increasing odometry stamp");
            return;
        }
        last_odom_stamp_ns_ = stamp_ns;

        const auto & linear = message->twist.twist.linear;
        const double speed = std::sqrt(
                linear.x * linear.x + linear.y * linear.y + linear.z * linear.z);
        if(!std::isfinite(speed)) {
            publishDiagnostic(
                    diagnostic_msgs::msg::DiagnosticStatus::WARN,
                    "Rejected non-finite odometry velocity");
            return;
        }
        if(speed > kMotionEpsilon) {
            has_seen_motion_ = true;
        }
        else if(has_seen_motion_) {
            scanning_stopped_ = true;
            RCLCPP_INFO(get_logger(), "trajectory complete, raw LaserScan acquisition stopped");
            return;
        }

        if(next_scan_stamp_ns_ != 0 && stamp_ns < next_scan_stamp_ns_) {
            return;
        }

        Pose3D pose;
        if(!poseFromOdom(*message, pose)) {
            publishDiagnostic(
                    diagnostic_msgs::msg::DiagnosticStatus::WARN,
                    "Rejected odometry outside the frozen map/body/fixed-attitude contract");
            return;
        }

        std::string diagnostic;
        const auto projected = projection_->project(lidar_->scanReturns(pose), &diagnostic);
        if(!projected.has_value()) {
            publishDiagnostic(
                    diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                    "Rejected complete raw scan: " + diagnostic);
            return;
        }

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = message->header.stamp;
        scan.header.frame_id = scan_frame_;
        scan.angle_min = projected->angle_min_rad;
        scan.angle_max = projected->angle_max_rad;
        scan.angle_increment = projected->angle_increment_rad;
        scan.time_increment = 0.0F;
        scan.scan_time = projected->scan_time_s;
        scan.range_min = projected->range_min_m;
        scan.range_max = projected->range_max_m;
        scan.ranges = projected->ranges;
        scan.intensities.clear();
        scan_publisher_->publish(scan);

        if(next_scan_stamp_ns_ == 0) {
            next_scan_stamp_ns_ = stamp_ns + scan_period_ns_;
        }
        else {
            do {
                next_scan_stamp_ns_ += scan_period_ns_;
            } while(next_scan_stamp_ns_ <= stamp_ns);
        }
    }

    void CaveLaserScanNode::publishDiagnostic(
            std::uint8_t level,
            const std::string & message)
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = get_clock()->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name() + std::string(": acquisition");
        status.hardware_id = "tree-cave-laser-scan";
        status.level = level;
        status.message = message;
        array.status.push_back(std::move(status));
        diagnostics_publisher_->publish(array);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", message.c_str());
    }

}// namespace DroneScanner
