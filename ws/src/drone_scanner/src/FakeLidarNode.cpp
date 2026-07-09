#include "drone_scanner/FakeLidarNode.hpp"

#include "drone_scanner/CaveFieldFromParameters.hpp"

#include <chrono>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>

namespace DroneScanner {

    FakeLidarNode::FakeLidarNode()
        : rclcpp::Node("fake_lidar")
    {
        declareParameters();
        map_frame_   = get_parameter("map_frame").as_string();
        lidar_frame_ = get_parameter("lidar_frame").as_string();
        scan_rate_hz_ = get_parameter("scan_rate").as_double();
        if(scan_rate_hz_ <= 0.0) {
            scan_rate_hz_ = 10.0;
        }

        initCaveField();
        initFakeLidar();
        initTf();
        initPublisher();
        stop_scan_when_trajectory_done_ = get_parameter("stop_scan_when_trajectory_done").as_bool();
        initOdomSubscription();
        initTimer();

        RCLCPP_INFO(
                get_logger(),
                "fake_lidar: %s -> %s, %.1f Hz, %ld beams, ring_pitch=%.3f rad, frame '%s'",
                map_frame_.c_str(), lidar_frame_.c_str(), scan_rate_hz_,
                static_cast<long>(get_parameter("num_beams").as_int()),
                get_parameter("ring_pitch_rad").as_double(), lidar_frame_.c_str());
    }

    void FakeLidarNode::declareParameters()
    {
        declareCaveFieldParameters(*this);

        declare_parameter("map_frame", "map");
        declare_parameter("lidar_frame", "lidar_link");
        declare_parameter("scan_rate", 10.0);
        declare_parameter("num_beams", 360);
        declare_parameter("max_range", 30.0);
        declare_parameter("range_noise_std", 0.0);
        declare_parameter("noise_seed", 0);
        declare_parameter("ring_pitch_rad", 0.0);
        declare_parameter("stop_scan_when_trajectory_done", true);
    }

    void FakeLidarNode::initCaveField()
    {
        field_ = createCaveFieldFromParameters(*this);
    }

    void FakeLidarNode::initFakeLidar()
    {
        FakeLidarConfig config;
        config.num_beams        = get_parameter("num_beams").as_int();
        config.max_range        = static_cast<float>(get_parameter("max_range").as_double());
        config.range_noise_std  = static_cast<float>(get_parameter("range_noise_std").as_double());
        config.noise_seed       = static_cast<std::uint32_t>(get_parameter("noise_seed").as_int());
        config.ring_pitch_rad   = static_cast<float>(get_parameter("ring_pitch_rad").as_double());
        fake_lidar_             = std::make_unique<FakeLidar>(field_, config);
    }

    void FakeLidarNode::initTf()
    {
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void FakeLidarNode::initPublisher()
    {
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
    }

    void FakeLidarNode::initOdomSubscription()
    {
        if(!stop_scan_when_trajectory_done_) {
            return;
        }

        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "odom", rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    onOdom(msg);
                });
    }

    void FakeLidarNode::initTimer()
    {
        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / scan_rate_hz_));
        timer_ = create_wall_timer(period, [this]() {
            onTimer();
        });
    }

    void FakeLidarNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(scanning_stopped_) {
            return;
        }

        const double linear_x = msg->twist.twist.linear.x;
        constexpr double kMotionEpsilon = 1e-6;
        if(std::abs(linear_x) > kMotionEpsilon) {
            has_seen_motion_ = true;
            return;
        }

        if(has_seen_motion_) {
            stopScanning();
        }
    }

    void FakeLidarNode::stopScanning()
    {
        if(scanning_stopped_) {
            return;
        }

        scanning_stopped_ = true;
        if(timer_) {
            timer_->cancel();
            timer_.reset();
        }
        RCLCPP_INFO(get_logger(), "fake_lidar: trajectory complete, scan stopped");
    }

    sensor_msgs::msg::PointCloud2 FakeLidarNode::makePointCloud(
            const rclcpp::Time & stamp, const std::string & frame_id, const std::vector<LidarPoint> & hits)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(hits.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        for(const LidarPoint & hit : hits) {
            *iter_x = hit.x;
            *iter_y = hit.y;
            *iter_z = hit.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        cloud.header.stamp    = stamp;
        cloud.header.frame_id = frame_id;
        cloud.is_dense        = true;
        return cloud;
    }

    void FakeLidarNode::onTimer()
    {
        if(scanning_stopped_) {
            return;
        }

        const rclcpp::Time now = get_clock()->now();
        geometry_msgs::msg::TransformStamped transform;
        try {
            transform = tf_buffer_->lookupTransform(map_frame_, lidar_frame_, tf2::TimePointZero);
        } catch(const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF %s -> %s: %s", map_frame_.c_str(),
                lidar_frame_.c_str(), ex.what());
            return;
        }

        Pose3D pose;
        pose.x   = static_cast<float>(transform.transform.translation.x);
        pose.y   = static_cast<float>(transform.transform.translation.y);
        pose.z   = static_cast<float>(transform.transform.translation.z);
        const auto & rot = transform.transform.rotation;
        // fake_odom 仅绕 Z 发布 yaw；与 yawToQuaternion 互逆
        pose.yaw = static_cast<float>(2.0 * std::atan2(rot.z, rot.w));

        const auto hits = fake_lidar_->scan(pose);
        publisher_->publish(makePointCloud(now, lidar_frame_, hits));
    }

}// namespace DroneScanner
