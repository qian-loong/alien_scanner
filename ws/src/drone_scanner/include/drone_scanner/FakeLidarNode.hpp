#ifndef DRONE_SCANNER_FAKELIDARNODE_HPP
#define DRONE_SCANNER_FAKELIDARNODE_HPP

#include "drone_scanner/FakeLidar.hpp"

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace DroneScanner {

    class FakeLidarNode : public rclcpp::Node
    {
    public:
        FakeLidarNode();

    private:
        void declareParameters();
        void initCaveField();
        void initFakeLidar();
        void initPublisher();
        void initTf();
        void initOdomSubscription();
        void initTimer();
        void onTimer();
        void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
        void stopScanning();

        static sensor_msgs::msg::PointCloud2 makePointCloud(
                const rclcpp::Time & stamp, const std::string & frame_id,
                const std::vector<LidarPoint> & hits);

        std::shared_ptr<CaveWorld::ICaveField>                       field_;
        std::unique_ptr<FakeLidar>                                   fake_lidar_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr          publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_subscription_;
        std::shared_ptr<tf2_ros::Buffer>                                     tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener>                          tf_listener_;
        rclcpp::TimerBase::SharedPtr                                         timer_;
        std::string                                                          map_frame_;
        std::string                                                          lidar_frame_;
        double                                                               scan_rate_hz_ {10.0};
        bool                                                                 stop_scan_when_trajectory_done_ {true};
        bool                                                                 has_seen_motion_ {false};
        bool                                                                 scanning_stopped_ {false};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_FAKELIDARNODE_HPP
