#ifndef DRONE_SCANNER_CAVELASERSCANNODE_HPP
#define DRONE_SCANNER_CAVELASERSCANNODE_HPP

#include "drone_scanner/FakeLidar.hpp"
#include "drone_scanner/LaserScanProjection.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace DroneScanner {

    class CaveLaserScanNode : public rclcpp::Node
    {
    public:
        CaveLaserScanNode();

    private:
        void onOdom(const nav_msgs::msg::Odometry::SharedPtr message);
        void publishDiagnostic(std::uint8_t level, const std::string & message);
        bool poseFromOdom(const nav_msgs::msg::Odometry & message, Pose3D & pose) const;

        std::shared_ptr<CaveWorld::ICaveField> field_;
        std::unique_ptr<FakeLidar>             lidar_;
        std::unique_ptr<LaserScanProjection>  projection_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                diagnostics_publisher_;

        std::string map_frame_;
        std::string base_frame_;
        std::string scan_frame_;
        std::int64_t scan_period_ns_ {100000000LL};
        std::int64_t next_scan_stamp_ns_ {0};
        std::int64_t last_odom_stamp_ns_ {0};
        bool has_seen_motion_ {false};
        bool scanning_stopped_ {false};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_CAVELASERSCANNODE_HPP
