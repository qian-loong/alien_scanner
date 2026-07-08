#ifndef DRONE_SCANNER_FAKEODOMNODE_HPP
#define DRONE_SCANNER_FAKEODOMNODE_HPP

#include "drone_scanner/LineTrajectory.hpp"

#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace DroneScanner {

class FakeOdomNode : public rclcpp::Node
{
public:
    FakeOdomNode();

private:
    void declareParameters();
    LineTrajectoryConfig loadLineConfigFromParameters() const;
    void initTrajectory();
    void initPublisher();
    void initTimer();
    void onTimer();

    static geometry_msgs::msg::Quaternion yawToQuaternion(float yaw);
    static geometry_msgs::msg::TransformStamped makeTransform(
            const rclcpp::Time & stamp, const std::string & odom_frame, const std::string & base_frame,
            const Pose3D & pose);

    std::unique_ptr<LineTrajectory>                              trajectory_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                 timer_;
    rclcpp::Time                                                 start_time_;
    std::string                                                  odom_frame_;
    std::string                                                  base_frame_;
    double                                                       publish_rate_hz_ {20.0};
};

}// namespace DroneScanner

#endif// DRONE_SCANNER_FAKEODOMNODE_HPP
