#include "drone_scanner/FakeOdomNode.hpp"

#include <cmath>

namespace DroneScanner {

FakeOdomNode::FakeOdomNode()
    : rclcpp::Node("fake_odom")
{
    declareParameters();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_rate_hz_ = get_parameter("publish_rate").as_double();
    if(publish_rate_hz_ <= 0.0) {
        publish_rate_hz_ = 20.0;
    }

    initTrajectory();
    initPublisher();
    start_time_ = get_clock()->now();
    initTimer();

    RCLCPP_INFO(
            get_logger(),
            "fake_odom: line (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) in %.1fs, %.2f m/s",
            get_parameter("line.start_x").as_double(), get_parameter("line.start_y").as_double(),
            get_parameter("line.start_z").as_double(), get_parameter("line.end_x").as_double(),
            get_parameter("line.end_y").as_double(), get_parameter("line.end_z").as_double(),
            get_parameter("line.duration_seconds").as_double(), trajectory_->speed());
}

void FakeOdomNode::declareParameters()
{
    declare_parameter("line.start_x", 0.0);
    declare_parameter("line.start_y", 0.0);
    declare_parameter("line.start_z", 1.5);
    declare_parameter("line.end_x", 11.0);
    declare_parameter("line.end_y", 0.0);
    declare_parameter("line.end_z", 1.5);
    declare_parameter("line.duration_seconds", 60.0);

    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("publish_rate", 20.0);
}

LineTrajectoryConfig FakeOdomNode::loadLineConfigFromParameters() const
{
    LineTrajectoryConfig config;
    config.start_x          = static_cast<float>(get_parameter("line.start_x").as_double());
    config.start_y          = static_cast<float>(get_parameter("line.start_y").as_double());
    config.start_z          = static_cast<float>(get_parameter("line.start_z").as_double());
    config.end_x            = static_cast<float>(get_parameter("line.end_x").as_double());
    config.end_y            = static_cast<float>(get_parameter("line.end_y").as_double());
    config.end_z            = static_cast<float>(get_parameter("line.end_z").as_double());
    config.duration_seconds = get_parameter("line.duration_seconds").as_double();
    return config;
}

void FakeOdomNode::initTrajectory()
{
    trajectory_ = std::make_unique<LineTrajectory>(loadLineConfigFromParameters());
}

void FakeOdomNode::initPublisher()
{
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}

void FakeOdomNode::initTimer()
{
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {
        onTimer();
    });
}

geometry_msgs::msg::Quaternion FakeOdomNode::yawToQuaternion(float yaw)
{
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5F);
    q.w = std::cos(yaw * 0.5F);
    return q;
}

geometry_msgs::msg::TransformStamped FakeOdomNode::makeTransform(
        const rclcpp::Time & stamp, const std::string & odom_frame, const std::string & base_frame,
        const Pose3D & pose)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp    = stamp;
    transform.header.frame_id = odom_frame;
    transform.child_frame_id  = base_frame;
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.translation.z = pose.z;
    transform.transform.rotation      = yawToQuaternion(pose.yaw);
    return transform;
}

void FakeOdomNode::onTimer()
{
    const rclcpp::Time now = get_clock()->now();
    const double       t   = (now - start_time_).seconds();
    const Pose3D       pose = trajectory_->pose(t);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = pose.z;
    odom.pose.pose.orientation = yawToQuaternion(pose.yaw);

    if(t < trajectory_->duration()) {
        // twist 在 child_frame（base_link）下：前进速度沿 +x（REP / nav_msgs 惯例）
        odom.twist.twist.linear.x = trajectory_->speed();
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
    }

    odom_publisher_->publish(odom);
    tf_broadcaster_->sendTransform(makeTransform(now, odom_frame_, base_frame_, pose));
}

}// namespace DroneScanner
