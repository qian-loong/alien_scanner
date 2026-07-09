#ifndef DRONE_SCANNER_FAKEODOMNODE_HPP
#define DRONE_SCANNER_FAKEODOMNODE_HPP

#include "drone_scanner/AltitudeAdapter.hpp"
#include "drone_scanner/LineTrajectory.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace DroneScanner {

class FakeOdomNode : public rclcpp::Node
{
public:
    FakeOdomNode();

private:
    void declareParameters();
    LineTrajectoryConfig loadLineConfigFromParameters() const;
    AltitudeAdaptConfig loadAltitudeConfigFromParameters() const;
    void initTrajectory();
    void initAltitudeAdapter();
    void initPublisher();
    void initScanSubscription();
    void initTimer();
    void onTimer();
    void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    static geometry_msgs::msg::Quaternion yawToQuaternion(float yaw);
    static geometry_msgs::msg::TransformStamped makeTransform(
            const rclcpp::Time & stamp, const std::string & odom_frame, const std::string & base_frame,
            const Pose3D & pose);
    static std::vector<LidarPoint> pointCloudToLidarPoints(const sensor_msgs::msg::PointCloud2 & cloud);

    std::unique_ptr<LineTrajectory>                                    trajectory_;
    std::unique_ptr<AltitudeAdapter>                                   altitude_adapter_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              odom_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr     points_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
    rclcpp::Time                                                       start_time_;
    std::string                                                        odom_frame_;
    std::string                                                        base_frame_;
    double                                                             publish_rate_hz_ {20.0};
    bool                                                               altitude_adapt_enable_ {true};
    double                                                             points_stale_sec_ {0.5};
    float                                                              adapted_z_ {1.5F};
    bool                                                               has_adapted_z_ {false};
    float                                                              last_pose_z_ {1.5F};
    rclcpp::Time                                                       last_adapt_time_;
    bool                                                               has_last_adapt_time_ {false};
    AltitudeBand                                                       last_band_ {};

    std::mutex              hits_mutex_;
    std::vector<LidarPoint> pending_hits_;
    float                   pending_scan_origin_z_ {0.0F};
    rclcpp::Time            pending_stamp_;
    bool                    has_pending_scan_ {false};

    static constexpr std::size_t kPoseHistorySize = 64;
    struct PoseZSample {
        rclcpp::Time stamp;
        float        z {0.0F};
    };
    mutable std::mutex         pose_history_mutex_;
    std::vector<PoseZSample>   pose_history_;

    float lookupPoseZNear(const rclcpp::Time & stamp) const;
};

}// namespace DroneScanner

#endif// DRONE_SCANNER_FAKEODOMNODE_HPP
