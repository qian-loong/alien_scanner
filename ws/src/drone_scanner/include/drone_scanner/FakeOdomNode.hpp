#ifndef DRONE_SCANNER_FAKEODOMNODE_HPP
#define DRONE_SCANNER_FAKEODOMNODE_HPP

#include "drone_scanner/AltitudeAdapter.hpp"
#include "drone_scanner/LineTrajectory.hpp"
#include "drone_scanner/PoseSegmentTrajectory.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
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
    void initGoalSubscription();
    void initTimer();
    void onTimer();
    void onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void onMotionGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void appendAndPublishPath(const rclcpp::Time & stamp, const Pose3D & pose);

    static geometry_msgs::msg::Quaternion yawToQuaternion(float yaw);
    static geometry_msgs::msg::TransformStamped makeTransform(
            const rclcpp::Time & stamp, const std::string & odom_frame, const std::string & base_frame,
            const Pose3D & pose);
    static std::vector<LidarPoint> pointCloudToLidarPoints(const sensor_msgs::msg::PointCloud2 & cloud);
    static float quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion);
    static float shortestYawDistance(float lhs, float rhs);

    std::unique_ptr<LineTrajectory>                                    trajectory_;
    std::unique_ptr<PoseSegmentTrajectory>                             goal_trajectory_;
    std::unique_ptr<AltitudeAdapter>                                   altitude_adapter_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              odom_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  path_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr     points_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   goal_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster>                     tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr                                       timer_;
    rclcpp::Time                                                       start_time_;
    std::string                                                        odom_frame_;
    std::string                                                        base_frame_;
    double                                                             publish_rate_hz_ {20.0};
    double                                                             path_min_distance_ {0.05};
    std::size_t                                                        path_max_poses_ {5000U};
    nav_msgs::msg::Path                                                flown_path_;
    bool                                                               goal_mode_ {false};
    std::string                                                        goal_topic_ {"motion_goal"};
    std::string                                                        goal_frame_ {"map"};
    float                                                              goal_linear_speed_ {0.4F};
    float                                                              goal_yaw_rate_ {0.5F};
    float                                                              goal_epsilon_ {1.0e-4F};
    bool                                                               altitude_adapt_enable_ {true};
    /// goal 模式：首段 XY 平移开始前冻结高度，避免入口悬停时错误下坠。
    bool                                                               altitude_adapt_unlocked_ {false};
    double                                                             points_stale_sec_ {0.5};
    float                                                              adapted_z_ {1.5F};
    bool                                                               has_adapted_z_ {false};
    float                                                              last_pose_z_ {1.5F};
    rclcpp::Time                                                       last_adapt_time_;
    bool                                                               has_last_adapt_time_ {false};
    AltitudeBand                                                       last_band_ {};

    std::mutex motion_mutex_;
    rclcpp::Time segment_start_time_;
    Pose3D       current_pose_ {};
    Pose3D       accepted_goal_ {};
    bool         has_current_pose_ {false};
    bool         has_accepted_goal_ {false};

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
