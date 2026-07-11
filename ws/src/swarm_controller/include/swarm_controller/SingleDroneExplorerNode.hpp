#ifndef SWARM_CONTROLLER_SINGLEDRONEEXPLORERNODE_HPP
#define SWARM_CONTROLLER_SINGLEDRONEEXPLORERNODE_HPP

#include "swarm_controller/FrontierExplorationStrategy.hpp"
#include "swarm_controller/SingleDroneExplorer.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace SwarmController {

    class SingleDroneExplorerNode : public rclcpp::Node
    {
    public:
        SingleDroneExplorerNode();

    private:
        void declareParameters();
        FrontierExplorationConfig loadStrategyConfig() const;
        SingleDroneExplorerConfig loadExplorerConfig() const;
        void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
        void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
        void onControlTimer();
        bool makeExplorerInput(ExplorerInput & input);
        void publishMotionGoal(const MotionCommand & command);
        void publishMarkers(const rclcpp::Time & stamp, const Pose3D & map_pose);

        std::shared_ptr<FrontierExplorationStrategy> strategy_;
        std::unique_ptr<SingleDroneExplorer>         explorer_;
        std::shared_ptr<octomap::OcTree>             map_;
        nav_msgs::msg::Odometry                      latest_odom_;
        bool                                         has_odom_ {false};
        std::uint64_t                                observation_epoch_ {0U};
        std::int64_t                                 observation_stamp_ns_ {0};
        std::string                                  map_frame_ {"map"};
        double                                       control_rate_hz_ {5.0};
        std::chrono::steady_clock::time_point         steady_start_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_subscription_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_SINGLEDRONEEXPLORERNODE_HPP
