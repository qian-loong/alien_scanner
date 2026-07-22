#ifndef SWARM_CONTROLLER_SINGLEDRONEEXPLORERNODE_HPP
#define SWARM_CONTROLLER_SINGLEDRONEEXPLORERNODE_HPP

#include "swarm_controller/FrontierExplorationStrategy.hpp"
#include "swarm_controller/PeerStateTracker.hpp"
#include "swarm_controller/PlanningSnapshotGuard.hpp"
#include "swarm_controller/SingleDroneExplorer.hpp"
#include "swarm_controller/TaskLeaseTracker.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <swarm_controller_interfaces/msg/exploration_task.hpp>

namespace SwarmController {

    class SingleDroneExplorerNode : public rclcpp::Node
    {
    public:
        SingleDroneExplorerNode();

    private:
        struct PeerTrack {
            std::string ns;
            nav_msgs::msg::Odometry odom;
            double                  odom_receive_time {-1.0};
            double                  odom_applied_time {-1.0};
            bool                    has_odom {false};
            geometry_msgs::msg::PoseStamped goal;
            double                  goal_receive_time {-1.0};
            double                  goal_applied_time {-1.0};
            bool                    has_goal {false};
        };

        void declareParameters();
        FrontierExplorationConfig loadStrategyConfig() const;
        SingleDroneExplorerConfig loadExplorerConfig() const;
        PlanningSnapshotConfig loadPlanningSnapshotConfig() const;
        void setupPeerSubscriptions(const std::vector<std::string> & peer_namespaces);
        void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
        void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
        void onPeerOdometry(std::size_t peer_index, const nav_msgs::msg::Odometry::SharedPtr msg);
        void onPeerMotionGoal(
                std::size_t peer_index, const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void onExplorationTask(
                const swarm_controller_interfaces::msg::ExplorationTask::SharedPtr msg);
        void onControlTimer();
        bool makeExplorerInput(
                ExplorerInput & input, std::shared_ptr<octomap::OcTree> & map_snapshot,
                double now_seconds);
        static PlanningSnapshot planningSnapshot(const ExplorerInput & input);
        void refreshPeerState(double now_seconds);
        double monotonicNow() const;
        bool transformPoseToMap(
                const std::string & source_frame, const builtin_interfaces::msg::Time & stamp,
                const geometry_msgs::msg::Pose & pose, Point3f & map_position, float * map_yaw);
        void publishMotionGoal(const MotionCommand & command);
        void publishMarkers(
                const rclcpp::Time & stamp, const Pose3D & map_pose,
                const PeerDispersionSnapshot & peers);
        void publishDiagnostics(const rclcpp::Time & stamp, bool input_ready);

        std::shared_ptr<FrontierExplorationStrategy> strategy_;
        std::unique_ptr<SingleDroneExplorer>         explorer_;
        PlanningSnapshotGuard                        planning_guard_;
        std::mutex                                   input_mutex_;
        std::shared_ptr<octomap::OcTree>             map_;
        nav_msgs::msg::Odometry                      latest_odom_;
        bool                                         has_odom_ {false};
        std::uint64_t                                observation_epoch_ {0U};
        std::int64_t                                 observation_stamp_ns_ {0};
        std::string                                  map_frame_ {"map"};
        double                                       control_rate_hz_ {5.0};
        double                                       peer_position_timeout_seconds_ {2.0};
        double                                       peer_goal_timeout_seconds_ {25.0};
        double                                       configured_altitude_clearance_ {0.41};
        BodyEnvelopeConfig                           body_envelope_config_ {};
        std::chrono::steady_clock::time_point         steady_start_;

        std::mutex                peer_mutex_;
        std::vector<PeerTrack>    peers_;
        std::unique_ptr<PeerStateTracker> peer_tracker_;
        std::unique_ptr<TaskLeaseTracker> task_tracker_;
        std::mutex                        task_mutex_;
        TaskGuidance                      task_guidance_ {};
        std::string                       task_update_status_ {"NoTask"};
        PeerDispersionSnapshot          peer_snapshot_;
        std::size_t                     tf_pending_count_ {0U};
        std::size_t                     tf_rejected_count_ {0U};
        std::size_t                     planning_revalidation_count_ {0U};
        std::size_t                     planning_discard_count_ {0U};
        double                          planning_snapshot_age_seconds_ {};
        bool                            planning_snapshot_changed_ {false};
        float                           required_vertical_clearance_ {};
        bool                            clearance_contract_valid_ {true};
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> peer_odom_subs_;
        std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
                peer_goal_subs_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_subscription_;
        rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr map_subscription_;
        rclcpp::Subscription<swarm_controller_interfaces::msg::ExplorationTask>::SharedPtr
                task_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
                diagnostics_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;
        rclcpp::CallbackGroup::SharedPtr control_callback_group_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_SINGLEDRONEEXPLORERNODE_HPP
