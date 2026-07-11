#include "swarm_controller/SingleDroneExplorerNode.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace SwarmController {

    namespace {

        geometry_msgs::msg::Point point(const Point3f & value)
        {
            geometry_msgs::msg::Point result;
            result.x = value.x;
            result.y = value.y;
            result.z = value.z;
            return result;
        }

        geometry_msgs::msg::Quaternion quaternion(float yaw)
        {
            geometry_msgs::msg::Quaternion result;
            result.z = std::sin(yaw * 0.5F);
            result.w = std::cos(yaw * 0.5F);
            return result;
        }

        visualization_msgs::msg::Marker marker(
                const std::string & frame, const rclcpp::Time & stamp,
                const std::string & name_space, int id, int type)
        {
            visualization_msgs::msg::Marker result;
            result.header.frame_id = frame;
            result.header.stamp    = stamp;
            result.ns              = name_space;
            result.id              = id;
            result.type            = type;
            result.action          = visualization_msgs::msg::Marker::ADD;
            result.pose.orientation.w = 1.0;
            return result;
        }

        tf2::TimePoint timePointFromStamp(const builtin_interfaces::msg::Time & stamp)
        {
            return tf2::TimePoint(
                    std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec));
        }

    }// namespace

    SingleDroneExplorerNode::SingleDroneExplorerNode()
        : Node("single_drone_explorer")
        , steady_start_(std::chrono::steady_clock::now())
    {
        declareParameters();
        map_frame_       = get_parameter("map_frame").as_string();
        control_rate_hz_ = get_parameter("control_rate").as_double();
        if(!std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0) {
            throw std::invalid_argument("control_rate must be positive and finite");
        }

        strategy_ = std::make_shared<FrontierExplorationStrategy>(loadStrategyConfig());
        explorer_ = std::make_unique<SingleDroneExplorer>(strategy_, loadExplorerConfig());
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "odom", rclcpp::QoS(10),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    onOdometry(msg);
                });
        map_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
                "octomap", rclcpp::QoS(1).reliable().transient_local(),
                [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                    onOctomap(msg);
                });
        goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "motion_goal", rclcpp::QoS(1).reliable().transient_local());
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                "exploration_markers", rclcpp::QoS(1));

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / control_rate_hz_));
        timer_ = create_wall_timer(period, [this]() {
            onControlTimer();
        });
        RCLCPP_INFO(
                get_logger(), "single_drone_explorer: odom + octomap -> motion_goal at %.1f Hz",
                control_rate_hz_);
    }

    void SingleDroneExplorerNode::declareParameters()
    {
        declare_parameter("map_frame", "map");
        declare_parameter("control_rate", 5.0);

        declare_parameter("goal.position_tolerance", 0.20);
        declare_parameter("goal.yaw_tolerance", 0.15);
        declare_parameter("motion.timeout", 20.0);
        declare_parameter("hold.timeout", 2.0);
        declare_parameter("hold.linear_speed_max", 0.02);
        declare_parameter("hold.angular_speed_max", 0.03);
        declare_parameter("map.stale_timeout", 2.0);
        declare_parameter("rescan.yaw_step", 0.78539816339);
        declare_parameter("rescan.max_steps", 8);
        declare_parameter("max_rejections_per_epoch", 16);
        declare_parameter("entry.enforce_forward_half_space", true);
        declare_parameter("entry.backward_margin", 0.10);
        declare_parameter("max_debug_faces", 2000);
        declare_parameter("max_debug_candidates", 500);

        declare_parameter("body.robot_radius", 0.25);
        declare_parameter("body.robot_half_height", 0.15);
        declare_parameter("body.safety_margin", 0.25);
        declare_parameter("body.vertical_margin", 0.20);
        declare_parameter("body.sample_spacing_fraction", 0.50);

        declare_parameter("frontier.planning_radius", 5.5);
        declare_parameter("frontier.min_goal_distance", 0.8);
        declare_parameter("frontier.max_goal_distance", 4.0);
        declare_parameter("frontier.goal_standoff", 0.6);
        declare_parameter("frontier.goal_search_radius", 0.4);
        declare_parameter("frontier.min_cluster_area", 0.20);
        declare_parameter("frontier.max_abs_normal_z", 0.60);
    }

    FrontierExplorationConfig SingleDroneExplorerNode::loadStrategyConfig() const
    {
        FrontierExplorationConfig config;
        config.planning_radius =
                static_cast<float>(get_parameter("frontier.planning_radius").as_double());
        config.min_goal_distance =
                static_cast<float>(get_parameter("frontier.min_goal_distance").as_double());
        config.max_goal_distance =
                static_cast<float>(get_parameter("frontier.max_goal_distance").as_double());
        config.goal_standoff =
                static_cast<float>(get_parameter("frontier.goal_standoff").as_double());
        config.goal_search_radius =
                static_cast<float>(get_parameter("frontier.goal_search_radius").as_double());
        config.min_cluster_area =
                static_cast<float>(get_parameter("frontier.min_cluster_area").as_double());
        config.max_abs_frontier_normal_z =
                static_cast<float>(get_parameter("frontier.max_abs_normal_z").as_double());
        config.robot_radius =
                static_cast<float>(get_parameter("body.robot_radius").as_double());
        config.robot_half_height =
                static_cast<float>(get_parameter("body.robot_half_height").as_double());
        config.safety_margin =
                static_cast<float>(get_parameter("body.safety_margin").as_double());
        config.vertical_margin =
                static_cast<float>(get_parameter("body.vertical_margin").as_double());
        return config;
    }

    SingleDroneExplorerConfig SingleDroneExplorerNode::loadExplorerConfig() const
    {
        SingleDroneExplorerConfig config;
        config.position_tolerance =
                static_cast<float>(get_parameter("goal.position_tolerance").as_double());
        config.yaw_tolerance =
                static_cast<float>(get_parameter("goal.yaw_tolerance").as_double());
        config.motion_timeout_seconds = get_parameter("motion.timeout").as_double();
        config.hold_timeout_seconds   = get_parameter("hold.timeout").as_double();
        config.stopped_linear_speed_max =
                static_cast<float>(get_parameter("hold.linear_speed_max").as_double());
        config.stopped_angular_speed_max =
                static_cast<float>(get_parameter("hold.angular_speed_max").as_double());
        config.map_stale_timeout_seconds = get_parameter("map.stale_timeout").as_double();
        config.rescan_yaw_step =
                static_cast<float>(get_parameter("rescan.yaw_step").as_double());
        config.rescan_max_steps =
                static_cast<std::size_t>(get_parameter("rescan.max_steps").as_int());
        config.max_rejections_per_epoch =
                static_cast<std::size_t>(get_parameter("max_rejections_per_epoch").as_int());
        config.enforce_entry_forward_half_space =
                get_parameter("entry.enforce_forward_half_space").as_bool();
        config.entry_backward_margin =
                static_cast<float>(get_parameter("entry.backward_margin").as_double());
        config.max_debug_faces =
                static_cast<std::size_t>(get_parameter("max_debug_faces").as_int());
        config.max_debug_candidates =
                static_cast<std::size_t>(get_parameter("max_debug_candidates").as_int());
        config.body_envelope.robot_radius =
                static_cast<float>(get_parameter("body.robot_radius").as_double());
        config.body_envelope.robot_half_height =
                static_cast<float>(get_parameter("body.robot_half_height").as_double());
        config.body_envelope.safety_margin =
                static_cast<float>(get_parameter("body.safety_margin").as_double());
        config.body_envelope.vertical_margin =
                static_cast<float>(get_parameter("body.vertical_margin").as_double());
        config.body_envelope.sample_spacing_fraction = static_cast<float>(
                get_parameter("body.sample_spacing_fraction").as_double());
        return config;
    }

    void SingleDroneExplorerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_ = *msg;
        has_odom_    = true;
    }

    void SingleDroneExplorerNode::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
    {
        if(msg->header.frame_id != map_frame_) {
            RCLCPP_WARN(
                    get_logger(), "ignored Octomap frame '%s', expected '%s'",
                    msg->header.frame_id.c_str(), map_frame_.c_str());
            return;
        }
        const std::int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        if(stamp <= observation_stamp_ns_) {
            return;
        }
        std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::fullMsgToMap(*msg));
        octomap::OcTree * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
        if(tree == nullptr) {
            RCLCPP_WARN(get_logger(), "ignored Octomap with unsupported tree type '%s'", msg->id.c_str());
            return;
        }
        abstract.release();
        map_.reset(tree);
        observation_stamp_ns_ = stamp;
        ++observation_epoch_;
    }

    bool SingleDroneExplorerNode::makeExplorerInput(ExplorerInput & input)
    {
        if(!has_odom_) {
            return false;
        }

        geometry_msgs::msg::PoseStamped odom_pose;
        odom_pose.header = latest_odom_.header;
        odom_pose.pose   = latest_odom_.pose.pose;
        geometry_msgs::msg::PoseStamped map_pose;
        try {
            const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    map_frame_, latest_odom_.header.frame_id,
                    timePointFromStamp(latest_odom_.header.stamp),
                    tf2::durationFromSec(0.05));
            tf2::doTransform(odom_pose, map_pose, transform);
        } catch(const tf2::TransformException & error) {
            RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000, "explorer TF: %s", error.what());
            return false;
        }

        const auto & q = map_pose.pose.orientation;
        const float yaw = static_cast<float>(
                std::atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        const float body_x = static_cast<float>(latest_odom_.twist.twist.linear.x);
        const float body_y = static_cast<float>(latest_odom_.twist.twist.linear.y);
        input.pose.position = Point3f {
                static_cast<float>(map_pose.pose.position.x),
                static_cast<float>(map_pose.pose.position.y),
                static_cast<float>(map_pose.pose.position.z),
        };
        input.pose.yaw = yaw;
        input.linear_velocity = Point3f {
                std::cos(yaw) * body_x - std::sin(yaw) * body_y,
                std::sin(yaw) * body_x + std::cos(yaw) * body_y,
                static_cast<float>(latest_odom_.twist.twist.linear.z),
        };
        input.angular_velocity_z =
                static_cast<float>(latest_odom_.twist.twist.angular.z);
        input.map                   = map_.get();
        input.observation_epoch     = observation_epoch_;
        input.observation_stamp_ns  = observation_stamp_ns_;
        input.odom_stamp_ns         = rclcpp::Time(latest_odom_.header.stamp).nanoseconds();
        input.monotonic_time_seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - steady_start_)
                                                   .count();
        return true;
    }

    void SingleDroneExplorerNode::onControlTimer()
    {
        ExplorerInput input;
        if(!makeExplorerInput(input)) {
            timer_->reset();
            return;
        }
        const ExplorerTickResult result = explorer_->tick(input);
        if(result.command.type != MotionCommandType::None) {
            publishMotionGoal(result.command);
        }
        publishMarkers(get_clock()->now(), input.pose);
        // 规划可能超过一个控制周期；从本次完成时重新计时，避免积压 timer
        // 在订阅回调处理 fresh map 前连续补跑并误判地图 stale。
        timer_->reset();
    }

    void SingleDroneExplorerNode::publishMotionGoal(const MotionCommand & command)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp    = get_clock()->now();
        goal.header.frame_id = map_frame_;
        goal.pose.position.x = command.goal.position.x;
        goal.pose.position.y = command.goal.position.y;
        goal.pose.position.z = command.goal.position.z;
        goal.pose.orientation = quaternion(command.goal.yaw);
        goal_publisher_->publish(goal);
    }

    void SingleDroneExplorerNode::publishMarkers(
            const rclcpp::Time & stamp, const Pose3D & map_pose)
    {
        const ExplorationDiagnostics & diagnostics = explorer_->diagnostics();
        visualization_msgs::msg::MarkerArray array;

        auto clear = marker(
                map_frame_, stamp, "clear", 0,
                visualization_msgs::msg::Marker::SPHERE_LIST);
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        array.markers.push_back(clear);

        auto frontier = marker(
                map_frame_, stamp, "frontier_faces", 0,
                visualization_msgs::msg::Marker::SPHERE_LIST);
        frontier.scale.x = frontier.scale.y = frontier.scale.z = 0.08;
        frontier.color.r = 1.0F;
        frontier.color.g = 0.55F;
        frontier.color.a = 0.75F;
        for(const DebugFrontierFace & face : diagnostics.frontier_faces) {
            frontier.points.push_back(point(face.position));
        }
        array.markers.push_back(frontier);

        auto clusters = marker(
                map_frame_, stamp, "frontier_clusters", 0,
                visualization_msgs::msg::Marker::SPHERE_LIST);
        clusters.scale.x = clusters.scale.y = clusters.scale.z = 0.18;
        for(const DebugFrontierCluster & cluster : diagnostics.frontier_clusters) {
            clusters.points.push_back(point(cluster.position));
            auto color = clusters.color;
            color.r = 1.0F;
            color.g = cluster.rejected ? 0.0F : 0.85F;
            color.b = 0.0F;
            color.a = 0.9F;
            clusters.colors.push_back(color);
        }
        array.markers.push_back(clusters);

        auto candidates = marker(
                map_frame_, stamp, "safe_candidates", 0,
                visualization_msgs::msg::Marker::SPHERE_LIST);
        candidates.scale.x = candidates.scale.y = candidates.scale.z = 0.12;
        candidates.color.g = 0.9F;
        candidates.color.b = 1.0F;
        candidates.color.a = 0.8F;
        for(const Point3f & candidate : diagnostics.locally_safe_candidates) {
            candidates.points.push_back(point(candidate));
        }
        array.markers.push_back(candidates);

        if(diagnostics.path_start.has_value() && diagnostics.path_goal.has_value()) {
            auto path = marker(
                    map_frame_, stamp, "checked_path", 0,
                    visualization_msgs::msg::Marker::LINE_STRIP);
            path.scale.x = 0.06;
            path.color.a = 1.0F;
            if(diagnostics.path_status == "Safe") {
                path.color.g = 1.0F;
            } else {
                path.color.r = 1.0F;
            }
            path.points.push_back(point(*diagnostics.path_start));
            path.points.push_back(point(*diagnostics.path_goal));
            array.markers.push_back(path);
        }

        if(diagnostics.selected_goal.has_value()) {
            auto selected = marker(
                    map_frame_, stamp, "selected_goal", 0,
                    visualization_msgs::msg::Marker::SPHERE);
            selected.pose.position = point(*diagnostics.selected_goal);
            selected.scale.x = selected.scale.y = selected.scale.z = 0.28;
            selected.color.g = 1.0F;
            selected.color.a = 1.0F;
            array.markers.push_back(selected);
        }

        if(diagnostics.first_blocked_position.has_value()) {
            auto blocked = marker(
                    map_frame_, stamp, "first_blocked", 0,
                    visualization_msgs::msg::Marker::CUBE);
            blocked.pose.position = point(*diagnostics.first_blocked_position);
            blocked.scale.x = blocked.scale.y = blocked.scale.z = 0.20;
            blocked.color.r = 1.0F;
            blocked.color.a = 1.0F;
            array.markers.push_back(blocked);
        }

        auto text = marker(
                map_frame_, stamp, "state", 0,
                visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        text.pose.position.x = map_pose.position.x;
        text.pose.position.y = map_pose.position.y;
        text.pose.position.z = map_pose.position.z + 0.6;
        text.scale.z = 0.25;
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
        text.text = diagnostics.controller_state;
        if(diagnostics.raw_candidate_count > 0U
           || diagnostics.unique_candidate_count > 0U)
        {
            std::ostringstream details;
            details << "\nraw/unique=" << diagnostics.raw_candidate_count << '/'
                    << diagnostics.unique_candidate_count
                    << " forward_filtered=" << diagnostics.forward_filtered_count
                    << " segment_checks=" << diagnostics.segment_check_count
                    << " select=" << std::fixed << std::setprecision(3)
                    << diagnostics.selection_elapsed_seconds << 's';
            text.text += details.str();
        }
        if(!diagnostics.failure_reason.empty()) {
            text.text += ": " + diagnostics.failure_reason;
        }
        array.markers.push_back(text);
        marker_publisher_->publish(array);
    }

}// namespace SwarmController
