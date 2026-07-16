#include "swarm_controller/SingleDroneExplorerNode.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

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

        const FrontierExplorationConfig strategy_config = loadStrategyConfig();
        const SingleDroneExplorerConfig explorer_config = loadExplorerConfig();
        body_envelope_config_ = explorer_config.body_envelope;
        planning_guard_ = PlanningSnapshotGuard(loadPlanningSnapshotConfig());
        configured_altitude_clearance_ =
                get_parameter("safety.altitude_min_clearance").as_double();
        if(!std::isfinite(configured_altitude_clearance_)
           || configured_altitude_clearance_ <= 0.0)
        {
            throw std::invalid_argument(
                    "safety.altitude_min_clearance must be positive and finite");
        }
        const auto peer_namespaces = get_parameter("peer_namespaces").as_string_array();
        const bool has_peer_namespace = std::any_of(
                peer_namespaces.begin(), peer_namespaces.end(),
                [](const std::string & value) { return !value.empty(); });
        peer_position_timeout_seconds_ = get_parameter("peer.position_timeout").as_double();
        peer_goal_timeout_seconds_     = get_parameter("peer.goal_timeout").as_double();
        if(!std::isfinite(peer_position_timeout_seconds_)
           || peer_position_timeout_seconds_ <= 0.0
           || !std::isfinite(peer_goal_timeout_seconds_)
           || (has_peer_namespace
               && peer_goal_timeout_seconds_ < explorer_config.motion_timeout_seconds))
        {
            throw std::invalid_argument(
                    "peer timeouts must be finite; position > 0 and goal >= motion.timeout");
        }
        strategy_ = std::make_shared<FrontierExplorationStrategy>(strategy_config);
        explorer_ = std::make_unique<SingleDroneExplorer>(strategy_, explorer_config);
        task_tracker_ = std::make_unique<TaskLeaseTracker>(TaskLeaseTrackerConfig {
                get_parameter("task.receive_watchdog").as_double()});
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        sensor_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        control_callback_group_ =
                create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        if(has_peer_namespace
           && strategy_config.min_peer_goal_separation
                       > 2.0F * strategy_config.forward_lookahead_max)
        {
            RCLCPP_WARN(
                    get_logger(),
                    "frontier.min_peer_goal_separation %.2f exceeds twice forward_lookahead_max "
                    "%.2f; nearby peers may have no simultaneously feasible local goals",
                    strategy_config.min_peer_goal_separation,
                    strategy_config.forward_lookahead_max);
        }
        setupPeerSubscriptions(peer_namespaces);
        peer_tracker_ = std::make_unique<PeerStateTracker>(
                peers_.size(),
                PeerStateConfig {
                        peer_position_timeout_seconds_,
                        peer_goal_timeout_seconds_,
                        explorer_config.position_tolerance,
                });

        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_callback_group_;
        odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                "odom", rclcpp::QoS(10),
                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    onOdometry(msg);
                }, sensor_options);
        map_subscription_ = create_subscription<octomap_msgs::msg::Octomap>(
                "octomap", rclcpp::QoS(1).reliable().transient_local(),
                [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
                    onOctomap(msg);
                }, sensor_options);
        task_subscription_ = create_subscription<
                swarm_controller_interfaces::msg::ExplorationTask>(
                "exploration_task", rclcpp::QoS(1).reliable().transient_local(),
                [this](const swarm_controller_interfaces::msg::ExplorationTask::SharedPtr msg) {
                    onExplorationTask(msg);
                }, sensor_options);
        goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
                "motion_goal", rclcpp::QoS(1).reliable().transient_local());
        marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                "exploration_markers", rclcpp::QoS(1));
        diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "exploration_diagnostics", rclcpp::QoS(1).reliable().transient_local());

        const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / control_rate_hz_));
        timer_ = create_wall_timer(
                period, [this]() { onControlTimer(); }, control_callback_group_);
        RCLCPP_INFO(
                get_logger(),
                "single_drone_explorer: odom + octomap -> motion_goal at %.1f Hz (%zu peers)",
                control_rate_hz_, peers_.size());
    }

    void SingleDroneExplorerNode::declareParameters()
    {
        declare_parameter("map_frame", "map");
        declare_parameter("control_rate", 5.0);
        declare_parameter("peer_namespaces", std::vector<std::string> {});

        declare_parameter("goal.position_tolerance", 0.20);
        declare_parameter("goal.yaw_tolerance", 0.15);
        declare_parameter("motion.timeout", 20.0);
        declare_parameter("hold.timeout", 2.0);
        declare_parameter("hold.linear_speed_max", 0.02);
        declare_parameter("hold.angular_speed_max", 0.03);
        declare_parameter("map.stale_timeout", 2.0);
        declare_parameter("peer.position_timeout", 2.0);
        declare_parameter("peer.goal_timeout", 25.0);
        declare_parameter("peer.retry_interval", 1.0);
        declare_parameter("task.receive_watchdog", 3.5);
        declare_parameter("task.retry_interval", 1.0);
        declare_parameter("task.rescan_max_steps", 4);
        declare_parameter("task.rescan_step_timeout", 2.5);
        declare_parameter("motion.travel_heading_update_distance", 0.35);
        declare_parameter("recovery.timeout", 8.0);
        declare_parameter("planning.max_snapshot_age", 1.0);
        declare_parameter("planning.max_position_drift", 0.10);
        declare_parameter("planning.max_yaw_drift", 0.10);
        declare_parameter("safety.altitude_min_clearance", 0.41);
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

        declare_parameter("frontier.forward_lookahead_min", 0.8);
        declare_parameter("frontier.forward_lookahead_max", 2.0);
        declare_parameter("frontier.forward_lateral_limit", 0.5);
        declare_parameter("frontier.forward_distance_samples", 4);
        declare_parameter("frontier.forward_lateral_samples", 5);
        declare_parameter("frontier.lateral_weight", 0.60);
        declare_parameter("frontier.heading_weight", 0.25);
        declare_parameter("frontier.dispersion_weight", 0.35);
        declare_parameter("frontier.task_progress_weight", 1.0);
        declare_parameter("task.min_progress", 0.15);
        declare_parameter("task.max_heading_error", 1.05);
        declare_parameter("frontier.min_peer_goal_separation", 0.8);
    }

    FrontierExplorationConfig SingleDroneExplorerNode::loadStrategyConfig() const
    {
        FrontierExplorationConfig config;
        const auto distance_samples =
                get_parameter("frontier.forward_distance_samples").as_int();
        const auto lateral_samples =
                get_parameter("frontier.forward_lateral_samples").as_int();
        if(distance_samples <= 0 || lateral_samples <= 0) {
            throw std::invalid_argument("frontier forward sample counts must be positive");
        }
        config.forward_lookahead_min = static_cast<float>(
                get_parameter("frontier.forward_lookahead_min").as_double());
        config.forward_lookahead_max = static_cast<float>(
                get_parameter("frontier.forward_lookahead_max").as_double());
        config.forward_lateral_limit = static_cast<float>(
                get_parameter("frontier.forward_lateral_limit").as_double());
        config.forward_distance_samples = static_cast<std::size_t>(distance_samples);
        config.forward_lateral_samples = static_cast<std::size_t>(lateral_samples);
        config.lateral_weight =
                static_cast<float>(get_parameter("frontier.lateral_weight").as_double());
        config.heading_weight =
                static_cast<float>(get_parameter("frontier.heading_weight").as_double());
        config.dispersion_weight =
                static_cast<float>(get_parameter("frontier.dispersion_weight").as_double());
        config.task_progress_weight = static_cast<float>(
                get_parameter("frontier.task_progress_weight").as_double());
        config.task_min_progress = static_cast<float>(
                get_parameter("task.min_progress").as_double());
        config.task_max_heading_error = static_cast<float>(
                get_parameter("task.max_heading_error").as_double());
        config.min_peer_goal_separation = static_cast<float>(
                get_parameter("frontier.min_peer_goal_separation").as_double());
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
        const auto rescan_max_steps = get_parameter("rescan.max_steps").as_int();
        const auto task_rescan_max_steps =
                get_parameter("task.rescan_max_steps").as_int();
        const auto max_rejections = get_parameter("max_rejections_per_epoch").as_int();
        const auto max_debug_faces = get_parameter("max_debug_faces").as_int();
        const auto max_debug_candidates = get_parameter("max_debug_candidates").as_int();
        if(rescan_max_steps <= 0 || task_rescan_max_steps <= 0 || max_rejections <= 0
           || max_debug_faces < 0 || max_debug_candidates < 0)
        {
            throw std::invalid_argument("invalid non-negative explorer count parameter");
        }
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
        config.peer_retry_interval_seconds =
                get_parameter("peer.retry_interval").as_double();
        config.task_retry_interval_seconds =
                get_parameter("task.retry_interval").as_double();
        config.task_rescan_step_timeout_seconds =
                get_parameter("task.rescan_step_timeout").as_double();
        config.travel_heading_update_distance = static_cast<float>(
                get_parameter("motion.travel_heading_update_distance").as_double());
        config.clearance_recovery_timeout_seconds =
                get_parameter("recovery.timeout").as_double();
        config.rescan_yaw_step =
                static_cast<float>(get_parameter("rescan.yaw_step").as_double());
        config.rescan_max_steps = static_cast<std::size_t>(rescan_max_steps);
        config.task_rescan_max_steps =
                static_cast<std::size_t>(task_rescan_max_steps);
        config.max_rejections_per_epoch = static_cast<std::size_t>(max_rejections);
        config.enforce_entry_forward_half_space =
                get_parameter("entry.enforce_forward_half_space").as_bool();
        config.entry_backward_margin =
                static_cast<float>(get_parameter("entry.backward_margin").as_double());
        config.max_debug_faces = static_cast<std::size_t>(max_debug_faces);
        config.max_debug_candidates = static_cast<std::size_t>(max_debug_candidates);
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

    PlanningSnapshotConfig SingleDroneExplorerNode::loadPlanningSnapshotConfig() const
    {
        PlanningSnapshotConfig config;
        config.max_snapshot_age_seconds =
                get_parameter("planning.max_snapshot_age").as_double();
        config.max_position_drift = static_cast<float>(
                get_parameter("planning.max_position_drift").as_double());
        config.max_yaw_drift = static_cast<float>(
                get_parameter("planning.max_yaw_drift").as_double());
        return config;
    }

    void SingleDroneExplorerNode::setupPeerSubscriptions(
            const std::vector<std::string> & peer_namespaces)
    {
        peers_.clear();
        peer_odom_subs_.clear();
        peer_goal_subs_.clear();
        peers_.reserve(peer_namespaces.size());
        peer_odom_subs_.reserve(peer_namespaces.size());
        peer_goal_subs_.reserve(peer_namespaces.size());

        const auto goal_qos = rclcpp::QoS(1).reliable().transient_local();
        rclcpp::SubscriptionOptions sensor_options;
        sensor_options.callback_group = sensor_callback_group_;
        for(const std::string & ns : peer_namespaces) {
            if(ns.empty()) {
                continue;
            }
            const std::size_t index = peers_.size();
            PeerTrack track;
            track.ns = ns;
            peers_.push_back(track);

            const std::string prefix = ns.front() == '/' ? ns : ("/" + ns);
            peer_odom_subs_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                    prefix + "/odom", rclcpp::QoS(10),
                    [this, index](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        onPeerOdometry(index, msg);
                    }, sensor_options));
            peer_goal_subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>(
                    prefix + "/motion_goal", goal_qos,
                    [this, index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                        onPeerMotionGoal(index, msg);
                    }, sensor_options));
        }
    }

    void SingleDroneExplorerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        latest_odom_ = *msg;
        has_odom_    = true;
    }

    void SingleDroneExplorerNode::onPeerOdometry(
            const std::size_t peer_index, const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(peer_mutex_);
        if(peer_index >= peers_.size()) {
            return;
        }
        peers_[peer_index].odom              = *msg;
        peers_[peer_index].odom_receive_time = monotonicNow();
        peers_[peer_index].has_odom          = true;
    }

    void SingleDroneExplorerNode::onPeerMotionGoal(
            const std::size_t peer_index,
            const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(peer_mutex_);
        if(peer_index >= peers_.size()) {
            return;
        }
        peers_[peer_index].goal              = *msg;
        peers_[peer_index].goal_receive_time = monotonicNow();
        peers_[peer_index].has_goal          = true;
    }

    void SingleDroneExplorerNode::onExplorationTask(
            const swarm_controller_interfaces::msg::ExplorationTask::SharedPtr msg)
    {
        if(msg->header.frame_id != map_frame_) {
            std::lock_guard<std::mutex> lock(task_mutex_);
            task_update_status_ = "RejectedInvalidFrame";
            return;
        }
        ExplorationTaskMode mode;
        switch(msg->mode) {
            case swarm_controller_interfaces::msg::ExplorationTask::MODE_LOCAL_FALLBACK:
                mode = ExplorationTaskMode::LocalFallback;
                break;
            case swarm_controller_interfaces::msg::ExplorationTask::MODE_ASSIGNED:
                mode = ExplorationTaskMode::Assigned;
                break;
            case swarm_controller_interfaces::msg::ExplorationTask::MODE_STANDBY:
                mode = ExplorationTaskMode::Standby;
                break;
            default:
                std::lock_guard<std::mutex> lock(task_mutex_);
                task_update_status_ = "RejectedInvalidMode";
                return;
        }
        const std::int64_t lease_ns =
                static_cast<std::int64_t>(msg->lease.sec) * 1'000'000'000LL
                + static_cast<std::int64_t>(msg->lease.nanosec);
        ExplorationTaskControl control;
        control.issued_time_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
        control.lease_duration_ns = lease_ns;
        control.allocator_epoch = msg->allocator_epoch;
        control.revision = msg->revision;
        control.task_id = msg->task_id;
        control.mode = mode;
        control.target = Point3f {
                static_cast<float>(msg->target.position.x),
                static_cast<float>(msg->target.position.y),
                static_cast<float>(msg->target.position.z)};

        std::lock_guard<std::mutex> lock(task_mutex_);
        const TaskUpdateResult result = task_tracker_->update(
                control, get_clock()->now().nanoseconds(), monotonicNow());
        switch(result.status) {
            case TaskUpdateStatus::AcceptedNew:
                task_update_status_ = "AcceptedNew";
                break;
            case TaskUpdateStatus::AcceptedRenewal:
                task_update_status_ = "AcceptedRenewal";
                break;
            case TaskUpdateStatus::RejectedInvalid:
                task_update_status_ = "RejectedInvalid:" + result.reason;
                break;
            case TaskUpdateStatus::RejectedStale:
                task_update_status_ = "RejectedStale:" + result.reason;
                break;
        }
    }

    bool SingleDroneExplorerNode::transformPoseToMap(
            const std::string & source_frame, const builtin_interfaces::msg::Time & stamp,
            const geometry_msgs::msg::Pose & pose, Point3f & map_position, float * map_yaw)
    {
        geometry_msgs::msg::PoseStamped source;
        source.header.frame_id = source_frame;
        source.header.stamp    = stamp;
        source.pose            = pose;
        geometry_msgs::msg::PoseStamped map_pose = source;
        if(source_frame.empty()) {
            return false;
        }
        if(source_frame != map_frame_) {
            try {
                const geometry_msgs::msg::TransformStamped transform =
                        tf_buffer_->lookupTransform(
                                map_frame_, source_frame, timePointFromStamp(stamp),
                                tf2::durationFromSec(0.0));
                tf2::doTransform(source, map_pose, transform);
            } catch(const tf2::TransformException & error) {
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000, "peer/self TF pending: %s",
                        error.what());
                return false;
            }
        }
        map_position = Point3f {
                static_cast<float>(map_pose.pose.position.x),
                static_cast<float>(map_pose.pose.position.y),
                static_cast<float>(map_pose.pose.position.z),
        };
        if(map_yaw != nullptr) {
            const auto & q = map_pose.pose.orientation;
            *map_yaw       = static_cast<float>(std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        }
        return std::isfinite(map_position.x) && std::isfinite(map_position.y)
               && std::isfinite(map_position.z)
               && (map_yaw == nullptr || std::isfinite(*map_yaw));
    }

    double SingleDroneExplorerNode::monotonicNow() const
    {
        return std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - steady_start_)
                .count();
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
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if(stamp <= observation_stamp_ns_) {
                return;
            }
        }
        std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::fullMsgToMap(*msg));
        octomap::OcTree * tree = dynamic_cast<octomap::OcTree *>(abstract.get());
        if(tree == nullptr) {
            RCLCPP_WARN(get_logger(), "ignored Octomap with unsupported tree type '%s'", msg->id.c_str());
            return;
        }
        abstract.release();
        std::shared_ptr<octomap::OcTree> map(tree);
        const KnownFreePathChecker checker(body_envelope_config_);
        const float required = checker.requiredVerticalClearance(
                static_cast<float>(map->getResolution()));
        const bool contract_valid =
                configured_altitude_clearance_ + 1.0e-6 >= required;
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if(stamp <= observation_stamp_ns_) {
                return;
            }
            map_                         = std::move(map);
            observation_stamp_ns_       = stamp;
            required_vertical_clearance_ = required;
            clearance_contract_valid_   = contract_valid;
            ++observation_epoch_;
        }
        if(!contract_valid) {
            RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "altitude clearance %.3f is below swept-body requirement %.3f",
                    configured_altitude_clearance_, required);
        }
    }

    void SingleDroneExplorerNode::refreshPeerState(const double now_seconds)
    {
        if(!peer_tracker_) {
            peer_snapshot_ = {};
            return;
        }
        for(std::size_t index = 0U; index < peers_.size(); ++index) {
            PeerTrack peer;
            {
                std::lock_guard<std::mutex> lock(peer_mutex_);
                peer = peers_[index];
            }

            if(peer.has_odom && peer.odom_receive_time > peer.odom_applied_time) {
                if(now_seconds - peer.odom_receive_time > peer_position_timeout_seconds_) {
                    std::lock_guard<std::mutex> lock(peer_mutex_);
                    if(peers_[index].odom_receive_time == peer.odom_receive_time) {
                        peers_[index].odom_applied_time = peer.odom_receive_time;
                        ++tf_rejected_count_;
                    }
                } else {
                    Point3f map_position;
                    if(transformPoseToMap(
                               peer.odom.header.frame_id, peer.odom.header.stamp,
                               peer.odom.pose.pose, map_position, nullptr))
                    {
                        bool current = false;
                        {
                            std::lock_guard<std::mutex> lock(peer_mutex_);
                            current = peers_[index].odom_receive_time == peer.odom_receive_time;
                            if(current) {
                                peers_[index].odom_applied_time = peer.odom_receive_time;
                            }
                        }
                        if(current) {
                            peer_tracker_->updatePosition(
                                    index, map_position, peer.odom_receive_time);
                        }
                    } else {
                        ++tf_pending_count_;
                    }
                }
            }

            if(peer.has_goal && peer.goal_receive_time > peer.goal_applied_time) {
                if(now_seconds - peer.goal_receive_time > peer_goal_timeout_seconds_) {
                    std::lock_guard<std::mutex> lock(peer_mutex_);
                    if(peers_[index].goal_receive_time == peer.goal_receive_time) {
                        peers_[index].goal_applied_time = peer.goal_receive_time;
                        ++tf_rejected_count_;
                    }
                } else {
                    Point3f map_goal;
                    if(transformPoseToMap(
                               peer.goal.header.frame_id, peer.goal.header.stamp,
                               peer.goal.pose, map_goal, nullptr))
                    {
                        bool current = false;
                        {
                            std::lock_guard<std::mutex> lock(peer_mutex_);
                            current = peers_[index].goal_receive_time == peer.goal_receive_time;
                            if(current) {
                                peers_[index].goal_applied_time = peer.goal_receive_time;
                            }
                        }
                        if(current) {
                            peer_tracker_->updateGoal(index, map_goal, peer.goal_receive_time);
                        }
                    } else {
                        ++tf_pending_count_;
                    }
                }
            }
        }
        peer_snapshot_ = peer_tracker_->snapshot(now_seconds);
    }

    bool SingleDroneExplorerNode::makeExplorerInput(
            ExplorerInput & input, std::shared_ptr<octomap::OcTree> & map_snapshot,
            const double now_seconds)
    {
        nav_msgs::msg::Odometry odom;
        std::uint64_t           observation_epoch = 0U;
        std::int64_t            observation_stamp = 0;
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            if(!has_odom_ || !clearance_contract_valid_) {
                return false;
            }
            odom              = latest_odom_;
            map_snapshot      = map_;
            observation_epoch = observation_epoch_;
            observation_stamp = observation_stamp_ns_;
        }

        Point3f map_position;
        float   yaw = 0.0F;
        if(!transformPoseToMap(
                   odom.header.frame_id, odom.header.stamp,
                   odom.pose.pose, map_position, &yaw))
        {
            ++tf_pending_count_;
            return false;
        }

        const float body_x = static_cast<float>(odom.twist.twist.linear.x);
        const float body_y = static_cast<float>(odom.twist.twist.linear.y);
        input.pose.position = map_position;
        input.pose.yaw      = yaw;
        input.linear_velocity = Point3f {
                std::cos(yaw) * body_x - std::sin(yaw) * body_y,
                std::sin(yaw) * body_x + std::cos(yaw) * body_y,
                static_cast<float>(odom.twist.twist.linear.z),
        };
        input.angular_velocity_z =
                static_cast<float>(odom.twist.twist.angular.z);
        input.map                   = map_snapshot.get();
        input.observation_epoch     = observation_epoch;
        input.observation_stamp_ns  = observation_stamp;
        input.odom_stamp_ns         = rclcpp::Time(odom.header.stamp).nanoseconds();
        input.monotonic_time_seconds = now_seconds;
        input.peer_positions         = peer_snapshot_.peer_positions;
        input.active_peer_goals      = peer_snapshot_.active_peer_goals;
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            const ExplorationTaskSnapshot snapshot = task_tracker_->snapshot(
                    get_clock()->now().nanoseconds(), now_seconds);
            task_guidance_ = {};
            if(snapshot.valid) {
                task_guidance_.valid = true;
                task_guidance_.mode = snapshot.control.mode;
                task_guidance_.allocator_epoch = snapshot.control.allocator_epoch;
                task_guidance_.revision = snapshot.control.revision;
                task_guidance_.task_id = snapshot.control.task_id;
                task_guidance_.target = snapshot.control.target;
            }
            input.task_guidance = task_guidance_;
        }
        return true;
    }

    PlanningSnapshot SingleDroneExplorerNode::planningSnapshot(const ExplorerInput & input)
    {
        PlanningSnapshot snapshot;
        snapshot.pose = input.pose;
        snapshot.map_epoch = input.observation_epoch;
        snapshot.map_stamp_ns = input.observation_stamp_ns;
        snapshot.monotonic_time_seconds = input.monotonic_time_seconds;
        snapshot.active_peer_goals = input.active_peer_goals;
        snapshot.task_valid = input.task_guidance.valid;
        snapshot.task_mode = input.task_guidance.mode;
        snapshot.task_allocator_epoch = input.task_guidance.allocator_epoch;
        snapshot.task_revision = input.task_guidance.revision;
        snapshot.task_id = input.task_guidance.task_id;
        snapshot.task_target = input.task_guidance.target;
        return snapshot;
    }

    void SingleDroneExplorerNode::onControlTimer()
    {
        tf_pending_count_       = 0U;
        const double now_seconds = monotonicNow();
        refreshPeerState(now_seconds);
        ExplorerInput input;
        std::shared_ptr<octomap::OcTree> map_snapshot;
        const bool input_ready = makeExplorerInput(input, map_snapshot, now_seconds);
        if(!input_ready) {
            publishDiagnostics(get_clock()->now(), false);
            timer_->reset();
            return;
        }
        const PlanningSnapshot before = planningSnapshot(input);
        SingleDroneExplorer    candidate = *explorer_;
        ExplorerTickResult     result    = candidate.tick(input);

        const double fresh_now = monotonicNow();
        refreshPeerState(fresh_now);
        ExplorerInput fresh_input;
        std::shared_ptr<octomap::OcTree> fresh_map_snapshot;
        const bool fresh_ready =
                makeExplorerInput(fresh_input, fresh_map_snapshot, fresh_now);
        PlanningSnapshotAssessment assessment;
        if(fresh_ready) {
            assessment = planning_guard_.assess(before, planningSnapshot(fresh_input));
        } else {
            assessment.age_seconds = std::max(0.0, fresh_now - now_seconds);
            assessment.age_exceeded = true;
        }
        planning_snapshot_age_seconds_ = assessment.age_seconds;
        planning_snapshot_changed_     = assessment.requiresRevalidation();

        bool commit = true;
        if(assessment.requiresRevalidation()) {
            ++planning_revalidation_count_;
            if(fresh_ready) {
                commit = candidate.revalidatePendingResult(fresh_input, result);
            } else {
                commit = result.command.type == MotionCommandType::Hold;
            }
        }
        if(!commit) {
            ++planning_discard_count_;
            const rclcpp::Time stamp = get_clock()->now();
            publishMarkers(stamp, fresh_ready ? fresh_input.pose : input.pose, peer_snapshot_);
            publishDiagnostics(stamp, fresh_ready);
            timer_->reset();
            return;
        }

        *explorer_ = std::move(candidate);
        if(result.command.type != MotionCommandType::None) {
            publishMotionGoal(result.command);
        }
        const rclcpp::Time stamp = get_clock()->now();
        publishMarkers(stamp, fresh_ready ? fresh_input.pose : input.pose, peer_snapshot_);
        publishDiagnostics(stamp, true);
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
            const rclcpp::Time & stamp, const Pose3D & map_pose,
            const PeerDispersionSnapshot & peers)
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

        auto peer_positions = marker(
                map_frame_, stamp, "peer_positions", 0,
                visualization_msgs::msg::Marker::SPHERE_LIST);
        peer_positions.scale.x = peer_positions.scale.y = peer_positions.scale.z = 0.24;
        peer_positions.color.r = 0.25F;
        peer_positions.color.g = 0.75F;
        peer_positions.color.b = 1.0F;
        peer_positions.color.a = 0.9F;
        for(const Point3f & peer_position : peers.peer_positions) {
            peer_positions.points.push_back(point(peer_position));
        }
        array.markers.push_back(peer_positions);

        auto peer_goals = marker(
                map_frame_, stamp, "peer_active_goals", 0,
                visualization_msgs::msg::Marker::CUBE_LIST);
        peer_goals.scale.x = peer_goals.scale.y = peer_goals.scale.z = 0.22;
        peer_goals.color.r = 1.0F;
        peer_goals.color.g = 0.25F;
        peer_goals.color.b = 0.8F;
        peer_goals.color.a = 0.9F;
        for(const Point3f & peer_goal : peers.active_peer_goals) {
            peer_goals.points.push_back(point(peer_goal));
        }
        array.markers.push_back(peer_goals);

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
        text.scale.z = 0.12;
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0F;
        text.text = diagnostics.controller_state;
        {
            std::ostringstream peer_details;
            peer_details << "\npeers=" << peers.fresh_positions << '/'
                         << peers.configured_peers << " active_goals=" << peers.active_goals
                         << " stale_pos/goal=" << peers.stale_positions << '/'
                         << peers.stale_goals << " tf_pending/rejected="
                         << tf_pending_count_ << '/' << tf_rejected_count_;
            text.text += peer_details.str();
        }
        {
            TaskGuidance task_snapshot;
            {
                std::lock_guard<std::mutex> lock(task_mutex_);
                task_snapshot = task_guidance_;
            }
            std::ostringstream task;
            task << "\ntask=";
            if(!task_snapshot.valid) {
                task << "LocalFallback(expired/unavailable)";
            } else {
                task << static_cast<int>(task_snapshot.mode)
                     << " id=" << task_snapshot.task_id
                     << " rev=" << task_snapshot.revision;
            }
            text.text += task.str();
        }
        if(diagnostics.raw_candidate_count > 0U
           || diagnostics.unique_candidate_count > 0U
           || diagnostics.forward_filtered_count > 0U
           || diagnostics.peer_goal_filtered_count > 0U)
        {
            std::ostringstream details;
            details << "\nraw/unique=" << diagnostics.raw_candidate_count << '/'
                    << diagnostics.unique_candidate_count
                    << " forward_filtered=" << diagnostics.forward_filtered_count
                    << " peer_filtered=" << diagnostics.peer_goal_filtered_count
                    << " task_filtered=" << diagnostics.task_filtered_count
                    << " segment_checks=" << diagnostics.segment_check_count
                    << " select=" << std::fixed << std::setprecision(3)
                    << diagnostics.selection_elapsed_seconds << 's';
            text.text += details.str();
        }
        {
            std::ostringstream safety;
            safety << "\nbody="
                   << (diagnostics.current_body_status.empty()
                               ? "unknown"
                               : diagnostics.current_body_status)
                   << " path="
                   << (diagnostics.path_status.empty() ? "none" : diagnostics.path_status)
                   << " snapshot=" << std::fixed << std::setprecision(3)
                   << planning_snapshot_age_seconds_ << "s"
                   << " revalidated/discarded=" << planning_revalidation_count_ << '/'
                   << planning_discard_count_;
            text.text += safety.str();
        }
        if(!diagnostics.failure_reason.empty()) {
            text.text += "\n" + diagnostics.failure_reason;
        }
        array.markers.push_back(text);
        marker_publisher_->publish(array);
    }

    void SingleDroneExplorerNode::publishDiagnostics(
            const rclcpp::Time & stamp, const bool input_ready)
    {
        const ExplorationDiagnostics & exploration = explorer_->diagnostics();
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = stamp;
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name        = get_fully_qualified_name() + std::string("/exploration");
        status.hardware_id = "simulated_drone";
        status.level       = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message     = exploration.controller_state;
        bool  has_odom = false;
        bool  clearance_contract_valid = true;
        float required_vertical_clearance = 0.0F;
        std::uint64_t observation_epoch = 0U;
        {
            std::lock_guard<std::mutex> lock(input_mutex_);
            has_odom                    = has_odom_;
            clearance_contract_valid    = clearance_contract_valid_;
            required_vertical_clearance = required_vertical_clearance_;
            observation_epoch           = observation_epoch_;
        }
        if(!clearance_contract_valid) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "ClearanceContractInvalid";
        } else if(!input_ready) {
            status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = has_odom ? "WaitingForTransform" : "WaitingForOdometry";
        } else if(explorer_->state() == ExplorerState::HoveringFailure) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        } else if(explorer_->state() == ExplorerState::WaitingForPeer
                  || explorer_->state() == ExplorerState::WaitingForTask
                  || explorer_->state() == ExplorerState::ExplorationStalled
                  || explorer_->state() == ExplorerState::RecoveringClearance
                  || tf_pending_count_ > 0U || peer_snapshot_.stale_positions > 0U
                  || peer_snapshot_.stale_goals > 0U)
        {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        }

        const auto add_value = [&status](const std::string & key, const auto & value) {
            diagnostic_msgs::msg::KeyValue item;
            item.key   = key;
            item.value = std::to_string(value);
            status.values.push_back(std::move(item));
        };
        const auto add_text = [&status](const std::string & key, const std::string & value) {
            diagnostic_msgs::msg::KeyValue item;
            item.key   = key;
            item.value = value;
            status.values.push_back(std::move(item));
        };
        add_text("controller_state", exploration.controller_state);
        add_text("goal_selection_status", exploration.last_goal_status);
        add_text("failure_reason", exploration.failure_reason);
        add_text("current_body_status", exploration.current_body_status);
        add_text("path_status", exploration.path_status);
        add_value("input_ready", input_ready ? 1 : 0);
        add_value("configured_peers", peer_snapshot_.configured_peers);
        add_value("fresh_peer_positions", peer_snapshot_.fresh_positions);
        add_value("stale_peer_positions", peer_snapshot_.stale_positions);
        add_value("missing_peer_positions", peer_snapshot_.missing_positions);
        add_value("fresh_peer_goals", peer_snapshot_.fresh_goals);
        add_value("stale_peer_goals", peer_snapshot_.stale_goals);
        add_value("missing_peer_goals", peer_snapshot_.missing_goals);
        add_value("active_peer_goals", peer_snapshot_.active_goals);
        add_value("pre_peer_candidates", exploration.pre_peer_candidate_count);
        add_value("peer_goal_filtered", exploration.peer_goal_filtered_count);
        add_value("post_peer_candidates", exploration.post_peer_candidate_count);
        add_value("task_filtered", exploration.task_filtered_count);
        add_value("raw_candidates", exploration.raw_candidate_count);
        add_value("unique_candidates", exploration.unique_candidate_count);
        add_value("forward_filtered", exploration.forward_filtered_count);
        add_value("segment_checks", exploration.segment_check_count);
        add_value("selection_elapsed_seconds", exploration.selection_elapsed_seconds);
        add_value("planning_snapshot_age_seconds", planning_snapshot_age_seconds_);
        add_value("planning_snapshot_changed", planning_snapshot_changed_ ? 1 : 0);
        add_value("planning_revalidations", planning_revalidation_count_);
        add_value("planning_discards", planning_discard_count_);
        add_value("observation_epoch", observation_epoch);
        add_value("configured_altitude_clearance", configured_altitude_clearance_);
        add_value("required_vertical_clearance", required_vertical_clearance);
        add_value("clearance_contract_valid", clearance_contract_valid ? 1 : 0);
        if(exploration.first_blocked_position.has_value()) {
            add_value("first_blocked_x", exploration.first_blocked_position->x);
            add_value("first_blocked_y", exploration.first_blocked_position->y);
            add_value("first_blocked_z", exploration.first_blocked_position->z);
        }
        add_value("tf_pending", tf_pending_count_);
        add_value("tf_rejected", tf_rejected_count_);
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            add_text("task_update_status", task_update_status_);
            add_value("task_valid", task_guidance_.valid ? 1 : 0);
            add_value("task_mode", static_cast<int>(task_guidance_.mode));
            add_value("task_allocator_epoch", task_guidance_.allocator_epoch);
            add_value("task_revision", task_guidance_.revision);
            add_value("task_id", task_guidance_.task_id);
            add_value("task_rejected", task_tracker_->rejectedCount());
        }
        message.status.push_back(std::move(status));
        diagnostics_publisher_->publish(message);
    }

}// namespace SwarmController
