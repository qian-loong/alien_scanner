#include "drone_scanner/FakeOdomNode.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>

#include <sensor_msgs/point_cloud2_iterator.hpp>

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
    path_min_distance_ = get_parameter("path.min_distance").as_double();
    if(path_min_distance_ <= 0.0) {
        path_min_distance_ = 0.05;
    }
    const int path_max_poses = get_parameter("path.max_poses").as_int();
    path_max_poses_ = path_max_poses > 0 ? static_cast<std::size_t>(path_max_poses) : 5000U;
    goal_mode_ = get_parameter("motion.mode").as_string() == "goal";
    goal_topic_ = get_parameter("motion.goal_topic").as_string();
    goal_frame_ = get_parameter("motion.goal_frame").as_string();
    goal_linear_speed_ =
            static_cast<float>(get_parameter("motion.linear_speed").as_double());
    goal_yaw_rate_ = static_cast<float>(get_parameter("motion.yaw_rate").as_double());
    goal_epsilon_  = static_cast<float>(get_parameter("motion.goal_epsilon").as_double());
    if(!std::isfinite(goal_linear_speed_) || goal_linear_speed_ <= 0.0F) {
        goal_linear_speed_ = 0.4F;
    }
    if(!std::isfinite(goal_yaw_rate_) || goal_yaw_rate_ <= 0.0F) {
        goal_yaw_rate_ = 0.5F;
    }
    if(!std::isfinite(goal_epsilon_) || goal_epsilon_ <= 0.0F) {
        goal_epsilon_ = 1.0e-4F;
    }
    altitude_adapt_enable_ = get_parameter("altitude_adapt.enable").as_bool();
    points_stale_sec_      = get_parameter("altitude_adapt.points_stale_sec").as_double();
    if(points_stale_sec_ <= 0.0) {
        points_stale_sec_ = 0.5;
    }

    initTrajectory();
    initAltitudeAdapter();
    initPublisher();
    initScanSubscription();
    start_time_   = get_clock()->now();
    adapted_z_    = static_cast<float>(get_parameter("line.start_z").as_double());
    last_pose_z_  = adapted_z_;
    current_pose_ = trajectory_->pose(0.0);
    has_current_pose_ = true;
    segment_start_time_ = start_time_;
    initGoalSubscription();
    initTimer();

    if(goal_mode_) {
        RCLCPP_INFO(
                get_logger(),
                "fake_odom: goal mode at (%.2f,%.2f,%.2f), %.2f m/s, %.2f rad/s, altitude_adapt=%s",
                current_pose_.x, current_pose_.y, current_pose_.z, goal_linear_speed_,
                goal_yaw_rate_, altitude_adapt_enable_ ? "on" : "off");
    } else {
        RCLCPP_INFO(
                get_logger(),
                "fake_odom: line (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f) in %.1fs, %.2f m/s, altitude_adapt=%s",
                get_parameter("line.start_x").as_double(), get_parameter("line.start_y").as_double(),
                get_parameter("line.start_z").as_double(), get_parameter("line.end_x").as_double(),
                get_parameter("line.end_y").as_double(), get_parameter("line.end_z").as_double(),
                get_parameter("line.duration_seconds").as_double(), trajectory_->speed(),
                altitude_adapt_enable_ ? "on" : "off");
    }
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
    declare_parameter("path.min_distance", 0.05);
    declare_parameter("path.max_poses", 5000);

    declare_parameter("motion.mode", "line");
    declare_parameter("motion.goal_topic", "motion_goal");
    declare_parameter("motion.goal_frame", "map");
    declare_parameter("motion.linear_speed", 0.4);
    declare_parameter("motion.yaw_rate", 0.5);
    declare_parameter("motion.goal_epsilon", 1.0e-4);

    declare_parameter("altitude_adapt.enable", true);
    declare_parameter("altitude_adapt.target_fraction", 0.5);
    declare_parameter("altitude_adapt.min_clearance", 0.41);
    declare_parameter("altitude_adapt.max_vertical_speed", 0.6);
    declare_parameter("altitude_adapt.band_ema_alpha", 0.25);
    declare_parameter("altitude_adapt.min_band_height", 0.8);
    declare_parameter("altitude_adapt.vertical_dot_min", 0.65);
    declare_parameter("altitude_adapt.ring_pitch_rad", 0.35);
    declare_parameter("altitude_adapt.points_stale_sec", 0.5);
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

AltitudeAdaptConfig FakeOdomNode::loadAltitudeConfigFromParameters() const
{
    AltitudeAdaptConfig config;
    config.target_fraction    = static_cast<float>(get_parameter("altitude_adapt.target_fraction").as_double());
    config.min_clearance      = static_cast<float>(get_parameter("altitude_adapt.min_clearance").as_double());
    config.max_vertical_speed = static_cast<float>(get_parameter("altitude_adapt.max_vertical_speed").as_double());
    config.band_ema_alpha     = static_cast<float>(get_parameter("altitude_adapt.band_ema_alpha").as_double());
    config.min_band_height    = static_cast<float>(get_parameter("altitude_adapt.min_band_height").as_double());
    config.vertical_dot_min   = static_cast<float>(get_parameter("altitude_adapt.vertical_dot_min").as_double());
    config.ring_pitch_rad     = static_cast<float>(get_parameter("altitude_adapt.ring_pitch_rad").as_double());
    return config;
}

void FakeOdomNode::initTrajectory()
{
    trajectory_ = std::make_unique<LineTrajectory>(loadLineConfigFromParameters());
}

void FakeOdomNode::initAltitudeAdapter()
{
    altitude_adapter_ = std::make_unique<AltitudeAdapter>(loadAltitudeConfigFromParameters());
    if(altitude_adapt_enable_ && !altitude_adapter_->geometryCompatible()) {
        RCLCPP_ERROR(
                get_logger(),
                "altitude_adapt: vertical_dot_min=%.3f incompatible with ring_pitch=%.3f rad "
                "(need vertical_dot_min < cos(pitch)=%.3f); altitude adaptation disabled",
                altitude_adapter_->config().vertical_dot_min, altitude_adapter_->config().ring_pitch_rad,
                std::cos(altitude_adapter_->config().ring_pitch_rad));
        altitude_adapt_enable_ = false;
    }
}

void FakeOdomNode::initPublisher()
{
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS(1).transient_local());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    flown_path_.header.frame_id = odom_frame_;
}

void FakeOdomNode::initScanSubscription()
{
    if(!altitude_adapt_enable_) {
        return;
    }
    points_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "points", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                onPoints(msg);
            });
}

void FakeOdomNode::initGoalSubscription()
{
    if(!goal_mode_) {
        return;
    }
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, rclcpp::QoS(1).reliable().transient_local(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                onMotionGoal(msg);
            });
}

void FakeOdomNode::initTimer()
{
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / publish_rate_hz_));
    timer_ = create_wall_timer(period, [this]() {
        onTimer();
    });
}

float FakeOdomNode::quaternionToYaw(const geometry_msgs::msg::Quaternion & quaternion)
{
    const double norm = std::sqrt(
            quaternion.x * quaternion.x + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z + quaternion.w * quaternion.w);
    if(!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
        return std::numeric_limits<float>::quiet_NaN();
    }
    const double x = quaternion.x / norm;
    const double y = quaternion.y / norm;
    const double z = quaternion.z / norm;
    const double w = quaternion.w / norm;
    return static_cast<float>(
            std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)));
}

float FakeOdomNode::shortestYawDistance(float lhs, float rhs)
{
    constexpr float pi = 3.14159265358979323846F;
    return std::remainder(lhs - rhs, 2.0F * pi);
}

void FakeOdomNode::onMotionGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    if(!goal_mode_) {
        return;
    }
    if(msg->header.frame_id != goal_frame_) {
        RCLCPP_WARN(
                get_logger(), "fake_odom: reject goal frame '%s', expected '%s'",
                msg->header.frame_id.c_str(), goal_frame_.c_str());
        return;
    }

    const float x   = static_cast<float>(msg->pose.position.x);
    const float y   = static_cast<float>(msg->pose.position.y);
    const float yaw = quaternionToYaw(msg->pose.orientation);
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(msg->pose.position.z)
       || !std::isfinite(yaw))
    {
        RCLCPP_WARN(get_logger(), "fake_odom: reject non-finite motion goal");
        return;
    }

    std::lock_guard<std::mutex> lock(motion_mutex_);
    const Pose3D goal {x, y, current_pose_.z, yaw};
    if(has_accepted_goal_
       && std::hypot(goal.x - accepted_goal_.x, goal.y - accepted_goal_.y) <= goal_epsilon_
       && std::fabs(shortestYawDistance(goal.yaw, accepted_goal_.yaw)) <= goal_epsilon_)
    {
        return;
    }

    PoseSegmentConfig config;
    config.start        = current_pose_;
    config.goal         = goal;
    config.linear_speed = goal_linear_speed_;
    config.yaw_rate     = goal_yaw_rate_;
    goal_trajectory_    = std::make_unique<PoseSegmentTrajectory>(config);
    segment_start_time_ = get_clock()->now();
    accepted_goal_      = goal;
    has_accepted_goal_  = true;
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

std::vector<LidarPoint> FakeOdomNode::pointCloudToLidarPoints(const sensor_msgs::msg::PointCloud2 & cloud)
{
    std::vector<LidarPoint> hits;
    if(cloud.width == 0 || cloud.data.empty()) {
        return hits;
    }
    hits.reserve(cloud.width * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");
    for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        hits.push_back(LidarPoint {*iter_x, *iter_y, *iter_z});
    }
    return hits;
}

void FakeOdomNode::onPoints(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    auto hits = pointCloudToLidarPoints(*msg);
    const rclcpp::Time stamp(msg->header.stamp);
    // 按点云时间戳在已发布位姿历史中取最近 z，避免用「接收时」的后续 z 解释旧 hits
    const float scan_origin_z = lookupPoseZNear(stamp);
    std::lock_guard<std::mutex> lock(hits_mutex_);
    pending_hits_          = std::move(hits);
    pending_scan_origin_z_ = scan_origin_z;
    pending_stamp_         = stamp;
    has_pending_scan_      = true;
}

float FakeOdomNode::lookupPoseZNear(const rclcpp::Time & stamp) const
{
    std::lock_guard<std::mutex> lock(pose_history_mutex_);
    if(pose_history_.empty()) {
        return last_pose_z_;
    }
    const PoseZSample * best = &pose_history_.front();
    int64_t             best_ns = std::llabs((stamp - best->stamp).nanoseconds());
    for(const PoseZSample & sample : pose_history_) {
        const int64_t d = std::llabs((stamp - sample.stamp).nanoseconds());
        if(d < best_ns) {
            best_ns = d;
            best    = &sample;
        }
    }
    return best->z;
}

void FakeOdomNode::onTimer()
{
    const rclcpp::Time now = get_clock()->now();
    const double       line_t = (now - start_time_).seconds();
    Pose3D             pose;
    PoseSegmentVelocity segment_velocity;
    bool                translation_active = false;
    if(goal_mode_) {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        if(goal_trajectory_) {
            const double segment_t = (now - segment_start_time_).seconds();
            pose                   = goal_trajectory_->pose(segment_t);
            segment_velocity       = goal_trajectory_->velocity(segment_t);
            translation_active     = goal_trajectory_->isTranslationActive(segment_t);
            if(translation_active) {
                altitude_adapt_unlocked_ = true;
            }
        } else {
            pose = current_pose_;
        }
        // 首段 XY 平移前冻结高度，避免入口悬停/重扫时 AltitudeAdapter 错误下坠。
        if(!altitude_adapt_unlocked_) {
            pose.z = current_pose_.z;
        }
    } else {
        pose = trajectory_->pose(line_t);
    }

    float dt = 1.0F / static_cast<float>(publish_rate_hz_);
    if(has_last_adapt_time_) {
        dt = static_cast<float>((now - last_adapt_time_).seconds());
    }
    last_adapt_time_     = now;
    has_last_adapt_time_ = true;
    const float previous_z = last_pose_z_;
    if(altitude_adapt_enable_ && altitude_adapter_) {
        std::vector<LidarPoint> new_hits;
        float                   scan_origin_z = 0.0F;
        rclcpp::Time            scan_stamp;
        bool                    consume_new = false;
        {
            std::lock_guard<std::mutex> lock(hits_mutex_);
            if(has_pending_scan_) {
                new_hits       = std::move(pending_hits_);
                scan_origin_z  = pending_scan_origin_z_;
                scan_stamp     = pending_stamp_;
                has_pending_scan_ = false;
                consume_new    = true;
            }
        }

        if(consume_new) {
            const double age = (now - scan_stamp).seconds();
            if(age <= points_stale_sec_) {
                // 仅新帧推进 EMA；用扫描时 z 解释相对命中点
                last_band_ = altitude_adapter_->estimateBand(new_hits, scan_origin_z);
            }
        }

        // line 模式：始终可调高。goal 模式：首段 XY 平移开始前冻结；平移中仍冻结 z。
        const bool allow_altitude_motion =
                !goal_mode_ || (altitude_adapt_unlocked_ && !translation_active);
        if(allow_altitude_motion) {
            const float reference_z = has_adapted_z_ ? adapted_z_ : pose.z;
            if(last_band_.valid) {
                adapted_z_     = altitude_adapter_->adaptZ(last_band_, reference_z, dt);
                has_adapted_z_ = true;
                pose.z         = adapted_z_;
            } else if(has_adapted_z_) {
                pose.z = adapted_z_;
            }
        }
    }

    if(goal_mode_) {
        std::lock_guard<std::mutex> lock(motion_mutex_);
        current_pose_     = pose;
        has_current_pose_ = true;
    }
    {
        std::lock_guard<std::mutex> lock(pose_history_mutex_);
        last_pose_z_ = static_cast<float>(pose.z);
        pose_history_.push_back(PoseZSample {now, last_pose_z_});
        if(pose_history_.size() > kPoseHistorySize) {
            pose_history_.erase(pose_history_.begin());
        }
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id  = base_frame_;
    odom.pose.pose.position.x = pose.x;
    odom.pose.pose.position.y = pose.y;
    odom.pose.pose.position.z = pose.z;
    odom.pose.pose.orientation = yawToQuaternion(pose.yaw);

    if(goal_mode_) {
        const float cos_yaw = std::cos(pose.yaw);
        const float sin_yaw = std::sin(pose.yaw);
        odom.twist.twist.linear.x =
                cos_yaw * segment_velocity.x + sin_yaw * segment_velocity.y;
        odom.twist.twist.linear.y =
                -sin_yaw * segment_velocity.x + cos_yaw * segment_velocity.y;
        odom.twist.twist.linear.z = dt > 0.0F ? (pose.z - previous_z) / dt : 0.0F;
        odom.twist.twist.angular.z = segment_velocity.yaw_rate;
    } else if(line_t < trajectory_->duration()) {
        odom.twist.twist.linear.x = trajectory_->speed();
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0.0;
    }

    odom_publisher_->publish(odom);
    tf_broadcaster_->sendTransform(makeTransform(now, odom_frame_, base_frame_, pose));
    appendAndPublishPath(now, pose);
}

void FakeOdomNode::appendAndPublishPath(const rclcpp::Time & stamp, const Pose3D & pose)
{
    geometry_msgs::msg::PoseStamped sample;
    sample.header.stamp    = stamp;
    sample.header.frame_id = odom_frame_;
    sample.pose.position.x = pose.x;
    sample.pose.position.y = pose.y;
    sample.pose.position.z = pose.z;
    sample.pose.orientation = yawToQuaternion(pose.yaw);

    bool append = flown_path_.poses.empty();
    if(!append) {
        const auto & last = flown_path_.poses.back().pose.position;
        const double dx   = pose.x - last.x;
        const double dy   = pose.y - last.y;
        const double dz   = pose.z - last.z;
        append = std::sqrt(dx * dx + dy * dy + dz * dz) >= path_min_distance_;
    }
    if(append) {
        flown_path_.poses.push_back(sample);
        while(flown_path_.poses.size() > path_max_poses_) {
            flown_path_.poses.erase(flown_path_.poses.begin());
        }
    }

    flown_path_.header.stamp    = stamp;
    flown_path_.header.frame_id = odom_frame_;
    path_publisher_->publish(flown_path_);
}

}// namespace DroneScanner
