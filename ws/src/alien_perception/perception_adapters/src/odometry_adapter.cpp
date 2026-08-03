#include "perception_adapters/odometry_adapter.hpp"
#include <algorithm>
#include <cmath>

namespace Perception::Adapters {

    OdometryAdapter::OdometryAdapter(const Config & config)
        : config_(config)
    {}

    PoseEstimate OdometryAdapter::convert(
            const nav_msgs::msg::Odometry & msg,
            const SourceID &                source_id,
            const SessionID &               session_id,
            const std::string &             clock_domain)
    {
        PoseEstimate estimate;

        // Identity and timing
        estimate.source_id    = source_id;
        estimate.session_id   = session_id;
        estimate.frame_id     = msg.header.frame_id;
        estimate.clock_domain = clock_domain;
        estimate.stamp        = Timestamp::from_nanoseconds(
                msg.header.stamp.sec * 1'000'000'000LL + msg.header.stamp.nanosec);

        // Pose
        estimate.position = Eigen::Vector3d(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z);
        estimate.orientation = Eigen::Quaterniond(
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z);

        // Covariance (6x6: position + orientation)
        PoseEstimate::Matrix6d cov;
        for(int i = 0; i < 6; ++i) {
            for(int j = 0; j < 6; ++j) {
                cov(i, j) = msg.pose.covariance[i * 6 + j];
            }
        }
        estimate.covariance = cov;

        // Compute quality from covariance
        estimate.quality = compute_quality(msg.pose.covariance);

        // Freshness (will be updated by node)
        estimate.freshness = Duration::from_seconds(0.0);

        // Reset epoch detection
        estimate.reset_epoch = current_reset_epoch_;

        if(last_pose_.has_value()) {
            if(detect_pose_reset(estimate, *last_pose_)) {
                ++current_reset_epoch_;
                estimate.reset_epoch = current_reset_epoch_;
            }
        }

        last_pose_ = estimate;

        return estimate;
    }

    PoseEstimate OdometryAdapter::from_tf(
            const geometry_msgs::msg::TransformStamped & transform,
            const SourceID &                             source_id,
            const SessionID &                            session_id,
            const std::string &                          clock_domain)
    {
        PoseEstimate estimate;

        // Identity and timing
        estimate.source_id    = source_id;
        estimate.session_id   = session_id;
        estimate.frame_id     = transform.header.frame_id;
        estimate.clock_domain = clock_domain;
        estimate.stamp        = Timestamp::from_nanoseconds(
                transform.header.stamp.sec * 1'000'000'000LL + transform.header.stamp.nanosec);

        // Pose from transform
        estimate.position = Eigen::Vector3d(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z);
        estimate.orientation = Eigen::Quaterniond(
                transform.transform.rotation.w,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z);

        // No covariance from TF
        estimate.covariance = std::nullopt;
        estimate.quality    = 1.0;// Assume good quality if from TF

        // Freshness
        estimate.freshness = Duration::from_seconds(0.0);

        // Reset epoch detection
        estimate.reset_epoch = current_reset_epoch_;

        if(last_pose_.has_value()) {
            if(detect_pose_reset(estimate, *last_pose_)) {
                ++current_reset_epoch_;
                estimate.reset_epoch = current_reset_epoch_;
            }
        }

        last_pose_ = estimate;

        return estimate;
    }

    double OdometryAdapter::compute_quality(const std::array<double, 36> & covariance) const
    {
        // Compute quality from position covariance (first 3x3 block)
        // Quality = 1 / (1 + trace(cov_position))
        double trace = covariance[0] + covariance[7] + covariance[14];

        // Clamp to avoid division issues
        trace = std::max(trace, 0.0);

        double quality = 1.0 / (1.0 + trace);
        return std::clamp(quality, 0.0, 1.0);
    }

    bool OdometryAdapter::detect_pose_reset(
            const PoseEstimate & current,
            const PoseEstimate & last)
    {
        // Frame change
        if(current.frame_id != last.frame_id) {
            return true;
        }

        // Time went backward
        if(current.stamp < last.stamp) {
            return true;
        }

        // Position jump detection
        double distance   = (current.position - last.position).norm();
        double time_delta = (current.stamp - last.stamp).to_seconds();

        if(distance > config_.position_jump_threshold_m && time_delta < config_.position_jump_time_window_s) {
            return true;
        }

        return false;
    }

}// namespace Perception::Adapters
