#ifndef PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_ODOMETRY_ADAPTER_HPP
#define PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_ODOMETRY_ADAPTER_HPP

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "perception_core/observation/pose_estimate.hpp"
#include <memory>
#include <optional>

namespace Perception::Adapters {

    /// Odometry to PoseEstimate adapter
    class OdometryAdapter
    {
    public:
        struct Config {
            double position_jump_threshold_m;
            double position_jump_time_window_s;

            Config()
                : position_jump_threshold_m(5.0)
                , position_jump_time_window_s(1.0)
            {}
        };

        explicit OdometryAdapter(const Config & config = Config {});

        /// Convert Odometry to PoseEstimate
        PoseEstimate convert(
                const nav_msgs::msg::Odometry & msg,
                const SourceID &                source_id,
                const SessionID &               session_id,
                const std::string &             clock_domain = "vehicle_steady_clock");

        /// Convert TF to PoseEstimate
        PoseEstimate from_tf(
                const geometry_msgs::msg::TransformStamped & transform,
                const SourceID &                             source_id,
                const SessionID &                            session_id,
                const std::string &                          clock_domain = "vehicle_steady_clock");

        /// Get current reset epoch
        uint64_t current_reset_epoch() const { return current_reset_epoch_; }

    private:
        Config                      config_;
        std::optional<PoseEstimate> last_pose_;
        uint64_t                    current_reset_epoch_ = 0;

        /// Compute quality score from covariance
        double compute_quality(const std::array<double, 36> & covariance) const;

        /// Check if pose reset occurred
        bool detect_pose_reset(
                const PoseEstimate & current,
                const PoseEstimate & last);
    };

}// namespace Perception::Adapters

#endif// PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_ODOMETRY_ADAPTER_HPP
