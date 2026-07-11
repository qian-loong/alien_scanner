#include "drone_scanner/PoseSegmentTrajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace DroneScanner {

    namespace {

        constexpr float PI     = 3.14159265358979323846F;
        constexpr float TWO_PI = 2.0F * PI;

        bool isFinite(const Pose3D & pose)
        {
            return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z)
                   && std::isfinite(pose.yaw);
        }

        float shortestYawDelta(float from, float to)
        {
            float delta = std::fmod(to - from, TWO_PI);
            if(delta > PI) {
                delta -= TWO_PI;
            } else if(delta < -PI) {
                delta += TWO_PI;
            }
            return delta;
        }

        float interpolationFactor(double time, double duration)
        {
            if(duration <= 0.0) {
                return 1.0F;
            }
            return static_cast<float>(std::clamp(time / duration, 0.0, 1.0));
        }

    }// namespace

    PoseSegmentTrajectory::PoseSegmentTrajectory(const PoseSegmentConfig & config)
        : config_(config)
    {
        if(!isFinite(config_.start) || !isFinite(config_.goal)
           || !std::isfinite(config_.linear_speed) || config_.linear_speed <= 0.0F
           || !std::isfinite(config_.yaw_rate) || config_.yaw_rate <= 0.0F)
        {
            throw std::invalid_argument("pose segment config must be finite with positive rates");
        }

        config_.goal.z = config_.start.z;
        const float dx = config_.goal.x - config_.start.x;
        const float dy = config_.goal.y - config_.start.y;
        distance_xy_   = std::sqrt(dx * dx + dy * dy);
        yaw_delta_     = shortestYawDelta(config_.start.yaw, config_.goal.yaw);

        const double translation_duration =
                static_cast<double>(distance_xy_ / config_.linear_speed);
        const double rotation_duration =
                static_cast<double>(std::fabs(yaw_delta_) / config_.yaw_rate);
        duration_seconds_ = std::max(translation_duration, rotation_duration);
    }

    Pose3D PoseSegmentTrajectory::pose(double t_seconds) const
    {
        const float factor = interpolationFactor(t_seconds, duration_seconds_);
        return Pose3D {
                config_.start.x + (config_.goal.x - config_.start.x) * factor,
                config_.start.y + (config_.goal.y - config_.start.y) * factor,
                config_.start.z,
                config_.start.yaw + yaw_delta_ * factor,
        };
    }

    double PoseSegmentTrajectory::duration() const
    {
        return duration_seconds_;
    }

    PoseSegmentVelocity PoseSegmentTrajectory::velocity(double t_seconds) const
    {
        if(duration_seconds_ <= 0.0 || t_seconds < 0.0 || t_seconds >= duration_seconds_) {
            return {};
        }
        return PoseSegmentVelocity {
                static_cast<float>((config_.goal.x - config_.start.x) / duration_seconds_),
                static_cast<float>((config_.goal.y - config_.start.y) / duration_seconds_),
                0.0F,
                static_cast<float>(yaw_delta_ / duration_seconds_),
        };
    }

    bool PoseSegmentTrajectory::isTranslationActive(double t_seconds) const
    {
        return distance_xy_ > 1.0e-6F && t_seconds >= 0.0 && t_seconds < duration_seconds_;
    }

}// namespace DroneScanner
