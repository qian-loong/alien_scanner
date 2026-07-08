#include "drone_scanner/LineTrajectory.hpp"

#include <algorithm>
#include <cmath>

namespace DroneScanner {

    namespace {

        float clampUnit(double u)
        {
            return static_cast<float>(std::clamp(u, 0.0, 1.0));
        }

        float pathLength3(float sx, float sy, float sz, float ex, float ey, float ez)
        {
            const float dx = ex - sx;
            const float dy = ey - sy;
            const float dz = ez - sz;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

    }// namespace

    LineTrajectory::LineTrajectory(const LineTrajectoryConfig & config)
        : config_(config)
    {
        if(config_.duration_seconds <= 0.0) {
            config_.duration_seconds = 1.0;
        }

        path_length_ = pathLength3(
                config_.start_x, config_.start_y, config_.start_z, config_.end_x, config_.end_y, config_.end_z);

        const float dx = config_.end_x - config_.start_x;
        const float dy = config_.end_y - config_.start_y;
        if(path_length_ > 1e-6F) {
            yaw_ = std::atan2(dy, dx);
        }
    }

    Pose3D LineTrajectory::pose(double t_seconds) const
    {
        const double u = (config_.duration_seconds > 0.0) ? (t_seconds / config_.duration_seconds) : 0.0;
        const float  f = clampUnit(u);

        Pose3D result;
        result.x   = config_.start_x + (config_.end_x - config_.start_x) * f;
        result.y   = config_.start_y + (config_.end_y - config_.start_y) * f;
        result.z   = config_.start_z + (config_.end_z - config_.start_z) * f;
        result.yaw = yaw_;
        return result;
    }

    double LineTrajectory::duration() const
    {
        return config_.duration_seconds;
    }

    float LineTrajectory::speed() const
    {
        return path_length_ / static_cast<float>(config_.duration_seconds);
    }

}// namespace DroneScanner
