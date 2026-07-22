#ifndef DRONE_SCANNER_POSESEGMENTTRAJECTORY_HPP
#define DRONE_SCANNER_POSESEGMENTTRAJECTORY_HPP

#include "drone_scanner/ITrajectory.hpp"

namespace DroneScanner {

    struct PoseSegmentConfig {
        Pose3D start {};
        Pose3D goal {};
        float  linear_speed {0.4F};
        float  yaw_rate {0.5F};
    };

    struct PoseSegmentVelocity {
        float x {};
        float y {};
        float z {};
        float yaw_rate {};
    };

    /// XY + yaw 短段；z 始终锁定为 start.z，由高度适配器在非平移期间独占控制。
    class PoseSegmentTrajectory final : public ITrajectory
    {
    public:
        explicit PoseSegmentTrajectory(const PoseSegmentConfig & config);

        Pose3D pose(double t_seconds) const override;
        double duration() const override;

        PoseSegmentVelocity velocity(double t_seconds) const;
        bool isTranslationActive(double t_seconds) const;

    private:
        PoseSegmentConfig config_;
        float             distance_xy_ {};
        float             yaw_delta_ {};
        double            duration_seconds_ {};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_POSESEGMENTTRAJECTORY_HPP
