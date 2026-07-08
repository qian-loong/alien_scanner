#ifndef DRONE_SCANNER_LINETRAJECTORY_HPP
#define DRONE_SCANNER_LINETRAJECTORY_HPP

#include "drone_scanner/ITrajectory.hpp"

namespace DroneScanner {

    struct LineTrajectoryConfig {
        float start_x {0.0F};
        float start_y {0.0F};
        float start_z {1.5F};
        float end_x {11.0F};   ///< 沿 +x 进洞（与 TreeCaveField 接入段一致）
        float end_y {0.0F};
        float end_z {1.5F};
        double duration_seconds {60.0}; ///< 从起点飞到终点的总时间 (s)，须 > 0
    };

    /// 起点到终点的匀速直线；yaw 沿 xy 投影方向，全程恒定。
    class LineTrajectory : public ITrajectory
    {
    public:
        explicit LineTrajectory(const LineTrajectoryConfig & config);

        Pose3D pose(double t_seconds) const override;
        double duration() const override;

        /// 恒定线速度模长 (m/s)。
        float speed() const;

    private:
        LineTrajectoryConfig config_;
        float                yaw_ {0.0F};
        float                path_length_ {0.0F};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_LINETRAJECTORY_HPP
