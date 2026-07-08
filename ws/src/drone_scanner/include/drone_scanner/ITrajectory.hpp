#ifndef DRONE_SCANNER_ITRAJECTORY_HPP
#define DRONE_SCANNER_ITRAJECTORY_HPP

#include "drone_scanner/Pose3D.hpp"

namespace DroneScanner {

    /// 时间参数化轨迹：t=0 为起点，t=duration() 为终点（秒）。
    class ITrajectory
    {
    public:
        virtual ~ITrajectory() = default;

        virtual Pose3D pose(double t_seconds) const = 0;

        /// 轨迹总时长 (s)；t 超出 [0, duration] 时由具体实现决定（LineTrajectory 会 clamp）。
        virtual double duration() const = 0;
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_ITRAJECTORY_HPP
