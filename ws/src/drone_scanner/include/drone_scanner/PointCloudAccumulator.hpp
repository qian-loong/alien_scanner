#ifndef DRONE_SCANNER_POINTCLOUDACCUMULATOR_HPP
#define DRONE_SCANNER_POINTCLOUDACCUMULATOR_HPP

#include "drone_scanner/Point3f.hpp"
#include "drone_scanner/RigidTransform.hpp"

#include <cstddef>
#include <vector>

namespace DroneScanner {

    /// 将多帧扫描点累积在 map 系；无 ROS 依赖。
    /// @param max_points 容量上限；0 表示不限制（默认）。
    class PointCloudAccumulator
    {
    public:
        explicit PointCloudAccumulator(std::size_t max_points = 0);

        void appendPointsInMapFrame(const std::vector<Point3f> & points);
        void appendTransformed(const std::vector<Point3f> & points, const RigidTransform & transform);
        void clear();

        std::size_t                    size() const;
        const std::vector<Point3f> & points() const;

    private:
        void mergePoints(const std::vector<Point3f> & incoming);

        std::vector<Point3f> points_;
        std::size_t          max_points_ {0};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_POINTCLOUDACCUMULATOR_HPP
