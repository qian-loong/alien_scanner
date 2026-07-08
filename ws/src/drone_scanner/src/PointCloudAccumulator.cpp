#include "drone_scanner/PointCloudAccumulator.hpp"

namespace DroneScanner {

    PointCloudAccumulator::PointCloudAccumulator(std::size_t max_points)
        : max_points_(max_points)
    {
    }

    void PointCloudAccumulator::mergePoints(const std::vector<Point3f> & incoming)
    {
        if(incoming.empty()) {
            return;
        }

        // max_points_ == 0：不限制容量
        if(max_points_ == 0) {
            points_.insert(points_.end(), incoming.begin(), incoming.end());
            return;
        }

        // 单批已超过 cap：只保留该批最新的 max_points_ 个点，避免短暂超额内存
        if(incoming.size() >= max_points_) {
            points_.assign(incoming.end() - static_cast<std::ptrdiff_t>(max_points_), incoming.end());
            return;
        }

        const std::size_t combined = points_.size() + incoming.size();
        if(combined <= max_points_) {
            points_.insert(points_.end(), incoming.begin(), incoming.end());
            return;
        }

        const std::size_t keep_old = max_points_ - incoming.size();
        std::vector<Point3f> merged;
        merged.reserve(max_points_);
        if(keep_old > 0 && points_.size() >= keep_old) {
            merged.insert(
                    merged.end(), points_.end() - static_cast<std::ptrdiff_t>(keep_old), points_.end());
        }
        merged.insert(merged.end(), incoming.begin(), incoming.end());
        points_.swap(merged);
    }

    void PointCloudAccumulator::appendPointsInMapFrame(const std::vector<Point3f> & points)
    {
        mergePoints(points);
    }

    void PointCloudAccumulator::appendTransformed(
            const std::vector<Point3f> & points, const RigidTransform & transform)
    {
        std::vector<Point3f> transformed;
        transformed.reserve(points.size());
        for(const Point3f & point : points) {
            transformed.push_back(transformPoint(point, transform));
        }
        mergePoints(transformed);
    }

    void PointCloudAccumulator::clear()
    {
        points_.clear();
    }

    std::size_t PointCloudAccumulator::size() const
    {
        return points_.size();
    }

    const std::vector<Point3f> & PointCloudAccumulator::points() const
    {
        return points_;
    }

}// namespace DroneScanner
