#include "swarm_controller/KnownFreePathChecker.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace SwarmController {

    namespace {

        bool isFinite(const Point3f & point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        Point3f toPoint3f(const octomap::point3d & point)
        {
            return Point3f {point.x(), point.y(), point.z()};
        }

    }// namespace

    const char * pathCheckStatusName(const PathCheckStatus status)
    {
        switch(status) {
            case PathCheckStatus::Safe:
                return "Safe";
            case PathCheckStatus::UnknownBlocked:
                return "UnknownBlocked";
            case PathCheckStatus::OccupiedBlocked:
                return "OccupiedBlocked";
            case PathCheckStatus::InvalidInput:
                return "InvalidInput";
        }
        return "InvalidInput";
    }

    KnownFreePathChecker::KnownFreePathChecker(BodyEnvelopeConfig config)
        : config_(config)
    {
        const bool finite = std::isfinite(config_.robot_radius)
                            && std::isfinite(config_.robot_half_height)
                            && std::isfinite(config_.safety_margin)
                            && std::isfinite(config_.vertical_margin)
                            && std::isfinite(config_.sample_spacing_fraction);
        if(!finite || config_.robot_radius < 0.0F || config_.robot_half_height < 0.0F
           || config_.safety_margin < 0.0F || config_.vertical_margin < 0.0F
           || config_.sample_spacing_fraction <= 0.0F
           || config_.sample_spacing_fraction > 1.0F)
        {
            throw std::invalid_argument("invalid body envelope config");
        }
    }

    PathCheckResult KnownFreePathChecker::checkBody(
            const octomap::OcTree & tree, const Point3f & center) const
    {
        if(!isFinite(center) || !std::isfinite(tree.getResolution())
           || tree.getResolution() <= 0.0)
        {
            return {PathCheckStatus::InvalidInput, std::nullopt};
        }

        const float resolution       = static_cast<float>(tree.getResolution());
        const float horizontal_limit = requiredHorizontalClearance(resolution);
        const float vertical_limit   = requiredVerticalClearance(resolution);
        const float range = std::max(horizontal_limit, vertical_limit);

        octomap::OcTreeKey min_key;
        octomap::OcTreeKey max_key;
        if(!tree.coordToKeyChecked(
                   octomap::point3d(center.x - range, center.y - range, center.z - range),
                   min_key)
           || !tree.coordToKeyChecked(
                   octomap::point3d(center.x + range, center.y + range, center.z + range),
                   max_key))
        {
            return {PathCheckStatus::InvalidInput, std::nullopt};
        }

        for(std::uint32_t x = min_key[0]; x <= max_key[0]; ++x) {
            for(std::uint32_t y = min_key[1]; y <= max_key[1]; ++y) {
                for(std::uint32_t z = min_key[2]; z <= max_key[2]; ++z) {
                    const octomap::OcTreeKey key {
                            static_cast<octomap::key_type>(x),
                            static_cast<octomap::key_type>(y),
                            static_cast<octomap::key_type>(z),
                    };
                    const Point3f voxel = toPoint3f(tree.keyToCoord(key));
                    const float  dx    = voxel.x - center.x;
                    const float  dy    = voxel.y - center.y;
                    if(dx * dx + dy * dy > horizontal_limit * horizontal_limit
                       || std::fabs(voxel.z - center.z) > vertical_limit)
                    {
                        continue;
                    }

                    const octomap::OcTreeNode * node = tree.search(key);
                    if(node == nullptr) {
                        return {PathCheckStatus::UnknownBlocked, voxel};
                    }
                    if(tree.isNodeOccupied(node)) {
                        return {PathCheckStatus::OccupiedBlocked, voxel};
                    }
                }
            }
        }
        return {PathCheckStatus::Safe, std::nullopt};
    }

    PathCheckResult KnownFreePathChecker::checkSegment(
            const octomap::OcTree & tree, const Point3f & start, const Point3f & goal) const
    {
        if(!isFinite(start) || !isFinite(goal)) {
            return {PathCheckStatus::InvalidInput, std::nullopt};
        }

        const float dx       = goal.x - start.x;
        const float dy       = goal.y - start.y;
        const float dz       = goal.z - start.z;
        const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const float spacing =
                static_cast<float>(tree.getResolution()) * config_.sample_spacing_fraction;
        if(!std::isfinite(spacing) || spacing <= 0.0F) {
            return {PathCheckStatus::InvalidInput, std::nullopt};
        }

        const std::uint32_t steps =
                distance > 0.0F
                        ? static_cast<std::uint32_t>(std::ceil(distance / spacing))
                        : 0U;
        for(std::uint32_t i = 0; i <= steps; ++i) {
            const float fraction =
                    steps > 0U ? static_cast<float>(i) / static_cast<float>(steps) : 0.0F;
            const Point3f center {
                    start.x + dx * fraction,
                    start.y + dy * fraction,
                    start.z + dz * fraction,
            };
            PathCheckResult result = checkBody(tree, center);
            if(!result.safe()) {
                return result;
            }
        }
        return {PathCheckStatus::Safe, std::nullopt};
    }

    PathCheckResult KnownFreePathChecker::checkEgressSegment(
            const octomap::OcTree & tree, const Point3f & start, const Point3f & goal,
            const float max_initial_conflict_distance) const
    {
        if(!isFinite(start) || !isFinite(goal)
           || !std::isfinite(max_initial_conflict_distance)
           || max_initial_conflict_distance < 0.0F)
        {
            return {PathCheckStatus::InvalidInput, std::nullopt};
        }

        const float dx       = goal.x - start.x;
        const float dy       = goal.y - start.y;
        const float dz       = goal.z - start.z;
        const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        const float spacing =
                static_cast<float>(tree.getResolution()) * config_.sample_spacing_fraction;
        if(!std::isfinite(spacing) || spacing <= 0.0F) {
            return {PathCheckStatus::InvalidInput, std::nullopt};
        }

        const std::uint32_t steps =
                distance > 0.0F
                        ? static_cast<std::uint32_t>(std::ceil(distance / spacing))
                        : 0U;
        bool            reached_safe_body = false;
        PathCheckResult initial_conflict {PathCheckStatus::InvalidInput, std::nullopt};
        for(std::uint32_t i = 0; i <= steps; ++i) {
            const float fraction =
                    steps > 0U ? static_cast<float>(i) / static_cast<float>(steps) : 0.0F;
            const Point3f center {
                    start.x + dx * fraction,
                    start.y + dy * fraction,
                    start.z + dz * fraction,
            };
            PathCheckResult result = checkBody(tree, center);
            if(result.status == PathCheckStatus::InvalidInput) {
                return result;
            }
            if(result.safe()) {
                reached_safe_body = true;
                continue;
            }
            if(reached_safe_body || distance * fraction > max_initial_conflict_distance) {
                return result;
            }
            if(initial_conflict.status == PathCheckStatus::InvalidInput) {
                initial_conflict = result;
            }
        }
        if(!reached_safe_body) {
            return initial_conflict;
        }
        return {PathCheckStatus::Safe, std::nullopt};
    }

    float KnownFreePathChecker::requiredHorizontalClearance(const float map_resolution) const
    {
        if(!std::isfinite(map_resolution) || map_resolution <= 0.0F) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return config_.robot_radius + config_.safety_margin
               + map_resolution * std::sqrt(0.5F);
    }

    float KnownFreePathChecker::requiredVerticalClearance(const float map_resolution) const
    {
        if(!std::isfinite(map_resolution) || map_resolution <= 0.0F) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return config_.robot_half_height + config_.vertical_margin
               + map_resolution * 0.5F;
    }

    const BodyEnvelopeConfig & KnownFreePathChecker::config() const
    {
        return config_;
    }

}// namespace SwarmController
