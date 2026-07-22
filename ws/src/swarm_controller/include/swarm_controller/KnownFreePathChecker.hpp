#ifndef SWARM_CONTROLLER_KNOWNFREEPATHCHECKER_HPP
#define SWARM_CONTROLLER_KNOWNFREEPATHCHECKER_HPP

#include "swarm_controller/Point3f.hpp"

#include <optional>

#include <octomap/OcTree.h>

namespace SwarmController {

    struct BodyEnvelopeConfig {
        float robot_radius {0.25F};
        float robot_half_height {0.15F};
        float safety_margin {0.25F};
        float vertical_margin {0.20F};
        float sample_spacing_fraction {0.50F};
    };

    enum class PathCheckStatus {
        Safe,
        UnknownBlocked,
        OccupiedBlocked,
        InvalidInput,
    };

    const char * pathCheckStatusName(PathCheckStatus status);

    struct PathCheckResult {
        PathCheckStatus       status {PathCheckStatus::InvalidInput};
        std::optional<Point3f> first_blocked_position;

        bool safe() const
        {
            return status == PathCheckStatus::Safe;
        }
    };

    class KnownFreePathChecker
    {
    public:
        explicit KnownFreePathChecker(BodyEnvelopeConfig config = {});

        PathCheckResult checkBody(const octomap::OcTree & tree, const Point3f & center) const;
        PathCheckResult checkSegment(
                const octomap::OcTree & tree, const Point3f & start,
                const Point3f & goal) const;
        /// 仅用于从“当前包络已冲突”状态退出：允许起点附近连续冲突，
        /// 但必须在 max_initial_conflict_distance 内进入 known-free，之后整段保持安全。
        PathCheckResult checkEgressSegment(
                const octomap::OcTree & tree, const Point3f & start,
                const Point3f & goal, float max_initial_conflict_distance) const;

        float requiredHorizontalClearance(float map_resolution) const;
        float requiredVerticalClearance(float map_resolution) const;

        const BodyEnvelopeConfig & config() const;

    private:
        BodyEnvelopeConfig config_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_KNOWNFREEPATHCHECKER_HPP
