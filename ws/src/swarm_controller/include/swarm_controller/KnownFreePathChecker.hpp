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

        const BodyEnvelopeConfig & config() const;

    private:
        BodyEnvelopeConfig config_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_KNOWNFREEPATHCHECKER_HPP
