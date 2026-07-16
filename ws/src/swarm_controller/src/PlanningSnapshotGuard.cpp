#include "swarm_controller/PlanningSnapshotGuard.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace SwarmController {

    namespace {

        constexpr float PI = 3.14159265358979323846F;

        float yawDistance(const float lhs, const float rhs)
        {
            return std::fabs(std::remainder(lhs - rhs, 2.0F * PI));
        }

        bool samePoints(const std::vector<Point3f> & lhs, const std::vector<Point3f> & rhs)
        {
            return lhs.size() == rhs.size()
                   && std::equal(
                           lhs.begin(), lhs.end(), rhs.begin(),
                           [](const Point3f & left, const Point3f & right) {
                               return left.x == right.x && left.y == right.y
                                      && left.z == right.z;
                           });
        }

    }// namespace

    PlanningSnapshotGuard::PlanningSnapshotGuard(PlanningSnapshotConfig config)
        : config_(config)
    {
        if(!std::isfinite(config_.max_position_drift)
           || !std::isfinite(config_.max_yaw_drift)
           || !std::isfinite(config_.max_snapshot_age_seconds)
           || config_.max_position_drift < 0.0F || config_.max_yaw_drift < 0.0F
           || config_.max_snapshot_age_seconds <= 0.0)
        {
            throw std::invalid_argument("invalid planning snapshot config");
        }
    }

    PlanningSnapshotAssessment PlanningSnapshotGuard::assess(
            const PlanningSnapshot & before, const PlanningSnapshot & after) const
    {
        PlanningSnapshotAssessment result;
        result.map_changed = before.map_epoch != after.map_epoch
                             || before.map_stamp_ns != after.map_stamp_ns;
        const float dx = after.pose.position.x - before.pose.position.x;
        const float dy = after.pose.position.y - before.pose.position.y;
        const float dz = after.pose.position.z - before.pose.position.z;
        result.pose_changed = std::sqrt(dx * dx + dy * dy + dz * dz)
                                      > config_.max_position_drift
                              || yawDistance(after.pose.yaw, before.pose.yaw)
                                         > config_.max_yaw_drift;
        result.active_peer_goals_changed =
                !samePoints(before.active_peer_goals, after.active_peer_goals);
        result.task_changed = before.task_valid != after.task_valid
                              || before.task_mode != after.task_mode
                              || before.task_allocator_epoch != after.task_allocator_epoch
                              || before.task_revision != after.task_revision
                              || before.task_id != after.task_id
                              || before.task_target.x != after.task_target.x
                              || before.task_target.y != after.task_target.y
                              || before.task_target.z != after.task_target.z;
        result.age_seconds = std::max(
                0.0, after.monotonic_time_seconds - before.monotonic_time_seconds);
        result.age_exceeded = result.age_seconds > config_.max_snapshot_age_seconds;
        return result;
    }

    const PlanningSnapshotConfig & PlanningSnapshotGuard::config() const
    {
        return config_;
    }

}// namespace SwarmController
