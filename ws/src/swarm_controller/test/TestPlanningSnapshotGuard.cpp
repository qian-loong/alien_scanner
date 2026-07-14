#include "swarm_controller/PlanningSnapshotGuard.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace SwarmController {

    namespace {

        PlanningSnapshot snapshot()
        {
            PlanningSnapshot value;
            value.pose.position          = Point3f {1.0F, 2.0F, 3.0F};
            value.pose.yaw               = 0.25F;
            value.map_epoch              = 4U;
            value.map_stamp_ns           = 100;
            value.monotonic_time_seconds = 10.0;
            value.active_peer_goals      = {Point3f {5.0F, 6.0F, 1.0F}};
            return value;
        }

    }// namespace

    TEST(PlanningSnapshotGuardTest, StableSnapshotNeedsNoRevalidation)
    {
        const PlanningSnapshot before = snapshot();
        const auto assessment = PlanningSnapshotGuard {}.assess(before, before);

        EXPECT_FALSE(assessment.requiresRevalidation());
        EXPECT_DOUBLE_EQ(assessment.age_seconds, 0.0);
    }

    TEST(PlanningSnapshotGuardTest, DetectsEachMutablePlanningDependency)
    {
        PlanningSnapshotConfig config;
        config.max_position_drift       = 0.1F;
        config.max_yaw_drift            = 0.1F;
        config.max_snapshot_age_seconds = 1.0;
        const PlanningSnapshotGuard guard(config);
        const PlanningSnapshot before = snapshot();

        PlanningSnapshot changed = before;
        changed.map_epoch += 1U;
        EXPECT_TRUE(guard.assess(before, changed).map_changed);

        changed = before;
        changed.pose.position.x += 0.11F;
        EXPECT_TRUE(guard.assess(before, changed).pose_changed);

        changed = before;
        changed.pose.yaw += 0.11F;
        EXPECT_TRUE(guard.assess(before, changed).pose_changed);

        changed = before;
        changed.active_peer_goals.front().y += 0.1F;
        EXPECT_TRUE(guard.assess(before, changed).active_peer_goals_changed);

        changed = before;
        changed.monotonic_time_seconds += 1.01;
        EXPECT_TRUE(guard.assess(before, changed).age_exceeded);
    }

    TEST(PlanningSnapshotGuardTest, RejectsInvalidConfiguration)
    {
        PlanningSnapshotConfig config;
        config.max_snapshot_age_seconds = 0.0;
        EXPECT_THROW(PlanningSnapshotGuard {config}, std::invalid_argument);
    }

}// namespace SwarmController
