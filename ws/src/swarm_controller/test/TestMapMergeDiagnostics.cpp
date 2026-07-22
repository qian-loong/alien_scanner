#include "swarm_controller/MapMergeDiagnostics.hpp"

#include <gtest/gtest.h>

namespace SwarmController {

    TEST(MapMergeDiagnosticsTest, ComputesAgesOnlyInsideOneRosClockEpoch)
    {
        const MergeHeaderAge valid = mergeHeaderAge(
                10'000'000'000LL, 13'500'000'000LL, 4U, 4U);
        EXPECT_TRUE(valid.valid);
        EXPECT_DOUBLE_EQ(valid.seconds, 3.5);

        const MergeHeaderAge future = mergeHeaderAge(
                14'000'000'000LL, 13'500'000'000LL, 4U, 4U);
        EXPECT_FALSE(future.valid);
        EXPECT_EQ(future.seconds, -1.0);

        const MergeHeaderAge rollback = mergeHeaderAge(
                10'000'000'000LL, 13'500'000'000LL, 3U, 4U);
        EXPECT_FALSE(rollback.valid);
        EXPECT_EQ(rollback.seconds, -1.0);

        const MergeHeaderAge invalid_stamp = mergeHeaderAge(0, 1, 4U, 4U);
        EXPECT_FALSE(invalid_stamp.valid);
        EXPECT_EQ(invalid_stamp.seconds, -1.0);
    }

    TEST(MapMergeDiagnosticsTest, UsesExplicitPointsForSteadyDurations)
    {
        EXPECT_DOUBLE_EQ(mergeElapsedSeconds(10.0, 13.25), 3.25);
        EXPECT_EQ(mergeElapsedSeconds(13.0, 12.0), -1.0);
        EXPECT_EQ(mergeElapsedSeconds(-1.0, 12.0), -1.0);
    }

    TEST(MapMergeDiagnosticsTest, AccountsForEveryMeasuredCycleStage)
    {
        MergeCycleStageDurations stages;
        stages.claim_seconds = 0.01;
        stages.decode_seconds = 0.10;
        stages.decode_cleanup_seconds = 0.05;
        stages.normalize_seconds = 0.20;
        stages.snapshot_compare_seconds = 0.05;
        stages.delta_preflight_seconds = 0.10;
        stages.contribution_tree_apply_seconds = 0.15;
        stages.update_inner_occupancy_seconds = 0.10;
        stages.source_commit_seconds = 0.01;
        stages.bookkeeping_seconds = 0.01;
        stages.serialize_seconds = 0.15;
        stages.output_prepare_seconds = 0.01;
        stages.publish_seconds = 0.01;

        stages.finishAccounting(1.05);

        EXPECT_NEAR(stages.accountedSeconds(), 0.95, 1.0e-12);
        EXPECT_DOUBLE_EQ(stages.cycle_total_seconds, 1.05);
        EXPECT_NEAR(stages.accounting_remainder_seconds, 0.10, 1.0e-12);
    }

    TEST(MapMergeDiagnosticsTest, SerializationFailureKeepsAppliedStateDistinctFromPublish)
    {
        const MergeCycleCompletion failed = completeMergeCycle(
                2U, 0U, false, false);
        EXPECT_TRUE(failed.merge_applied);
        EXPECT_FALSE(failed.published);
        EXPECT_STREQ(failed.outcome, "serialization_failure");

        const MergeCycleCompletion rejected = completeMergeCycle(
                0U, 2U, false, false);
        EXPECT_FALSE(rejected.merge_applied);
        EXPECT_FALSE(rejected.published);
        EXPECT_STREQ(rejected.outcome, "all_sources_rejected");

        const MergeCycleCompletion partial = completeMergeCycle(
                1U, 1U, true, true);
        EXPECT_TRUE(partial.merge_applied);
        EXPECT_TRUE(partial.published);
        EXPECT_STREQ(partial.outcome, "partial_accept");
    }

}// namespace SwarmController
