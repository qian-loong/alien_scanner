#include "swarm_controller/TaskLeaseTracker.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace SwarmController {

    namespace {

        ExplorationTaskControl assigned(
                const std::int64_t issued_ns = 1'000'000'000LL,
                const std::uint64_t epoch = 7U, const std::uint64_t revision = 1U,
                const std::uint64_t task_id = 11U)
        {
            ExplorationTaskControl task;
            task.issued_time_ns    = issued_ns;
            task.lease_duration_ns = 3'000'000'000LL;
            task.allocator_epoch   = epoch;
            task.revision          = revision;
            task.task_id           = task_id;
            task.mode              = ExplorationTaskMode::Assigned;
            task.target            = Point3f {4.0F, 1.0F, 1.5F};
            return task;
        }

    }// namespace

    TEST(TaskLeaseTrackerTest, RejectsInvalidConfig)
    {
        EXPECT_THROW(TaskLeaseTracker(TaskLeaseTrackerConfig {0.0}), std::invalid_argument);
    }

    TEST(TaskLeaseTrackerTest, AcceptsNewTaskAndAbsoluteLeaseExpires)
    {
        TaskLeaseTracker tracker;
        EXPECT_EQ(
                tracker.update(assigned(), 1'100'000'000LL, 0.1).status,
                TaskUpdateStatus::AcceptedNew);
        const auto fresh = tracker.snapshot(3'999'999'999LL, 2.9);
        EXPECT_TRUE(fresh.valid);
        EXPECT_EQ(fresh.control.task_id, 11U);
        EXPECT_FALSE(tracker.snapshot(4'000'000'000LL, 3.0).valid);
    }

    TEST(TaskLeaseTrackerTest, RenewalKeepsSemanticRevision)
    {
        TaskLeaseTracker tracker;
        ASSERT_TRUE(tracker.update(assigned(), 1'100'000'000LL, 0.1).accepted());
        auto renewal = assigned(2'000'000'000LL);
        EXPECT_EQ(
                tracker.update(renewal, 2'100'000'000LL, 1.1).status,
                TaskUpdateStatus::AcceptedRenewal);
        EXPECT_TRUE(tracker.snapshot(4'900'000'000LL, 3.8).valid);
    }

    TEST(TaskLeaseTrackerTest, RejectsExpiredTransientReplay)
    {
        TaskLeaseTracker tracker;
        EXPECT_EQ(
                tracker.update(assigned(), 5'000'000'000LL, 0.0).status,
                TaskUpdateStatus::RejectedStale);
        EXPECT_FALSE(tracker.snapshot(5'000'000'000LL, 0.0).valid);
    }

    TEST(TaskLeaseTrackerTest, RejectsChangedPayloadWithoutRevision)
    {
        TaskLeaseTracker tracker;
        ASSERT_TRUE(tracker.update(assigned(), 1'100'000'000LL, 0.0).accepted());
        auto changed = assigned(2'000'000'000LL);
        changed.target.y = 2.0F;
        EXPECT_EQ(
                tracker.update(changed, 2'100'000'000LL, 1.0).status,
                TaskUpdateStatus::RejectedInvalid);
    }

    TEST(TaskLeaseTrackerTest, RejectsRevisionAndOldEpochReplay)
    {
        TaskLeaseTracker tracker;
        ASSERT_TRUE(tracker.update(assigned(2'000'000'000LL, 7U, 2U), 2'100'000'000LL, 0.0)
                            .accepted());
        EXPECT_EQ(
                tracker.update(assigned(3'000'000'000LL, 7U, 1U), 3'100'000'000LL, 1.0)
                        .status,
                TaskUpdateStatus::RejectedStale);
        EXPECT_EQ(
                tracker.update(assigned(1'500'000'000LL, 8U, 1U), 2'200'000'000LL, 1.1)
                        .status,
                TaskUpdateStatus::RejectedStale);
    }

    TEST(TaskLeaseTrackerTest, NewerEpochTakesControl)
    {
        TaskLeaseTracker tracker;
        ASSERT_TRUE(tracker.update(assigned(), 1'100'000'000LL, 0.0).accepted());
        const auto replacement = assigned(2'000'000'000LL, 9U, 1U, 3U);
        EXPECT_EQ(
                tracker.update(replacement, 2'100'000'000LL, 1.0).status,
                TaskUpdateStatus::AcceptedNew);
        EXPECT_EQ(tracker.snapshot(2'200'000'000LL, 1.1).control.allocator_epoch, 9U);
    }

    TEST(TaskLeaseTrackerTest, ExplicitFallbackRevokesAssignedTask)
    {
        TaskLeaseTracker tracker;
        ASSERT_TRUE(tracker.update(assigned(), 1'100'000'000LL, 0.0).accepted());
        auto fallback = assigned(2'000'000'000LL, 7U, 2U, 0U);
        fallback.mode = ExplorationTaskMode::LocalFallback;
        EXPECT_TRUE(tracker.update(fallback, 2'100'000'000LL, 1.0).accepted());
        const auto snapshot = tracker.snapshot(2'200'000'000LL, 1.1);
        ASSERT_TRUE(snapshot.valid);
        EXPECT_EQ(snapshot.control.mode, ExplorationTaskMode::LocalFallback);
        EXPECT_EQ(snapshot.control.task_id, 0U);
    }

    TEST(TaskLeaseTrackerTest, ReceiveWatchdogIsSecondaryExpiry)
    {
        TaskLeaseTracker tracker(TaskLeaseTrackerConfig {2.0});
        ASSERT_TRUE(tracker.update(assigned(), 1'100'000'000LL, 1.0).accepted());
        EXPECT_TRUE(tracker.snapshot(2'000'000'000LL, 3.0).valid);
        EXPECT_FALSE(tracker.snapshot(2'000'000'000LL, 3.0001).valid);
    }

    TEST(TaskLeaseTrackerTest, RejectsOverflowingLease)
    {
        TaskLeaseTracker tracker;
        auto task = assigned(std::numeric_limits<std::int64_t>::max() - 10);
        EXPECT_EQ(
                tracker.update(task, 1, 0.0).status,
                TaskUpdateStatus::RejectedStale);
    }

}// namespace SwarmController
