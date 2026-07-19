#include "swarm_controller/LatestSnapshotSlot.hpp"

#include <gtest/gtest.h>

#include <string>

namespace SwarmController {

    TEST(LatestSnapshotSlotTest, NonDecreasingStampCoalescesPendingInput)
    {
        LatestSnapshotSlot<std::string, int> slot(
                SnapshotStampPolicy::NonDecreasing);

        EXPECT_EQ(
                slot.submit("first", 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        EXPECT_EQ(
                slot.submit("same", 10, 1.1),
                SnapshotSubmitStatus::SameStampAccepted);
        EXPECT_EQ(slot.latestAcceptedRevision(), 2U);
        EXPECT_EQ(slot.sameStampAcceptedCount(), 1U);
        EXPECT_EQ(slot.pendingCoalescedCount(), 1U);

        const auto pending = slot.takePending();
        ASSERT_TRUE(pending.has_value());
        EXPECT_EQ(pending->revision, 2U);
        EXPECT_EQ(pending->payload, "same");
        EXPECT_EQ(
                slot.submit("older", 9, 1.2),
                SnapshotSubmitStatus::RejectedRegressed);
        EXPECT_EQ(slot.latestAcceptedRevision(), 2U);
        EXPECT_EQ(slot.regressedRejectedCount(), 1U);
    }

    TEST(LatestSnapshotSlotTest, StrictPolicyRejectsDuplicateAndRegressedStamp)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::StrictlyIncreasing);

        EXPECT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        EXPECT_EQ(
                slot.submit(2, 10, 1.1),
                SnapshotSubmitStatus::RejectedDuplicate);
        EXPECT_EQ(
                slot.submit(3, 9, 1.2),
                SnapshotSubmitStatus::RejectedRegressed);
        EXPECT_EQ(slot.duplicateRejectedCount(), 1U);
        EXPECT_EQ(slot.regressedRejectedCount(), 1U);
    }

    TEST(LatestSnapshotSlotTest, InFlightResultCanAdvanceBeforeNewerPendingInput)
    {
        LatestSnapshotSlot<int, std::string> slot(
                SnapshotStampPolicy::NonDecreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto first = slot.takePending();
        ASSERT_TRUE(first.has_value());
        ASSERT_EQ(
                slot.submit(2, 11, 1.1),
                SnapshotSubmitStatus::Accepted);

        EXPECT_TRUE(slot.publishResult(
                first->revision, first->stamp_ns, first->received_seconds,
                "first-result"));
        const auto first_ready = slot.takeReady();
        ASSERT_TRUE(first_ready.has_value());
        EXPECT_EQ(first_ready->revision, 1U);
        EXPECT_EQ(slot.consumedRevision(), 1U);

        const auto second = slot.takePending();
        ASSERT_TRUE(second.has_value());
        EXPECT_TRUE(slot.publishResult(
                second->revision, second->stamp_ns, second->received_seconds,
                "second-result"));
        const auto second_ready = slot.takeReady();
        ASSERT_TRUE(second_ready.has_value());
        EXPECT_EQ(second_ready->revision, 2U);
        EXPECT_EQ(slot.consumedRevision(), 2U);
    }

    TEST(LatestSnapshotSlotTest, InFlightInputCoalescesToLatestPendingRevision)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::NonDecreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto first = slot.takePending();
        ASSERT_TRUE(first.has_value());
        ASSERT_EQ(
                slot.submit(2, 11, 1.1),
                SnapshotSubmitStatus::Accepted);
        ASSERT_EQ(
                slot.submit(3, 12, 1.2),
                SnapshotSubmitStatus::Accepted);
        EXPECT_EQ(slot.pendingCoalescedCount(), 1U);

        ASSERT_TRUE(slot.publishResult(
                first->revision, first->stamp_ns, first->received_seconds, 10));
        ASSERT_TRUE(slot.takeReady().has_value());
        const auto latest = slot.takePending();
        ASSERT_TRUE(latest.has_value());
        EXPECT_EQ(latest->revision, 3U);
        EXPECT_EQ(latest->payload, 3);
    }

    TEST(LatestSnapshotSlotTest, NewerInputWaitsForReadyAcknowledgement)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::NonDecreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto first = slot.takePending();
        ASSERT_TRUE(first.has_value());
        ASSERT_TRUE(slot.publishResult(
                first->revision, first->stamp_ns, first->received_seconds, 10));

        ASSERT_EQ(
                slot.submit(2, 11, 1.1),
                SnapshotSubmitStatus::Accepted);
        EXPECT_FALSE(slot.hasProcessablePending());
        EXPECT_EQ(slot.pendingRevision(), 2U);
        const auto ready = slot.takeReady();
        ASSERT_TRUE(ready.has_value());
        EXPECT_EQ(ready->revision, 1U);
        EXPECT_EQ(ready->payload, 10);
        EXPECT_TRUE(slot.hasProcessablePending());
        const auto next = slot.takePending();
        ASSERT_TRUE(next.has_value());
        EXPECT_EQ(next->revision, 2U);
        ASSERT_TRUE(slot.publishResult(
                next->revision, next->stamp_ns, next->received_seconds, 20));
        EXPECT_EQ(slot.takeReady()->payload, 20);
    }

    TEST(LatestSnapshotSlotTest, OlderCompletionCannotOverwriteConsumedOrReadyResult)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::NonDecreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto first = slot.takePending();
        ASSERT_TRUE(first.has_value());
        ASSERT_TRUE(slot.publishResult(
                first->revision, first->stamp_ns, first->received_seconds, 10));
        ASSERT_TRUE(slot.takeReady().has_value());

        EXPECT_FALSE(slot.publishResult(
                first->revision, first->stamp_ns, first->received_seconds, 11));
        EXPECT_EQ(slot.supersededResultCount(), 1U);
    }

    TEST(LatestSnapshotSlotTest, RejectedLatestResultRollsBackAdmissionWatermark)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::StrictlyIncreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto pending = slot.takePending();
        ASSERT_TRUE(pending.has_value());
        ASSERT_TRUE(slot.publishResult(
                pending->revision, pending->stamp_ns, pending->received_seconds, 10));
        ASSERT_TRUE(slot.takeReady().has_value());

        ASSERT_EQ(
                slot.submit(2, 11, 2.0),
                SnapshotSubmitStatus::Accepted);
        const auto rejected = slot.takePending();
        ASSERT_TRUE(rejected.has_value());
        ASSERT_TRUE(slot.publishResult(
                rejected->revision, rejected->stamp_ns, rejected->received_seconds, 0));
        const auto failed = slot.claimReady();
        ASSERT_TRUE(failed.has_value());
        EXPECT_TRUE(slot.acknowledgeReady(failed->revision, false));
        EXPECT_EQ(slot.latestAcceptedStampNs(), 10);
        EXPECT_EQ(slot.lastAppliedStampNs(), 10);
        EXPECT_EQ(
                slot.submit(3, 11, 3.0),
                SnapshotSubmitStatus::Accepted);
    }

    TEST(LatestSnapshotSlotTest, RejectedOlderResultPreservesNewerPendingWatermark)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::StrictlyIncreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto first = slot.takePending();
        ASSERT_TRUE(first.has_value());
        ASSERT_EQ(
                slot.submit(2, 20, 2.0),
                SnapshotSubmitStatus::Accepted);
        ASSERT_TRUE(slot.publishResult(
                first->revision, first->stamp_ns, first->received_seconds, 0));
        const auto rejected = slot.claimReady();
        ASSERT_TRUE(rejected.has_value());
        ASSERT_TRUE(slot.acknowledgeReady(rejected->revision, false));

        EXPECT_EQ(slot.latestAcceptedStampNs(), 20);
        EXPECT_EQ(
                slot.submit(3, 19, 2.1),
                SnapshotSubmitStatus::RejectedRegressed);
        const auto pending = slot.takePending();
        ASSERT_TRUE(pending.has_value());
        EXPECT_EQ(pending->revision, 2U);
    }

    TEST(LatestSnapshotSlotTest, ClearingTransientStatePreservesStampWatermark)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::StrictlyIncreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        ASSERT_TRUE(slot.takePending().has_value());
        slot.clear();

        EXPECT_EQ(slot.latestAcceptedStampNs(), 10);
        EXPECT_EQ(
                slot.submit(2, 9, 2.0),
                SnapshotSubmitStatus::RejectedRegressed);
        EXPECT_EQ(
                slot.submit(3, 10, 2.1),
                SnapshotSubmitStatus::RejectedDuplicate);
        EXPECT_EQ(
                slot.submit(4, 11, 2.2),
                SnapshotSubmitStatus::Accepted);
    }

    TEST(LatestSnapshotSlotTest, ReadyClaimRequiresExplicitAcknowledgement)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::NonDecreasing);
        ASSERT_EQ(
                slot.submit(1, 10, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto pending = slot.takePending();
        ASSERT_TRUE(pending.has_value());
        ASSERT_TRUE(slot.publishResult(
                pending->revision, pending->stamp_ns, pending->received_seconds, 10));
        const auto claimed = slot.claimReady();
        ASSERT_TRUE(claimed.has_value());
        EXPECT_TRUE(slot.readyClaimed());
        EXPECT_FALSE(slot.claimReady().has_value());
        EXPECT_FALSE(slot.acknowledgeReady(claimed->revision + 1U));
        EXPECT_TRUE(slot.acknowledgeReady(claimed->revision));
        EXPECT_FALSE(slot.hasReady());
    }

    TEST(LatestSnapshotSlotTest, ClockResetAcceptsLowerStampAndRejectsOldCompletion)
    {
        LatestSnapshotSlot<int, int> slot(
                SnapshotStampPolicy::StrictlyIncreasing);
        ASSERT_EQ(
                slot.submit(1, 100, 1.0),
                SnapshotSubmitStatus::Accepted);
        const auto old = slot.takePending();
        ASSERT_TRUE(old.has_value());

        slot.resetStampWatermark();
        EXPECT_EQ(slot.consumedRevision(), 1U);
        EXPECT_FALSE(slot.publishResult(
                old->revision, old->stamp_ns, old->received_seconds, 10));
        EXPECT_EQ(
                slot.submit(2, 50, 2.0),
                SnapshotSubmitStatus::Accepted);
        const auto current = slot.takePending();
        ASSERT_TRUE(current.has_value());
        EXPECT_EQ(current->revision, 2U);
        EXPECT_EQ(current->stamp_ns, 50);
    }

}// namespace SwarmController
