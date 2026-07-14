#include "swarm_controller/PeerStateTracker.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace SwarmController {

    namespace {

        constexpr float NAN_F = std::numeric_limits<float>::quiet_NaN();

    }// namespace

    TEST(PeerStateTrackerTest, RejectsInvalidConfig)
    {
        EXPECT_THROW(
                PeerStateTracker(1U, PeerStateConfig {0.0, 25.0, 0.2F}), std::invalid_argument);
        EXPECT_THROW(
                PeerStateTracker(1U, PeerStateConfig {2.0, -1.0, 0.2F}), std::invalid_argument);
        EXPECT_THROW(
                PeerStateTracker(1U, PeerStateConfig {2.0, 25.0, -0.1F}), std::invalid_argument);
    }

    TEST(PeerStateTrackerTest, FreshPositionContributesSoftPenalty)
    {
        PeerStateTracker tracker(1U);
        tracker.updatePosition(0U, Point3f {1.0F, 2.0F, 0.0F}, 0.0);

        const auto snap = tracker.snapshot(1.0);

        ASSERT_EQ(snap.peer_positions.size(), 1U);
        EXPECT_FLOAT_EQ(snap.peer_positions.front().x, 1.0F);
        EXPECT_FLOAT_EQ(snap.peer_positions.front().y, 2.0F);
        EXPECT_EQ(snap.fresh_positions, 1U);
        EXPECT_EQ(snap.stale_positions, 0U);
        EXPECT_TRUE(snap.active_peer_goals.empty());
    }

    TEST(PeerStateTrackerTest, StalePositionRemovesSoftAndHard)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {2.0, 25.0, 0.2F});
        tracker.updatePosition(0U, Point3f {1.0F, 0.0F, 0.0F}, 0.0);
        tracker.updateGoal(0U, Point3f {5.0F, 0.0F, 0.0F}, 0.0);// far goal, would be active

        const auto snap = tracker.snapshot(3.0);// now - 0 = 3 > position_timeout 2

        EXPECT_TRUE(snap.peer_positions.empty());
        EXPECT_TRUE(snap.active_peer_goals.empty());
        EXPECT_EQ(snap.fresh_positions, 0U);
        EXPECT_EQ(snap.stale_positions, 1U);
    }

    TEST(PeerStateTrackerTest, FreshGoalFarFromPositionIsActive)
    {
        PeerStateTracker tracker(1U);
        tracker.updatePosition(0U, Point3f {0.0F, 0.0F, 0.0F}, 0.0);
        tracker.updateGoal(0U, Point3f {3.0F, 0.0F, 0.0F}, 0.0);

        const auto snap = tracker.snapshot(1.0);

        ASSERT_EQ(snap.active_peer_goals.size(), 1U);
        EXPECT_FLOAT_EQ(snap.active_peer_goals.front().x, 3.0F);
        EXPECT_EQ(snap.active_goals, 1U);
        EXPECT_EQ(snap.fresh_goals, 1U);
    }

    TEST(PeerStateTrackerTest, StaleGoalKeepsPositionRemovesHard)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {100.0, 5.0, 0.2F});
        tracker.updatePosition(0U, Point3f {0.0F, 0.0F, 0.0F}, 0.0);
        tracker.updateGoal(0U, Point3f {3.0F, 0.0F, 0.0F}, 0.0);

        const auto snap = tracker.snapshot(6.0);// goal 6>5 stale; position 6<100 fresh

        ASSERT_EQ(snap.peer_positions.size(), 1U);// 软惩罚保留
        EXPECT_TRUE(snap.active_peer_goals.empty());// 硬约束移除
        EXPECT_EQ(snap.stale_goals, 1U);
        EXPECT_EQ(snap.active_goals, 0U);
    }

    TEST(PeerStateTrackerTest, ContinuousOdomDoesNotExtendStaleGoal)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {100.0, 5.0, 0.2F});
        tracker.updateGoal(0U, Point3f {3.0F, 0.0F, 0.0F}, 0.0);
        // odom 持续新鲜，但目标未刷新
        tracker.updatePosition(0U, Point3f {0.0F, 0.0F, 0.0F}, 6.0);

        const auto snap = tracker.snapshot(6.0);// goal 6>5 stale，虽然位置新鲜

        EXPECT_EQ(snap.peer_positions.size(), 1U);
        EXPECT_TRUE(snap.active_peer_goals.empty());
    }

    TEST(PeerStateTrackerTest, HoldGoalIsNotActive)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {2.0, 25.0, 0.2F});
        tracker.updatePosition(0U, Point3f {1.0F, 1.0F, 0.0F}, 0.0);
        tracker.updateGoal(0U, Point3f {1.1F, 1.0F, 0.0F}, 0.0);// 0.1 < 0.2 → Hold

        const auto snap = tracker.snapshot(1.0);

        EXPECT_EQ(snap.peer_positions.size(), 1U);
        EXPECT_TRUE(snap.active_peer_goals.empty());
        EXPECT_EQ(snap.fresh_goals, 1U);
        EXPECT_EQ(snap.active_goals, 0U);
    }

    TEST(PeerStateTrackerTest, BoundaryAtTimeoutStillFresh)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {2.0, 25.0, 0.2F});
        tracker.updatePosition(0U, Point3f {1.0F, 0.0F, 0.0F}, 0.0);

        const auto snap = tracker.snapshot(2.0);// 恰好等于 timeout 仍视为新鲜

        EXPECT_EQ(snap.fresh_positions, 1U);
        EXPECT_EQ(snap.stale_positions, 0U);
    }

    TEST(PeerStateTrackerTest, NewMessageRecoversFromStale)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {2.0, 25.0, 0.2F});
        tracker.updatePosition(0U, Point3f {1.0F, 0.0F, 0.0F}, 0.0);
        EXPECT_TRUE(tracker.snapshot(5.0).peer_positions.empty());// 过期

        tracker.updatePosition(0U, Point3f {1.0F, 0.0F, 0.0F}, 5.0);// 新消息
        EXPECT_EQ(tracker.snapshot(5.5).peer_positions.size(), 1U);// 恢复新鲜
    }

    TEST(PeerStateTrackerTest, NewGoalRecoversFromStale)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {100.0, 2.0, 0.2F});
        tracker.updatePosition(0U, Point3f {}, 0.0);
        tracker.updateGoal(0U, Point3f {2.0F, 0.0F, 0.0F}, 0.0);
        EXPECT_TRUE(tracker.snapshot(3.0).active_peer_goals.empty());

        tracker.updateGoal(0U, Point3f {3.0F, 0.0F, 0.0F}, 3.0);
        const auto recovered = tracker.snapshot(3.1);
        ASSERT_EQ(recovered.active_peer_goals.size(), 1U);
        EXPECT_FLOAT_EQ(recovered.active_peer_goals.front().x, 3.0F);
    }

    TEST(PeerStateTrackerTest, IgnoresOutOfRangeAndNonFinite)
    {
        PeerStateTracker tracker(1U);
        tracker.updatePosition(5U, Point3f {1.0F, 0.0F, 0.0F}, 0.0);// 越界
        tracker.updatePosition(0U, Point3f {NAN_F, 0.0F, 0.0F}, 0.0);// 非有限

        EXPECT_TRUE(tracker.snapshot(0.1).peer_positions.empty());
    }

    TEST(PeerStateTrackerTest, RejectsNonFiniteSnapshotTime)
    {
        PeerStateTracker tracker(1U);

        EXPECT_THROW(
                tracker.snapshot(std::numeric_limits<double>::quiet_NaN()),
                std::invalid_argument);
        EXPECT_THROW(tracker.snapshot(-0.1), std::invalid_argument);
    }

    TEST(PeerStateTrackerTest, OlderUpdatesDoNotRegressState)
    {
        PeerStateTracker tracker(1U, PeerStateConfig {2.0, 5.0, 0.2F});
        tracker.updatePosition(0U, Point3f {2.0F, 0.0F, 0.0F}, 4.0);
        tracker.updatePosition(0U, Point3f {1.0F, 0.0F, 0.0F}, 3.0);
        tracker.updateGoal(0U, Point3f {4.0F, 0.0F, 0.0F}, 4.0);
        tracker.updateGoal(0U, Point3f {3.0F, 0.0F, 0.0F}, 3.0);

        const auto snap = tracker.snapshot(4.5);

        ASSERT_EQ(snap.peer_positions.size(), 1U);
        ASSERT_EQ(snap.active_peer_goals.size(), 1U);
        EXPECT_FLOAT_EQ(snap.peer_positions.front().x, 2.0F);
        EXPECT_FLOAT_EQ(snap.active_peer_goals.front().x, 4.0F);
    }

    TEST(PeerStateTrackerTest, FreshnessCountsAreIndependentAndComplete)
    {
        PeerStateTracker tracker(3U, PeerStateConfig {2.0, 5.0, 0.2F});
        tracker.updatePosition(0U, Point3f {}, 10.0);
        tracker.updateGoal(0U, Point3f {1.0F, 0.0F, 0.0F}, 4.0);
        tracker.updateGoal(1U, Point3f {2.0F, 0.0F, 0.0F}, 10.0);

        const auto snap = tracker.snapshot(10.0);

        EXPECT_EQ(snap.configured_peers, 3U);
        EXPECT_EQ(snap.fresh_positions, 1U);
        EXPECT_EQ(snap.missing_positions, 2U);
        EXPECT_EQ(snap.stale_positions, 0U);
        EXPECT_EQ(snap.fresh_goals, 1U);
        EXPECT_EQ(snap.stale_goals, 1U);
        EXPECT_EQ(snap.missing_goals, 1U);
        EXPECT_EQ(snap.active_goals, 0U);
    }

    TEST(PeerStateTrackerTest, ZeroPeersProducesEmptySnapshot)
    {
        PeerStateTracker tracker(0U);

        const auto snap = tracker.snapshot(1.0);

        EXPECT_TRUE(snap.peer_positions.empty());
        EXPECT_TRUE(snap.active_peer_goals.empty());
        EXPECT_EQ(snap.configured_peers, 0U);
        EXPECT_EQ(snap.fresh_positions, 0U);
    }

}// namespace SwarmController
