#include "swarm_controller/SingleDroneExplorer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <thread>
#include <utility>

namespace SwarmController {

    namespace {

        class SequenceStrategy final : public IExplorationStrategy
        {
        public:
            mutable std::vector<GoalSelectionRequest> requests;
            mutable std::deque<GoalSelectionResult>   results;

            GoalSelectionResult selectGoal(
                    const GoalSelectionRequest & request, const octomap::OcTree &,
                    ExplorationDiagnostics * diagnostics) const override
            {
                requests.push_back(request);
                if(diagnostics != nullptr) {
                    diagnostics->clear();
                }
                if(results.empty()) {
                    return {GoalSelectionStatus::NoSafeCandidate, std::nullopt};
                }
                GoalSelectionResult result = results.front();
                results.pop_front();
                return result;
            }
        };

        FrontierClusterId cluster(std::uint16_t x)
        {
            return octomap::OcTreeKey {x, 0U, 0U};
        }

        GoalSelectionResult goal(float x, float y, std::uint16_t cluster_key)
        {
            return {
                    GoalSelectionStatus::Success,
                    ExplorationGoal {
                            Point3f {x, y, 0.0F}, cluster(cluster_key), 1.0F, 1.0F},
            };
        }

        void fillFree(octomap::OcTree & tree)
        {
            const octomap::OcTreeKey minimum = tree.coordToKey(-1.0, -1.0, -1.0);
            const octomap::OcTreeKey maximum = tree.coordToKey(2.0, 2.0, 1.0);
            for(std::uint32_t x = minimum[0]; x <= maximum[0]; ++x) {
                for(std::uint32_t y = minimum[1]; y <= maximum[1]; ++y) {
                    for(std::uint32_t z = minimum[2]; z <= maximum[2]; ++z) {
                        tree.updateNode(
                                octomap::OcTreeKey {
                                        static_cast<octomap::key_type>(x),
                                        static_cast<octomap::key_type>(y),
                                        static_cast<octomap::key_type>(z),
                                },
                                false, true);
                    }
                }
            }
            tree.updateInnerOccupancy();
        }

        ExplorerInput input(
                const octomap::OcTree & tree, Pose3D pose = {},
                std::uint64_t epoch = 1U, std::int64_t stamp = 100,
                double monotonic = 0.0)
        {
            ExplorerInput value;
            value.pose                   = pose;
            value.map                    = &tree;
            value.observation_epoch      = epoch;
            value.observation_stamp_ns   = stamp;
            value.odom_stamp_ns          = stamp;
            value.monotonic_time_seconds = monotonic;
            return value;
        }

    }// namespace

    TEST(SingleDroneExplorerTest, CommandsSafeGoal)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorer explorer(strategy);

        const ExplorerTickResult result = explorer.tick(input(tree));

        EXPECT_EQ(result.state, ExplorerState::Moving);
        EXPECT_EQ(result.command.type, MotionCommandType::MoveTo);
        EXPECT_FLOAT_EQ(result.command.goal.position.x, 1.0F);
        EXPECT_EQ(explorer.diagnostics().path_status, "Safe");
    }

    TEST(SingleDroneExplorerTest, PassesPeerInputsToStrategy)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        SingleDroneExplorer explorer(strategy);
        ExplorerInput value = input(tree);
        value.peer_positions = {Point3f {1.0F, 2.0F, 0.0F}};
        value.active_peer_goals = {Point3f {3.0F, 4.0F, 0.0F}};

        explorer.tick(value);

        ASSERT_EQ(strategy->requests.size(), 1U);
        ASSERT_EQ(strategy->requests.front().peer_positions.size(), 1U);
        ASSERT_EQ(strategy->requests.front().active_peer_goals.size(), 1U);
        EXPECT_FLOAT_EQ(strategy->requests.front().peer_positions.front().y, 2.0F);
        EXPECT_FLOAT_EQ(strategy->requests.front().active_peer_goals.front().x, 3.0F);
    }

    TEST(SingleDroneExplorerTest, PeerConflictWaitsWithoutYawRescanAndRetriesOnNewMap)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::PeerGoalConflict, std::nullopt});
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorerConfig config;
        config.peer_retry_interval_seconds = 0.5;
        SingleDroneExplorer explorer(strategy, config);
        ExplorerInput value = input(tree);
        value.active_peer_goals = {Point3f {2.0F, 0.0F, 0.0F}};

        const ExplorerTickResult waiting = explorer.tick(value);
        ASSERT_EQ(waiting.state, ExplorerState::WaitingForPeer);
        ASSERT_EQ(waiting.command.type, MotionCommandType::Hold);
        EXPECT_FLOAT_EQ(waiting.command.goal.yaw, value.pose.yaw);

        value.observation_epoch       = 2U;
        value.observation_stamp_ns    = 200;
        value.monotonic_time_seconds  = 0.1;
        EXPECT_EQ(explorer.tick(value).state, ExplorerState::Selecting);
        EXPECT_EQ(strategy->requests.size(), 1U);

        value.monotonic_time_seconds = 0.2;
        EXPECT_EQ(explorer.tick(value).state, ExplorerState::Moving);
        EXPECT_EQ(strategy->requests.size(), 2U);
    }

    TEST(SingleDroneExplorerTest, PeerGoalChangeWakesWaitingImmediately)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::PeerGoalConflict, std::nullopt});
        SingleDroneExplorer explorer(strategy);
        ExplorerInput value = input(tree);
        value.active_peer_goals = {Point3f {2.0F, 0.0F, 0.0F}};
        ASSERT_EQ(explorer.tick(value).state, ExplorerState::WaitingForPeer);

        value.active_peer_goals = {Point3f {2.0F, 1.0F, 0.0F}};
        value.monotonic_time_seconds = 0.1;
        EXPECT_EQ(explorer.tick(value).state, ExplorerState::Selecting);
    }

    TEST(SingleDroneExplorerTest, PeerWaitRetriesWithoutMapChange)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::PeerGoalConflict, std::nullopt});
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorerConfig config;
        config.peer_retry_interval_seconds = 0.5;
        SingleDroneExplorer explorer(strategy, config);
        ExplorerInput value = input(tree);
        value.active_peer_goals = {Point3f {2.0F, 0.0F, 0.0F}};
        ASSERT_EQ(explorer.tick(value).state, ExplorerState::WaitingForPeer);

        value.monotonic_time_seconds = 0.6;
        EXPECT_EQ(explorer.tick(value).state, ExplorerState::Selecting);
        value.monotonic_time_seconds = 0.7;
        EXPECT_EQ(explorer.tick(value).state, ExplorerState::Moving);
        EXPECT_EQ(strategy->requests.size(), 2U);
    }

    TEST(SingleDroneExplorerTest, RejectsUnsafeClusterAndSelectsNextGoal)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        tree.updateNode(octomap::point3d(0.8F, 0.0F, 0.0F), true, true);
        tree.updateInnerOccupancy();
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        strategy->results.push_back(goal(0.0F, 1.5F, 2U));
        SingleDroneExplorer explorer(strategy);

        const ExplorerTickResult result = explorer.tick(input(tree));

        ASSERT_EQ(result.command.type, MotionCommandType::MoveTo);
        EXPECT_FLOAT_EQ(result.command.goal.position.y, 1.5F);
        ASSERT_EQ(strategy->requests.size(), 2U);
        ASSERT_EQ(strategy->requests[1].rejected_cluster_ids.size(), 1U);
        EXPECT_EQ(strategy->requests[1].rejected_cluster_ids.front(), cluster(1U));
    }

    TEST(SingleDroneExplorerTest, LocksEntryConstraintAndUsesCurrentFixedAltitude)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorer explorer(strategy);

        Pose3D entry;
        entry.position = Point3f {0.2F, -0.3F, 0.0F};
        entry.yaw      = 0.25F;
        const ExplorerTickResult rescan = explorer.tick(input(tree, entry));
        ASSERT_EQ(rescan.state, ExplorerState::Rescanning);

        Pose3D reached = rescan.command.goal;
        reached.position.z = 0.5F;
        ExplorerInput reached_input = input(tree, reached, 1U, 100, 0.5);
        reached_input.odom_stamp_ns = 200;
        explorer.tick(reached_input);
        explorer.tick(input(tree, reached, 2U, 201, 0.6));
        explorer.tick(input(tree, reached, 2U, 201, 0.7));

        ASSERT_EQ(strategy->requests.size(), 2U);
        ASSERT_TRUE(strategy->requests[0].forward_half_space.has_value());
        ASSERT_TRUE(strategy->requests[1].forward_half_space.has_value());
        EXPECT_FLOAT_EQ(
                strategy->requests[1].forward_half_space->origin.x,
                entry.position.x);
        EXPECT_FLOAT_EQ(
                strategy->requests[1].forward_half_space->origin.y,
                entry.position.y);
        EXPECT_FLOAT_EQ(strategy->requests[1].forward_half_space->yaw, entry.yaw);
        ASSERT_TRUE(strategy->requests[1].fixed_altitude.has_value());
        EXPECT_FLOAT_EQ(strategy->requests[1].fixed_altitude->altitude, reached.position.z);
    }

    TEST(SingleDroneExplorerTest, CanDisableEntryForwardConstraint)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorerConfig config;
        config.enforce_entry_forward_half_space = false;
        SingleDroneExplorer explorer(strategy, config);

        explorer.tick(input(tree));

        ASSERT_EQ(strategy->requests.size(), 1U);
        EXPECT_FALSE(strategy->requests.front().forward_half_space.has_value());
        EXPECT_TRUE(strategy->requests.front().fixed_altitude.has_value());
    }

    TEST(SingleDroneExplorerTest, AdvancesNoRetreatPlaneWithReachedForwardProgress)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        SingleDroneExplorer explorer(strategy);
        explorer.tick(input(tree));

        Pose3D reached;
        reached.position.x = 1.0F;
        ExplorerInput arrival = input(tree, reached, 1U, 100, 1.0);
        arrival.odom_stamp_ns = 200;
        explorer.tick(arrival);
        explorer.tick(input(tree, reached, 2U, 201, 1.1));
        explorer.tick(input(tree, reached, 2U, 201, 1.2));

        ASSERT_EQ(strategy->requests.size(), 2U);
        ASSERT_TRUE(strategy->requests[1].forward_half_space.has_value());
        EXPECT_FLOAT_EQ(strategy->requests[1].forward_half_space->origin.x, 1.0F);
        EXPECT_FLOAT_EQ(strategy->requests[1].forward_half_space->origin.y, 0.0F);
        EXPECT_FLOAT_EQ(strategy->requests[1].forward_half_space->yaw, 0.0F);
    }

    TEST(SingleDroneExplorerTest, RejectsNegativeEntryBackwardMargin)
    {
        auto strategy = std::make_shared<SequenceStrategy>();
        SingleDroneExplorerConfig config;
        config.entry_backward_margin = -0.1F;

        EXPECT_THROW(SingleDroneExplorer(strategy, config), std::invalid_argument);
    }

    TEST(SingleDroneExplorerTest, RejectsNonPositivePeerRetryInterval)
    {
        auto strategy = std::make_shared<SequenceStrategy>();
        SingleDroneExplorerConfig config;
        config.peer_retry_interval_seconds = 0.0;

        EXPECT_THROW(SingleDroneExplorer(strategy, config), std::invalid_argument);
    }

    TEST(SingleDroneExplorerTest, ArrivalRequiresEpochAndStampToBeFresh)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorer explorer(strategy);
        explorer.tick(input(tree));

        Pose3D reached;
        reached.position.x = 1.0F;
        ExplorerInput arrival = input(tree, reached, 1U, 100, 1.0);
        arrival.odom_stamp_ns = 200;
        EXPECT_EQ(
                explorer.tick(arrival).state,
                ExplorerState::AwaitingFreshObservation);

        EXPECT_EQ(
                explorer.tick(input(tree, reached, 2U, 200, 1.1)).state,
                ExplorerState::AwaitingFreshObservation);
        EXPECT_EQ(
                explorer.tick(input(tree, reached, 3U, 201, 1.2)).state,
                ExplorerState::Selecting);
    }

    TEST(SingleDroneExplorerTest, FreshBlockedPathStopsBeforeReplanning)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorer explorer(strategy);
        explorer.tick(input(tree));

        tree.updateNode(octomap::point3d(0.5F, 0.0F, 0.0F), true, true);
        tree.updateInnerOccupancy();
        ExplorerInput blocked = input(tree, Pose3D {}, 2U, 200, 0.5);
        const ExplorerTickResult stop = explorer.tick(blocked);
        EXPECT_EQ(stop.state, ExplorerState::Stopping);
        EXPECT_EQ(stop.command.type, MotionCommandType::Hold);
        ASSERT_EQ(explorer.rejectedClusterIds().size(), 1U);
        EXPECT_EQ(explorer.rejectedClusterIds().front(), cluster(1U));

        const ExplorerTickResult stopped = explorer.tick(
                input(tree, stop.command.goal, 2U, 200, 0.6));
        EXPECT_EQ(stopped.state, ExplorerState::RecoveringClearance);
        EXPECT_EQ(stopped.command.type, MotionCommandType::Hold);
    }

    TEST(SingleDroneExplorerTest, OccupiedStartRecoversWhenBodyBecomesSafe)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        tree.updateNode(octomap::point3d(0.5F, 0.0F, 0.0F), true, true);
        tree.updateInnerOccupancy();
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::StartBodyConflict, std::nullopt});
        SingleDroneExplorer explorer(strategy);

        const ExplorerTickResult conflict = explorer.tick(input(tree));
        ASSERT_EQ(conflict.state, ExplorerState::RecoveringClearance);
        ASSERT_EQ(conflict.command.type, MotionCommandType::Hold);

        tree.deleteNode(octomap::point3d(0.5F, 0.0F, 0.0F));
        tree.updateNode(octomap::point3d(0.5F, 0.0F, 0.0F), false, true);
        tree.updateInnerOccupancy();
        const ExplorerTickResult recovered = explorer.tick(
                input(tree, conflict.command.goal, 2U, 200, 0.1));
        EXPECT_EQ(recovered.state, ExplorerState::Selecting);
        EXPECT_EQ(recovered.command.type, MotionCommandType::None);
    }

    TEST(SingleDroneExplorerTest, OccupiedBodyHoldsWithoutHistoricalEgress)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorerConfig config;
        SingleDroneExplorer explorer(strategy, config);
        ASSERT_EQ(explorer.tick(input(tree)).state, ExplorerState::Moving);

        Pose3D blocked_pose;
        blocked_pose.position.x = 0.7F;
        tree.updateNode(octomap::point3d(0.7F, 0.0F, 0.0F), true, true);
        tree.updateInnerOccupancy();
        const ExplorerTickResult stopping = explorer.tick(
                input(tree, blocked_pose, 2U, 200, 0.2));
        ASSERT_EQ(stopping.state, ExplorerState::Stopping);
        const ExplorerTickResult recovery = explorer.tick(
                input(tree, stopping.command.goal, 2U, 200, 0.3));
        ASSERT_EQ(recovery.state, ExplorerState::RecoveringClearance);

        const ExplorerTickResult holding = explorer.tick(
                input(tree, stopping.command.goal, 2U, 200, 0.5));
        ASSERT_EQ(holding.state, ExplorerState::RecoveringClearance);
        EXPECT_NE(holding.command.type, MotionCommandType::MoveTo);
        EXPECT_EQ(explorer.diagnostics().path_status, "OccupiedBlocked");
    }

    TEST(SingleDroneExplorerTest, ClearanceRecoveryTimeoutFailsExplicitly)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        tree.updateNode(octomap::point3d(0.5F, 0.0F, 0.0F), true, true);
        tree.updateInnerOccupancy();
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::StartBodyConflict, std::nullopt});
        SingleDroneExplorerConfig config;
        config.clearance_recovery_timeout_seconds = 0.1;
        SingleDroneExplorer explorer(strategy, config);
        ASSERT_EQ(explorer.tick(input(tree)).state, ExplorerState::RecoveringClearance);

        const ExplorerTickResult timed_out = explorer.tick(
                input(tree, Pose3D {}, 1U, 100, 0.2));
        EXPECT_EQ(timed_out.state, ExplorerState::Stopping);
        EXPECT_EQ(timed_out.command.type, MotionCommandType::Hold);
        EXPECT_EQ(
                explorer.diagnostics().failure_reason,
                "body clearance recovery exhausted");
    }

    TEST(SingleDroneExplorerTest, UnknownStartBodyUsesYawRescan)
    {
        octomap::OcTree tree(0.1);
        tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.0F), false, true);
        tree.updateInnerOccupancy();
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::StartBodyConflict, std::nullopt});
        SingleDroneExplorer explorer(strategy);

        const ExplorerTickResult result = explorer.tick(input(tree));
        EXPECT_EQ(result.state, ExplorerState::Rescanning);
        EXPECT_EQ(result.command.type, MotionCommandType::MoveTo);
        EXPECT_NE(result.command.goal.yaw, 0.0F);
    }

    TEST(SingleDroneExplorerTest, RevalidatesPendingMoveAgainstLatestSnapshot)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorer explorer(strategy);
        ExplorerTickResult pending = explorer.tick(input(tree));
        ASSERT_EQ(pending.command.type, MotionCommandType::MoveTo);

        ExplorerInput shifted = input(tree, Pose3D {}, 2U, 200, 0.1);
        shifted.pose.position.x = 0.05F;
        shifted.pose.position.z = 0.2F;
        EXPECT_TRUE(explorer.revalidatePendingResult(shifted, pending));
        EXPECT_FLOAT_EQ(pending.command.goal.position.z, 0.2F);

        tree.updateNode(octomap::point3d(0.6F, 0.0F, 0.2F), true, true);
        tree.updateInnerOccupancy();
        ExplorerInput blocked = input(tree, shifted.pose, 3U, 300, 0.2);
        EXPECT_FALSE(explorer.revalidatePendingResult(blocked, pending));
    }

    TEST(SingleDroneExplorerTest, RescanAlsoUsesPostReachFreshGate)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        SingleDroneExplorer explorer(strategy);

        const ExplorerTickResult command = explorer.tick(input(tree));
        ASSERT_EQ(command.state, ExplorerState::Rescanning);
        ASSERT_EQ(command.command.type, MotionCommandType::MoveTo);

        Pose3D reached = command.command.goal;
        ExplorerInput reached_input = input(tree, reached, 1U, 100, 0.5);
        reached_input.odom_stamp_ns = 200;
        EXPECT_EQ(
                explorer.tick(reached_input).state,
                ExplorerState::Rescanning);
        EXPECT_EQ(
                explorer.tick(input(tree, reached, 2U, 200, 0.6)).state,
                ExplorerState::Rescanning);
        EXPECT_EQ(
                explorer.tick(input(tree, reached, 3U, 201, 0.7)).state,
                ExplorerState::Selecting);
        explorer.tick(input(tree, reached, 3U, 201, 0.8));
        ASSERT_GE(strategy->requests.size(), 2U);
        ASSERT_TRUE(strategy->requests.back().preferred_travel_yaw.has_value());
        EXPECT_NEAR(*strategy->requests.back().preferred_travel_yaw, 0.0F, 1.0e-5F);
    }

    TEST(SingleDroneExplorerTest, ExhaustedRescanStallsAndRetriesAfterMapStructureChanges)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        strategy->results.push_back(goal(1.0F, 0.0F, 3U));
        SingleDroneExplorerConfig config;
        config.rescan_max_steps = 1U;
        SingleDroneExplorer explorer(strategy, config);

        const ExplorerTickResult rescan = explorer.tick(input(tree));
        ASSERT_EQ(rescan.state, ExplorerState::Rescanning);
        ExplorerInput reached = input(tree, rescan.command.goal, 1U, 100, 0.1);
        reached.odom_stamp_ns = 200;
        explorer.tick(reached);
        explorer.tick(input(tree, rescan.command.goal, 2U, 201, 0.2));

        const ExplorerTickResult stalled = explorer.tick(
                input(tree, rescan.command.goal, 2U, 201, 0.3));
        ASSERT_EQ(stalled.state, ExplorerState::ExplorationStalled);
        ASSERT_EQ(stalled.command.type, MotionCommandType::Hold);
        EXPECT_NE(
                explorer.diagnostics().failure_reason.find("sensor-limited hold"),
                std::string::npos);

        EXPECT_EQ(
                explorer.tick(input(tree, stalled.command.goal, 3U, 300, 0.4)).state,
                ExplorerState::ExplorationStalled);
        tree.updateNode(octomap::point3d(4.0F, 4.0F, 4.0F), false, true);
        tree.updateInnerOccupancy();
        EXPECT_EQ(
                explorer.tick(input(tree, stalled.command.goal, 4U, 400, 0.9)).state,
                ExplorerState::Selecting);
        const ExplorerTickResult resumed = explorer.tick(
                input(tree, stalled.command.goal, 4U, 400, 1.0));
        ASSERT_EQ(resumed.state, ExplorerState::Moving);
        EXPECT_FLOAT_EQ(resumed.command.goal.position.x, 1.0F);
    }

    TEST(SingleDroneExplorerTest, ExhaustedRescanNeverCommandsHistoricalBackwardMove)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.2F, 0.0F, 1U));
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        strategy->results.push_back(
                {GoalSelectionStatus::NoSafeCandidate, std::nullopt});
        SingleDroneExplorerConfig config;
        config.rescan_max_steps = 1U;
        SingleDroneExplorer explorer(strategy, config);

        ASSERT_EQ(explorer.tick(input(tree)).state, ExplorerState::Moving);
        Pose3D reached;
        reached.position.x = 1.2F;
        ExplorerInput arrival = input(tree, reached, 1U, 100, 0.2);
        arrival.odom_stamp_ns = 200;
        explorer.tick(arrival);
        explorer.tick(input(tree, reached, 2U, 201, 0.3));
        const ExplorerTickResult rescan = explorer.tick(
                input(tree, reached, 2U, 201, 0.4));
        ASSERT_EQ(rescan.state, ExplorerState::Rescanning);

        ExplorerInput yaw_reached = input(tree, rescan.command.goal, 2U, 201, 0.5);
        yaw_reached.odom_stamp_ns = 300;
        explorer.tick(yaw_reached);
        explorer.tick(input(tree, rescan.command.goal, 3U, 301, 0.6));
        const ExplorerTickResult stalled = explorer.tick(
                input(tree, rescan.command.goal, 3U, 301, 0.7));

        ASSERT_EQ(stalled.state, ExplorerState::ExplorationStalled);
        ASSERT_EQ(stalled.command.type, MotionCommandType::Hold);
        EXPECT_NEAR(stalled.command.goal.position.x, reached.position.x, 1e-5F);
        EXPECT_NE(stalled.command.type, MotionCommandType::MoveTo);
    }

    TEST(SingleDroneExplorerTest, HoldTimeoutFailsSafely)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);
        auto strategy = std::make_shared<SequenceStrategy>();
        strategy->results.push_back(goal(1.0F, 0.0F, 1U));
        SingleDroneExplorerConfig config;
        config.motion_timeout_seconds = 0.1;
        config.hold_timeout_seconds   = 0.1;
        SingleDroneExplorer explorer(strategy, config);
        explorer.tick(input(tree));

        ExplorerInput timed_out = input(tree, Pose3D {}, 1U, 100, 0.2);
        timed_out.linear_velocity.x = 1.0F;
        EXPECT_EQ(explorer.tick(timed_out).state, ExplorerState::Stopping);

        timed_out.monotonic_time_seconds = 0.4;
        EXPECT_EQ(explorer.tick(timed_out).state, ExplorerState::HoveringFailure);
    }

    TEST(SingleDroneExplorerTest, LongSelectDoesNotBurnRescanMotionTimeout)
    {
        octomap::OcTree tree(0.1);
        fillFree(tree);

        class SlowNoCandidateStrategy final : public IExplorationStrategy
        {
        public:
            GoalSelectionResult selectGoal(
                    const GoalSelectionRequest &, const octomap::OcTree &,
                    ExplorationDiagnostics * diagnostics) const override
            {
                if(diagnostics != nullptr) {
                    diagnostics->clear();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                return {GoalSelectionStatus::NoSafeCandidate, std::nullopt};
            }
        };

        SingleDroneExplorerConfig config;
        config.motion_timeout_seconds = 0.05;
        config.map_stale_timeout_seconds = 30.0;
        SingleDroneExplorer explorer(std::make_shared<SlowNoCandidateStrategy>(), config);

        const ExplorerTickResult first = explorer.tick(input(tree, Pose3D {}, 1U, 100, 0.0));
        ASSERT_EQ(first.state, ExplorerState::Rescanning);
        ASSERT_EQ(first.command.type, MotionCommandType::MoveTo);

        // mono 只前进 10ms（小于 timeout），但 select 已墙钟阻塞 200ms。
        // 若用 tick 开始时刻当 state_start，下一拍会误判超时；effectiveNow 应避免。
        const ExplorerTickResult second =
                explorer.tick(input(tree, first.command.goal, 2U, 200, 0.01));
        EXPECT_EQ(second.state, ExplorerState::Rescanning);
        EXPECT_NE(second.command.type, MotionCommandType::Hold);
        EXPECT_EQ(explorer.diagnostics().failure_reason.find("yaw rescan timeout"), std::string::npos);
    }

}// namespace SwarmController
