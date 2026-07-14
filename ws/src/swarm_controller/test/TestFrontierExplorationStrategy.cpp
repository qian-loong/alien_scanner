#include "swarm_controller/FrontierExplorationStrategy.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

namespace SwarmController {

    namespace {

        constexpr float HALF_PI = 1.57079632679F;
        constexpr float PI      = 3.14159265359F;

        void fillFreeBox(
                octomap::OcTree & tree, const Point3f & minimum, const Point3f & maximum)
        {
            const octomap::OcTreeKey min_key = tree.coordToKey(
                    minimum.x, minimum.y, minimum.z);
            const octomap::OcTreeKey max_key = tree.coordToKey(
                    maximum.x, maximum.y, maximum.z);
            for(std::uint32_t x = min_key[0]; x <= max_key[0]; ++x) {
                for(std::uint32_t y = min_key[1]; y <= max_key[1]; ++y) {
                    for(std::uint32_t z = min_key[2]; z <= max_key[2]; ++z) {
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

        FrontierExplorationConfig compactConfig()
        {
            FrontierExplorationConfig config;
            config.forward_lookahead_min     = 1.0F;
            config.forward_lookahead_max     = 2.0F;
            config.forward_lateral_limit     = 1.0F;
            config.forward_distance_samples = 2U;
            config.forward_lateral_samples  = 3U;
            config.robot_radius              = 0.05F;
            config.robot_half_height         = 0.05F;
            config.safety_margin             = 0.0F;
            config.vertical_margin           = 0.0F;
            config.lateral_weight            = 0.5F;
            config.heading_weight            = 0.25F;
            return config;
        }

        Pose3D originPose(float yaw = 0.0F)
        {
            Pose3D pose;
            pose.yaw = yaw;
            return pose;
        }

    }// namespace

    TEST(FrontierExplorationStrategyTest, SelectsFarthestStraightKnownFreeCandidate)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});

        FrontierExplorationStrategy strategy(compactConfig());
        ExplorationDiagnostics diagnostics;
        const GoalSelectionResult result = strategy.selectGoal(
                GoalSelectionRequest {originPose(), {}}, tree, &diagnostics);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_NEAR(result.goal->position.x, 2.0F, 1e-5F);
        EXPECT_NEAR(result.goal->position.y, 0.0F, 1e-5F);
        EXPECT_LE(
                diagnostics.pre_peer_candidate_count,
                compactConfig().forward_distance_samples
                        * compactConfig().forward_lateral_samples);
        EXPECT_EQ(diagnostics.last_goal_status, "Success");
    }

    TEST(FrontierExplorationStrategyTest, UsesPreferredTravelYawInsteadOfPoseYaw)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-2.0F, -1.0F, -1.0F}, {2.0F, 3.0F, 1.0F});
        GoalSelectionRequest request {originPose(), {}};
        request.preferred_travel_yaw = HALF_PI;

        const GoalSelectionResult result =
                FrontierExplorationStrategy(compactConfig()).selectGoal(request, tree);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_NEAR(result.goal->position.x, 0.0F, 1e-5F);
        EXPECT_NEAR(result.goal->position.y, 2.0F, 1e-5F);
    }

    TEST(FrontierExplorationStrategyTest, UsesReachableLateralCandidateWhenCenterPathIsBlocked)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        tree.updateNode(octomap::point3d(1.5F, 0.0F, 0.0F), true, true);
        tree.updateInnerOccupancy();

        FrontierExplorationStrategy strategy(compactConfig());
        ExplorationDiagnostics diagnostics;
        const GoalSelectionResult result = strategy.selectGoal(
                GoalSelectionRequest {originPose(), {}}, tree, &diagnostics);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_GT(std::fabs(result.goal->position.y), 0.5F);
        EXPECT_GT(diagnostics.segment_check_count, 0U);
    }

    TEST(FrontierExplorationStrategyTest, ReportsStartBodyConflictForUnknownOrOccupiedBody)
    {
        FrontierExplorationStrategy strategy(compactConfig());
        octomap::OcTree unknown_tree(0.1);
        unknown_tree.updateNode(octomap::point3d(2.0F, 0.0F, 0.0F), false, true);
        unknown_tree.updateInnerOccupancy();
        EXPECT_EQ(
                strategy.selectGoal(GoalSelectionRequest {originPose(), {}}, unknown_tree).status,
                GoalSelectionStatus::StartBodyConflict);

        octomap::OcTree occupied_tree(0.1);
        fillFreeBox(occupied_tree, {-1.0F, -1.0F, -1.0F}, {2.0F, 1.0F, 1.0F});
        occupied_tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.0F), true, true);
        occupied_tree.updateInnerOccupancy();
        EXPECT_EQ(
                strategy.selectGoal(GoalSelectionRequest {originPose(), {}}, occupied_tree).status,
                GoalSelectionStatus::StartBodyConflict);
    }

    TEST(FrontierExplorationStrategyTest, FixedSampleCountBoundsAllCandidateWork)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        FrontierExplorationConfig config = compactConfig();
        config.forward_distance_samples = 3U;
        config.forward_lateral_samples  = 5U;
        ExplorationDiagnostics diagnostics;

        FrontierExplorationStrategy(config).selectGoal(
                GoalSelectionRequest {originPose(), {}}, tree, &diagnostics);

        const std::size_t maximum = config.forward_distance_samples
                                    * config.forward_lateral_samples;
        EXPECT_LE(diagnostics.pre_peer_candidate_count, maximum);
        EXPECT_LE(diagnostics.post_peer_candidate_count, maximum);
        EXPECT_LE(diagnostics.raw_candidate_count, maximum);
        EXPECT_LE(diagnostics.unique_candidate_count, maximum);
        EXPECT_LE(diagnostics.segment_check_count, maximum);
    }

    TEST(FrontierExplorationStrategyTest, ForwardHalfSpaceNeverReturnsBackwardGoal)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-3.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        GoalSelectionRequest request {originPose(PI), {}};
        request.preferred_travel_yaw = PI;
        request.forward_half_space = ForwardHalfSpaceConstraint {
                {0.0F, 0.0F, 0.0F},
                0.0F,
                0.0F,
        };

        const GoalSelectionResult result =
                FrontierExplorationStrategy(compactConfig()).selectGoal(request, tree);

        EXPECT_EQ(result.status, GoalSelectionStatus::NoSafeCandidate);
    }

    TEST(FrontierExplorationStrategyTest, PeerGoalHardSeparationSelectsUnclaimedOffset)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        FrontierExplorationConfig config = compactConfig();
        config.min_peer_goal_separation = 0.6F;
        GoalSelectionRequest request {originPose(), {}};
        request.active_peer_goals = {{2.0F, 0.0F, 0.0F}};

        const GoalSelectionResult result =
                FrontierExplorationStrategy(config).selectGoal(request, tree);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_GT(std::fabs(result.goal->position.y), 0.5F);
    }

    TEST(FrontierExplorationStrategyTest, ReportsPeerConflictWhenAllSamplesAreClaimed)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        FrontierExplorationConfig config = compactConfig();
        config.min_peer_goal_separation = 10.0F;
        GoalSelectionRequest request {originPose(), {}};
        request.active_peer_goals = {{2.0F, 0.0F, 0.0F}};
        ExplorationDiagnostics diagnostics;

        const GoalSelectionResult result =
                FrontierExplorationStrategy(config).selectGoal(request, tree, &diagnostics);

        EXPECT_EQ(result.status, GoalSelectionStatus::PeerGoalConflict);
        EXPECT_GT(diagnostics.pre_peer_candidate_count, 0U);
        EXPECT_EQ(diagnostics.post_peer_candidate_count, 0U);
        EXPECT_EQ(
                FrontierExplorationStrategy(config).selectGoal(request, tree).status,
                GoalSelectionStatus::PeerGoalConflict);
    }

    TEST(FrontierExplorationStrategyTest, SoftPeerPositionBiasesAwayFromPeer)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        FrontierExplorationConfig config = compactConfig();
        config.lateral_weight    = 0.0F;
        config.heading_weight    = 0.0F;
        config.dispersion_weight = 5.0F;
        GoalSelectionRequest request {originPose(), {}};
        request.peer_positions = {{2.0F, -1.0F, 0.0F}};

        const GoalSelectionResult result =
                FrontierExplorationStrategy(config).selectGoal(request, tree);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_GT(result.goal->position.y, 0.5F);
    }

    TEST(FrontierExplorationStrategyTest, ActivePositionAndGoalEachAddSoftPenalty)
    {
        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -2.0F, -1.0F}, {3.0F, 2.0F, 1.0F});
        FrontierExplorationConfig config = compactConfig();
        config.min_peer_goal_separation = 0.0F;
        GoalSelectionRequest position_only {originPose(), {}};
        position_only.peer_positions = {{2.0F, 0.0F, 0.0F}};
        GoalSelectionRequest position_and_goal = position_only;
        position_and_goal.active_peer_goals = {{2.0F, 0.0F, 0.0F}};
        FrontierExplorationStrategy strategy(config);

        const GoalSelectionResult once = strategy.selectGoal(position_only, tree);
        const GoalSelectionResult twice = strategy.selectGoal(position_and_goal, tree);

        ASSERT_TRUE(once.goal.has_value());
        ASSERT_TRUE(twice.goal.has_value());
        EXPECT_LT(twice.goal->utility, once.goal->utility);
    }

    TEST(FrontierExplorationStrategyTest, RejectsInvalidInputAndConfiguration)
    {
        FrontierExplorationConfig invalid = compactConfig();
        invalid.forward_distance_samples = 0U;
        EXPECT_THROW((void) FrontierExplorationStrategy {invalid}, std::invalid_argument);

        octomap::OcTree tree(0.1);
        fillFreeBox(tree, {-1.0F, -1.0F, -1.0F}, {3.0F, 1.0F, 1.0F});
        Pose3D pose = originPose();
        pose.position.x = std::numeric_limits<float>::quiet_NaN();
        EXPECT_EQ(
                FrontierExplorationStrategy(compactConfig())
                        .selectGoal(GoalSelectionRequest {pose, {}}, tree)
                        .status,
                GoalSelectionStatus::InvalidInput);
    }

}// namespace SwarmController
