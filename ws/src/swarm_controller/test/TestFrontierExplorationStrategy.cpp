#include "swarm_controller/FrontierExplorationStrategy.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace SwarmController {

    namespace {

        octomap::OcTreeKey offsetKey(
                const octomap::OcTreeKey & base, const int dx, const int dy, const int dz)
        {
            return octomap::OcTreeKey {
                    static_cast<octomap::key_type>(static_cast<int>(base[0]) + dx),
                    static_cast<octomap::key_type>(static_cast<int>(base[1]) + dy),
                    static_cast<octomap::key_type>(static_cast<int>(base[2]) + dz),
            };
        }

        Pose3D poseAtKey(const octomap::OcTree & tree, const octomap::OcTreeKey & key)
        {
            const octomap::point3d point = tree.keyToCoord(key);
            return Pose3D {Point3f {point.x(), point.y(), point.z()}, 0.0F};
        }

        struct CellUpdate {
            octomap::OcTreeKey key;
            bool               occupied;
        };

        void populateOpenCorridorAt(
                octomap::OcTree & tree, const octomap::OcTreeKey & base, const int length,
                const int half_width, const int half_height, const bool reverse_order = false)
        {
            std::vector<CellUpdate> updates;
            for(int x = 0; x <= length; ++x) {
                for(int y = -half_width; y <= half_width; ++y) {
                    for(int z = -half_height; z <= half_height; ++z) {
                        updates.push_back(CellUpdate {offsetKey(base, x, y, z), false});
                    }
                }
            }

            for(int y = -half_width; y <= half_width; ++y) {
                for(int z = -half_height; z <= half_height; ++z) {
                    updates.push_back(CellUpdate {offsetKey(base, -1, y, z), true});
                }
            }
            for(int x = -1; x <= length; ++x) {
                for(int z = -half_height; z <= half_height; ++z) {
                    updates.push_back(
                            CellUpdate {offsetKey(base, x, -half_width - 1, z), true});
                    updates.push_back(
                            CellUpdate {offsetKey(base, x, half_width + 1, z), true});
                }
                for(int y = -half_width; y <= half_width; ++y) {
                    updates.push_back(
                            CellUpdate {offsetKey(base, x, y, -half_height - 1), true});
                    updates.push_back(
                            CellUpdate {offsetKey(base, x, y, half_height + 1), true});
                }
            }

            if(reverse_order) {
                std::reverse(updates.begin(), updates.end());
            }
            for(const CellUpdate & update : updates) {
                tree.updateNode(update.key, update.occupied, true);
            }
            tree.updateInnerOccupancy();
        }

        void populateOpenCorridor(
                octomap::OcTree & tree, const int length, const int half_width,
                const int half_height, const bool reverse_order = false)
        {
            populateOpenCorridorAt(
                    tree, tree.coordToKey(0.0, 0.0, 0.0), length, half_width, half_height,
                    reverse_order);
        }

        void surroundFreeCell(
                octomap::OcTree & tree, const octomap::OcTreeKey & center,
                const bool leave_top_unknown)
        {
            tree.updateNode(center, false, true);
            const std::array<std::array<int, 3>, 6> offsets {{
                    {{-1, 0, 0}},
                    {{1, 0, 0}},
                    {{0, -1, 0}},
                    {{0, 1, 0}},
                    {{0, 0, -1}},
                    {{0, 0, 1}},
            }};
            for(std::size_t i = 0; i < offsets.size(); ++i) {
                if(leave_top_unknown && i == offsets.size() - 1U) {
                    continue;
                }
                tree.updateNode(
                        offsetKey(center, offsets[i][0], offsets[i][1], offsets[i][2]), true,
                        true);
            }
            tree.updateInnerOccupancy();
        }

        bool keysEqual(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs)
        {
            return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
        }

    }// namespace

    TEST(FrontierExplorationStrategyTest, SelectsKnownFreeStandoffInsideOpenCorridor)
    {
        octomap::OcTree tree(0.2);
        populateOpenCorridor(tree, 15, 5, 3);
        const octomap::OcTreeKey base = tree.coordToKey(0.0, 0.0, 0.0);
        const Pose3D             pose = poseAtKey(tree, base);

        const GoalSelectionResult result =
                FrontierExplorationStrategy {}.selectGoal(GoalSelectionRequest {pose, {}}, tree);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_GT(result.goal->position.x, pose.position.x + 1.0F);
        EXPECT_LT(result.goal->position.x, pose.position.x + 3.1F);
        EXPECT_GE(result.goal->frontier_area, 0.2F);
        const octomap::OcTreeNode * node = tree.search(octomap::point3d(
                result.goal->position.x, result.goal->position.y, result.goal->position.z));
        ASSERT_NE(node, nullptr);
        EXPECT_FALSE(tree.isNodeOccupied(node));
    }

    TEST(FrontierExplorationStrategyTest, RejectedClusterIsNotReturnedAgain)
    {
        octomap::OcTree tree(0.2);
        const auto base = tree.coordToKey(0.0, 0.0, 0.0);
        populateOpenCorridorAt(tree, base, 15, 5, 3);
        populateOpenCorridorAt(tree, offsetKey(base, 0, 12, 0), 15, 5, 3);
        const Pose3D pose = poseAtKey(tree, base);
        FrontierExplorationStrategy strategy;

        const GoalSelectionResult first =
                strategy.selectGoal(GoalSelectionRequest {pose, {}}, tree);
        ASSERT_EQ(first.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(first.goal.has_value());
        EXPECT_LT(first.goal->position.y, 1.2F);

        const GoalSelectionResult rejected = strategy.selectGoal(
                GoalSelectionRequest {pose, {first.goal->cluster_id}}, tree);
        ASSERT_EQ(rejected.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(rejected.goal.has_value());
        EXPECT_FALSE(keysEqual(first.goal->cluster_id, rejected.goal->cluster_id));
        EXPECT_GT(rejected.goal->position.y, 1.2F);
    }

    TEST(FrontierExplorationStrategyTest, ReportsEmptyAndClosedMapStates)
    {
        FrontierExplorationStrategy strategy;
        octomap::OcTree             empty_tree(0.2);
        const Pose3D                origin_pose {Point3f {0.0F, 0.0F, 0.0F}, 0.0F};
        EXPECT_EQ(
                strategy.selectGoal(GoalSelectionRequest {origin_pose, {}}, empty_tree).status,
                GoalSelectionStatus::NoKnownFree);

        octomap::OcTree closed_tree(0.2);
        const auto      center = closed_tree.coordToKey(0.0, 0.0, 0.0);
        surroundFreeCell(closed_tree, center, false);
        EXPECT_EQ(
                strategy
                        .selectGoal(
                                GoalSelectionRequest {poseAtKey(closed_tree, center), {}},
                                closed_tree)
                        .status,
                GoalSelectionStatus::NoFrontier);
    }

    TEST(FrontierExplorationStrategyTest, FiltersVerticalOnlyFrontier)
    {
        octomap::OcTree tree(0.2);
        const auto      base = tree.coordToKey(0.0, 0.0, 0.0);
        for(int x = -3; x <= 3; ++x) {
            for(int y = -3; y <= 3; ++y) {
                for(int z = 0; z <= 10; ++z) {
                    tree.updateNode(offsetKey(base, x, y, z), false, true);
                }
                tree.updateNode(offsetKey(base, x, y, -1), true, true);
            }
        }
        for(int z = 0; z <= 10; ++z) {
            for(int axis = -3; axis <= 3; ++axis) {
                tree.updateNode(offsetKey(base, -4, axis, z), true, true);
                tree.updateNode(offsetKey(base, 4, axis, z), true, true);
                tree.updateNode(offsetKey(base, axis, -4, z), true, true);
                tree.updateNode(offsetKey(base, axis, 4, z), true, true);
            }
        }
        tree.updateInnerOccupancy();

        const GoalSelectionResult result = FrontierExplorationStrategy {}.selectGoal(
                GoalSelectionRequest {poseAtKey(tree, base), {}}, tree);

        EXPECT_EQ(result.status, GoalSelectionStatus::NoSafeCandidate);
        EXPECT_FALSE(result.goal.has_value());

        FrontierExplorationConfig vertical_config;
        vertical_config.max_abs_frontier_normal_z = 1.0F;
        EXPECT_EQ(
                FrontierExplorationStrategy {vertical_config}
                        .selectGoal(GoalSelectionRequest {poseAtKey(tree, base), {}}, tree)
                        .status,
                GoalSelectionStatus::Success);
    }

    TEST(FrontierExplorationStrategyTest, TreatsUnknownInsideBodyEnvelopeAsUnsafe)
    {
        octomap::OcTree tree(0.2);
        const auto      center = tree.coordToKey(0.0, 0.0, 0.0);
        for(int x = 0; x <= 15; ++x) {
            tree.updateNode(offsetKey(center, x, 0, 0), false, true);
        }
        tree.updateInnerOccupancy();

        const GoalSelectionResult result = FrontierExplorationStrategy {}.selectGoal(
                GoalSelectionRequest {poseAtKey(tree, center), {}}, tree);

        EXPECT_EQ(result.status, GoalSelectionStatus::NoSafeCandidate);
    }

    TEST(FrontierExplorationStrategyTest, FiltersFrontierBelowPhysicalAreaThreshold)
    {
        octomap::OcTree tree(0.2);
        const auto      base = tree.coordToKey(0.0, 0.0, 0.0);
        populateOpenCorridorAt(tree, base, 15, 5, 3);
        for(int y = -5; y <= 5; ++y) {
            for(int z = -3; z <= 3; ++z) {
                if(y != 0 || z != 0) {
                    tree.updateNode(offsetKey(base, 15, y, z), true, true);
                }
            }
        }
        tree.updateInnerOccupancy();

        FrontierExplorationConfig filtered_config;
        filtered_config.goal_standoff = 0.8F;
        const GoalSelectionResult result =
                FrontierExplorationStrategy {filtered_config}.selectGoal(
                        GoalSelectionRequest {poseAtKey(tree, base), {}}, tree);

        EXPECT_EQ(result.status, GoalSelectionStatus::NoSafeCandidate);

        FrontierExplorationConfig permissive_config = filtered_config;
        permissive_config.min_cluster_area           = 0.01F;
        EXPECT_EQ(
                FrontierExplorationStrategy {permissive_config}
                        .selectGoal(GoalSelectionRequest {poseAtKey(tree, base), {}}, tree)
                        .status,
                GoalSelectionStatus::Success);
    }

    TEST(FrontierExplorationStrategyTest, DoesNotReturnFrontierInsideMinimumGoalDistance)
    {
        octomap::OcTree tree(0.2);
        populateOpenCorridor(tree, 2, 5, 3);
        const auto center = tree.coordToKey(0.0, 0.0, 0.0);

        const GoalSelectionResult result = FrontierExplorationStrategy {}.selectGoal(
                GoalSelectionRequest {poseAtKey(tree, center), {}}, tree);

        EXPECT_EQ(result.status, GoalSelectionStatus::NoSafeCandidate);
    }

    TEST(FrontierExplorationStrategyTest, UsesIndividualFacesWhenClusterNormalsCancel)
    {
        octomap::OcTree tree(0.2);
        const auto      base = tree.coordToKey(0.0, 0.0, 0.0);
        for(int x = -10; x <= 10; ++x) {
            for(int y = -5; y <= 5; ++y) {
                for(int z = -3; z <= 3; ++z) {
                    tree.updateNode(offsetKey(base, x, y, z), false, true);
                }
            }
        }
        for(int x = -10; x <= 10; ++x) {
            for(int z = -3; z <= 3; ++z) {
                tree.updateNode(offsetKey(base, x, -6, z), true, true);
            }
            for(int y = -5; y <= 5; ++y) {
                tree.updateNode(offsetKey(base, x, y, -4), true, true);
                tree.updateNode(offsetKey(base, x, y, 4), true, true);
            }
        }
        tree.updateInnerOccupancy();

        const GoalSelectionResult result = FrontierExplorationStrategy {}.selectGoal(
                GoalSelectionRequest {poseAtKey(tree, base), {}}, tree);

        ASSERT_EQ(result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(result.goal.has_value());
        EXPECT_GT(result.goal->position.x, 0.8F);
    }

    TEST(FrontierExplorationStrategyTest, UsesPhysicalAreaAcrossResolutions)
    {
        FrontierExplorationStrategy strategy;
        octomap::OcTree             coarse_tree(0.2);
        octomap::OcTree             fine_tree(0.1);
        populateOpenCorridor(coarse_tree, 15, 5, 3);
        populateOpenCorridor(fine_tree, 30, 10, 6);

        const auto coarse_result = strategy.selectGoal(
                GoalSelectionRequest {
                        poseAtKey(coarse_tree, coarse_tree.coordToKey(0.0, 0.0, 0.0)), {}},
                coarse_tree);
        const auto fine_result = strategy.selectGoal(
                GoalSelectionRequest {
                        poseAtKey(fine_tree, fine_tree.coordToKey(0.0, 0.0, 0.0)), {}},
                fine_tree);

        ASSERT_EQ(coarse_result.status, GoalSelectionStatus::Success);
        ASSERT_EQ(fine_result.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(coarse_result.goal.has_value());
        ASSERT_TRUE(fine_result.goal.has_value());
        EXPECT_NEAR(coarse_result.goal->frontier_area, 3.08F, 1.0e-4F);
        EXPECT_NEAR(fine_result.goal->frontier_area, 2.73F, 1.0e-4F);
    }

    TEST(FrontierExplorationStrategyTest, SelectionIsIndependentOfInsertionOrder)
    {
        octomap::OcTree forward_tree(0.2);
        octomap::OcTree reverse_tree(0.2);
        populateOpenCorridor(forward_tree, 15, 5, 3, false);
        populateOpenCorridor(reverse_tree, 15, 5, 3, true);
        FrontierExplorationStrategy strategy;

        const auto forward = strategy.selectGoal(
                GoalSelectionRequest {
                        poseAtKey(forward_tree, forward_tree.coordToKey(0.0, 0.0, 0.0)), {}},
                forward_tree);
        const auto reverse = strategy.selectGoal(
                GoalSelectionRequest {
                        poseAtKey(reverse_tree, reverse_tree.coordToKey(0.0, 0.0, 0.0)), {}},
                reverse_tree);

        ASSERT_EQ(forward.status, GoalSelectionStatus::Success);
        ASSERT_EQ(reverse.status, GoalSelectionStatus::Success);
        ASSERT_TRUE(forward.goal.has_value());
        ASSERT_TRUE(reverse.goal.has_value());
        EXPECT_TRUE(keysEqual(forward.goal->cluster_id, reverse.goal->cluster_id));
        EXPECT_FLOAT_EQ(forward.goal->position.x, reverse.goal->position.x);
        EXPECT_FLOAT_EQ(forward.goal->position.y, reverse.goal->position.y);
        EXPECT_FLOAT_EQ(forward.goal->position.z, reverse.goal->position.z);
    }

    TEST(FrontierExplorationStrategyTest, RejectsInvalidInputAndConfiguration)
    {
        octomap::OcTree tree(0.2);
        const float     nan = std::numeric_limits<float>::quiet_NaN();
        const auto result = FrontierExplorationStrategy {}.selectGoal(
                GoalSelectionRequest {Pose3D {Point3f {nan, 0.0F, 0.0F}, 0.0F}, {}}, tree);
        EXPECT_EQ(result.status, GoalSelectionStatus::InvalidInput);

        FrontierExplorationConfig config;
        config.planning_radius = 1.0F;
        EXPECT_THROW(FrontierExplorationStrategy {config}, std::invalid_argument);
    }

}// namespace SwarmController
