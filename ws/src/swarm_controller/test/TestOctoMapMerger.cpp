#include "swarm_controller/OctoMapMerger.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <string>

namespace SwarmController {

    namespace {

        enum class QueryState {
            Unknown,
            Free,
            Occupied,
        };

        void setState(
                octomap::OcTree & tree, const float x, const float y, const float z,
                const bool occupied)
        {
            tree.setNodeValue(
                    octomap::point3d(x, y, z),
                    occupied ? tree.getClampingThresMaxLog()
                             : tree.getClampingThresMinLog(),
                    true);
            tree.updateInnerOccupancy();
        }

        QueryState query(
                const octomap::OcTree & tree, const float x, const float y, const float z)
        {
            const octomap::OcTreeNode * node = tree.search(x, y, z);
            if(node == nullptr) {
                return QueryState::Unknown;
            }
            return tree.isNodeOccupied(node) ? QueryState::Occupied : QueryState::Free;
        }

        OctoMapMergerConfig config(
                const std::size_t per_source = 1'000U,
                const std::size_t global = 2'000U)
        {
            OctoMapMergerConfig result;
            result.resolution             = 0.1;
            result.max_voxels_per_source  = per_source;
            result.max_global_voxels      = global;
            return result;
        }

        octomap::OcTreeKey makePrunedBlock(
                octomap::OcTree & tree, const bool occupied)
        {
            octomap::OcTreeKey base = tree.coordToKey(
                    octomap::point3d(0.0F, 0.0F, 0.0F));
            base[0] = static_cast<octomap::key_type>(
                    base[0] & ~octomap::key_type {1U});
            base[1] = static_cast<octomap::key_type>(
                    base[1] & ~octomap::key_type {1U});
            base[2] = static_cast<octomap::key_type>(
                    base[2] & ~octomap::key_type {1U});
            for(octomap::key_type x = 0U; x < 2U; ++x) {
                for(octomap::key_type y = 0U; y < 2U; ++y) {
                    for(octomap::key_type z = 0U; z < 2U; ++z) {
                        tree.setNodeValue(
                                octomap::OcTreeKey {
                                        static_cast<octomap::key_type>(base[0] + x),
                                        static_cast<octomap::key_type>(base[1] + y),
                                        static_cast<octomap::key_type>(base[2] + z),
                                },
                                occupied ? tree.getClampingThresMaxLog()
                                         : tree.getClampingThresMinLog(),
                                true);
                    }
                }
            }
            tree.updateInnerOccupancy();
            tree.prune();
            return base;
        }

    }// namespace

    TEST(OctoMapMergerTest, RejectsInvalidConfiguration)
    {
        OctoMapMergerConfig invalid = config();
        invalid.resolution = 0.0;
        EXPECT_THROW((void) OctoMapMerger {invalid}, std::invalid_argument);

        invalid = config();
        invalid.max_voxels_per_source = 0U;
        EXPECT_THROW((void) OctoMapMerger {invalid}, std::invalid_argument);

        invalid = config();
        invalid.max_global_voxels = 0U;
        EXPECT_THROW((void) OctoMapMerger {invalid}, std::invalid_argument);
    }

    TEST(OctoMapMergerTest, MergesDisjointSourcesAndReportsCounts)
    {
        OctoMapMerger merger(config());
        octomap::OcTree first(0.1);
        octomap::OcTree second(0.1);
        setState(first, 0.05F, 0.05F, 0.05F, false);
        setState(first, 0.15F, 0.05F, 0.05F, true);
        setState(second, 1.05F, 0.05F, 0.05F, false);

        const SourceUpdateResult a = merger.updateSource("/drone_0/octomap", first);
        const SourceUpdateResult b = merger.updateSource("/drone_1/octomap", second);

        EXPECT_EQ(a.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(b.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(merger.sourceCount(), 2U);
        EXPECT_EQ(merger.totalSourceVoxelCount(), 3U);
        EXPECT_EQ(merger.knownCount(), 3U);
        EXPECT_EQ(merger.freeCount(), 2U);
        EXPECT_EQ(merger.occupiedCount(), 1U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Free);
        EXPECT_EQ(query(merger.tree(), 0.15F, 0.05F, 0.05F), QueryState::Occupied);
        EXPECT_EQ(query(merger.tree(), 1.05F, 0.05F, 0.05F), QueryState::Free);
    }

    TEST(OctoMapMergerTest, OccupiedWinsOverFreeRegardlessOfSourceOrder)
    {
        octomap::OcTree free_tree(0.1);
        octomap::OcTree occupied_tree(0.1);
        setState(free_tree, 0.05F, 0.05F, 0.05F, false);
        setState(occupied_tree, 0.05F, 0.05F, 0.05F, true);

        OctoMapMerger forward(config());
        forward.updateSource("free", free_tree);
        forward.updateSource("occupied", occupied_tree);

        OctoMapMerger reverse(config());
        reverse.updateSource("occupied", occupied_tree);
        reverse.updateSource("free", free_tree);

        EXPECT_EQ(query(forward.tree(), 0.05F, 0.05F, 0.05F), QueryState::Occupied);
        EXPECT_EQ(query(reverse.tree(), 0.05F, 0.05F, 0.05F), QueryState::Occupied);
        EXPECT_EQ(forward.knownCount(), reverse.knownCount());
        EXPECT_EQ(forward.occupiedCount(), reverse.occupiedCount());
    }

    TEST(OctoMapMergerTest, RepeatedSnapshotIsIdempotent)
    {
        OctoMapMerger merger(config());
        octomap::OcTree source(0.1);
        setState(source, 0.05F, 0.05F, 0.05F, true);

        const SourceUpdateResult first = merger.updateSource("source", source);
        const SourceUpdateResult repeat = merger.updateSource("source", source);

        EXPECT_EQ(first.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(repeat.status, SourceUpdateStatus::AcceptedUnchanged);
        EXPECT_FALSE(repeat.global_changed);
        EXPECT_EQ(repeat.source_revision, first.source_revision);
        EXPECT_EQ(repeat.global_revision, first.global_revision);
        EXPECT_EQ(merger.sourceVoxelCount("source"), 1U);
        EXPECT_EQ(merger.knownCount(), 1U);
    }

    TEST(OctoMapMergerTest, ReplacesSourceAndPreservesOtherContributions)
    {
        OctoMapMerger merger(config());
        octomap::OcTree first(0.1);
        octomap::OcTree replacement(0.1);
        octomap::OcTree peer(0.1);
        setState(first, 0.05F, 0.05F, 0.05F, false);
        setState(first, 0.15F, 0.05F, 0.05F, false);
        setState(peer, 0.15F, 0.05F, 0.05F, true);
        setState(replacement, 0.25F, 0.05F, 0.05F, true);
        merger.updateSource("source", first);
        merger.updateSource("peer", peer);

        const SourceUpdateResult update = merger.updateSource("source", replacement);

        EXPECT_EQ(update.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(update.added_keys, 1U);
        EXPECT_EQ(update.removed_keys, 2U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Unknown);
        EXPECT_EQ(query(merger.tree(), 0.15F, 0.05F, 0.05F), QueryState::Occupied);
        EXPECT_EQ(query(merger.tree(), 0.25F, 0.05F, 0.05F), QueryState::Occupied);
    }

    TEST(OctoMapMergerTest, TracksSourceAndGlobalRevisionsSeparately)
    {
        OctoMapMerger merger(config());
        octomap::OcTree occupied(0.1);
        octomap::OcTree peer_free(0.1);
        octomap::OcTree peer_occupied(0.1);
        setState(occupied, 0.05F, 0.05F, 0.05F, true);
        setState(peer_free, 0.05F, 0.05F, 0.05F, false);
        setState(peer_occupied, 0.05F, 0.05F, 0.05F, true);
        merger.updateSource("owner", occupied);
        merger.updateSource("peer", peer_free);
        const std::uint64_t global_before = merger.globalRevision();

        const SourceUpdateResult result = merger.updateSource("peer", peer_occupied);

        EXPECT_EQ(result.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_FALSE(result.global_changed);
        EXPECT_GT(result.source_revision, 0U);
        EXPECT_EQ(result.global_revision, global_before);
        EXPECT_EQ(result.flipped_keys, 1U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Occupied);
    }

    TEST(OctoMapMergerTest, EmptySnapshotAndRemoveSourceHaveDistinctSourceSemantics)
    {
        OctoMapMerger merger(config());
        octomap::OcTree source(0.1);
        octomap::OcTree empty(0.1);
        setState(source, 0.05F, 0.05F, 0.05F, true);
        merger.updateSource("source", source);

        const SourceUpdateResult cleared = merger.updateSource("source", empty);
        EXPECT_EQ(cleared.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(merger.sourceCount(), 1U);
        EXPECT_EQ(merger.sourceVoxelCount("source"), 0U);
        EXPECT_EQ(merger.knownCount(), 0U);
        EXPECT_EQ(merger.tree().getNumLeafNodes(), 0U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Unknown);

        const SourceUpdateResult removed = merger.removeSource("source");
        EXPECT_EQ(removed.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(merger.sourceCount(), 0U);
        EXPECT_FALSE(removed.global_changed);

        const SourceUpdateResult repeated = merger.removeSource("source");
        EXPECT_EQ(repeated.status, SourceUpdateStatus::AcceptedUnchanged);
    }

    TEST(OctoMapMergerTest, ExpandsPrunedLeafToAllCoveredMaxDepthKeys)
    {
        OctoMapMergerConfig merger_config;
        merger_config.resolution = 1.0;
        merger_config.max_voxels_per_source = 64U;
        merger_config.max_global_voxels = 64U;
        OctoMapMerger merger(merger_config);
        octomap::OcTree source(1.0);

        const octomap::OcTreeKey base = makePrunedBlock(source, true);
        ASSERT_LT(source.begin_leafs().getDepth(), source.getTreeDepth());

        const SourceUpdateResult result = merger.updateSource("pruned", source);

        ASSERT_EQ(result.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(merger.knownCount(), 8U);
        for(octomap::key_type x = 0U; x < 2U; ++x) {
            for(octomap::key_type y = 0U; y < 2U; ++y) {
                for(octomap::key_type z = 0U; z < 2U; ++z) {
                    const octomap::OcTreeKey key {
                            static_cast<octomap::key_type>(base[0] + x),
                            static_cast<octomap::key_type>(base[1] + y),
                            static_cast<octomap::key_type>(base[2] + z),
                    };
                    const octomap::OcTreeNode * node = merger.tree().search(key);
                    ASSERT_NE(node, nullptr);
                    EXPECT_TRUE(merger.tree().isNodeOccupied(node));
                }
            }
        }
    }

    TEST(OctoMapMergerTest, ExpandsPrunedFreeLeafWithoutChangingItsState)
    {
        OctoMapMergerConfig merger_config;
        merger_config.resolution = 1.0;
        merger_config.max_voxels_per_source = 64U;
        merger_config.max_global_voxels = 64U;
        OctoMapMerger merger(merger_config);
        octomap::OcTree source(1.0);
        const octomap::OcTreeKey base = makePrunedBlock(source, false);
        ASSERT_LT(source.begin_leafs().getDepth(), source.getTreeDepth());

        const SourceUpdateResult result = merger.updateSource("pruned_free", source);

        ASSERT_EQ(result.status, SourceUpdateStatus::AcceptedChanged);
        EXPECT_EQ(merger.knownCount(), 8U);
        EXPECT_EQ(merger.freeCount(), 8U);
        for(octomap::key_type x = 0U; x < 2U; ++x) {
            for(octomap::key_type y = 0U; y < 2U; ++y) {
                for(octomap::key_type z = 0U; z < 2U; ++z) {
                    const octomap::OcTreeKey key {
                            static_cast<octomap::key_type>(base[0] + x),
                            static_cast<octomap::key_type>(base[1] + y),
                            static_cast<octomap::key_type>(base[2] + z),
                    };
                    const octomap::OcTreeNode * node = merger.tree().search(key);
                    ASSERT_NE(node, nullptr);
                    EXPECT_FALSE(merger.tree().isNodeOccupied(node));
                }
            }
        }
    }

    TEST(OctoMapMergerTest, RejectsResolutionMismatchAtomically)
    {
        OctoMapMerger merger(config());
        octomap::OcTree valid(0.1);
        octomap::OcTree invalid(0.2);
        setState(valid, 0.05F, 0.05F, 0.05F, true);
        setState(invalid, 1.0F, 0.0F, 0.0F, true);
        merger.updateSource("source", valid);
        const std::uint64_t source_revision = merger.sourceRevision();
        const std::uint64_t global_revision = merger.globalRevision();

        const SourceUpdateResult result = merger.updateSource("source", invalid);

        EXPECT_EQ(result.status, SourceUpdateStatus::Invalid);
        EXPECT_EQ(merger.sourceRevision(), source_revision);
        EXPECT_EQ(merger.globalRevision(), global_revision);
        EXPECT_EQ(merger.knownCount(), 1U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Occupied);
    }

    TEST(OctoMapMergerTest, RejectsPerSourceLimitBeforeReplacingOldSnapshot)
    {
        OctoMapMerger merger(config(1U, 10U));
        octomap::OcTree valid(0.1);
        octomap::OcTree too_large(0.1);
        setState(valid, 0.05F, 0.05F, 0.05F, true);
        setState(too_large, 1.05F, 0.05F, 0.05F, true);
        setState(too_large, 1.15F, 0.05F, 0.05F, true);
        merger.updateSource("source", valid);

        const SourceUpdateResult result = merger.updateSource("source", too_large);

        EXPECT_EQ(result.status, SourceUpdateStatus::Invalid);
        EXPECT_EQ(merger.knownCount(), 1U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Occupied);
        EXPECT_EQ(query(merger.tree(), 1.05F, 0.05F, 0.05F), QueryState::Unknown);
    }

    TEST(OctoMapMergerTest, RejectsGlobalLimitAtomically)
    {
        OctoMapMerger merger(config(10U, 1U));
        octomap::OcTree first(0.1);
        octomap::OcTree second(0.1);
        setState(first, 0.05F, 0.05F, 0.05F, true);
        setState(second, 1.05F, 0.05F, 0.05F, true);
        merger.updateSource("first", first);

        const SourceUpdateResult result = merger.updateSource("second", second);

        EXPECT_EQ(result.status, SourceUpdateStatus::Invalid);
        EXPECT_EQ(merger.sourceCount(), 1U);
        EXPECT_EQ(merger.knownCount(), 1U);
        EXPECT_EQ(query(merger.tree(), 0.05F, 0.05F, 0.05F), QueryState::Occupied);
        EXPECT_EQ(query(merger.tree(), 1.05F, 0.05F, 0.05F), QueryState::Unknown);
    }

    TEST(OctoMapMergerTest, RejectsEmptySourceIdWithoutMutation)
    {
        OctoMapMerger merger(config());
        octomap::OcTree source(0.1);
        setState(source, 0.05F, 0.05F, 0.05F, true);

        const SourceUpdateResult result = merger.updateSource("", source);

        EXPECT_EQ(result.status, SourceUpdateStatus::Invalid);
        EXPECT_EQ(merger.sourceCount(), 0U);
        EXPECT_EQ(merger.knownCount(), 0U);
    }

}// namespace SwarmController
