#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MerklePrototypeApplier.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

namespace PerceptionMapUpdate::Test {

    namespace {

        CanonicalCell cell(
                std::int64_t x,
                std::int64_t y,
                std::int64_t z,
                CellState state)
        {
            return {{x, y, z}, state};
        }

        SourceIdentity source()
        {
            return {"prototype", {10U, 2U}, 1U};
        }

        Hash256 geometry_hash()
        {
            return ContentHasher::geometry_fingerprint(
                    {0.1, {0.0, 0.0, 0.0}, "map"});
        }

    }// namespace

    TEST(MerklePrototypeApplierTest, CandidateStoreAndTreeCommitTogether)
    {
        const std::vector<CanonicalCell> initial {
                cell(-1, 0, 0, CellState::Free),
                cell(0, 0, 0, CellState::Free),
                cell(16, 0, 0, CellState::Occupied)};
        const auto base = MerklePrototypeApplier::build(
                source(), geometry_hash(), initial);
        ASSERT_TRUE(base) << base.diagnostic;
        ASSERT_TRUE(base.candidate);
        const auto old_root = base.candidate->tree().content_root();

        const std::vector<DeltaOperation> operations {
                {{0, 0, 0}, DeltaOperationKind::UpsertOccupied},
                {{1, 0, 0}, DeltaOperationKind::UpsertFree},
                {{16, 0, 0}, DeltaOperationKind::RemoveToUnknown}};
        const auto applied = base.candidate->apply(operations);
        ASSERT_TRUE(applied) << applied.diagnostic;
        ASSERT_TRUE(applied.candidate);
        const std::vector<CanonicalCell> expected {
                cell(-1, 0, 0, CellState::Free),
                cell(0, 0, 0, CellState::Occupied),
                cell(1, 0, 0, CellState::Free)};
        const auto oracle = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), expected);
        ASSERT_TRUE(oracle) << oracle.diagnostic;
        EXPECT_EQ(applied.candidate->store().view(), expected);
        EXPECT_EQ(applied.candidate->tree().content_root(), oracle.tree->content_root());
        EXPECT_EQ(base.candidate->tree().content_root(), old_root);
        EXPECT_EQ(base.candidate->store().view(), initial);
    }

    TEST(MerklePrototypeApplierTest, FailedCandidateLeavesCommittedStateUntouched)
    {
        const std::vector<CanonicalCell> initial {
                cell(0, 0, 0, CellState::Free)};
        const auto base = MerklePrototypeApplier::build(
                source(), geometry_hash(), initial);
        ASSERT_TRUE(base) << base.diagnostic;
        const auto old_root = base.candidate->tree().content_root();

        const std::vector<DeltaOperation> redundant {
                {{0, 0, 0}, DeltaOperationKind::UpsertFree}};
        const auto rejected = base.candidate->apply(redundant);
        EXPECT_FALSE(rejected);
        EXPECT_EQ(base.candidate->tree().content_root(), old_root);
        EXPECT_EQ(base.candidate->store().view(), initial);

        const auto invalid_storage = MerklePrototypeApplier::build(
                source(),
                geometry_hash(),
                initial,
                {CellStorageMode::Vector, 16U, 256U});
        EXPECT_FALSE(invalid_storage);
    }

    TEST(MerklePrototypeApplierTest, RevisionOnlyCandidateReusesBothIdentities)
    {
        const auto base = MerklePrototypeApplier::build(
                source(), geometry_hash(),
                std::vector<CanonicalCell> {cell(0, 0, 0, CellState::Free)});
        ASSERT_TRUE(base) << base.diagnostic;
        const auto next = base.candidate->apply({});
        ASSERT_TRUE(next) << next.diagnostic;
        EXPECT_EQ(next.candidate->tree().content_root(), base.candidate->tree().content_root());
        EXPECT_EQ(next.candidate->store().view(), base.candidate->store().view());
        EXPECT_EQ(next.merkle_metrics.allocated_nodes, 0U);
        EXPECT_EQ(next.merkle_metrics.candidate_owned_bytes, 0U);
        EXPECT_EQ(next.storage_metrics.candidate_owned_bytes, 0U);
    }

    TEST(MerklePrototypeApplierTest, RemovingLastCellUsesEmptyMerkleRoot)
    {
        const auto base = MerklePrototypeApplier::build(
                source(), geometry_hash(),
                std::vector<CanonicalCell> {cell(0, 0, 0, CellState::Free)});
        ASSERT_TRUE(base) << base.diagnostic;
        const auto removed = base.candidate->apply({
                {{0, 0, 0}, DeltaOperationKind::RemoveToUnknown}});
        ASSERT_TRUE(removed) << removed.diagnostic;
        EXPECT_TRUE(removed.candidate->store().view().empty());
        EXPECT_TRUE(removed.candidate->tree().empty());
        EXPECT_FALSE(is_zero_hash(removed.candidate->tree().trie_root()));
        EXPECT_EQ(removed.candidate->tree().metrics().node_count, 0U);
    }

    TEST(MerkleMapStateTest, ResourcePreflightRejectsWithoutMutatingCommittedState)
    {
        MapUpdateLimits keyframe_limits;
        keyframe_limits.max_live_chunks = 1U;
        const auto rejected_keyframe = MerkleMapState::build(
                source(),
                geometry_hash(),
                {cell(0, 0, 0, CellState::Free),
                 cell(16, 0, 0, CellState::Occupied)},
                {CellStorageMode::Chunked,
                 kMerkleChunkEdge,
                 kMerkleChunkBucketCount},
                {},
                keyframe_limits);
        EXPECT_FALSE(rejected_keyframe);
        EXPECT_TRUE(rejected_keyframe.resource_limit);
        EXPECT_EQ(rejected_keyframe.candidate, nullptr);

        MapUpdateLimits delta_limits;
        delta_limits.max_candidate_merkle_bytes = 1U;
        const auto empty = MerkleMapState::build(
                source(),
                geometry_hash(),
                {},
                {CellStorageMode::Chunked,
                 kMerkleChunkEdge,
                 kMerkleChunkBucketCount},
                {},
                delta_limits);
        ASSERT_TRUE(empty) << empty.diagnostic;
        ASSERT_NE(empty.candidate, nullptr);
        const auto committed_root = empty.candidate->identity();

        const auto rejected_delta = empty.candidate->apply({
                {{0, 0, 0}, DeltaOperationKind::UpsertFree}});
        EXPECT_FALSE(rejected_delta);
        EXPECT_TRUE(rejected_delta.resource_limit);
        EXPECT_EQ(rejected_delta.candidate, nullptr);
        EXPECT_TRUE(empty.candidate->cells().empty());
        EXPECT_EQ(empty.candidate->identity(), committed_root);
    }

}// namespace PerceptionMapUpdate::Test
