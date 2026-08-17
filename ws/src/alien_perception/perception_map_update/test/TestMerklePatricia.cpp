#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MerklePatricia.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <initializer_list>
#include <map>
#include <string>
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

        MerkleChunkMutation upsert(
                ChunkCoordinate coordinate,
                std::initializer_list<CanonicalCell> cells)
        {
            return {coordinate, std::vector<CanonicalCell>(cells), false};
        }

        MerkleChunkMutation remove(ChunkCoordinate coordinate)
        {
            return {coordinate, {}, true};
        }

        SourceIdentity source()
        {
            return {"drone-merkle", {100U, 7U}, 3U};
        }

        Hash256 geometry_hash()
        {
            return ContentHasher::geometry_fingerprint(
                    {0.1, {-2.0, 1.0, 0.5}, "map"});
        }

        std::string key_hex(const MerkleCoordinateKey & key)
        {
            constexpr char digits[] = "0123456789abcdef";
            std::string result;
            result.reserve(key.bytes.size() * 2U);
            for(const auto byte : key.bytes) {
                result.push_back(digits[(byte >> 4U) & 0x0fU]);
                result.push_back(digits[byte & 0x0fU]);
            }
            return result;
        }

    }// namespace

    TEST(MerklePatriciaTest, CoordinateKeyUsesSignedOrderAndBigEndianEncoding)
    {
        const auto key = merkle_coordinate_key(ChunkCoordinate {-1, 0, 1});
        EXPECT_EQ(
                key_hex(key),
                "7fffffffffffffff80000000000000008000000000000001");
        EXPECT_LT(
                merkle_coordinate_key(ChunkCoordinate {-1, 0, 0}),
                merkle_coordinate_key(ChunkCoordinate {0, 0, 0}));
        EXPECT_LT(
                merkle_coordinate_key(ChunkCoordinate {0, -1, 0}),
                merkle_coordinate_key(ChunkCoordinate {0, 0, 0}));
    }

    TEST(MerklePatriciaTest, FullRebuildHasDeterministicRootAndChunkNodeBound)
    {
        const std::vector<CanonicalCell> cells {
                cell(-17, 0, 0, CellState::Free),
                cell(-1, 0, 0, CellState::Occupied),
                cell(0, 0, 0, CellState::Free),
                cell(15, 15, 15, CellState::Occupied),
                cell(16, 0, 0, CellState::Free),
                cell(32, 1, 0, CellState::Occupied)};
        const auto first = MerklePatriciaTree::full_rebuild(source(), geometry_hash(), cells);
        ASSERT_TRUE(first) << first.diagnostic;
        ASSERT_TRUE(first.tree);
        EXPECT_EQ(first.tree->leaf_count(), 5U);
        EXPECT_EQ(first.tree->metrics().node_count, 9U);
        EXPECT_EQ(first.tree->metrics().branch_count, 4U);
        EXPECT_FALSE(is_zero_hash(first.tree->trie_root()));
        EXPECT_FALSE(is_zero_hash(first.tree->content_root()));
        EXPECT_EQ(first.tree->versioned_digest().descriptor.chunk_edge, 16U);

        const auto second = MerklePatriciaTree::full_rebuild(source(), geometry_hash(), cells);
        ASSERT_TRUE(second) << second.diagnostic;
        EXPECT_EQ(second.tree->trie_root(), first.tree->trie_root());
        EXPECT_EQ(second.tree->content_root(), first.tree->content_root());
    }

    TEST(MerklePatriciaTest, FullRebuildStreamsCanonicalCellView)
    {
        const std::vector<CanonicalCell> cells {
                cell(-17, 0, 0, CellState::Free),
                cell(0, 0, 0, CellState::Free),
                cell(16, 0, 0, CellState::Occupied)};
        CellSnapshotStore store({CellStorageMode::Chunked, 16U, 256U});
        ASSERT_TRUE(store.replace(cells).success);
        const auto from_view = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), store.view());
        const auto from_store = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), store);
        const auto from_vector = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), cells);
        ASSERT_TRUE(from_view) << from_view.diagnostic;
        ASSERT_TRUE(from_store) << from_store.diagnostic;
        ASSERT_TRUE(from_vector) << from_vector.diagnostic;
        EXPECT_EQ(from_view.tree->trie_root(), from_vector.tree->trie_root());
        EXPECT_EQ(from_store.tree->trie_root(), from_vector.tree->trie_root());
        EXPECT_EQ(from_view.tree->content_root(), from_vector.tree->content_root());
        EXPECT_EQ(from_store.tree->content_root(), from_vector.tree->content_root());
    }

    TEST(MerklePatriciaTest, IncrementalBatchMatchesIndependentFullRebuild)
    {
        const std::vector<CanonicalCell> initial {
                cell(-17, 0, 0, CellState::Free),
                cell(0, 0, 0, CellState::Free),
                cell(16, 0, 0, CellState::Occupied)};
        const auto base = MerklePatriciaTree::full_rebuild(source(), geometry_hash(), initial);
        ASSERT_TRUE(base) << base.diagnostic;

        const auto candidate = base.tree->apply({
                upsert({2, 0, 0}, {cell(32, 0, 0, CellState::Occupied)}),
                upsert({0, 0, 0}, {cell(0, 0, 0, CellState::Occupied)}),
        });
        ASSERT_TRUE(candidate) << candidate.diagnostic;
        ASSERT_TRUE(candidate.tree);
        const std::vector<CanonicalCell> expected {
                cell(-17, 0, 0, CellState::Free),
                cell(0, 0, 0, CellState::Occupied),
                cell(16, 0, 0, CellState::Occupied),
                cell(32, 0, 0, CellState::Occupied)};
        const auto rebuilt = MerklePatriciaTree::full_rebuild(source(), geometry_hash(), expected);
        ASSERT_TRUE(rebuilt) << rebuilt.diagnostic;
        EXPECT_EQ(candidate.tree->trie_root(), rebuilt.tree->trie_root());
        EXPECT_EQ(candidate.tree->content_root(), rebuilt.tree->content_root());
        EXPECT_GT(candidate.metrics.path_nodes_rebuilt, 0U);
        EXPECT_GT(candidate.metrics.allocated_nodes, 0U);

        const auto after_remove = candidate.tree->apply({remove({1, 0, 0})});
        ASSERT_TRUE(after_remove) << after_remove.diagnostic;
        const std::vector<CanonicalCell> after_remove_cells {
                cell(-17, 0, 0, CellState::Free),
                cell(0, 0, 0, CellState::Occupied),
                cell(32, 0, 0, CellState::Occupied)};
        const auto remove_rebuilt = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), after_remove_cells);
        ASSERT_TRUE(remove_rebuilt) << remove_rebuilt.diagnostic;
        EXPECT_EQ(after_remove.tree->content_root(), remove_rebuilt.tree->content_root());
    }

    TEST(MerklePatriciaTest, EmptyMutationReusesRootWithoutAllocatingNodes)
    {
        const auto base = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(),
                std::vector<CanonicalCell> {cell(0, 0, 0, CellState::Free)});
        ASSERT_TRUE(base) << base.diagnostic;
        const auto revision_only = base.tree->apply({});
        ASSERT_TRUE(revision_only) << revision_only.diagnostic;
        EXPECT_EQ(revision_only.tree->trie_root(), base.tree->trie_root());
        EXPECT_EQ(revision_only.tree->content_root(), base.tree->content_root());
        EXPECT_EQ(revision_only.metrics.allocated_nodes, 0U);
        EXPECT_EQ(revision_only.metrics.path_nodes_rebuilt, 0U);
    }

    TEST(MerklePatriciaTest, InvalidMutationDoesNotChangePersistentTree)
    {
        const std::vector<CanonicalCell> cells {
                cell(0, 0, 0, CellState::Free)};
        const auto base = MerklePatriciaTree::full_rebuild(source(), geometry_hash(), cells);
        ASSERT_TRUE(base) << base.diagnostic;
        const auto before = base.tree->content_root();

        EXPECT_FALSE(base.tree->apply({remove({9, 0, 0})}));
        EXPECT_FALSE(base.tree->apply({
                upsert({0, 0, 0}, {cell(16, 0, 0, CellState::Occupied)})}));
        EXPECT_FALSE(base.tree->apply({
                upsert({0, 0, 0}, {cell(0, 0, 0, CellState::Occupied)}),
                remove({0, 0, 0})}));
        EXPECT_EQ(base.tree->content_root(), before);
    }

    TEST(MerklePatriciaTest, EmptyKeyframeHasNonZeroRoot)
    {
        const auto empty = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), std::vector<CanonicalCell> {});
        ASSERT_TRUE(empty) << empty.diagnostic;
        EXPECT_TRUE(empty.tree->empty());
        EXPECT_EQ(empty.tree->leaf_count(), 0U);
        EXPECT_EQ(empty.tree->metrics().node_count, 0U);
        EXPECT_FALSE(is_zero_hash(empty.tree->trie_root()));
        EXPECT_EQ(
                hash_to_hex(empty.tree->trie_root()),
                "f3f23d08429f3c2397ed5a83435d64a595a91e44d38a4f4e87ba255e2a18ab00");
        EXPECT_FALSE(is_zero_hash(empty.tree->content_root()));
        EXPECT_NE(empty.tree->content_root(), Hash256 {});
    }

    TEST(MerklePatriciaTest, SingleLeafGoldenVectorIsStable)
    {
        const auto result = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(),
                std::vector<CanonicalCell> {cell(0, 0, 0, CellState::Free)});
        ASSERT_TRUE(result) << result.diagnostic;
        EXPECT_EQ(
                hash_to_hex(result.tree->trie_root()),
                "1227dc3fd60b638954c08ef185769f3cdfeb91fca3a03113edd89aeb46a3e90c");
    }

    TEST(MerklePatriciaTest, InvalidDescriptorIsRejected)
    {
        ContentIdentityDescriptor descriptor;
        descriptor.chunk_edge = 8U;
        const auto result = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), std::vector<CanonicalCell> {}, descriptor);
        EXPECT_FALSE(result);
        EXPECT_FALSE(result.tree);
    }

    TEST(MerklePatriciaTest, DeterministicThreeDimensionalReplayMatchesEveryFullRebuild)
    {
        std::map<ChunkCoordinate, std::vector<CanonicalCell>> chunks;
        const auto initial = MerklePatriciaTree::full_rebuild(
                source(), geometry_hash(), std::vector<CanonicalCell> {});
        ASSERT_TRUE(initial) << initial.diagnostic;
        auto incremental = initial.tree;

        for(std::int64_t step = 0; step < 400; ++step) {
            const ChunkCoordinate coordinate {
                    (step * 17) % 41 - 20,
                    (step * 7) % 13 - 6,
                    (step * 11) % 17 - 8};
            std::vector<MerkleChunkMutation> mutations;
            const auto found = chunks.find(coordinate);
            if(step % 5 == 0 && found != chunks.end()) {
                chunks.erase(found);
                mutations.push_back(remove(coordinate));
            }
            else {
                std::vector<CanonicalCell> replacement {
                        cell(
                                coordinate.x * 16,
                                coordinate.y * 16,
                                coordinate.z * 16,
                                step % 2 == 0 ? CellState::Free : CellState::Occupied),
                        cell(
                                coordinate.x * 16 + 15,
                                coordinate.y * 16 + 15,
                                coordinate.z * 16 + 15,
                                step % 2 == 0 ? CellState::Occupied : CellState::Free)};
                chunks[coordinate] = replacement;
                mutations.push_back({coordinate, std::move(replacement), false});
            }

            const auto candidate = incremental->apply(mutations);
            ASSERT_TRUE(candidate) << "step=" << step << ' ' << candidate.diagnostic;
            std::vector<CanonicalCell> expected;
            for(const auto & chunk : chunks) {
                expected.insert(expected.end(), chunk.second.begin(), chunk.second.end());
            }
            std::sort(
                    expected.begin(),
                    expected.end(),
                    [](const CanonicalCell & left, const CanonicalCell & right) {
                        return left.index < right.index;
                    });
            const auto rebuilt = MerklePatriciaTree::full_rebuild(
                    source(), geometry_hash(), expected);
            ASSERT_TRUE(rebuilt) << "step=" << step << ' ' << rebuilt.diagnostic;
            EXPECT_EQ(candidate.tree->trie_root(), rebuilt.tree->trie_root())
                    << "step=" << step;
            EXPECT_EQ(candidate.tree->content_root(), rebuilt.tree->content_root())
                    << "step=" << step;
            EXPECT_EQ(candidate.tree->metrics().leaf_count, chunks.size())
                    << "step=" << step;
            EXPECT_EQ(candidate.tree->metrics().node_count, rebuilt.tree->metrics().node_count)
                    << "step=" << step;
            incremental = candidate.tree;
        }
    }

}// namespace PerceptionMapUpdate::Test
