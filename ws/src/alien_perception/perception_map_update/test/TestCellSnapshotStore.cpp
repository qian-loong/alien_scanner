#include "perception_map_update/CellSnapshotStore.hpp"

#include "perception_map_update/ContentHasher.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <utility>
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

        CellSnapshotStore make_store(CellStorageMode mode)
        {
            return CellSnapshotStore({mode, 16U, 256U});
        }

        const std::vector<CanonicalCell> kInterleavedCells {
                cell(-17, 0, 0, CellState::Free),
                cell(-16, -1, 7, CellState::Occupied),
                cell(-1, 0, 0, CellState::Free),
                cell(0, -17, 0, CellState::Occupied),
                cell(0, 0, -17, CellState::Free),
                cell(0, 0, 0, CellState::Occupied),
                cell(15, 15, 15, CellState::Free),
                cell(16, -16, 16, CellState::Occupied),
                cell(16, 0, 0, CellState::Free)};

    }// namespace

    TEST(CellSnapshotStore, VectorAndChunkedViewsPreserveCanonicalOrderAndFlatHash)
    {
        auto vector_store = make_store(CellStorageMode::Vector);
        auto chunked_store = make_store(CellStorageMode::Chunked);
        ASSERT_TRUE(vector_store.replace(kInterleavedCells).success);
        ASSERT_TRUE(chunked_store.replace(kInterleavedCells).success);

        EXPECT_EQ(vector_store.view(), kInterleavedCells);
        EXPECT_EQ(chunked_store.view(), kInterleavedCells);
        EXPECT_EQ(vector_store.view(), chunked_store.view());
        EXPECT_EQ(chunked_store.view().front(), kInterleavedCells.front());

        const SourceIdentity source {"drone-a", {10U, 20U}, 3U};
        const MapGeometry geometry {0.1, {-1.0, 2.0, 3.0}, "drone-a/map"};
        const auto geometry_hash = ContentHasher::geometry_fingerprint(geometry);
        const auto vector_hash = ContentHasher::content_hash(
                source, geometry_hash, vector_store.view());
        const auto chunked_hash = ContentHasher::content_hash(
                source, geometry_hash, chunked_store.view());
        EXPECT_EQ(vector_hash, chunked_hash);
        EXPECT_EQ(
                chunked_hash,
                ContentHasher::content_hash(source, geometry_hash, kInterleavedCells));
    }

    TEST(CellSnapshotStore, ChunkVisitorIsReadOnlyAndDeterministicAcrossStorageModes)
    {
        auto vector_store = make_store(CellStorageMode::Vector);
        auto chunked_store = make_store(CellStorageMode::Chunked);
        ASSERT_TRUE(vector_store.replace(kInterleavedCells).success);
        ASSERT_TRUE(chunked_store.replace(kInterleavedCells).success);

        std::vector<ChunkCoordinate> vector_coordinates;
        std::vector<std::size_t> vector_sizes;
        vector_store.for_each_chunk([&](
                                             const ChunkCoordinate & coordinate,
                                             const std::vector<CanonicalCell> & cells) {
            vector_coordinates.push_back(coordinate);
            vector_sizes.push_back(cells.size());
        });
        std::vector<ChunkCoordinate> chunked_coordinates;
        std::vector<std::size_t> chunked_sizes;
        chunked_store.for_each_chunk([&](
                                              const ChunkCoordinate & coordinate,
                                              const std::vector<CanonicalCell> & cells) {
            chunked_coordinates.push_back(coordinate);
            chunked_sizes.push_back(cells.size());
        });

        EXPECT_EQ(vector_coordinates, chunked_coordinates);
        EXPECT_EQ(vector_sizes, chunked_sizes);
        ASSERT_FALSE(vector_coordinates.empty());
        const ChunkCoordinate expected_first {-2, 0, 0};
        EXPECT_EQ(vector_coordinates.front(), expected_first);
        EXPECT_EQ(vector_store.view(), kInterleavedCells);
        EXPECT_EQ(chunked_store.view(), kInterleavedCells);

        std::vector<CanonicalCell> copied;
        ASSERT_TRUE(chunked_store.copy_chunk({0, 0, 0}, copied));
        EXPECT_EQ(copied, std::vector<CanonicalCell>({
                                   cell(0, 0, 0, CellState::Occupied),
                                   cell(15, 15, 15, CellState::Free)}));
        EXPECT_FALSE(chunked_store.copy_chunk({99, 99, 99}, copied));
        EXPECT_TRUE(copied.empty());
    }

    TEST(CellSnapshotStore, ChunkedApplyCopiesOnlyTouchedChunksAndSharesTheRest)
    {
        auto store = make_store(CellStorageMode::Chunked);
        const std::vector<CanonicalCell> initial {
                cell(-17, 0, 0, CellState::Free),
                cell(-1, 0, 0, CellState::Occupied),
                cell(0, 0, 0, CellState::Free),
                cell(1, 0, 0, CellState::Occupied),
                cell(16, 0, 0, CellState::Free),
                cell(32, 0, 0, CellState::Occupied)};
        ASSERT_TRUE(store.replace(initial).success);

        const auto untouched_identity = store.chunk_identity({32, 0, 0});
        const auto touched_identity = store.chunk_identity({0, 0, 0});
        ASSERT_NE(untouched_identity, nullptr);
        ASSERT_NE(touched_identity, nullptr);

        const std::vector<DeltaOperation> operations {
                {{-17, 0, 0}, DeltaOperationKind::UpsertOccupied},
                {{-1, 0, 0}, DeltaOperationKind::RemoveToUnknown},
                {{2, 0, 0}, DeltaOperationKind::UpsertFree},
                {{16, 0, 0}, DeltaOperationKind::UpsertOccupied}};
        const auto upper = store.estimate_apply_upper_bound(operations);
        ASSERT_TRUE(upper.success) << upper.diagnostic;
        const auto applied = store.apply(operations);
        ASSERT_TRUE(applied.success) << applied.diagnostic;

        const std::vector<CanonicalCell> expected {
                cell(-17, 0, 0, CellState::Occupied),
                cell(0, 0, 0, CellState::Free),
                cell(1, 0, 0, CellState::Occupied),
                cell(2, 0, 0, CellState::Free),
                cell(16, 0, 0, CellState::Occupied),
                cell(32, 0, 0, CellState::Occupied)};
        EXPECT_EQ(store.view(), expected);
        EXPECT_EQ(store.chunk_identity({32, 0, 0}), untouched_identity);
        EXPECT_NE(store.chunk_identity({0, 0, 0}), touched_identity);
        EXPECT_EQ(applied.metrics.touched_chunks, 4U);
        EXPECT_EQ(applied.metrics.shared_chunks, 1U);
        EXPECT_EQ(applied.metrics.copied_cells, 5U);
        EXPECT_GT(applied.metrics.copied_bucket_entries, 0U);
        EXPECT_GT(applied.metrics.candidate_owned_bytes, 0U);
        EXPECT_EQ(upper.metrics.touched_chunks, applied.metrics.touched_chunks);
        EXPECT_EQ(upper.metrics.copied_cells, applied.metrics.copied_cells);
        EXPECT_EQ(
                upper.metrics.copied_bucket_entries,
                applied.metrics.copied_bucket_entries);
        EXPECT_GE(
                upper.metrics.candidate_owned_bytes,
                applied.metrics.candidate_owned_bytes);
    }

    TEST(CellSnapshotStore, RevisionOnlyApplySharesEveryChunkWithoutCandidateAllocation)
    {
        auto store = make_store(CellStorageMode::Chunked);
        ASSERT_TRUE(store.replace(kInterleavedCells).success);
        const auto before = store.view();
        const auto first_identity = store.chunk_identity(kInterleavedCells.front().index);
        const auto total_chunks = store.metrics().total_chunks;

        const std::vector<DeltaOperation> operations;
        const auto upper = store.estimate_apply_upper_bound(operations);
        ASSERT_TRUE(upper.success);
        const auto applied = store.apply(operations);
        ASSERT_TRUE(applied.success);

        EXPECT_EQ(store.view(), before);
        EXPECT_EQ(store.chunk_identity(kInterleavedCells.front().index), first_identity);
        EXPECT_EQ(applied.metrics.shared_chunks, total_chunks);
        EXPECT_EQ(applied.metrics.touched_chunks, 0U);
        EXPECT_EQ(applied.metrics.candidate_owned_bytes, 0U);
        EXPECT_EQ(upper.metrics.candidate_owned_bytes, 0U);
    }

    TEST(CellSnapshotStore, FailedReplacementAndApplyLeaveCommittedSnapshotUntouched)
    {
        auto store = make_store(CellStorageMode::Chunked);
        ASSERT_TRUE(store.replace(kInterleavedCells).success);
        const auto before = store.view();
        const auto identity = store.chunk_identity({0, 0, 0});

        auto invalid_cells = kInterleavedCells;
        invalid_cells[1].index = invalid_cells[0].index;
        EXPECT_FALSE(store.replace(std::move(invalid_cells)).success);
        EXPECT_EQ(store.view(), before);

        const std::vector<DeltaOperation> redundant {
                {{0, 0, 0}, DeltaOperationKind::UpsertOccupied}};
        EXPECT_FALSE(store.apply(redundant).success);
        EXPECT_EQ(store.view(), before);
        EXPECT_EQ(store.chunk_identity({0, 0, 0}), identity);

        const std::vector<DeltaOperation> unknown_remove {
                {{5, 5, 5}, DeltaOperationKind::RemoveToUnknown}};
        EXPECT_FALSE(store.apply(unknown_remove).success);
        EXPECT_EQ(store.view(), before);

        const std::vector<DeltaOperation> unsorted {
                {{2, 0, 0}, DeltaOperationKind::UpsertFree},
                {{1, 0, 0}, DeltaOperationKind::UpsertFree}};
        EXPECT_FALSE(store.estimate_apply_upper_bound(unsorted).success);
        EXPECT_FALSE(store.apply(unsorted).success);
        EXPECT_EQ(store.view(), before);

        const std::vector<DeltaOperation> invalid_kind {
                {{3, 0, 0}, static_cast<DeltaOperationKind>(255U)}};
        EXPECT_FALSE(store.estimate_apply_upper_bound(invalid_kind).success);
        EXPECT_FALSE(store.apply(invalid_kind).success);
        EXPECT_EQ(store.view(), before);
    }

    TEST(CellSnapshotStore, CursorMoveRetainsSnapshotAndReportsExhaustion)
    {
        auto store = make_store(CellStorageMode::Chunked);
        ASSERT_TRUE(store.replace(kInterleavedCells).success);
        const auto old_view = store.view();
        auto cursor = old_view.cursor();
        ASSERT_FALSE(cursor.done());
        EXPECT_EQ(cursor.value(), kInterleavedCells.front());
        cursor.advance();
        auto moved = std::move(cursor);

        ASSERT_TRUE(store.replace({cell(100, 0, 0, CellState::Free)}).success);
        std::size_t index = 1U;
        while(!moved.done()) {
            ASSERT_LT(index, kInterleavedCells.size());
            EXPECT_EQ(moved.value(), kInterleavedCells[index]);
            moved.advance();
            ++index;
        }
        EXPECT_EQ(index, kInterleavedCells.size());
        EXPECT_THROW(moved.value(), std::out_of_range);
        EXPECT_THROW(moved.advance(), std::out_of_range);
        EXPECT_EQ(old_view, kInterleavedCells);
    }

    TEST(CellSnapshotStore, LocalDeltaUpperBoundDoesNotScaleCopiedCellsWithMapSize)
    {
        auto store = make_store(CellStorageMode::Chunked);
        std::vector<CanonicalCell> cells;
        cells.reserve(128U);
        for(std::int64_t chunk = 0; chunk < 128; ++chunk) {
            cells.push_back(cell(chunk * 16, 0, 0, CellState::Free));
        }
        ASSERT_TRUE(store.replace(cells).success);

        const std::vector<DeltaOperation> operations {
                {{64 * 16, 0, 0}, DeltaOperationKind::UpsertOccupied}};
        const auto upper = store.estimate_apply_upper_bound(operations);
        ASSERT_TRUE(upper.success) << upper.diagnostic;
        EXPECT_EQ(upper.metrics.touched_chunks, 1U);
        EXPECT_EQ(upper.metrics.copied_cells, 1U);
        EXPECT_EQ(upper.metrics.shared_chunks, 127U);
        EXPECT_LT(upper.metrics.copied_cells, store.size());
    }

    TEST(CellSnapshotStore, ChunkedReplacementUpperBoundIncludesConcurrentBuildState)
    {
        auto store = make_store(CellStorageMode::Chunked);
        constexpr std::size_t kCellCount = 512U;
        const auto upper = store.estimate_replace_upper_bound(kCellCount);
        ASSERT_TRUE(upper.success) << upper.diagnostic;

        const auto decoded_and_chunk_payload =
                2U * kCellCount * sizeof(CanonicalCell);
        EXPECT_EQ(upper.metrics.total_chunks, kCellCount);
        EXPECT_GT(
                upper.metrics.candidate_owned_bytes,
                decoded_and_chunk_payload);
        EXPECT_GT(
                upper.metrics.traversal_scratch_bytes,
                kCellCount * (sizeof(void *) + sizeof(std::size_t)));
    }

    TEST(CellSnapshotStore, RejectsUnknownStorageMode)
    {
        CellStorageConfig config;
        config.mode = static_cast<CellStorageMode>(255U);
        EXPECT_THROW((void) CellSnapshotStore {config}, std::invalid_argument);
    }

}// namespace PerceptionMapUpdate::Test
