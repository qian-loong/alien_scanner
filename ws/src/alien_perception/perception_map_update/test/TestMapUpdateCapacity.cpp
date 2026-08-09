#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

namespace PerceptionMapUpdate::Test {

    namespace {

        constexpr std::size_t kCapacityCellCount = 1'812'520U;
        constexpr std::size_t kSparseChangeStride = 1'777U;

        CanonicalSnapshot make_snapshot(
                std::uint64_t revision,
                std::vector<CanonicalCell> cells,
                std::uint32_t changed_cells)
        {
            CanonicalSnapshot snapshot;
            snapshot.source = {"capacity-vehicle", {10'000U, 11U}, 1U};
            snapshot.geometry = {0.1, {0.0, 0.0, 0.0}, "capacity/map"};
            snapshot.revision = revision;
            snapshot.latest_commit = {
                    "capacity-lidar",
                    {20'000U, 12U},
                    Perception::Timestamp {static_cast<std::int64_t>(revision * 1'000'000U)},
                    "capacity-clock",
                    changed_cells};
            snapshot.cells = std::move(cells);
            snapshot.geometry_fingerprint = ContentHasher::geometry_fingerprint(
                    snapshot.geometry);
            snapshot.content_hash = ContentHasher::content_hash(
                    snapshot.source, snapshot.geometry_fingerprint, snapshot.cells);
            return snapshot;
        }

        std::int64_t elapsed_ms(std::chrono::steady_clock::time_point start)
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                    .count();
        }

    }// namespace

    TEST(MapUpdateCapacityTest, ReconstructsSparseDeltaAtExpandingReferenceCapacity)
    {
        std::vector<CanonicalCell> base_cells;
        base_cells.reserve(kCapacityCellCount);
        for(std::size_t index = 0U; index < kCapacityCellCount; ++index) {
            base_cells.push_back(
                    {{static_cast<std::int64_t>(index), 0, 0},
                     index % 5U == 0U ? CellState::Occupied : CellState::Free});
        }
        auto target_cells = base_cells;
        std::uint32_t changed_cells = 0U;
        for(std::size_t index = 0U; index < target_cells.size();
            index += kSparseChangeStride) {
            target_cells[index].state = target_cells[index].state == CellState::Free
                                                ? CellState::Occupied
                                                : CellState::Free;
            ++changed_cells;
        }

        const auto base = std::make_shared<const CanonicalSnapshot>(make_snapshot(
                1U,
                std::move(base_cells),
                static_cast<std::uint32_t>(kCapacityCellCount)));
        const auto target = std::make_shared<const CanonicalSnapshot>(
                make_snapshot(2U, std::move(target_cells), changed_cells));

        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(base->source));

        const auto keyframe_prepare_start = std::chrono::steady_clock::now();
        const auto keyframe = producer.prepare(base);
        const auto keyframe_prepare_ms = elapsed_ms(keyframe_prepare_start);
        ASSERT_TRUE(keyframe.update.has_value()) << keyframe.diagnostic;
        ASSERT_EQ(keyframe.update->kind, UpdateKind::Keyframe);
        ASSERT_TRUE(producer.commit_published(keyframe));
        const auto keyframe_apply_start = std::chrono::steady_clock::now();
        ASSERT_EQ(
                receiver.apply(*keyframe.update).status,
                ApplyUpdateStatus::AppliedKeyframe);
        const auto keyframe_apply_ms = elapsed_ms(keyframe_apply_start);

        const auto delta_prepare_start = std::chrono::steady_clock::now();
        const auto delta = producer.prepare(target);
        const auto delta_prepare_ms = elapsed_ms(delta_prepare_start);
        ASSERT_TRUE(delta.update.has_value()) << delta.diagnostic;
        ASSERT_EQ(delta.update->kind, UpdateKind::Delta);
        ASSERT_EQ(delta.update->operation_count, changed_cells);
        EXPECT_LT(delta.update->payload.size(), keyframe.update->payload.size());
        ASSERT_TRUE(producer.commit_published(delta));
        const auto delta_apply_start = std::chrono::steady_clock::now();
        ASSERT_EQ(receiver.apply(*delta.update).status, ApplyUpdateStatus::AppliedDelta);
        const auto delta_apply_ms = elapsed_ms(delta_apply_start);

        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, target->revision);
        EXPECT_EQ(receiver.reconstructed_map()->content_hash, target->content_hash);
        EXPECT_EQ(receiver.reconstructed_map()->cells, target->cells);

        MapUpdateLimits rejected_limits;
        rejected_limits.max_known_cells = kCapacityCellCount - 1U;
        MapUpdateProducer rejecting_producer(rejected_limits);
        const auto rejected = rejecting_producer.prepare(target);
        EXPECT_EQ(rejected.status, ProduceStatus::RejectedInvalidSnapshot);
        EXPECT_FALSE(rejected.update.has_value());

        std::cout << "[C3_CAPACITY] known_cells=" << kCapacityCellCount
                  << " changed_cells=" << changed_cells
                  << " keyframe_bytes=" << keyframe.update->payload.size()
                  << " delta_bytes=" << delta.update->payload.size()
                  << " keyframe_prepare_ms=" << keyframe_prepare_ms
                  << " keyframe_encode_ns=" << keyframe.timing.encode_duration_ns
                  << " keyframe_hash_ns=" << keyframe.timing.update_hash_duration_ns
                  << " keyframe_apply_ms=" << keyframe_apply_ms
                  << " delta_prepare_ms=" << delta_prepare_ms
                  << " delta_diff_ns=" << delta.timing.diff_duration_ns
                  << " delta_encode_ns=" << delta.timing.encode_duration_ns
                  << " delta_hash_ns=" << delta.timing.update_hash_duration_ns
                  << " delta_apply_ms=" << delta_apply_ms << std::endl;
    }

}// namespace PerceptionMapUpdate::Test
