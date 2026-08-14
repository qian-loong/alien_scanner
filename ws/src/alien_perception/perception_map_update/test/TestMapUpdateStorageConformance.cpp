#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace PerceptionMapUpdate::Test {

    namespace {

        SourceIdentity source()
        {
            return {"conformance-drone", {100U, 7U}, 1U};
        }

        MapGeometry geometry()
        {
            return {0.1, {-2.0, 1.0, 3.0}, "conformance-drone/map"};
        }

        CanonicalCell cell(
                std::int64_t x,
                std::int64_t y,
                std::int64_t z,
                CellState state)
        {
            return {{x, y, z}, state};
        }

        CanonicalSnapshot snapshot(
                std::uint64_t revision,
                std::vector<CanonicalCell> cells)
        {
            CanonicalSnapshot result;
            result.source = source();
            result.geometry = geometry();
            result.revision = revision;
            result.latest_commit = {
                    "lidar",
                    {200U, 8U},
                    Perception::Timestamp {static_cast<std::int64_t>(revision * 1'000U)},
                    "simulation",
                    static_cast<std::uint32_t>(cells.size())};
            result.cells = std::move(cells);
            result.geometry_fingerprint = ContentHasher::geometry_fingerprint(
                    result.geometry);
            result.content_hash = ContentHasher::content_hash(
                    result.source, result.geometry_fingerprint, result.cells);
            return result;
        }

        PreparedUpdate publish(
                MapUpdateProducer & producer,
                const CanonicalSnapshot & value)
        {
            auto prepared = producer.prepare(value);
            EXPECT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
            EXPECT_TRUE(producer.commit_published(prepared));
            return prepared;
        }

        CellStorageConfig storage(CellStorageMode mode)
        {
            return {mode, 16U, 256U};
        }

        void expect_same_map(
                const MapUpdateApplier & vector_receiver,
                const MapUpdateApplier & chunked_receiver)
        {
            ASSERT_EQ(
                    vector_receiver.reconstructed_map().has_value(),
                    chunked_receiver.reconstructed_map().has_value());
            if(!vector_receiver.reconstructed_map().has_value()) {
                return;
            }
            const auto & vector_map = *vector_receiver.reconstructed_map();
            const auto & chunked_map = *chunked_receiver.reconstructed_map();
            EXPECT_EQ(vector_map.source, chunked_map.source);
            EXPECT_EQ(vector_map.geometry, chunked_map.geometry);
            EXPECT_EQ(vector_map.revision, chunked_map.revision);
            EXPECT_EQ(vector_map.content_hash, chunked_map.content_hash);
            EXPECT_EQ(vector_map.cells, chunked_map.cells);
        }

        void expect_same_apply(
                MapUpdateApplier & vector_receiver,
                MapUpdateApplier & chunked_receiver,
                const MapUpdate & update,
                ApplyUpdateStatus expected)
        {
            const auto vector_result = vector_receiver.apply(update);
            const auto chunked_result = chunked_receiver.apply(update);
            EXPECT_EQ(vector_result.status, expected) << vector_result.diagnostic;
            EXPECT_EQ(chunked_result.status, expected) << chunked_result.diagnostic;
            EXPECT_EQ(vector_result.status, chunked_result.status);
            EXPECT_EQ(vector_result.state_changed, chunked_result.state_changed);
            EXPECT_EQ(vector_result.diagnostic, chunked_result.diagnostic);
            EXPECT_EQ(vector_receiver.state(), chunked_receiver.state());
            expect_same_map(vector_receiver, chunked_receiver);
        }

        void admit_both(
                MapUpdateApplier & vector_receiver,
                MapUpdateApplier & chunked_receiver)
        {
            ASSERT_TRUE(vector_receiver.admit_source(source()));
            ASSERT_TRUE(chunked_receiver.admit_source(source()));
        }

    }// namespace

    TEST(MapUpdateStorageConformance, AppliesKeyframeDeltaRevisionOnlySummaryAndRemove)
    {
        MapUpdateApplier vector_receiver({}, storage(CellStorageMode::Vector));
        MapUpdateApplier chunked_receiver({}, storage(CellStorageMode::Chunked));
        admit_both(vector_receiver, chunked_receiver);
        MapUpdateProducer producer;

        const auto first_snapshot = snapshot(
                1U,
                {cell(-17, 0, 0, CellState::Free),
                 cell(0, 0, 0, CellState::Occupied),
                 cell(32, 1, -1, CellState::Free)});
        const auto first = publish(producer, first_snapshot);
        ASSERT_TRUE(first.update.has_value());
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *first.update,
                ApplyUpdateStatus::AppliedKeyframe);

        const auto second_snapshot = snapshot(
                2U,
                {cell(-17, 0, 0, CellState::Occupied),
                 cell(1, -1, 0, CellState::Free),
                 cell(32, 1, -1, CellState::Free)});
        const auto second = publish(producer, second_snapshot);
        ASSERT_TRUE(second.update.has_value());
        ASSERT_EQ(second.update->kind, UpdateKind::Delta);
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *second.update,
                ApplyUpdateStatus::AppliedDelta);

        const auto third = publish(producer, snapshot(3U, second_snapshot.cells));
        ASSERT_TRUE(third.update.has_value());
        ASSERT_EQ(third.update->kind, UpdateKind::Delta);
        ASSERT_EQ(third.update->operation_count, 0U);
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *third.update,
                ApplyUpdateStatus::AppliedDelta);

        MapUpdate summary = *third.update;
        summary.kind = UpdateKind::Summary;
        summary.base_revision = summary.new_revision;
        summary.revision_span = 0U;
        summary.base_content_hash = summary.content_hash;
        summary.operation_count = 0U;
        summary.canonical_payload_bytes = 0U;
        summary.payload.clear();
        summary.update_hash = ContentHasher::update_hash(summary);
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                summary,
                ApplyUpdateStatus::AcceptedSummary);

        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *third.update,
                ApplyUpdateStatus::IgnoredDuplicate);

        MapUpdate remove = summary;
        remove.kind = UpdateKind::Remove;
        remove.base_revision = 3U;
        remove.new_revision = 4U;
        remove.revision_span = 1U;
        remove.content_hash = {};
        remove.known_cell_count = 0U;
        remove.update_hash = ContentHasher::update_hash(remove);
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                remove,
                ApplyUpdateStatus::AppliedRemove);

        MapUpdateProducer reactivation_producer;
        const auto reactivation = publish(reactivation_producer, first_snapshot);
        ASSERT_TRUE(reactivation.update.has_value());
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *reactivation.update,
                ApplyUpdateStatus::RejectedStale);
    }

    TEST(MapUpdateStorageConformance, RejectsGapConflictCorruptionAndPeakLimitAtomically)
    {
        const auto base_snapshot = snapshot(
                1U,
                {cell(-1, 0, 0, CellState::Free),
                 cell(0, 0, 0, CellState::Occupied)});
        const auto target_snapshot = snapshot(
                2U,
                {cell(-1, 0, 0, CellState::Occupied),
                 cell(0, 0, 0, CellState::Free)});

        MapUpdateProducer producer;
        const auto keyframe = publish(producer, base_snapshot);
        const auto delta = publish(producer, target_snapshot);
        ASSERT_TRUE(keyframe.update.has_value());
        ASSERT_TRUE(delta.update.has_value());

        {
            MapUpdateApplier vector_receiver({}, storage(CellStorageMode::Vector));
            MapUpdateApplier chunked_receiver({}, storage(CellStorageMode::Chunked));
            admit_both(vector_receiver, chunked_receiver);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    *keyframe.update,
                    ApplyUpdateStatus::AppliedKeyframe);
            auto gap = *delta.update;
            gap.base_revision = 2U;
            gap.new_revision = 3U;
            gap.revision_span = 1U;
            gap.update_hash = ContentHasher::update_hash(gap);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    gap,
                    ApplyUpdateStatus::RejectedGap);
            ASSERT_TRUE(vector_receiver.reconstructed_map().has_value());
            EXPECT_EQ(vector_receiver.reconstructed_map()->revision, 1U);
        }

        {
            MapUpdateApplier vector_receiver({}, storage(CellStorageMode::Vector));
            MapUpdateApplier chunked_receiver({}, storage(CellStorageMode::Chunked));
            admit_both(vector_receiver, chunked_receiver);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    *keyframe.update,
                    ApplyUpdateStatus::AppliedKeyframe);
            auto conflict = *delta.update;
            conflict.content_hash[0] ^= 0xFFU;
            conflict.update_hash = ContentHasher::update_hash(conflict);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    conflict,
                    ApplyUpdateStatus::RejectedConflict);
            ASSERT_TRUE(vector_receiver.reconstructed_map().has_value());
            EXPECT_EQ(vector_receiver.reconstructed_map()->revision, 1U);
        }

        {
            MapUpdateApplier vector_receiver({}, storage(CellStorageMode::Vector));
            MapUpdateApplier chunked_receiver({}, storage(CellStorageMode::Chunked));
            admit_both(vector_receiver, chunked_receiver);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    *keyframe.update,
                    ApplyUpdateStatus::AppliedKeyframe);
            auto corrupted = *delta.update;
            ASSERT_FALSE(corrupted.payload.empty());
            corrupted.payload.back() ^= 0xFFU;
            corrupted.update_hash = ContentHasher::update_hash(corrupted);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    corrupted,
                    ApplyUpdateStatus::RejectedInvalid);
            ASSERT_TRUE(vector_receiver.reconstructed_map().has_value());
            EXPECT_EQ(vector_receiver.reconstructed_map()->revision, 1U);
        }

        {
            MapUpdateLimits limits;
            limits.max_peak_apply_bytes = 1U;
            MapUpdateApplier vector_receiver(limits, storage(CellStorageMode::Vector));
            MapUpdateApplier chunked_receiver(limits, storage(CellStorageMode::Chunked));
            admit_both(vector_receiver, chunked_receiver);
            expect_same_apply(
                    vector_receiver,
                    chunked_receiver,
                    *keyframe.update,
                    ApplyUpdateStatus::RejectedResourceLimit);
            EXPECT_FALSE(vector_receiver.reconstructed_map().has_value());
        }
    }

    TEST(MapUpdateStorageConformance, RejectsUnadmittedAndRetiredSourcesAtomically)
    {
        MapUpdateApplier vector_receiver({}, storage(CellStorageMode::Vector));
        MapUpdateApplier chunked_receiver({}, storage(CellStorageMode::Chunked));
        MapUpdateProducer producer;
        const auto initial = publish(
                producer,
                snapshot(1U, {cell(0, 0, 0, CellState::Occupied)}));
        ASSERT_TRUE(initial.update.has_value());

        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *initial.update,
                ApplyUpdateStatus::RejectedAdmission);
        EXPECT_FALSE(vector_receiver.reconstructed_map().has_value());

        admit_both(vector_receiver, chunked_receiver);
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *initial.update,
                ApplyUpdateStatus::AppliedKeyframe);

        auto newer_source = source();
        ++newer_source.map_epoch;
        ASSERT_TRUE(vector_receiver.admit_source(newer_source));
        ASSERT_TRUE(chunked_receiver.admit_source(newer_source));
        EXPECT_EQ(vector_receiver.state(), ReceiverState::ResyncRequired);
        EXPECT_EQ(chunked_receiver.state(), ReceiverState::ResyncRequired);

        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *initial.update,
                ApplyUpdateStatus::RejectedAdmission);
        ASSERT_TRUE(vector_receiver.reconstructed_map().has_value());
        EXPECT_EQ(vector_receiver.reconstructed_map()->revision, 1U);
    }

    TEST(MapUpdateStorageConformance, FullCellCapacityAllowsNonGrowingDeltaInBothModes)
    {
        MapUpdateLimits limits;
        limits.max_known_cells = 2U;
        limits.max_receiver_cells = 2U;
        limits.max_delta_operations = 2U;
        MapUpdateApplier vector_receiver(limits, storage(CellStorageMode::Vector));
        MapUpdateApplier chunked_receiver(limits, storage(CellStorageMode::Chunked));
        admit_both(vector_receiver, chunked_receiver);
        MapUpdateProducer producer(limits);

        const auto initial = publish(
                producer,
                snapshot(
                        1U,
                        {cell(0, 0, 0, CellState::Free),
                         cell(1, 0, 0, CellState::Occupied)}));
        ASSERT_TRUE(initial.update.has_value());
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *initial.update,
                ApplyUpdateStatus::AppliedKeyframe);

        const auto flips = publish(
                producer,
                snapshot(
                        2U,
                        {cell(0, 0, 0, CellState::Occupied),
                         cell(1, 0, 0, CellState::Free)}));
        ASSERT_TRUE(flips.update.has_value());
        expect_same_apply(
                vector_receiver,
                chunked_receiver,
                *flips.update,
                ApplyUpdateStatus::AppliedDelta);
    }

}// namespace PerceptionMapUpdate::Test
