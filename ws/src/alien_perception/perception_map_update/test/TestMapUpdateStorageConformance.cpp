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
            result.geometry_fingerprint = ContentHasher::geometry_fingerprint(result.geometry);
            // Retain the flat digest as an isolated canonical-content oracle. The v2 producer
            // and receiver use only the versioned Merkle identity on the wire.
            result.content_hash = ContentHasher::content_hash(
                    result.source, result.geometry_fingerprint, result.cells);
            return result;
        }

        PreparedUpdate publish(MapUpdateProducer & producer, const CanonicalSnapshot & value)
        {
            auto prepared = producer.prepare(value);
            EXPECT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
            EXPECT_TRUE(producer.commit_published(prepared));
            return prepared;
        }

        ApplyUpdateResult apply(
                MapUpdateApplier & receiver,
                const MapUpdate & update,
                ApplyUpdateStatus expected)
        {
            const auto result = receiver.apply(update);
            EXPECT_EQ(result.status, expected) << result.diagnostic;
            return result;
        }

    }// namespace

    TEST(MapUpdateStorageConformance, V2MerkleReceiverAppliesFullStateLifecycle)
    {
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        MapUpdateProducer producer;

        const auto first = publish(
                producer,
                snapshot(
                        1U,
                        {cell(-17, 0, 0, CellState::Free),
                         cell(0, 0, 0, CellState::Occupied),
                         cell(32, 1, -1, CellState::Free)}));
        ASSERT_TRUE(first.update.has_value());
        apply(receiver, *first.update, ApplyUpdateStatus::AppliedKeyframe);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->cells, first.target_snapshot->cells);
        const VersionedContentDigest first_identity {
                first.update->content_identity,
                first.update->content_hash};
        EXPECT_EQ(
                receiver.reconstructed_map()->content_identity,
                first_identity);

        const auto second = publish(
                producer,
                snapshot(
                        2U,
                        {cell(-17, 0, 0, CellState::Occupied),
                         cell(1, -1, 0, CellState::Free),
                         cell(32, 1, -1, CellState::Free)}));
        ASSERT_TRUE(second.update.has_value());
        ASSERT_EQ(second.update->kind, UpdateKind::Delta);
        apply(receiver, *second.update, ApplyUpdateStatus::AppliedDelta);
        EXPECT_EQ(receiver.reconstructed_map()->cells, second.target_snapshot->cells);

        const auto revision_only = publish(
                producer,
                snapshot(3U, second.target_snapshot->cells));
        ASSERT_TRUE(revision_only.update.has_value());
        ASSERT_EQ(revision_only.update->operation_count, 0U);
        apply(receiver, *revision_only.update, ApplyUpdateStatus::AppliedDelta);
        EXPECT_EQ(receiver.reconstructed_map()->cells, revision_only.target_snapshot->cells);

        MapUpdate summary = *revision_only.update;
        summary.kind = UpdateKind::Summary;
        summary.base_revision = summary.new_revision;
        summary.revision_span = 0U;
        summary.base_content_hash = summary.content_hash;
        summary.operation_count = 0U;
        summary.canonical_payload_bytes = 0U;
        summary.payload.clear();
        summary.update_hash = ContentHasher::update_hash(summary);
        apply(receiver, summary, ApplyUpdateStatus::AcceptedSummary);

        MapUpdate remove = summary;
        remove.kind = UpdateKind::Remove;
        remove.base_revision = 3U;
        remove.new_revision = 4U;
        remove.revision_span = 1U;
        remove.content_hash = {};
        remove.known_cell_count = 0U;
        remove.update_hash = ContentHasher::update_hash(remove);
        apply(receiver, remove, ApplyUpdateStatus::AppliedRemove);
        EXPECT_EQ(receiver.state(), ReceiverState::Removed);
        EXPECT_TRUE(receiver.reconstructed_map()->cells.empty());
        EXPECT_TRUE(receiver.reconstructed_map()->content_identity.digest == Hash256 {});
        EXPECT_EQ(receiver.storage_metrics().total_chunks, 0U);
        EXPECT_EQ(receiver.merkle_metrics().node_count, 0U);
        EXPECT_EQ(receiver.apply(remove).status, ApplyUpdateStatus::IgnoredDuplicate);
    }

    TEST(MapUpdateStorageConformance, RejectsInvalidIdentityAndPreservesCommittedState)
    {
        MapUpdateProducer producer;
        const auto first = publish(producer, snapshot(1U, {cell(0, 0, 0, CellState::Free)}));
        const auto second = publish(producer, snapshot(2U, {cell(0, 0, 0, CellState::Occupied)}));
        ASSERT_TRUE(first.update.has_value());
        ASSERT_TRUE(second.update.has_value());

        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        apply(receiver, *first.update, ApplyUpdateStatus::AppliedKeyframe);

        auto conflict = *second.update;
        conflict.content_hash[0] ^= 0xFFU;
        conflict.update_hash = ContentHasher::update_hash(conflict);
        apply(receiver, conflict, ApplyUpdateStatus::RejectedConflict);
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
        EXPECT_EQ(
                receiver.reconstructed_map()->content_identity.digest,
                first.update->content_hash);

        auto unknown = *first.update;
        unknown.content_identity.chunk_edge = 8U;
        unknown.update_hash = ContentHasher::update_hash(unknown);
        apply(receiver, unknown, ApplyUpdateStatus::RejectedInvalid);
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
    }

    TEST(MapUpdateStorageConformance, ResourceLimitRejectsBeforeCommit)
    {
        MapUpdateLimits limits;
        limits.max_peak_apply_bytes = 1U;
        MapUpdateProducer producer(limits);
        const auto first = producer.prepare(
                snapshot(
                        1U,
                        {cell(-1, 0, 0, CellState::Free),
                         cell(0, 0, 0, CellState::Occupied)}));
        EXPECT_EQ(first.status, ProduceStatus::RejectedResourceLimit);
        EXPECT_FALSE(first.update.has_value());

        MapUpdateApplier receiver(limits);
        ASSERT_TRUE(receiver.admit_source(source()));
        MapUpdateProducer normal;
        const auto valid = publish(normal, snapshot(1U, {cell(0, 0, 0, CellState::Free)}));
        ASSERT_TRUE(valid.update.has_value());
        apply(receiver, *valid.update, ApplyUpdateStatus::RejectedResourceLimit);
        EXPECT_FALSE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.state(), ReceiverState::Empty);
    }

}// namespace PerceptionMapUpdate::Test
