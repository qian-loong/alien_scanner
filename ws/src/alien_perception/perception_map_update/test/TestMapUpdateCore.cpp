#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "perception_map_update/ResyncStateMachine.hpp"
#include "perception_map_update/SnapshotDiffer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace PerceptionMapUpdate::Test {

    namespace {

        SourceIdentity source(
                std::uint64_t map_epoch = 1U,
                std::uint64_t boot_time = 100U)
        {
            return {"drone_0", {boot_time, 7U}, map_epoch};
        }

        MapGeometry geometry(double origin_x = 0.0)
        {
            return {0.1, {origin_x, 0.0, 0.0}, "drone_0/map"};
        }

        RevisionProvenance provenance(std::uint32_t changed = 1U)
        {
            return {"lidar", {200U, 9U}, Perception::Timestamp {3'000'000U},
                    "sim-clock", changed};
        }

        CanonicalCell cell(std::int64_t x, CellState state)
        {
            return {{x, 0, 0}, state};
        }

        CanonicalSnapshot snapshot(
                std::uint64_t revision,
                std::vector<CanonicalCell> cells,
                SourceIdentity identity = source(),
                MapGeometry map_geometry = geometry())
        {
            CanonicalSnapshot result;
            result.source = std::move(identity);
            result.geometry = std::move(map_geometry);
            result.revision = revision;
            result.latest_commit = provenance(static_cast<std::uint32_t>(cells.size()));
            result.cells = std::move(cells);
            result.geometry_fingerprint = ContentHasher::geometry_fingerprint(result.geometry);
            result.content_hash = ContentHasher::content_hash(
                    result.source, result.geometry_fingerprint, result.cells);
            return result;
        }

        PreparedUpdate publish(
                MapUpdateProducer & producer,
                const CanonicalSnapshot & value,
                std::uint64_t coalesced = 0U)
        {
            auto prepared = producer.prepare(value, coalesced);
            EXPECT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
            EXPECT_TRUE(producer.commit_published(prepared));
            return prepared;
        }

        MapUpdate with_recomputed_update_hash(MapUpdate update)
        {
            update.update_hash = ContentHasher::update_hash(update);
            return update;
        }

    }// namespace

    TEST(CanonicalCodecTest, KeyframeRoundTripUsesStableBigEndianLayout)
    {
        const std::vector<CanonicalCell> cells {
                cell(-1, CellState::Free), cell(2, CellState::Occupied)};
        const auto encoded = CanonicalCodec::encode_keyframe_payload(cells, {});
        ASSERT_TRUE(encoded.success) << encoded.diagnostic;
        ASSERT_EQ(encoded.bytes.size(), 61U);
        EXPECT_EQ(encoded.bytes[0], 0U);
        EXPECT_EQ(encoded.bytes[1], 1U);
        EXPECT_EQ(encoded.bytes[2], static_cast<std::uint8_t>(UpdateKind::Keyframe));
        EXPECT_EQ(encoded.bytes[10], 2U);

        const auto decoded = CanonicalCodec::decode_keyframe_payload(encoded.bytes, {});
        ASSERT_TRUE(decoded.success) << decoded.diagnostic;
        EXPECT_EQ(decoded.cells, cells);
    }

    TEST(CanonicalCodecTest, RejectsTrailingTruncatedAndNonCanonicalEntries)
    {
        std::vector<CanonicalCell> duplicate {
                cell(1, CellState::Free), cell(1, CellState::Occupied)};
        EXPECT_FALSE(CanonicalCodec::encode_keyframe_payload(duplicate, {}).success);

        const auto valid = CanonicalCodec::encode_delta_payload(
                {{{1, 0, 0}, DeltaOperationKind::UpsertFree}}, {});
        ASSERT_TRUE(valid.success);
        auto trailing = valid.bytes;
        trailing.push_back(0U);
        EXPECT_FALSE(CanonicalCodec::decode_delta_payload(trailing, {}).success);
        auto truncated = valid.bytes;
        truncated.pop_back();
        EXPECT_FALSE(CanonicalCodec::decode_delta_payload(truncated, {}).success);

        EXPECT_FALSE(CanonicalCodec::validate_delta_payload_size(
                             std::numeric_limits<std::uint64_t>::max(),
                             valid.bytes.size(),
                             {}));
    }

    TEST(ContentHasherTest, NormalizesNegativeZeroAndSeparatesHashDomains)
    {
        const auto positive = ContentHasher::geometry_fingerprint(geometry(0.0));
        const auto negative = ContentHasher::geometry_fingerprint(geometry(-0.0));
        EXPECT_EQ(positive, negative);

        const auto empty = snapshot(1U, {});
        EXPECT_NE(empty.geometry_fingerprint, empty.content_hash);
        EXPECT_EQ(hash_to_hex(empty.geometry_fingerprint).size(), 64U);
        EXPECT_EQ(hash_to_hex(empty.content_hash).size(), 64U);
    }

    TEST(ContentHasherTest, MatchesGoldenHashesAcrossDigestBufferBoundary)
    {
        const MapGeometry map_geometry {0.5, {-0.0, 1.25, -2.5}, "map"};
        const auto geometry_hash = ContentHasher::geometry_fingerprint(map_geometry);
        EXPECT_EQ(
                hash_to_hex(geometry_hash),
                "1eb8467467d328a5a7fa3c2bb6639462d6138e99fc06dd447a825dfd3ecc27b6");

        std::vector<CanonicalCell> cells;
        cells.reserve(3'000U);
        for(std::int64_t index = 0; index < 3'000; ++index) {
            cells.push_back(
                    {{index, -index, 7},
                     index % 2 == 0 ? CellState::Free : CellState::Occupied});
        }
        const SourceIdentity identity {"drone", {1U, 2U}, 3U};
        EXPECT_EQ(
                hash_to_hex(ContentHasher::content_hash(identity, geometry_hash, cells)),
                "6acaf06cb3977547549e891ffbb71e5adee885c4369827719bb4aa41883bdc9c");
    }

    TEST(SnapshotDifferTest, ExpressesAddRemoveFlipAndRevisionOnlyTransitions)
    {
        const auto base = snapshot(
                4U,
                {cell(0, CellState::Free),
                 cell(1, CellState::Occupied),
                 cell(3, CellState::Free)});
        const auto target = snapshot(
                7U,
                {cell(0, CellState::Occupied),
                 cell(2, CellState::Free),
                 cell(3, CellState::Free)});
        const auto diff = SnapshotDiffer::compare(base, target, {});
        ASSERT_TRUE(diff.success) << diff.diagnostic;
        const std::vector<DeltaOperation> expected {
                {{0, 0, 0}, DeltaOperationKind::UpsertOccupied},
                {{1, 0, 0}, DeltaOperationKind::RemoveToUnknown},
                {{2, 0, 0}, DeltaOperationKind::UpsertFree}};
        EXPECT_EQ(diff.operations, expected);

        const auto revision_only = SnapshotDiffer::compare(
                target,
                snapshot(8U, target.cells),
                {});
        ASSERT_TRUE(revision_only.success);
        EXPECT_TRUE(revision_only.operations.empty());
    }

    TEST(MapUpdateProducerTest, BaselineAdvancesOnlyAfterPublishCommit)
    {
        MapUpdateProducer producer;
        const auto rejected = producer.prepare(
                std::shared_ptr<const CanonicalSnapshot> {});
        EXPECT_EQ(rejected.status, ProduceStatus::RejectedInvalidSnapshot);
        EXPECT_FALSE(rejected.update.has_value());

        const auto initial = std::make_shared<const CanonicalSnapshot>(
                snapshot(1U, {cell(0, CellState::Free)}));
        const auto prepared = producer.prepare(initial);
        ASSERT_EQ(prepared.status, ProduceStatus::ProducedKeyframe);
        EXPECT_FALSE(producer.baseline());
        ASSERT_TRUE(prepared.update.has_value());
        ASSERT_TRUE(prepared.target_snapshot);
        EXPECT_EQ(prepared.target_snapshot.get(), initial.get());
        EXPECT_EQ(prepared.update->kind, UpdateKind::Keyframe);
        EXPECT_TRUE(producer.commit_published(prepared));
        ASSERT_TRUE(producer.baseline());
        EXPECT_EQ(producer.baseline().get(), initial.get());
        EXPECT_EQ(producer.baseline()->revision, 1U);
    }

    TEST(MapUpdateProducerTest, StalePublishCommitCannotRollBackBaseline)
    {
        MapUpdateProducer producer;
        publish(producer, snapshot(1U, {cell(0, CellState::Free)}));

        const auto stale = producer.prepare(
                snapshot(2U, {cell(0, CellState::Occupied)}));
        const auto latest = producer.prepare(
                snapshot(3U, {cell(0, CellState::Free), cell(1, CellState::Occupied)}));
        ASSERT_TRUE(stale.update.has_value());
        ASSERT_TRUE(latest.update.has_value());
        ASSERT_TRUE(producer.commit_published(latest));
        ASSERT_TRUE(producer.baseline());
        ASSERT_EQ(producer.baseline()->revision, 3U);

        EXPECT_FALSE(producer.commit_published(stale));
        ASSERT_TRUE(producer.baseline());
        EXPECT_EQ(producer.baseline()->revision, 3U);
        EXPECT_EQ(producer.baseline()->content_hash, latest.target_snapshot->content_hash);
    }

    TEST(MapUpdateProducerTest, CoalescesRevisionSpanAndProducesRevisionOnlyDelta)
    {
        MapUpdateProducer producer;
        const auto initial = snapshot(2U, {cell(0, CellState::Free)});
        publish(producer, initial);

        const auto target = snapshot(
                5U,
                {cell(0, CellState::Free), cell(1, CellState::Occupied)});
        const auto delta = publish(producer, target, 2U);
        ASSERT_EQ(delta.status, ProduceStatus::ProducedDelta);
        ASSERT_TRUE(delta.update.has_value());
        EXPECT_EQ(delta.update->base_revision, 2U);
        EXPECT_EQ(delta.update->new_revision, 5U);
        EXPECT_EQ(delta.update->revision_span, 3U);
        EXPECT_EQ(delta.update->observed_coalesced_receipt_count, 2U);

        const auto revision_only = publish(producer, snapshot(6U, target.cells));
        ASSERT_TRUE(revision_only.update.has_value());
        EXPECT_EQ(revision_only.update->kind, UpdateKind::Delta);
        EXPECT_EQ(revision_only.update->operation_count, 0U);
        EXPECT_EQ(revision_only.update->base_content_hash, revision_only.update->content_hash);
    }

    TEST(MapUpdateProducerTest, ResyncRequestForcesCorrelatedKeyframeIdempotently)
    {
        MapUpdateProducer producer;
        publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(producer.request_keyframe("request-1"));
        EXPECT_TRUE(producer.request_keyframe("request-1"));
        EXPECT_FALSE(producer.request_keyframe("request-2"));
        const auto keyframe = producer.prepare(snapshot(2U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(keyframe.update.has_value());
        EXPECT_EQ(keyframe.update->kind, UpdateKind::Keyframe);
        EXPECT_EQ(keyframe.update->correlation_id, "request-1");
        EXPECT_TRUE(producer.commit_published(keyframe));
        EXPECT_FALSE(producer.keyframe_pending());
    }

    TEST(MapUpdateProducerTest, PeriodicAndOversizedRevisionSpansFallBackToKeyframes)
    {
        MapUpdateLimits periodic_limits;
        periodic_limits.periodic_keyframe_revision_interval = 3U;
        MapUpdateProducer periodic(periodic_limits);
        publish(periodic, snapshot(1U, {cell(0, CellState::Free)}));
        EXPECT_EQ(
                publish(periodic, snapshot(2U, {cell(0, CellState::Occupied)})).update->kind,
                UpdateKind::Delta);
        EXPECT_EQ(
                publish(periodic, snapshot(3U, {cell(0, CellState::Free)})).update->kind,
                UpdateKind::Delta);
        EXPECT_EQ(
                publish(periodic, snapshot(4U, {cell(0, CellState::Occupied)})).update->kind,
                UpdateKind::Keyframe);

        MapUpdateLimits span_limits;
        span_limits.max_revision_span = 2U;
        MapUpdateProducer spanning(span_limits);
        publish(spanning, snapshot(1U, {cell(0, CellState::Free)}));
        const auto fallback = publish(
                spanning, snapshot(4U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(fallback.update.has_value());
        EXPECT_EQ(fallback.update->kind, UpdateKind::Keyframe);
        EXPECT_EQ(fallback.update->new_revision, 4U);
    }

    TEST(MapUpdateApplierTest, AppliesKeyframeDeltaAndRevisionOnlyDelta)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));

        const auto first = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(first.update.has_value());
        EXPECT_EQ(receiver.apply(*first.update).status, ApplyUpdateStatus::AppliedKeyframe);
        EXPECT_EQ(receiver.state(), ReceiverState::Ready);

        const auto second = publish(
                producer,
                snapshot(3U, {cell(0, CellState::Occupied), cell(2, CellState::Free)}));
        ASSERT_TRUE(second.update.has_value());
        EXPECT_EQ(receiver.apply(*second.update).status, ApplyUpdateStatus::AppliedDelta);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 3U);
        EXPECT_EQ(receiver.reconstructed_map()->cells, second.target_snapshot->cells);

        const auto third = publish(producer, snapshot(4U, second.target_snapshot->cells));
        ASSERT_TRUE(third.update.has_value());
        EXPECT_EQ(third.update->operation_count, 0U);
        EXPECT_EQ(receiver.apply(*third.update).status, ApplyUpdateStatus::AppliedDelta);
        EXPECT_EQ(receiver.reconstructed_map()->revision, 4U);
    }

    TEST(MapUpdateApplierTest, GapPreservesStateAndSameRevisionKeyframeRecovers)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        const auto first = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(first.update.has_value());
        ASSERT_EQ(receiver.apply(*first.update).status, ApplyUpdateStatus::AppliedKeyframe);

        const auto valid_delta = publish(
                producer,
                snapshot(2U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(valid_delta.update.has_value());
        auto gap = *valid_delta.update;
        gap.base_revision = 3U;
        gap.new_revision = 4U;
        gap.revision_span = 1U;
        gap = with_recomputed_update_hash(std::move(gap));
        EXPECT_EQ(receiver.apply(gap).status, ApplyUpdateStatus::RejectedGap);
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);

        MapUpdateProducer resync_producer;
        const auto recovery = publish(
                resync_producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(recovery.update.has_value());
        EXPECT_EQ(receiver.apply(*recovery.update).status, ApplyUpdateStatus::AppliedKeyframe);
        EXPECT_EQ(receiver.state(), ReceiverState::Ready);
    }

    TEST(MapUpdateApplierTest, DuplicateRequiresTheCommittedOccupancyUpdateHash)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        const auto first = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(first.update.has_value());
        ASSERT_EQ(receiver.apply(*first.update).status, ApplyUpdateStatus::AppliedKeyframe);
        EXPECT_EQ(receiver.apply(*first.update).status, ApplyUpdateStatus::IgnoredDuplicate);

        auto semantic_conflict = *first.update;
        semantic_conflict.observed_coalesced_receipt_count = 1U;
        semantic_conflict = with_recomputed_update_hash(std::move(semantic_conflict));
        EXPECT_EQ(
                receiver.apply(semantic_conflict).status,
                ApplyUpdateStatus::RejectedConflict);
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
    }

    TEST(CanonicalCodecTest, RejectsNonCanonicalUtf8AcrossCoreStrings)
    {
        const std::string invalid_utf8("\xC0\xAF", 2U);
        auto invalid_source = source();
        invalid_source.vehicle_id = invalid_utf8;
        EXPECT_FALSE(CanonicalCodec::validate_identity(invalid_source, {}));

        MapUpdateProducer producer;
        EXPECT_FALSE(producer.request_keyframe(invalid_utf8));

        ResyncRequestLedger ledger;
        ResyncRequest request {
                {invalid_utf8, {500U, 1U}}, "client-1", std::nullopt, 0U, {},
                ResyncReason::InitialBaseline};
        EXPECT_FALSE(ledger.accept(request, source(), 1U).accepted);
    }

    TEST(MapUpdateApplierTest, NewAdmittedEpochCanStartAtLowerRevision)
    {
        MapUpdateApplier receiver;
        MapUpdateProducer first_producer;
        ASSERT_TRUE(receiver.admit_source(source(1U)));
        const auto old_map = publish(
                first_producer, snapshot(9U, {cell(0, CellState::Free)}, source(1U)));
        ASSERT_TRUE(old_map.update.has_value());
        ASSERT_EQ(receiver.apply(*old_map.update).status, ApplyUpdateStatus::AppliedKeyframe);

        ASSERT_TRUE(receiver.admit_source(source(2U)));
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        MapUpdateProducer second_producer;
        const auto new_map = publish(
                second_producer, snapshot(1U, {cell(1, CellState::Occupied)}, source(2U)));
        ASSERT_TRUE(new_map.update.has_value());
        EXPECT_EQ(receiver.apply(*new_map.update).status, ApplyUpdateStatus::AppliedKeyframe);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->source.map_epoch, 2U);
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
        EXPECT_EQ(receiver.apply(*old_map.update).status, ApplyUpdateStatus::RejectedAdmission);
    }

    TEST(MapUpdateApplierTest, SourceAdmissionRejectsUnseenOlderEpochAndSession)
    {
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source(2U, 200U)));
        EXPECT_FALSE(receiver.admit_source(source(1U, 200U)));
        EXPECT_FALSE(receiver.admit_source(source(9U, 100U)));
        auto other_vehicle = source(3U, 200U);
        other_vehicle.vehicle_id = "drone_1";
        EXPECT_FALSE(receiver.admit_source(other_vehicle));
        EXPECT_TRUE(receiver.admit_source(source(1U, 300U)));
        ASSERT_TRUE(receiver.expected_source().has_value());
        EXPECT_EQ(*receiver.expected_source(), source(1U, 300U));
    }

    TEST(MapUpdateApplierTest, CorruptPayloadCannotPartiallyCommit)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        const auto initial = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(initial.update.has_value());
        ASSERT_EQ(receiver.apply(*initial.update).status, ApplyUpdateStatus::AppliedKeyframe);
        const auto next = publish(producer, snapshot(2U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(next.update.has_value());
        auto corrupt = *next.update;
        ASSERT_FALSE(corrupt.payload.empty());
        corrupt.payload.back() = static_cast<std::uint8_t>(DeltaOperationKind::UpsertFree);
        corrupt = with_recomputed_update_hash(std::move(corrupt));

        EXPECT_EQ(receiver.apply(corrupt).status, ApplyUpdateStatus::RejectedInvalid);
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
        EXPECT_EQ(receiver.reconstructed_map()->cells,
                  std::vector<CanonicalCell>({cell(0, CellState::Free)}));
    }

    TEST(MapUpdateApplierTest, EnvelopeCountMismatchIsRejectedBeforeDecodeAllocation)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        const auto initial = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(initial.update.has_value());
        ASSERT_EQ(receiver.apply(*initial.update).status, ApplyUpdateStatus::AppliedKeyframe);
        const auto next = publish(producer, snapshot(2U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(next.update.has_value());

        auto mismatched = *next.update;
        mismatched.operation_count = 0U;
        mismatched = with_recomputed_update_hash(std::move(mismatched));
        EXPECT_EQ(receiver.apply(mismatched).status, ApplyUpdateStatus::RejectedInvalid);
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
    }

    TEST(MapUpdateApplierTest, ExternalDecodeFailureRequiresKeyframeWithoutMutatingState)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        const auto initial = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(initial.update.has_value());
        ASSERT_EQ(receiver.apply(*initial.update).status, ApplyUpdateStatus::AppliedKeyframe);

        const auto next = publish(producer, snapshot(2U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(next.update.has_value());
        ASSERT_TRUE(receiver.require_resync());
        EXPECT_EQ(receiver.state(), ReceiverState::ResyncRequired);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);
        EXPECT_EQ(receiver.reconstructed_map()->cells,
                  std::vector<CanonicalCell>({cell(0, CellState::Free)}));
        EXPECT_EQ(receiver.apply(*next.update).status, ApplyUpdateStatus::RejectedGap);
        EXPECT_EQ(receiver.reconstructed_map()->revision, 1U);

        MapUpdateProducer recovery_producer;
        const auto recovery = publish(
                recovery_producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(recovery.update.has_value());
        EXPECT_EQ(receiver.apply(*recovery.update).status, ApplyUpdateStatus::AppliedKeyframe);
        EXPECT_EQ(receiver.state(), ReceiverState::Ready);
    }

    TEST(MapUpdateApplierTest, FullCapacityAllowsNonGrowingDelta)
    {
        MapUpdateLimits limits;
        limits.max_known_cells = 2U;
        limits.max_receiver_cells = 2U;
        limits.max_delta_operations = 2U;
        MapUpdateProducer producer(limits);
        MapUpdateApplier receiver(limits);
        ASSERT_TRUE(receiver.admit_source(source()));

        const auto initial = publish(
                producer,
                snapshot(
                        1U,
                        {cell(0, CellState::Free), cell(1, CellState::Occupied)}));
        ASSERT_TRUE(initial.update.has_value());
        ASSERT_EQ(receiver.apply(*initial.update).status, ApplyUpdateStatus::AppliedKeyframe);

        const auto flips = publish(
                producer,
                snapshot(
                        2U,
                        {cell(0, CellState::Occupied), cell(1, CellState::Free)}));
        ASSERT_TRUE(flips.update.has_value());
        EXPECT_EQ(receiver.apply(*flips.update).status, ApplyUpdateStatus::AppliedDelta);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_EQ(receiver.reconstructed_map()->cells, flips.target_snapshot->cells);

        const auto removal = publish(
                producer, snapshot(3U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(removal.update.has_value());
        EXPECT_EQ(receiver.apply(*removal.update).status, ApplyUpdateStatus::AppliedDelta);
        EXPECT_EQ(receiver.reconstructed_map()->cells, removal.target_snapshot->cells);
    }

    TEST(MapUpdateApplierTest, TombstoneRejectsSameChainReactivation)
    {
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        const auto initial = publish(producer, snapshot(1U, {cell(0, CellState::Free)}));
        ASSERT_TRUE(initial.update.has_value());
        ASSERT_EQ(receiver.apply(*initial.update).status, ApplyUpdateStatus::AppliedKeyframe);

        auto remove = *initial.update;
        remove.kind = UpdateKind::Remove;
        remove.base_revision = 1U;
        remove.new_revision = 2U;
        remove.revision_span = 1U;
        remove.base_content_hash = initial.update->content_hash;
        remove.content_hash = {};
        remove.known_cell_count = 0U;
        remove.operation_count = 0U;
        remove.canonical_payload_bytes = 0U;
        remove.payload.clear();
        remove = with_recomputed_update_hash(std::move(remove));
        ASSERT_EQ(receiver.apply(remove).status, ApplyUpdateStatus::AppliedRemove);
        EXPECT_EQ(receiver.state(), ReceiverState::Removed);
        EXPECT_EQ(receiver.apply(remove).status, ApplyUpdateStatus::IgnoredDuplicate);

        const auto delta = publish(
                producer, snapshot(2U, {cell(0, CellState::Occupied)}));
        ASSERT_TRUE(delta.update.has_value());
        EXPECT_EQ(receiver.apply(*delta.update).status, ApplyUpdateStatus::RejectedStale);
        EXPECT_EQ(receiver.state(), ReceiverState::Removed);

        MapUpdateProducer replacement_producer;
        const auto replacement = publish(
                replacement_producer,
                snapshot(3U, {cell(1, CellState::Occupied)}));
        ASSERT_TRUE(replacement.update.has_value());
        EXPECT_EQ(
                receiver.apply(*replacement.update).status,
                ApplyUpdateStatus::RejectedStale);
        EXPECT_EQ(receiver.state(), ReceiverState::Removed);
        ASSERT_TRUE(receiver.reconstructed_map().has_value());
        EXPECT_TRUE(receiver.reconstructed_map()->cells.empty());
        EXPECT_EQ(receiver.reconstructed_map()->revision, 2U);

        auto malformed = replacement.update.value();
        malformed.update_hash.fill(0xffU);
        EXPECT_EQ(
                receiver.apply(malformed).status,
                ApplyUpdateStatus::RejectedInvalid);
        EXPECT_FALSE(receiver.require_resync());
        EXPECT_EQ(receiver.state(), ReceiverState::Removed);
    }

    TEST(ResyncRequestLedgerTest, IsIdempotentRejectsConflictsAndIsBounded)
    {
        MapUpdateLimits limits;
        limits.max_recent_resync_requests = 1U;
        ResyncRequestLedger ledger(limits);
        ResyncRequest request {
                {"receiver", {500U, 1U}}, "client-1", std::nullopt, 0U, {},
                ResyncReason::InitialBaseline};
        const auto first = ledger.accept(request, source(), 4U);
        ASSERT_TRUE(first.accepted) << first.diagnostic;
        const auto duplicate = ledger.accept(request, source(), 4U);
        EXPECT_TRUE(duplicate.accepted);
        EXPECT_EQ(duplicate.correlation_id, first.correlation_id);
        EXPECT_EQ(ledger.size(), 1U);
        const auto retired_duplicate = ledger.accept(request, source(2U), 1U);
        EXPECT_FALSE(retired_duplicate.accepted);
        EXPECT_EQ(retired_duplicate.current_source, source(2U));
        EXPECT_EQ(ledger.size(), 1U);

        auto conflict = request;
        conflict.receiver_revision = 1U;
        EXPECT_FALSE(ledger.accept(conflict, source(), 4U).accepted);
        auto second = request;
        second.client_request_id = "client-2";
        EXPECT_FALSE(ledger.accept(second, source(), 4U).accepted);
        EXPECT_EQ(ledger.size(), 1U);
    }

    TEST(ResyncRequestLedgerTest, LongValidVehicleIdDoesNotBreakCorrelationGeneration)
    {
        MapUpdateLimits limits;
        limits.max_identity_string_bytes = 256U;
        limits.max_correlation_id_bytes = 64U;
        ResyncRequestLedger ledger(limits);
        auto long_source = source();
        long_source.vehicle_id.assign(256U, 'v');
        const ResyncRequest request {
                {"receiver", {500U, 1U}}, "client-1", std::nullopt, 0U, {},
                ResyncReason::InitialBaseline};
        const auto response = ledger.accept(request, long_source, 1U);
        EXPECT_TRUE(response.accepted) << response.diagnostic;
        EXPECT_LE(response.correlation_id.size(), limits.max_correlation_id_bytes);
    }

    TEST(ResyncRequestLedgerTest, RejectsUnknownReasonWithoutConsumingCapacity)
    {
        ResyncRequestLedger ledger;
        ResyncRequest request {
                {"receiver", {500U, 1U}}, "client-invalid", std::nullopt, 0U, {},
                static_cast<ResyncReason>(0U)};
        const auto response = ledger.accept(request, source(), 1U);
        EXPECT_FALSE(response.accepted);
        EXPECT_EQ(response.diagnostic, "resync reason is invalid");
        EXPECT_EQ(ledger.size(), 0U);
    }

    TEST(MapUpdateProducerTest, FailedResyncIntentCanBeCancelledBeforeAnotherRequest)
    {
        MapUpdateProducer producer;
        ASSERT_TRUE(producer.request_keyframe("request-1"));
        EXPECT_FALSE(producer.cancel_keyframe_request("request-2"));
        EXPECT_TRUE(producer.keyframe_pending());
        EXPECT_TRUE(producer.cancel_keyframe_request("request-1"));
        EXPECT_FALSE(producer.keyframe_pending());
        EXPECT_TRUE(producer.request_keyframe("request-2"));
    }

    TEST(MapUpdateReplayTest, FixedSeedCheckpointsRecoverOneDroppedDelta)
    {
        std::mt19937 random(42U);
        std::uniform_int_distribution<int> index_distribution(-200, 200);
        std::uniform_int_distribution<int> action_distribution(0, 2);
        std::map<VoxelIndex, CellState> oracle_cells;
        MapUpdateProducer producer;
        MapUpdateApplier receiver;
        ASSERT_TRUE(receiver.admit_source(source()));
        std::optional<MapUpdate> previous_update;
        std::uint64_t last_applied_revision = 0U;

        for(std::uint64_t revision = 1U; revision <= 100U; ++revision) {
            if(revision % 10U != 0U) {
                for(int mutation = 0; mutation < 5; ++mutation) {
                    const VoxelIndex index {index_distribution(random), 0, 0};
                    switch(action_distribution(random)) {
                        case 0:
                            oracle_cells[index] = CellState::Free;
                            break;
                        case 1:
                            oracle_cells[index] = CellState::Occupied;
                            break;
                        default:
                            oracle_cells.erase(index);
                            break;
                    }
                }
            }
            std::vector<CanonicalCell> cells;
            cells.reserve(oracle_cells.size());
            for(const auto & entry : oracle_cells) {
                cells.push_back({entry.first, entry.second});
            }
            const auto checkpoint = snapshot(revision, std::move(cells));
            auto prepared = producer.prepare(checkpoint);
            ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
            ASSERT_TRUE(producer.commit_published(prepared));

            if(revision == 50U) {
                previous_update = prepared.update;
                continue;
            }
            const auto result = receiver.apply(*prepared.update);
            if(revision == 51U) {
                ASSERT_EQ(result.status, ApplyUpdateStatus::RejectedGap);
                ASSERT_TRUE(receiver.reconstructed_map().has_value());
                EXPECT_EQ(receiver.reconstructed_map()->revision, last_applied_revision);

                ASSERT_TRUE(producer.request_keyframe("fixture-replay-resync"));
                auto recovery = producer.prepare(checkpoint);
                ASSERT_TRUE(recovery.update.has_value()) << recovery.diagnostic;
                ASSERT_EQ(recovery.update->kind, UpdateKind::Keyframe);
                ASSERT_TRUE(producer.commit_published(recovery));
                ASSERT_EQ(
                        receiver.apply(*recovery.update).status,
                        ApplyUpdateStatus::AppliedKeyframe);
                previous_update = recovery.update;
            } else {
                ASSERT_TRUE(
                        result.status == ApplyUpdateStatus::AppliedKeyframe
                        || result.status == ApplyUpdateStatus::AppliedDelta);
                previous_update = prepared.update;
            }

            ASSERT_TRUE(receiver.reconstructed_map().has_value());
            EXPECT_EQ(receiver.reconstructed_map()->revision, revision);
            EXPECT_EQ(receiver.reconstructed_map()->content_hash, checkpoint.content_hash);
            EXPECT_EQ(receiver.reconstructed_map()->cells, checkpoint.cells);
            last_applied_revision = revision;

            if(revision % 7U == 0U) {
                EXPECT_EQ(
                        receiver.apply(*previous_update).status,
                        ApplyUpdateStatus::IgnoredDuplicate);
            }
        }
    }

}// namespace PerceptionMapUpdate::Test
