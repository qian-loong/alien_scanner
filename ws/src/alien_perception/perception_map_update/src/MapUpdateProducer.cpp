#include "perception_map_update/MapUpdateProducer.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/SnapshotDiffer.hpp"

#include <chrono>
#include <limits>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        MapUpdate make_common_update(
                const CanonicalSnapshot & target,
                std::uint64_t observed_coalesced_receipt_count)
        {
            MapUpdate update;
            update.source = target.source;
            update.geometry = target.geometry;
            update.new_revision = target.revision;
            update.observed_coalesced_receipt_count = observed_coalesced_receipt_count;
            update.geometry_fingerprint = target.geometry_fingerprint;
            update.content_hash = target.content_hash;
            update.latest_commit = target.latest_commit;
            update.known_cell_count = static_cast<std::uint64_t>(target.cells.size());
            return update;
        }

        std::int64_t elapsed_ns(std::chrono::steady_clock::time_point start) noexcept
        {
            return std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - start)
                    .count();
        }

    }// namespace

    MapUpdateProducer::MapUpdateProducer(MapUpdateLimits limits) : limits_(std::move(limits)) {}

    ValidationResult MapUpdateProducer::validate_snapshot(
            const CanonicalSnapshot & snapshot) const
    {
        const auto identity = CanonicalCodec::validate_identity(snapshot.source, limits_);
        if(!identity) {
            return identity;
        }
        const auto geometry = CanonicalCodec::validate_geometry(snapshot.geometry, limits_);
        if(!geometry) {
            return geometry;
        }
        const auto provenance = CanonicalCodec::validate_provenance(
                snapshot.latest_commit, limits_);
        if(!provenance) {
            return provenance;
        }
        const auto cells = CanonicalCodec::validate_cells(snapshot.cells, limits_);
        if(!cells) {
            return cells;
        }
        if(snapshot.revision == 0U) {
            return {false, "snapshot revision must be non-zero"};
        }
        const auto expected_geometry = ContentHasher::geometry_fingerprint(snapshot.geometry);
        if(expected_geometry != snapshot.geometry_fingerprint) {
            return {false, "snapshot geometry fingerprint mismatch"};
        }
        const auto expected_content = ContentHasher::content_hash(
                snapshot.source, snapshot.geometry_fingerprint, snapshot.cells);
        if(expected_content != snapshot.content_hash) {
            return {false, "snapshot content hash mismatch"};
        }
        if(snapshot.cells.size()
           > std::numeric_limits<std::size_t>::max() / sizeof(CanonicalCell)) {
            return {false, "snapshot retained-state estimate overflows size_t"};
        }
        const std::size_t estimated = snapshot.cells.size() * sizeof(CanonicalCell);
        if(estimated > limits_.max_retained_snapshot_bytes) {
            return {false, "snapshot retained-state estimate exceeds configured limit"};
        }
        return {true, {}};
    }

    PreparedUpdate MapUpdateProducer::make_keyframe(
            std::shared_ptr<const CanonicalSnapshot> target,
            std::uint64_t observed_coalesced_receipt_count,
            PrepareTiming timing) const
    {
        if(!target) {
            return {ProduceStatus::RejectedInvalidSnapshot, std::nullopt, nullptr,
                    "target snapshot is null", timing, std::nullopt};
        }
        const auto & snapshot = *target;
        const auto encode_start = std::chrono::steady_clock::now();
        auto encoded = CanonicalCodec::encode_keyframe_payload(snapshot.cells, limits_);
        timing.encode_duration_ns = elapsed_ns(encode_start);
        if(!encoded.success) {
            return {ProduceStatus::RejectedResourceLimit, std::nullopt, nullptr,
                    encoded.diagnostic, timing, std::nullopt};
        }
        auto update = make_common_update(snapshot, observed_coalesced_receipt_count);
        update.kind = UpdateKind::Keyframe;
        update.base_revision = 0U;
        update.revision_span = snapshot.revision;
        update.base_content_hash = {};
        update.operation_count = 0U;
        update.payload = std::move(encoded.bytes);
        update.canonical_payload_bytes = static_cast<std::uint64_t>(update.payload.size());
        if(pending_correlation_id_.has_value()) {
            update.correlation_id = *pending_correlation_id_;
        }
        const auto hash_start = std::chrono::steady_clock::now();
        update.update_hash = ContentHasher::update_hash(update);
        timing.update_hash_duration_ns = elapsed_ns(hash_start);
        PreparedUpdate prepared {
                ProduceStatus::ProducedKeyframe,
                std::move(update),
                std::move(target),
                {},
                timing,
                std::nullopt};
        prepared.expected_baseline = current_baseline_token();
        return prepared;
    }

    PreparedUpdate MapUpdateProducer::prepare(
            const CanonicalSnapshot & target,
            std::uint64_t observed_coalesced_receipt_count) const
    {
        return prepare(
                std::make_shared<const CanonicalSnapshot>(target),
                observed_coalesced_receipt_count);
    }

    PreparedUpdate MapUpdateProducer::prepare(
            std::shared_ptr<const CanonicalSnapshot> target,
            std::uint64_t observed_coalesced_receipt_count) const
    {
        PrepareTiming timing;
        if(!target) {
            return {ProduceStatus::RejectedInvalidSnapshot, std::nullopt, nullptr,
                    "target snapshot is null", timing, std::nullopt};
        }
        const auto & snapshot = *target;
        const auto validation_start = std::chrono::steady_clock::now();
        const auto validation = validate_snapshot(snapshot);
        timing.validation_duration_ns = elapsed_ns(validation_start);
        if(!validation) {
            return {ProduceStatus::RejectedInvalidSnapshot, std::nullopt, nullptr,
                    validation.diagnostic, timing, std::nullopt};
        }
        if(!baseline_ || pending_correlation_id_.has_value()
           || !limits_.delta_enabled || baseline_->source != snapshot.source
           || !(baseline_->geometry == snapshot.geometry)
           || baseline_->geometry_fingerprint != snapshot.geometry_fingerprint
           || (limits_.max_delta_chain_length != 0U
               && delta_chain_length_ >= limits_.max_delta_chain_length)
           || (limits_.periodic_keyframe_revision_interval != 0U
               && snapshot.revision > last_keyframe_revision_
               && snapshot.revision - last_keyframe_revision_
                          >= limits_.periodic_keyframe_revision_interval)) {
            return make_keyframe(target, observed_coalesced_receipt_count, timing);
        }
        if(snapshot.revision == baseline_->revision
           && snapshot.content_hash == baseline_->content_hash) {
            return {ProduceStatus::NoNewRevision,
                    std::nullopt,
                    nullptr,
                    {},
                    timing,
                    std::nullopt};
        }
        if(snapshot.revision <= baseline_->revision) {
            return {ProduceStatus::RejectedInvalidSnapshot, std::nullopt, nullptr,
                    "target revision does not advance the producer baseline",
                    timing,
                    std::nullopt};
        }
        if(snapshot.revision - baseline_->revision > limits_.max_revision_span) {
            return make_keyframe(target, observed_coalesced_receipt_count, timing);
        }
        const auto diff_start = std::chrono::steady_clock::now();
        const auto diff = SnapshotDiffer::compare(*baseline_, snapshot, limits_);
        timing.diff_duration_ns = elapsed_ns(diff_start);
        if(!diff.success) {
            return make_keyframe(target, observed_coalesced_receipt_count, timing);
        }
        const auto encode_start = std::chrono::steady_clock::now();
        auto encoded = CanonicalCodec::encode_delta_payload(diff.operations, limits_);
        timing.encode_duration_ns = elapsed_ns(encode_start);
        if(!encoded.success) {
            return make_keyframe(target, observed_coalesced_receipt_count, timing);
        }
        auto update = make_common_update(snapshot, observed_coalesced_receipt_count);
        update.kind = UpdateKind::Delta;
        update.base_revision = baseline_->revision;
        update.revision_span = snapshot.revision - baseline_->revision;
        update.base_content_hash = baseline_->content_hash;
        update.operation_count = static_cast<std::uint64_t>(diff.operations.size());
        update.payload = std::move(encoded.bytes);
        update.canonical_payload_bytes = static_cast<std::uint64_t>(update.payload.size());
        const auto hash_start = std::chrono::steady_clock::now();
        update.update_hash = ContentHasher::update_hash(update);
        timing.update_hash_duration_ns = elapsed_ns(hash_start);
        PreparedUpdate prepared {
                ProduceStatus::ProducedDelta,
                std::move(update),
                std::move(target),
                {},
                timing,
                std::nullopt};
        prepared.expected_baseline = current_baseline_token();
        return prepared;
    }

    bool MapUpdateProducer::commit_published(const PreparedUpdate & prepared)
    {
        if(!prepared.update.has_value() || !prepared.target_snapshot) {
            return false;
        }
        const auto & update = *prepared.update;
        const auto & target = *prepared.target_snapshot;
        if(prepared.expected_baseline.has_value() != static_cast<bool>(baseline_)) {
            return false;
        }
        if(prepared.expected_baseline.has_value()) {
            const auto & expected = *prepared.expected_baseline;
            if(baseline_->source != expected.source
               || baseline_->revision != expected.revision
               || baseline_->content_hash != expected.content_hash) {
                return false;
            }
        }
        if(update.source != target.source || update.new_revision != target.revision
           || update.content_hash != target.content_hash
           || update.update_hash != ContentHasher::update_hash(update)) {
            return false;
        }
        if((prepared.status == ProduceStatus::ProducedKeyframe
            && update.kind != UpdateKind::Keyframe)
           || (prepared.status == ProduceStatus::ProducedDelta
               && update.kind != UpdateKind::Delta)) {
            return false;
        }
        if(update.kind == UpdateKind::Delta
           && delta_chain_length_ == std::numeric_limits<std::uint64_t>::max()) {
            return false;
        }
        if(update.kind == UpdateKind::Keyframe) {
            baseline_ = prepared.target_snapshot;
            delta_chain_length_ = 0U;
            last_keyframe_revision_ = target.revision;
            if(pending_correlation_id_.has_value()
               && update.correlation_id == pending_correlation_id_) {
                pending_correlation_id_.reset();
            }
        } else if(update.kind == UpdateKind::Delta) {
            baseline_ = prepared.target_snapshot;
            ++delta_chain_length_;
        } else {
            return false;
        }
        return true;
    }

    std::optional<ProducerBaselineToken>
    MapUpdateProducer::current_baseline_token() const
    {
        if(!baseline_) {
            return std::nullopt;
        }
        return ProducerBaselineToken {
                baseline_->source, baseline_->revision, baseline_->content_hash};
    }

    bool MapUpdateProducer::request_keyframe(std::string correlation_id)
    {
        if(!CanonicalCodec::validate_string(
                   correlation_id,
                   limits_.max_correlation_id_bytes,
                   "correlation_id",
                   false)) {
            return false;
        }
        if(pending_correlation_id_.has_value()) {
            return *pending_correlation_id_ == correlation_id;
        }
        pending_correlation_id_ = std::move(correlation_id);
        return true;
    }

    bool MapUpdateProducer::cancel_keyframe_request(
            const std::string & correlation_id) noexcept
    {
        if(!pending_correlation_id_.has_value()
           || *pending_correlation_id_ != correlation_id) {
            return false;
        }
        pending_correlation_id_.reset();
        return true;
    }

    const std::shared_ptr<const CanonicalSnapshot> &
    MapUpdateProducer::baseline() const noexcept
    {
        return baseline_;
    }

    std::uint64_t MapUpdateProducer::delta_chain_length() const noexcept
    {
        return delta_chain_length_;
    }

    bool MapUpdateProducer::keyframe_pending() const noexcept
    {
        return pending_correlation_id_.has_value();
    }

}// namespace PerceptionMapUpdate
