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
                const VersionedContentDigest & content_identity,
                std::uint64_t observed_coalesced_receipt_count)
        {
            MapUpdate update;
            update.content_identity = content_identity.descriptor;
            update.source = target.source;
            update.geometry = target.geometry;
            update.new_revision = target.revision;
            update.observed_coalesced_receipt_count = observed_coalesced_receipt_count;
            update.geometry_fingerprint = target.geometry_fingerprint;
            update.content_hash = content_identity.digest;
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

        void record_merkle_timing(
                const MerkleMapStateResult & candidate,
                PrepareTiming & timing) noexcept
        {
            timing.store_candidate_duration_ns = static_cast<std::int64_t>(
                    candidate.timings.storage_ns + candidate.timings.mutation_build_ns);
            timing.merkle_duration_ns = static_cast<std::int64_t>(
                    candidate.timings.merkle_ns);
        }

    }// namespace

    bool ProducerBaselineToken::operator==(
            const ProducerBaselineToken & other) const noexcept
    {
        return source == other.source && revision == other.revision
               && content_identity == other.content_identity;
    }

    bool ProducerBaselineToken::operator!=(
            const ProducerBaselineToken & other) const noexcept
    {
        return !(*this == other);
    }

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
                    "target snapshot is null", timing, std::nullopt, nullptr};
        }
        const auto & snapshot = *target;
        const auto state = MerkleMapState::build(
                snapshot.source,
                snapshot.geometry_fingerprint,
                snapshot.cells,
                {CellStorageMode::Chunked, kMerkleChunkEdge, kMerkleChunkBucketCount},
                {},
                limits_);
        record_merkle_timing(state, timing);
        if(!state || !state.candidate) {
            return {state.resource_limit
                            ? ProduceStatus::RejectedResourceLimit
                            : ProduceStatus::RejectedInvalidSnapshot,
                    std::nullopt,
                    nullptr,
                    state.diagnostic,
                    timing,
                    std::nullopt,
                    nullptr};
        }

        const auto encode_start = std::chrono::steady_clock::now();
        auto encoded = CanonicalCodec::encode_keyframe_payload(snapshot.cells, limits_);
        timing.encode_duration_ns = elapsed_ns(encode_start);
        if(!encoded.success) {
            return {ProduceStatus::RejectedResourceLimit, std::nullopt, nullptr,
                    encoded.diagnostic, timing, std::nullopt, nullptr};
        }
        auto update = make_common_update(
                snapshot,
                state.candidate->identity(),
                observed_coalesced_receipt_count);
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
                current_baseline_token(),
                state.candidate};
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
                    "target snapshot is null", timing, std::nullopt, nullptr};
        }
        const auto & snapshot = *target;
        const auto validation_start = std::chrono::steady_clock::now();
        const auto validation = validate_snapshot(snapshot);
        timing.validation_duration_ns = elapsed_ns(validation_start);
        if(!validation) {
            return {ProduceStatus::RejectedInvalidSnapshot, std::nullopt, nullptr,
                    validation.diagnostic, timing, std::nullopt, nullptr};
        }
        if(!baseline_ || !committed_state_ || pending_correlation_id_.has_value()
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
        if(snapshot.revision == baseline_->revision) {
            if(snapshot.cells == baseline_->cells) {
                return {ProduceStatus::NoNewRevision,
                        std::nullopt,
                        nullptr,
                        {},
                        timing,
                        std::nullopt,
                        nullptr};
            }
            return {ProduceStatus::RejectedInvalidSnapshot,
                    std::nullopt,
                    nullptr,
                    "target changes cells without advancing revision",
                    timing,
                    std::nullopt,
                    nullptr};
        }
        if(snapshot.revision < baseline_->revision) {
            return {ProduceStatus::RejectedInvalidSnapshot, std::nullopt, nullptr,
                    "target revision does not advance the producer baseline",
                    timing, std::nullopt, nullptr};
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
        const auto state = committed_state_->apply(diff.operations);
        record_merkle_timing(state, timing);
        if(!state || !state.candidate) {
            return {state.resource_limit
                            ? ProduceStatus::RejectedResourceLimit
                            : ProduceStatus::RejectedInvalidSnapshot,
                    std::nullopt,
                    nullptr,
                    state.diagnostic,
                    timing,
                    current_baseline_token(),
                    nullptr};
        }

        const auto encode_start = std::chrono::steady_clock::now();
        auto encoded = CanonicalCodec::encode_delta_payload(diff.operations, limits_);
        timing.encode_duration_ns = elapsed_ns(encode_start);
        if(!encoded.success) {
            return {ProduceStatus::RejectedResourceLimit, std::nullopt, nullptr,
                    encoded.diagnostic, timing, current_baseline_token(), nullptr};
        }
        auto update = make_common_update(
                snapshot,
                state.candidate->identity(),
                observed_coalesced_receipt_count);
        update.kind = UpdateKind::Delta;
        update.base_revision = baseline_->revision;
        update.revision_span = snapshot.revision - baseline_->revision;
        update.base_content_hash = committed_state_->identity().digest;
        update.operation_count = static_cast<std::uint64_t>(diff.operations.size());
        update.payload = std::move(encoded.bytes);
        update.canonical_payload_bytes = static_cast<std::uint64_t>(update.payload.size());
        const auto hash_start = std::chrono::steady_clock::now();
        update.update_hash = ContentHasher::update_hash(update);
        timing.update_hash_duration_ns = elapsed_ns(hash_start);
        return {ProduceStatus::ProducedDelta,
                std::move(update),
                std::move(target),
                {},
                timing,
                current_baseline_token(),
                state.candidate};
    }

    bool MapUpdateProducer::commit_published(const PreparedUpdate & prepared)
    {
        if(!prepared.update.has_value() || !prepared.target_snapshot
           || !prepared.candidate_state) {
            return false;
        }
        const auto & update = *prepared.update;
        const auto & target = *prepared.target_snapshot;
        if(prepared.expected_baseline.has_value() != static_cast<bool>(baseline_)
           || static_cast<bool>(baseline_) != static_cast<bool>(committed_state_)) {
            return false;
        }
        if(prepared.expected_baseline.has_value()
           && *prepared.expected_baseline != *current_baseline_token()) {
            return false;
        }
        if(update.source != target.source || update.new_revision != target.revision
           || update.content_identity != prepared.candidate_state->identity().descriptor
           || update.content_hash != prepared.candidate_state->identity().digest
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

        baseline_ = prepared.target_snapshot;
        committed_state_ = prepared.candidate_state;
        if(update.kind == UpdateKind::Keyframe) {
            delta_chain_length_ = 0U;
            last_keyframe_revision_ = target.revision;
            if(pending_correlation_id_.has_value()
               && update.correlation_id == pending_correlation_id_) {
                pending_correlation_id_.reset();
            }
        }
        else if(update.kind == UpdateKind::Delta) {
            ++delta_chain_length_;
        }
        else {
            return false;
        }
        return true;
    }

    std::optional<ProducerBaselineToken>
    MapUpdateProducer::current_baseline_token() const
    {
        if(!baseline_ || !committed_state_) {
            return std::nullopt;
        }
        return ProducerBaselineToken {
                baseline_->source,
                baseline_->revision,
                committed_state_->identity()};
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

    const std::shared_ptr<const MerkleMapState> &
    MapUpdateProducer::committed_state() const noexcept
    {
        return committed_state_;
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
