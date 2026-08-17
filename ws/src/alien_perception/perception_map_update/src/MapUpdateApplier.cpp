#include "perception_map_update/MapUpdateApplier.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

        using SteadyClock = std::chrono::steady_clock;

        std::uint64_t elapsed_ns(SteadyClock::time_point start) noexcept
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            SteadyClock::now() - start)
                            .count());
        }

        bool checked_add(std::size_t left, std::size_t right, std::size_t & result) noexcept
        {
            if(right > std::numeric_limits<std::size_t>::max() - left) {
                return false;
            }
            result = left + right;
            return true;
        }

        bool checked_peak(
                std::size_t committed_bytes,
                std::size_t candidate_bytes,
                std::size_t payload_bytes,
                std::size_t operation_count,
                std::size_t & result) noexcept
        {
            std::size_t total = 0U;
            if(!checked_add(committed_bytes, candidate_bytes, total)
               || !checked_add(total, payload_bytes, total)) {
                return false;
            }
            if(operation_count > std::numeric_limits<std::size_t>::max()
                                       / sizeof(DeltaOperation)) {
                return false;
            }
            return checked_add(
                    total,
                    operation_count * sizeof(DeltaOperation),
                    result);
        }

        std::size_t committed_state_bytes(const MerkleMapState & state) noexcept
        {
            std::size_t result = state.store().metrics().committed_live_bytes;
            if(!checked_add(result, state.tree().metrics().owned_bytes, result)) {
                return std::numeric_limits<std::size_t>::max();
            }
            return result;
        }

        const ContentIdentityDescriptor & v2_descriptor() noexcept
        {
            static const ContentIdentityDescriptor descriptor {};
            return descriptor;
        }

        bool descriptor_matches(
                const ContentIdentityDescriptor & descriptor,
                const ContentIdentityDescriptor & expected) noexcept
        {
            return descriptor.valid() && descriptor == expected;
        }

    }// namespace

    MapUpdateApplier::MapUpdateApplier(MapUpdateLimits limits)
        : limits_(std::move(limits))
    {
    }

    bool MapUpdateApplier::is_retired(const SourceIdentity & source) const
    {
        return std::find(retired_sources_.begin(), retired_sources_.end(), source)
               != retired_sources_.end();
    }

    bool MapUpdateApplier::admit_source(const SourceIdentity & source)
    {
        if(!CanonicalCodec::validate_identity(source, limits_) || is_retired(source)) {
            return false;
        }
        if(expected_source_.has_value() && *expected_source_ == source) {
            return true;
        }
        if(expected_source_.has_value()) {
            if(source.vehicle_id != expected_source_->vehicle_id
               || (source.mapper_session == expected_source_->mapper_session
                   && source.map_epoch <= expected_source_->map_epoch)
               || (source.mapper_session != expected_source_->mapper_session
                   && !(expected_source_->mapper_session < source.mapper_session))) {
                return false;
            }
            if(retired_sources_.size() == limits_.max_recent_resync_requests) {
                return false;
            }
            retired_sources_.push_back(*expected_source_);
        }
        expected_source_ = source;
        if(reconstructed_map_.has_value()) {
            state_ = ReceiverState::ResyncRequired;
        }
        else {
            state_ = ReceiverState::Empty;
        }
        map_state_.reset();
        storage_metrics_ = {};
        merkle_metrics_ = {};
        last_occupancy_update_hash_.reset();
        return true;
    }

    bool MapUpdateApplier::require_resync() noexcept
    {
        if(!reconstructed_map_.has_value() || state_ == ReceiverState::Removed) {
            return false;
        }
        state_ = ReceiverState::ResyncRequired;
        return true;
    }

    ValidationResult MapUpdateApplier::validate_envelope(const MapUpdate & update) const
    {
        if(update.protocol_version != kProtocolVersion
           || update.canonical_encoding_version != kCanonicalEncodingVersion
           || update.hash_algorithm != HashAlgorithm::Sha256) {
            return {false, "map update protocol, encoding, or hash version mismatch"};
        }
        if(!descriptor_matches(update.content_identity, v2_descriptor())) {
            return {false, "map update content identity descriptor is unsupported"};
        }
        const auto identity = CanonicalCodec::validate_identity(update.source, limits_);
        if(!identity) {
            return identity;
        }
        const auto geometry = CanonicalCodec::validate_geometry(update.geometry, limits_);
        if(!geometry) {
            return geometry;
        }
        const auto provenance = CanonicalCodec::validate_provenance(
                update.latest_commit, limits_);
        if(!provenance) {
            return provenance;
        }
        const auto correlation = CanonicalCodec::validate_string(
                update.correlation_id,
                limits_.max_correlation_id_bytes,
                "correlation_id",
                true);
        if(!correlation) {
            return correlation;
        }
        if(update.geometry_fingerprint != ContentHasher::geometry_fingerprint(update.geometry)) {
            return {false, "map update geometry fingerprint mismatch"};
        }
        if(update.canonical_payload_bytes != update.payload.size()) {
            return {false, "map update payload byte count mismatch"};
        }
        switch(update.kind) {
            case UpdateKind::Keyframe:
                if(update.known_cell_count > limits_.max_known_cells
                   || update.payload.size() > limits_.max_keyframe_payload_bytes) {
                    return {false, "keyframe envelope exceeds configured resource limits"};
                }
                break;
            case UpdateKind::Delta:
                if(update.operation_count > limits_.max_delta_operations
                   || update.payload.size() > limits_.max_delta_payload_bytes) {
                    return {false, "delta envelope exceeds configured resource limits"};
                }
                break;
            case UpdateKind::Summary:
            case UpdateKind::Remove:
                if(!update.payload.empty()) {
                    return {false, "metadata-only update carries an unexpected payload"};
                }
                break;
            default:
                return {false, "map update kind is unknown"};
        }
        if(update.update_hash != ContentHasher::update_hash(update)) {
            return {false, "map update hash mismatch"};
        }
        return {true, {}};
    }

    ApplyUpdateResult MapUpdateApplier::reject_for_current_chain(
            ApplyUpdateStatus status,
            std::string diagnostic,
            bool require_resync)
    {
        if(require_resync && reconstructed_map_.has_value()
           && state_ != ReceiverState::Removed) {
            state_ = ReceiverState::ResyncRequired;
        }
        return {status, false, std::move(diagnostic)};
    }

    ApplyUpdateResult MapUpdateApplier::apply(const MapUpdate & update)
    {
        last_apply_timing_ = {};
        const auto identity = CanonicalCodec::validate_identity(update.source, limits_);
        if(!identity || !expected_source_.has_value() || update.source != *expected_source_
           || is_retired(update.source)) {
            return {ApplyUpdateStatus::RejectedAdmission, false,
                    identity ? "map update source is not admitted" : identity.diagnostic};
        }
        const auto envelope = validate_envelope(update);
        if(!envelope) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid, envelope.diagnostic, true);
        }
        const bool occupancy_update = update.kind == UpdateKind::Keyframe
                                      || update.kind == UpdateKind::Delta
                                      || update.kind == UpdateKind::Remove;
        if(state_ == ReceiverState::Removed && occupancy_update) {
            if(reconstructed_map_.has_value()
               && reconstructed_map_->source == update.source
               && update.new_revision == reconstructed_map_->revision
               && last_occupancy_update_hash_.has_value()
               && update.update_hash == *last_occupancy_update_hash_) {
                return {ApplyUpdateStatus::IgnoredDuplicate, false, {}};
            }
            return {ApplyUpdateStatus::RejectedStale, false,
                    "removed source chain cannot be reactivated"};
        }
        if(occupancy_update && state_ != ReceiverState::ResyncRequired
           && reconstructed_map_.has_value()
           && reconstructed_map_->source == update.source
           && update.new_revision == reconstructed_map_->revision) {
            if(last_occupancy_update_hash_.has_value()
               && update.update_hash == *last_occupancy_update_hash_) {
                return {ApplyUpdateStatus::IgnoredDuplicate, false, {}};
            }
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "same source revision carries a different occupancy update",
                    true);
        }
        switch(update.kind) {
            case UpdateKind::Keyframe:
                return apply_keyframe(update);
            case UpdateKind::Delta:
                return apply_delta(update);
            case UpdateKind::Summary:
                return apply_summary(update);
            case UpdateKind::Remove:
                return apply_remove(update);
        }
        return reject_for_current_chain(
                ApplyUpdateStatus::RejectedInvalid, "unknown map update kind", true);
    }

    ApplyUpdateResult MapUpdateApplier::apply_keyframe(const MapUpdate & update)
    {
        if(update.base_revision != 0U || !is_zero_hash(update.base_content_hash)
           || update.new_revision == 0U || update.revision_span != update.new_revision
           || update.operation_count != 0U) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "keyframe revision/base fields are inconsistent",
                    true);
        }
        if(reconstructed_map_.has_value() && reconstructed_map_->source == update.source
           && update.new_revision < reconstructed_map_->revision) {
            return {ApplyUpdateStatus::RejectedStale, false, "keyframe revision is stale"};
        }
        const auto payload_size = CanonicalCodec::validate_keyframe_payload_size(
                update.known_cell_count, update.payload.size(), limits_);
        if(!payload_size) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid, payload_size.diagnostic, true);
        }
        if(update.known_cell_count > limits_.max_receiver_cells) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "keyframe declared cell count exceeds receiver limit",
                    true);
        }
        const auto decode_start = SteadyClock::now();
        auto decoded = CanonicalCodec::decode_keyframe_payload(update.payload, limits_);
        last_apply_timing_.payload_decode_duration_ns = elapsed_ns(decode_start);
        if(!decoded.success) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid, decoded.diagnostic, true);
        }
        if(decoded.cells.size() != update.known_cell_count) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "keyframe cell count mismatches envelope",
                    true);
        }

        const auto candidate_start = SteadyClock::now();
        auto result = MerkleMapState::build(
                update.source,
                update.geometry_fingerprint,
                decoded.cells,
                {CellStorageMode::Chunked, kMerkleChunkEdge, kMerkleChunkBucketCount},
                update.content_identity,
                limits_);
        last_apply_timing_.candidate_build_duration_ns = elapsed_ns(candidate_start);
        last_apply_timing_.merkle_duration_ns = result.timings.merkle_ns;
        if(!result) {
            return reject_for_current_chain(
                    result.resource_limit ? ApplyUpdateStatus::RejectedResourceLimit
                                          : ApplyUpdateStatus::RejectedInvalid,
                    result.diagnostic,
                    true);
        }
        if(!result.candidate || result.candidate->cells().size() != update.known_cell_count
           || result.candidate->identity().descriptor != update.content_identity
           || result.candidate->identity().digest != update.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "keyframe locally recomputed content identity mismatches envelope",
                    true);
        }
        std::size_t peak = result.actual_peak_bytes;
        const std::size_t committed = map_state_ ? committed_state_bytes(*map_state_) : 0U;
        if(!checked_peak(
                   committed,
                   peak,
                   update.payload.size(),
                   0U,
                   peak)
           || peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "keyframe peak apply memory exceeds configured limit",
                    true);
        }

        const auto commit_start = SteadyClock::now();
        map_state_ = std::move(result.candidate);
        storage_metrics_ = map_state_->store().metrics();
        merkle_metrics_ = map_state_->tree().metrics();
        reconstructed_map_ = ReconstructedMap {
                update.source,
                update.geometry,
                update.new_revision,
                map_state_->identity(),
                map_state_->cells()};
        last_occupancy_update_hash_ = update.update_hash;
        state_ = ReceiverState::Ready;
        last_apply_timing_.commit_duration_ns = elapsed_ns(commit_start);
        return {ApplyUpdateStatus::AppliedKeyframe, true, {}};
    }

    ApplyUpdateResult MapUpdateApplier::apply_delta(const MapUpdate & update)
    {
        if(state_ != ReceiverState::Ready || !reconstructed_map_.has_value() || !map_state_) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedGap,
                    "delta requires a ready receiver baseline",
                    reconstructed_map_.has_value());
        }
        const auto & current = *reconstructed_map_;
        if(current.source != update.source || !(current.geometry == update.geometry)
           || current.content_identity.descriptor != update.content_identity
           || ContentHasher::geometry_fingerprint(current.geometry)
                      != update.geometry_fingerprint) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta source, geometry, or descriptor mismatches receiver baseline",
                    true);
        }
        if(update.base_revision < current.revision || update.new_revision <= current.revision) {
            return {ApplyUpdateStatus::RejectedStale, false, "delta revision is stale or overlaps"};
        }
        if(update.base_revision > current.revision) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedGap, "delta base revision is in the future", true);
        }
        if(update.base_content_hash != current.content_identity.digest) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta base content identity mismatches receiver state",
                    true);
        }
        if(update.revision_span != update.new_revision - update.base_revision
           || update.revision_span == 0U || update.revision_span > limits_.max_revision_span) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "delta revision span is inconsistent or exceeds limit",
                    true);
        }
        const auto payload_size = CanonicalCodec::validate_delta_payload_size(
                update.operation_count, update.payload.size(), limits_);
        if(!payload_size) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid, payload_size.diagnostic, true);
        }
        if(update.known_cell_count > limits_.max_receiver_cells) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta declared result count exceeds receiver limit",
                    true);
        }
        const auto decode_start = SteadyClock::now();
        const auto decoded = CanonicalCodec::decode_delta_payload(update.payload, limits_);
        last_apply_timing_.payload_decode_duration_ns = elapsed_ns(decode_start);
        if(!decoded.success) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid, decoded.diagnostic, true);
        }
        if(decoded.operations.size() != update.operation_count) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "delta operation count mismatches envelope",
                    true);
        }

        const auto candidate_start = SteadyClock::now();
        auto result = map_state_->apply(decoded.operations);
        last_apply_timing_.candidate_build_duration_ns = elapsed_ns(candidate_start);
        last_apply_timing_.merkle_duration_ns = result.timings.merkle_ns;
        if(!result) {
            return reject_for_current_chain(
                    result.resource_limit ? ApplyUpdateStatus::RejectedResourceLimit
                                          : ApplyUpdateStatus::RejectedInvalid,
                    result.diagnostic,
                    true);
        }
        if(!result.candidate || result.candidate->cells().size() != update.known_cell_count
           || result.candidate->identity().descriptor != update.content_identity
           || result.candidate->identity().digest != update.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta locally recomputed content identity mismatches envelope",
                    true);
        }
        std::size_t peak = 0U;
        const std::size_t committed = committed_state_bytes(*map_state_);
        if(!checked_peak(
                   committed,
                   result.actual_peak_bytes,
                   update.payload.size(),
                   decoded.operations.size(),
                   peak)
           || peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta peak apply memory exceeds configured limit",
                    true);
        }

        const auto commit_start = SteadyClock::now();
        map_state_ = std::move(result.candidate);
        storage_metrics_ = map_state_->store().metrics();
        merkle_metrics_ = map_state_->tree().metrics();
        reconstructed_map_->revision = update.new_revision;
        reconstructed_map_->content_identity = map_state_->identity();
        reconstructed_map_->cells = map_state_->cells();
        last_occupancy_update_hash_ = update.update_hash;
        last_apply_timing_.commit_duration_ns = elapsed_ns(commit_start);
        return {ApplyUpdateStatus::AppliedDelta, true, {}};
    }

    ApplyUpdateResult MapUpdateApplier::apply_summary(const MapUpdate & update)
    {
        if(!reconstructed_map_.has_value()
           || reconstructed_map_->source != update.source
           || !(reconstructed_map_->geometry == update.geometry)
           || update.base_revision != reconstructed_map_->revision
           || update.new_revision != reconstructed_map_->revision
           || update.revision_span != 0U
           || update.content_identity != reconstructed_map_->content_identity.descriptor
           || update.base_content_hash != reconstructed_map_->content_identity.digest
           || update.content_hash != reconstructed_map_->content_identity.digest
           || !update.payload.empty() || update.operation_count != 0U
           || update.known_cell_count != reconstructed_map_->cells.size()) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "summary attempts to alter occupancy state",
                    reconstructed_map_.has_value());
        }
        return {ApplyUpdateStatus::AcceptedSummary, false, {}};
    }

    ApplyUpdateResult MapUpdateApplier::apply_remove(const MapUpdate & update)
    {
        if(!reconstructed_map_.has_value()
           || reconstructed_map_->source != update.source
           || !(reconstructed_map_->geometry == update.geometry)
           || update.base_revision != reconstructed_map_->revision
           || update.content_identity != reconstructed_map_->content_identity.descriptor
           || update.base_content_hash != reconstructed_map_->content_identity.digest
           || update.new_revision <= update.base_revision
           || update.revision_span != update.new_revision - update.base_revision
           || !is_zero_hash(update.content_hash) || !update.payload.empty()
           || update.known_cell_count != 0U || update.operation_count != 0U) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "remove tombstone fields are inconsistent",
                    reconstructed_map_.has_value());
        }
        const auto commit_start = SteadyClock::now();
        map_state_.reset();
        storage_metrics_ = {};
        merkle_metrics_ = {};
        reconstructed_map_->revision = update.new_revision;
        reconstructed_map_->content_identity = {
                update.content_identity,
                {}};
        reconstructed_map_->cells = CanonicalCellView {};
        last_occupancy_update_hash_ = update.update_hash;
        state_ = ReceiverState::Removed;
        last_apply_timing_.commit_duration_ns = elapsed_ns(commit_start);
        return {ApplyUpdateStatus::AppliedRemove, true, {}};
    }

    ReceiverState MapUpdateApplier::state() const noexcept
    {
        return state_;
    }

    const std::optional<SourceIdentity> & MapUpdateApplier::expected_source() const noexcept
    {
        return expected_source_;
    }

    const std::optional<ReconstructedMap> & MapUpdateApplier::reconstructed_map() const noexcept
    {
        return reconstructed_map_;
    }

    const CellStorageMetrics & MapUpdateApplier::storage_metrics() const noexcept
    {
        return storage_metrics_;
    }

    const MerkleTreeMetrics & MapUpdateApplier::merkle_metrics() const noexcept
    {
        return merkle_metrics_;
    }

    const ApplyUpdateTiming & MapUpdateApplier::last_apply_timing() const noexcept
    {
        return last_apply_timing_;
    }

    const void * MapUpdateApplier::chunk_identity(const VoxelIndex & index) const noexcept
    {
        return map_state_ ? map_state_->store().chunk_identity(index) : nullptr;
    }

}// namespace PerceptionMapUpdate
