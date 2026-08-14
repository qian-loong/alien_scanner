#include "perception_map_update/MapUpdateApplier.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <new>
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

        bool checked_add(std::size_t left, std::size_t right, std::size_t & result)
        {
            if(right > std::numeric_limits<std::size_t>::max() - left) {
                return false;
            }
            result = left + right;
            return true;
        }

        bool checked_bytes(std::size_t count, std::size_t element_size, std::size_t & result)
        {
            if(count > std::numeric_limits<std::size_t>::max() / element_size) {
                return false;
            }
            result = count * element_size;
            return true;
        }

        bool estimate_peak_bytes(
                std::size_t committed_live_bytes,
                const CellStorageMetrics & candidate,
                std::size_t operation_count,
                std::size_t payload_bytes,
                std::size_t & result)
        {
            std::size_t operations = 0U;
            std::size_t total = 0U;
            return checked_bytes(operation_count, sizeof(DeltaOperation), operations)
                   && checked_add(committed_live_bytes, candidate.candidate_owned_bytes, total)
                   && checked_add(total, operations, total)
                   && checked_add(total, payload_bytes, total)
                   && checked_add(total, candidate.traversal_scratch_bytes, result);
        }

    }// namespace

    MapUpdateApplier::MapUpdateApplier(
            MapUpdateLimits limits,
            CellStorageConfig storage)
        : limits_(std::move(limits))
        , cell_store_(storage)
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
        } else {
            state_ = ReceiverState::Empty;
        }
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
        const auto preflight = cell_store_.estimate_replace_upper_bound(
                decoded.cells.size());
        std::size_t preflight_peak = 0U;
        if(!preflight.success
           || !estimate_peak_bytes(
                   cell_store_.metrics().committed_live_bytes,
                   preflight.metrics,
                   0U,
                   update.payload.size(),
                   preflight_peak)
           || preflight_peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    preflight.success
                            ? "keyframe peak apply memory exceeds configured limit"
                            : preflight.diagnostic,
                    true);
        }
        CellSnapshotStore candidate_store = cell_store_;
        const auto stored = candidate_store.replace(std::move(decoded.cells));
        if(!stored.success) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    stored.diagnostic,
                    true);
        }
        const auto candidate_cells = candidate_store.view();
        std::size_t peak = 0U;
        if(!estimate_peak_bytes(
                   cell_store_.metrics().committed_live_bytes,
                   stored.metrics,
                   0U,
                   update.payload.size(),
                   peak)
           || peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "keyframe peak apply memory exceeds configured limit",
                    true);
        }
        last_apply_timing_.candidate_build_duration_ns = elapsed_ns(candidate_start);
        const auto hash_start = SteadyClock::now();
        const auto content = ContentHasher::content_hash(
                update.source, update.geometry_fingerprint, candidate_cells);
        last_apply_timing_.canonical_hash_duration_ns = elapsed_ns(hash_start);
        if(content != update.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "keyframe reconstructed content hash mismatch",
                    true);
        }
        const auto commit_start = SteadyClock::now();
        cell_store_ = std::move(candidate_store);
        ReconstructedMap candidate {
                update.source,
                update.geometry,
                update.new_revision,
                content,
                cell_store_.view()};
        reconstructed_map_ = std::move(candidate);
        last_occupancy_update_hash_ = update.update_hash;
        state_ = ReceiverState::Ready;
        last_apply_timing_.commit_duration_ns = elapsed_ns(commit_start);
        return {ApplyUpdateStatus::AppliedKeyframe, true, {}};
    }

    ApplyUpdateResult MapUpdateApplier::apply_delta(const MapUpdate & update)
    {
        if(state_ != ReceiverState::Ready || !reconstructed_map_.has_value()) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedGap,
                    "delta requires a ready receiver baseline",
                    reconstructed_map_.has_value());
        }
        const auto & current = *reconstructed_map_;
        if(current.source != update.source || !(current.geometry == update.geometry)
           || ContentHasher::geometry_fingerprint(current.geometry)
                      != update.geometry_fingerprint) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta source or geometry mismatches receiver baseline",
                    true);
        }
        if(update.base_revision < current.revision || update.new_revision <= current.revision) {
            return {ApplyUpdateStatus::RejectedStale, false, "delta revision is stale or overlaps"};
        }
        if(update.base_revision > current.revision) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedGap, "delta base revision is in the future", true);
        }
        if(update.base_content_hash != current.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta base content hash mismatches receiver state",
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
                    "delta declared counts exceed configured limits",
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
        const auto preflight = cell_store_.estimate_apply_upper_bound(
                decoded.operations);
        std::size_t preflight_peak = 0U;
        if(!preflight.success
           || !estimate_peak_bytes(
                   cell_store_.metrics().committed_live_bytes,
                   preflight.metrics,
                   decoded.operations.size(),
                   update.payload.size(),
                   preflight_peak)
           || preflight_peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    preflight.success
                            ? "delta peak apply memory exceeds configured limit"
                            : preflight.diagnostic,
                    true);
        }

        CellSnapshotStore candidate_store = cell_store_;
        const auto stored = candidate_store.apply(decoded.operations);
        if(!stored.success) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    stored.diagnostic,
                    true);
        }
        if(candidate_store.size() > limits_.max_receiver_cells
           || candidate_store.size() != update.known_cell_count) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta result count mismatches envelope or receiver limit",
                    true);
        }
        std::size_t peak = 0U;
        if(!estimate_peak_bytes(
                   cell_store_.metrics().committed_live_bytes,
                   stored.metrics,
                   decoded.operations.size(),
                   update.payload.size(),
                   peak)
           || peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta peak apply memory exceeds configured limit",
                    true);
        }

        const auto candidate_cells = candidate_store.view();
        last_apply_timing_.candidate_build_duration_ns = elapsed_ns(candidate_start);
        const auto hash_start = SteadyClock::now();
        const auto content = ContentHasher::content_hash(
                update.source, update.geometry_fingerprint, candidate_cells);
        last_apply_timing_.canonical_hash_duration_ns = elapsed_ns(hash_start);
        if(content != update.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta reconstructed content hash mismatch",
                    true);
        }
        const auto commit_start = SteadyClock::now();
        cell_store_ = std::move(candidate_store);
        reconstructed_map_->revision = update.new_revision;
        reconstructed_map_->content_hash = content;
        reconstructed_map_->cells = cell_store_.view();
        last_occupancy_update_hash_ = update.update_hash;
        last_apply_timing_.commit_duration_ns = elapsed_ns(commit_start);
        return {ApplyUpdateStatus::AppliedDelta, true, {}};
    }

    ApplyUpdateResult MapUpdateApplier::apply_summary(const MapUpdate & update)
    {
        if(!reconstructed_map_.has_value() || reconstructed_map_->source != update.source
           || !(reconstructed_map_->geometry == update.geometry)
           || update.base_revision != reconstructed_map_->revision
           || update.new_revision != reconstructed_map_->revision || update.revision_span != 0U
           || update.base_content_hash != reconstructed_map_->content_hash
           || update.content_hash != reconstructed_map_->content_hash || !update.payload.empty()
           || update.operation_count != 0U
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
        if(!reconstructed_map_.has_value() || reconstructed_map_->source != update.source
           || !(reconstructed_map_->geometry == update.geometry)
           || update.base_revision != reconstructed_map_->revision
           || update.base_content_hash != reconstructed_map_->content_hash
           || update.new_revision <= update.base_revision
           || update.revision_span != update.new_revision - update.base_revision
           || !is_zero_hash(update.content_hash) || !update.payload.empty()
           || update.known_cell_count != 0U || update.operation_count != 0U) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "remove tombstone fields are inconsistent",
                    reconstructed_map_.has_value());
        }
        CellSnapshotStore candidate_store = cell_store_;
        try {
            candidate_store.clear();
        }
        catch(const std::bad_alloc &) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "remove tombstone storage allocation failed",
                    true);
        }
        cell_store_ = std::move(candidate_store);
        reconstructed_map_->revision = update.new_revision;
        reconstructed_map_->content_hash = {};
        reconstructed_map_->cells = cell_store_.view();
        last_occupancy_update_hash_ = update.update_hash;
        state_ = ReceiverState::Removed;
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
        return cell_store_.metrics();
    }

    const ApplyUpdateTiming & MapUpdateApplier::last_apply_timing() const noexcept
    {
        return last_apply_timing_;
    }

    const void * MapUpdateApplier::chunk_identity(const VoxelIndex & index) const noexcept
    {
        return cell_store_.chunk_identity(index);
    }

}// namespace PerceptionMapUpdate
