#include "perception_map_update/MapUpdateApplier.hpp"

#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/ContentHasher.hpp"

#include <algorithm>
#include <limits>
#include <new>
#include <utility>

namespace PerceptionMapUpdate {

    namespace {

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
                std::size_t current_cells,
                std::size_t candidate_cells,
                std::size_t operation_count,
                std::size_t payload_bytes,
                std::size_t & result)
        {
            std::size_t current = 0U;
            std::size_t candidate = 0U;
            std::size_t operations = 0U;
            std::size_t total = 0U;
            return checked_bytes(current_cells, sizeof(CanonicalCell), current)
                   && checked_bytes(candidate_cells, sizeof(CanonicalCell), candidate)
                   && checked_bytes(operation_count, sizeof(DeltaOperation), operations)
                   && checked_add(current, candidate, total)
                   && checked_add(total, operations, total)
                   && checked_add(total, payload_bytes, result);
        }

        CellState state_for(DeltaOperationKind kind)
        {
            return kind == DeltaOperationKind::UpsertFree ? CellState::Free
                                                          : CellState::Occupied;
        }

        struct DeltaMergePlan {
            bool valid = false;
            std::size_t candidate_count = 0U;
            std::string diagnostic;
        };

        DeltaMergePlan plan_delta_merge(
                const std::vector<CanonicalCell> & current,
                const std::vector<DeltaOperation> & operations)
        {
            std::size_t additions = 0U;
            std::size_t removals = 0U;
            std::size_t cell_index = 0U;
            std::size_t operation_index = 0U;
            while(cell_index < current.size() || operation_index < operations.size()) {
                if(operation_index == operations.size()) {
                    break;
                }
                const auto & operation = operations[operation_index];
                if(cell_index == current.size()
                   || operation.index < current[cell_index].index) {
                    if(operation.kind == DeltaOperationKind::RemoveToUnknown) {
                        return {false, 0U, "delta removes an unknown cell"};
                    }
                    ++additions;
                    ++operation_index;
                    continue;
                }
                const auto & cell = current[cell_index];
                if(cell.index < operation.index) {
                    ++cell_index;
                    continue;
                }
                if(operation.kind == DeltaOperationKind::RemoveToUnknown) {
                    ++removals;
                } else if(state_for(operation.kind) == cell.state) {
                    return {false, 0U, "delta contains a redundant upsert"};
                }
                ++cell_index;
                ++operation_index;
            }
            std::size_t expanded = 0U;
            if(!checked_add(current.size(), additions, expanded) || removals > expanded) {
                return {false, 0U, "delta result cell count overflows size_t"};
            }
            return {true, expanded - removals, {}};
        }

    }// namespace

    MapUpdateApplier::MapUpdateApplier(MapUpdateLimits limits) : limits_(std::move(limits)) {}

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
        const std::size_t current_count = reconstructed_map_.has_value()
                                                  ? reconstructed_map_->cells.size()
                                                  : 0U;
        std::size_t peak = 0U;
        if(!estimate_peak_bytes(
                   current_count,
                   static_cast<std::size_t>(update.known_cell_count),
                   0U,
                   update.payload.size(),
                   peak)
           || peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "keyframe peak apply memory exceeds configured limit",
                    true);
        }
        auto decoded = CanonicalCodec::decode_keyframe_payload(update.payload, limits_);
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
        const auto content = ContentHasher::content_hash(
                update.source, update.geometry_fingerprint, decoded.cells);
        if(content != update.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "keyframe reconstructed content hash mismatch",
                    true);
        }
        ReconstructedMap candidate {
                update.source,
                update.geometry,
                update.new_revision,
                content,
                std::move(decoded.cells)};
        reconstructed_map_ = std::move(candidate);
        last_occupancy_update_hash_ = update.update_hash;
        state_ = ReceiverState::Ready;
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
        std::size_t minimum_peak = 0U;
        if(!estimate_peak_bytes(
                   current.cells.size(),
                   0U,
                   static_cast<std::size_t>(update.operation_count),
                   update.payload.size(),
                   minimum_peak)
           || minimum_peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta decode memory exceeds configured peak limit",
                    true);
        }
        const auto decoded = CanonicalCodec::decode_delta_payload(update.payload, limits_);
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

        const auto plan = plan_delta_merge(current.cells, decoded.operations);
        if(!plan.valid) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    plan.diagnostic,
                    true);
        }
        if(plan.candidate_count > limits_.max_receiver_cells
           || plan.candidate_count != update.known_cell_count) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta result count mismatches envelope or receiver limit",
                    true);
        }
        std::size_t peak = 0U;
        if(!estimate_peak_bytes(
                   current.cells.size(),
                   plan.candidate_count,
                   decoded.operations.size(),
                   update.payload.size(),
                   peak)
           || peak > limits_.max_peak_apply_bytes) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta peak apply memory exceeds configured limit",
                    true);
        }

        std::vector<CanonicalCell> candidate;
        try {
            candidate.reserve(plan.candidate_count);
        }
        catch(const std::bad_alloc &) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedResourceLimit,
                    "delta candidate allocation failed",
                    true);
        }
        std::size_t cell_index = 0U;
        std::size_t operation_index = 0U;
        while(cell_index < current.cells.size() || operation_index < decoded.operations.size()) {
            if(operation_index == decoded.operations.size()) {
                candidate.insert(
                        candidate.end(), current.cells.begin() + cell_index, current.cells.end());
                break;
            }
            const auto & operation = decoded.operations[operation_index];
            if(cell_index == current.cells.size()
               || operation.index < current.cells[cell_index].index) {
                candidate.push_back({operation.index, state_for(operation.kind)});
                ++operation_index;
                continue;
            }
            const auto & cell = current.cells[cell_index];
            if(cell.index < operation.index) {
                candidate.push_back(cell);
                ++cell_index;
                continue;
            }
            if(operation.kind == DeltaOperationKind::RemoveToUnknown) {
                ++cell_index;
                ++operation_index;
                continue;
            }
            const auto new_state = state_for(operation.kind);
            candidate.push_back({operation.index, new_state});
            ++cell_index;
            ++operation_index;
        }
        if(candidate.size() != plan.candidate_count) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedInvalid,
                    "delta merge result differs from its validated plan",
                    true);
        }
        const auto content = ContentHasher::content_hash(
                update.source, update.geometry_fingerprint, candidate);
        if(content != update.content_hash) {
            return reject_for_current_chain(
                    ApplyUpdateStatus::RejectedConflict,
                    "delta reconstructed content hash mismatch",
                    true);
        }
        reconstructed_map_->revision = update.new_revision;
        reconstructed_map_->content_hash = content;
        reconstructed_map_->cells.swap(candidate);
        last_occupancy_update_hash_ = update.update_hash;
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
        reconstructed_map_->revision = update.new_revision;
        reconstructed_map_->content_hash = {};
        reconstructed_map_->cells.clear();
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

}// namespace PerceptionMapUpdate
