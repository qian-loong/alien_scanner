#ifndef PERCEPTION_MAP_UPDATE_MAP_UPDATE_APPLIER_HPP
#define PERCEPTION_MAP_UPDATE_MAP_UPDATE_APPLIER_HPP

#include "perception_map_update/MapUpdateLimits.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"
#include "perception_map_update/MerkleMapState.hpp"

#include <optional>

namespace PerceptionMapUpdate {

    enum class ReceiverState : std::uint8_t
    {
        Empty,
        Ready,
        ResyncRequired,
        Removed
    };

    enum class ApplyUpdateStatus : std::uint8_t
    {
        AppliedKeyframe,
        AppliedDelta,
        AppliedRemove,
        AcceptedSummary,
        IgnoredDuplicate,
        RejectedAdmission,
        RejectedStale,
        RejectedGap,
        RejectedConflict,
        RejectedInvalid,
        RejectedResourceLimit
    };

    struct ApplyUpdateResult {
        ApplyUpdateStatus status = ApplyUpdateStatus::RejectedInvalid;
        bool state_changed = false;
        std::string diagnostic;
    };

    struct ApplyUpdateTiming {
        std::uint64_t payload_decode_duration_ns = 0U;
        std::uint64_t candidate_build_duration_ns = 0U;
        std::uint64_t merkle_duration_ns = 0U;
        std::uint64_t commit_duration_ns = 0U;
    };

    struct ReconstructedMap {
        SourceIdentity              source;
        MapGeometry                geometry;
        std::uint64_t              revision = 0U;
        VersionedContentDigest     content_identity;
        CanonicalCellView          cells;
    };

    class MapUpdateApplier
    {
    public:
        explicit MapUpdateApplier(MapUpdateLimits limits = {});

        bool admit_source(const SourceIdentity & source);
        bool require_resync() noexcept;
        ApplyUpdateResult apply(const MapUpdate & update);

        ReceiverState state() const noexcept;
        const std::optional<SourceIdentity> & expected_source() const noexcept;
        const std::optional<ReconstructedMap> & reconstructed_map() const noexcept;
        const CellStorageMetrics & storage_metrics() const noexcept;
        const MerkleTreeMetrics & merkle_metrics() const noexcept;
        const ApplyUpdateTiming & last_apply_timing() const noexcept;
        const void * chunk_identity(const VoxelIndex & index) const noexcept;

    private:
        ValidationResult validate_envelope(const MapUpdate & update) const;
        ApplyUpdateResult apply_keyframe(const MapUpdate & update);
        ApplyUpdateResult apply_delta(const MapUpdate & update);
        ApplyUpdateResult apply_summary(const MapUpdate & update);
        ApplyUpdateResult apply_remove(const MapUpdate & update);
        ApplyUpdateResult reject_for_current_chain(
                ApplyUpdateStatus status,
                std::string diagnostic,
                bool require_resync);
        bool is_retired(const SourceIdentity & source) const;

        MapUpdateLimits limits_;
        ReceiverState state_ = ReceiverState::Empty;
        std::optional<SourceIdentity> expected_source_;
        std::optional<ReconstructedMap> reconstructed_map_;
        std::shared_ptr<const MerkleMapState> map_state_;
        CellStorageMetrics storage_metrics_;
        MerkleTreeMetrics merkle_metrics_;
        ApplyUpdateTiming last_apply_timing_;
        std::optional<Hash256> last_occupancy_update_hash_;
        std::vector<SourceIdentity> retired_sources_;
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_MAP_UPDATE_APPLIER_HPP
