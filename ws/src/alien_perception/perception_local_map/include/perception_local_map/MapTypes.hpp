#ifndef PERCEPTION_LOCAL_MAP_MAP_TYPES_HPP
#define PERCEPTION_LOCAL_MAP_MAP_TYPES_HPP

#include "perception_core/health/MapperContractFingerprint.hpp"
#include "perception_core/health/health_state.hpp"
#include "perception_core/observation/ray_evidence_capability.hpp"
#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace PerceptionLocalMap {

    enum class OccupancyState : std::uint8_t
    {
        Unknown,
        Free,
        Occupied
    };

    struct MapPoint {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct VoxelIndex {
        std::int64_t x = 0;
        std::int64_t y = 0;
        std::int64_t z = 0;

        bool operator==(const VoxelIndex & other) const noexcept;
        bool operator!=(const VoxelIndex & other) const noexcept;
        bool operator<(const VoxelIndex & other) const noexcept;
    };

    struct AxisAlignedBounds {
        MapPoint minimum;
        MapPoint maximum;
    };

    struct MapGeometry {
        double      resolution_m = 0.0;
        MapPoint    lattice_origin;
        std::string frame_id;
    };

    struct OccupancyCell {
        VoxelIndex     index;
        MapPoint       center;
        OccupancyState state = OccupancyState::Unknown;
    };

    enum class QueryStatus : std::uint8_t
    {
        Ok,
        Invalid,
        OutOfRange,
        Unavailable,
        TransactionClosed
    };

    struct MapQueryResult {
        QueryStatus    status = QueryStatus::Invalid;
        OccupancyState state  = OccupancyState::Unknown;
    };

    struct RegionQueryResult {
        QueryStatus                status = QueryStatus::Invalid;
        std::vector<OccupancyCell> cells;
    };

    struct BackendCapabilities {
        bool supports_point_query          = false;
        bool supports_bounded_region_query = false;
        bool supports_reset                = false;
        bool supports_known_bounds         = false;
        bool supports_hierarchy            = false;
        bool supports_dirty_region         = false;
        bool supports_serialization        = false;

        bool has_required_capabilities() const noexcept;
    };

    struct EvidenceBatch {
        std::vector<VoxelIndex> free_cells;
        std::vector<VoxelIndex> occupied_cells;
    };

    enum class ApplyStatus : std::uint8_t
    {
        Applied,
        NoEvidence,
        Rejected
    };

    struct ApplyResult {
        ApplyStatus status             = ApplyStatus::Rejected;
        std::size_t changed_cell_count = 0;
        std::string diagnostic;
    };

    struct MapIdentity {
        std::string           vehicle_id;
        Perception::SessionID mapper_session {0, 0};
        std::uint64_t         map_epoch = 1;
        std::uint64_t         revision  = 0;
    };

    struct EffectiveMapCapabilities {
        std::string satisfied_combination_id;
        bool has_active_ray_evidence = false;
        Perception::RayEvidenceCapability maximum_active_ray_evidence =
                Perception::RayEvidenceCapability::HitOnly;
        std::uint32_t active_sensor_count                   = 0;
        std::uint32_t active_2d_hit_only_sensor_count       = 0;
        std::uint32_t active_2d_hit_ray_sensor_count        = 0;
        std::uint32_t active_2d_full_ray_sensor_count       = 0;
        std::uint32_t active_3d_hit_only_sensor_count       = 0;
        std::uint32_t active_3d_hit_ray_sensor_count        = 0;
        std::uint32_t active_3d_full_ray_sensor_count       = 0;

        bool is_consistent() const noexcept;
    };

    enum class AlignmentStatus : std::uint8_t
    {
        Candidate,
        Committed,
        Degraded,
        Revoked
    };

    struct AlignmentReference {
        std::string           vehicle_id;
        Perception::SessionID mapper_session {0, 0};
        std::uint64_t         map_epoch = 0;
        std::string           provider_id;
        Perception::SessionID provider_session {0, 0};
        std::uint64_t         alignment_epoch    = 0;
        std::uint64_t         alignment_revision = 0;
        AlignmentStatus      status = AlignmentStatus::Candidate;
    };

    struct CommitReceipt {
        Perception::SessionID mapper_session {0, 0};
        std::uint64_t         map_epoch = 0;
        std::uint64_t         revision  = 0;
    };

    struct CommitProvenance {
        Perception::SensorID  sensor_id;
        Perception::SessionID sensor_session {0, 0};
        Perception::Timestamp observation_stamp {0};
        std::string           clock_domain;
        std::uint32_t         changed_cell_count = 0;
    };

    struct LocalMapState {
        MapIdentity                           identity;
        MapGeometry                           geometry;
        Perception::HealthState               health = Perception::HealthState::Unavailable;
        bool                                  map_fresh = false;
        std::int64_t                          validity_remaining_ns = 0;
        std::optional<AxisAlignedBounds>       known_bounds;
        Perception::MapperContractFingerprint contract_fingerprint;
        EffectiveMapCapabilities              effective_capabilities;
        BackendCapabilities                   backend_capabilities;
        std::optional<CommitProvenance>        last_commit;
        std::optional<AlignmentReference>      alignment;
    };

    bool is_valid_geometry(const MapGeometry & geometry) noexcept;
    bool is_valid_bounds(const AxisAlignedBounds & bounds) noexcept;
    bool is_representable_voxel(
            const VoxelIndex & index,
            const MapGeometry & geometry) noexcept;

    struct VoxelIndexResult {
        QueryStatus status = QueryStatus::Invalid;
        VoxelIndex  index;
    };

    VoxelIndexResult quantize_point(
            const MapPoint & point,
            const MapGeometry & geometry) noexcept;
    MapPoint voxel_center(const VoxelIndex & index, const MapGeometry & geometry) noexcept;
    AxisAlignedBounds voxel_bounds(
            const VoxelIndex & minimum,
            const VoxelIndex & maximum_exclusive,
            const MapGeometry & geometry) noexcept;

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_MAP_TYPES_HPP
