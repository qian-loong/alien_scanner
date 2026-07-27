#include "perception_local_map/MapTypes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace PerceptionLocalMap {

    namespace {

        constexpr double kMinimumOctoMapKeyCoordinate = -32768.0;
        constexpr double kMaximumOctoMapKeyCoordinate = 32767.0;

        bool representable_axis(
                std::int64_t index,
                double origin,
                double resolution) noexcept
        {
            const double center = origin
                                  + (static_cast<double>(index) + 0.5) * resolution;
            const float backend_coordinate = static_cast<float>(center);
            if(!std::isfinite(backend_coordinate)) {
                return false;
            }
            const double key_coordinate = std::floor(
                    static_cast<double>(backend_coordinate) / resolution);
            return key_coordinate >= kMinimumOctoMapKeyCoordinate
                   && key_coordinate <= kMaximumOctoMapKeyCoordinate;
        }

    }// namespace

    bool VoxelIndex::operator==(const VoxelIndex & other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool VoxelIndex::operator!=(const VoxelIndex & other) const noexcept
    {
        return !(*this == other);
    }

    bool VoxelIndex::operator<(const VoxelIndex & other) const noexcept
    {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }

    bool BackendCapabilities::has_required_capabilities() const noexcept
    {
        return supports_point_query && supports_bounded_region_query
               && supports_reset && supports_known_bounds;
    }

    bool EffectiveMapCapabilities::is_consistent() const noexcept
    {
        const std::uint64_t sum = static_cast<std::uint64_t>(active_2d_hit_only_sensor_count)
                                  + active_2d_hit_ray_sensor_count
                                  + active_2d_full_ray_sensor_count
                                  + active_3d_hit_only_sensor_count
                                  + active_3d_hit_ray_sensor_count
                                  + active_3d_full_ray_sensor_count;
        if(sum != active_sensor_count) {
            return false;
        }
        if(active_3d_hit_ray_sensor_count != 0U
           || active_3d_full_ray_sensor_count != 0U
           || !Perception::is_valid_ray_evidence(maximum_active_ray_evidence)) {
            return false;
        }
        if(active_sensor_count == 0U) {
            return !has_active_ray_evidence
                   && maximum_active_ray_evidence
                              == Perception::RayEvidenceCapability::HitOnly;
        }
        if(!has_active_ray_evidence) {
            return false;
        }
        const auto expected = active_2d_full_ray_sensor_count != 0U
                                      ? Perception::RayEvidenceCapability::FullRay
                              : active_2d_hit_ray_sensor_count != 0U
                                      ? Perception::RayEvidenceCapability::HitRay
                                      : Perception::RayEvidenceCapability::HitOnly;
        return maximum_active_ray_evidence == expected;
    }

    bool is_valid_geometry(const MapGeometry & geometry) noexcept
    {
        const bool basic = std::isfinite(geometry.resolution_m)
                           && geometry.resolution_m > 0.0
                           && std::isfinite(geometry.lattice_origin.x)
                           && std::isfinite(geometry.lattice_origin.y)
                           && std::isfinite(geometry.lattice_origin.z)
                           && !geometry.frame_id.empty();
        return basic && is_representable_voxel({}, geometry);
    }

    bool is_valid_bounds(const AxisAlignedBounds & bounds) noexcept
    {
        return std::isfinite(bounds.minimum.x) && std::isfinite(bounds.minimum.y)
               && std::isfinite(bounds.minimum.z) && std::isfinite(bounds.maximum.x)
               && std::isfinite(bounds.maximum.y) && std::isfinite(bounds.maximum.z)
               && bounds.minimum.x < bounds.maximum.x
               && bounds.minimum.y < bounds.maximum.y
               && bounds.minimum.z < bounds.maximum.z;
    }

    bool is_representable_voxel(
            const VoxelIndex & index,
            const MapGeometry & geometry) noexcept
    {
        if(!std::isfinite(geometry.resolution_m) || geometry.resolution_m <= 0.0
           || !std::isfinite(geometry.lattice_origin.x)
           || !std::isfinite(geometry.lattice_origin.y)
           || !std::isfinite(geometry.lattice_origin.z)) {
            return false;
        }
        return representable_axis(
                       index.x, geometry.lattice_origin.x, geometry.resolution_m)
               && representable_axis(
                       index.y, geometry.lattice_origin.y, geometry.resolution_m)
               && representable_axis(
                       index.z, geometry.lattice_origin.z, geometry.resolution_m);
    }

    VoxelIndexResult quantize_point(
            const MapPoint & point,
            const MapGeometry & geometry) noexcept
    {
        if(!is_valid_geometry(geometry) || !std::isfinite(point.x)
           || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            return {QueryStatus::Invalid, {}};
        }
        const double x = std::floor((point.x - geometry.lattice_origin.x) / geometry.resolution_m);
        const double y = std::floor((point.y - geometry.lattice_origin.y) / geometry.resolution_m);
        const double z = std::floor((point.z - geometry.lattice_origin.z) / geometry.resolution_m);
        constexpr double minimum_index = static_cast<double>(std::numeric_limits<std::int64_t>::min());
        constexpr double maximum_index = static_cast<double>(std::numeric_limits<std::int64_t>::max());
        if(x < minimum_index || x >= maximum_index || y < minimum_index || y >= maximum_index
           || z < minimum_index || z >= maximum_index) {
            return {QueryStatus::OutOfRange, {}};
        }
        const VoxelIndex index {
                static_cast<std::int64_t>(x), static_cast<std::int64_t>(y),
                static_cast<std::int64_t>(z)};
        return is_representable_voxel(index, geometry)
                       ? VoxelIndexResult {QueryStatus::Ok, index}
                       : VoxelIndexResult {QueryStatus::OutOfRange, {}};
    }

    MapPoint voxel_center(const VoxelIndex & index, const MapGeometry & geometry) noexcept
    {
        return {
                geometry.lattice_origin.x
                        + (static_cast<double>(index.x) + 0.5) * geometry.resolution_m,
                geometry.lattice_origin.y
                        + (static_cast<double>(index.y) + 0.5) * geometry.resolution_m,
                geometry.lattice_origin.z
                        + (static_cast<double>(index.z) + 0.5) * geometry.resolution_m};
    }

    AxisAlignedBounds voxel_bounds(
            const VoxelIndex & minimum,
            const VoxelIndex & maximum_exclusive,
            const MapGeometry & geometry) noexcept
    {
        return {
                {geometry.lattice_origin.x + static_cast<double>(minimum.x) * geometry.resolution_m,
                 geometry.lattice_origin.y + static_cast<double>(minimum.y) * geometry.resolution_m,
                 geometry.lattice_origin.z + static_cast<double>(minimum.z) * geometry.resolution_m},
                {geometry.lattice_origin.x
                         + static_cast<double>(maximum_exclusive.x) * geometry.resolution_m,
                 geometry.lattice_origin.y
                         + static_cast<double>(maximum_exclusive.y) * geometry.resolution_m,
                 geometry.lattice_origin.z
                         + static_cast<double>(maximum_exclusive.z) * geometry.resolution_m}};
    }

}// namespace PerceptionLocalMap
