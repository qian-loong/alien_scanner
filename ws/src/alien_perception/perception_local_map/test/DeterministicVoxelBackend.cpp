#include "DeterministicVoxelBackend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace PerceptionLocalMap::Test {

    DeterministicVoxelBackend::DeterministicVoxelBackend(const MapGeometry & geometry)
        : geometry_(geometry)
    {
        if(!is_valid_geometry(geometry_)) {
            throw std::invalid_argument("deterministic backend requires valid geometry");
        }
    }

    MapGeometry DeterministicVoxelBackend::geometry() const noexcept { return geometry_; }

    BackendCapabilities DeterministicVoxelBackend::capabilities() const noexcept
    {
        return {true, true, true, true, false, false, false};
    }

    std::optional<AxisAlignedBounds> DeterministicVoxelBackend::known_bounds() const
    {
        if(cells_.empty()) {
            return std::nullopt;
        }
        VoxelIndex minimum = cells_.begin()->first;
        VoxelIndex maximum = minimum;
        for(const auto & entry : cells_) {
            minimum.x = std::min(minimum.x, entry.first.x);
            minimum.y = std::min(minimum.y, entry.first.y);
            minimum.z = std::min(minimum.z, entry.first.z);
            maximum.x = std::max(maximum.x, entry.first.x);
            maximum.y = std::max(maximum.y, entry.first.y);
            maximum.z = std::max(maximum.z, entry.first.z);
        }
        return voxel_bounds(
                minimum, {maximum.x + 1, maximum.y + 1, maximum.z + 1}, geometry_);
    }

    MapQueryResult DeterministicVoxelBackend::query(const MapPoint & point) const
    {
        const auto index = quantize_point(point, geometry_);
        if(index.status != QueryStatus::Ok) {
            return {index.status, OccupancyState::Unknown};
        }
        const auto value = cells_.find(index.index);
        return {QueryStatus::Ok,
                value == cells_.end() ? OccupancyState::Unknown : value->second};
    }

    RegionQueryResult DeterministicVoxelBackend::query_region(
            const AxisAlignedBounds & bounds) const
    {
        if(!is_valid_bounds(bounds)) {
            return {QueryStatus::Invalid, {}};
        }
        const auto minimum = quantize_point(bounds.minimum, geometry_);
        const auto maximum = quantize_point(
                {std::nextafter(bounds.maximum.x, -std::numeric_limits<double>::infinity()),
                 std::nextafter(bounds.maximum.y, -std::numeric_limits<double>::infinity()),
                 std::nextafter(bounds.maximum.z, -std::numeric_limits<double>::infinity())},
                geometry_);
        if(minimum.status != QueryStatus::Ok || maximum.status != QueryStatus::Ok) {
            return {minimum.status != QueryStatus::Ok ? minimum.status : maximum.status, {}};
        }
        RegionQueryResult result {QueryStatus::Ok, {}};
        for(const auto & entry : cells_) {
            if(entry.first.x >= minimum.index.x && entry.first.x <= maximum.index.x
               && entry.first.y >= minimum.index.y && entry.first.y <= maximum.index.y
               && entry.first.z >= minimum.index.z && entry.first.z <= maximum.index.z) {
                result.cells.push_back(
                        {entry.first, voxel_center(entry.first, geometry_), entry.second});
            }
        }
        return result;
    }

    bool DeterministicVoxelBackend::for_each_known_cell(
            const std::function<void(OccupancyCell)> & visitor) const
    {
        if(!visitor) {
            return false;
        }
        for(const auto & entry : cells_) {
            visitor({entry.first, voxel_center(entry.first, geometry_), entry.second});
        }
        return true;
    }

    void DeterministicVoxelBackend::reset(const MapGeometry & geometry)
    {
        if(!is_valid_geometry(geometry)) {
            throw std::invalid_argument("deterministic backend reset requires valid geometry");
        }
        geometry_ = geometry;
        cells_.clear();
    }

    ApplyResult DeterministicVoxelBackend::apply(const EvidenceBatch & batch)
    {
        if(batch.free_cells.empty() && batch.occupied_cells.empty()) {
            return {ApplyStatus::NoEvidence, 0, {}};
        }
        std::set<VoxelIndex> free(batch.free_cells.begin(), batch.free_cells.end());
        std::set<VoxelIndex> occupied(
                batch.occupied_cells.begin(), batch.occupied_cells.end());
        for(const auto & cell : occupied) {
            free.erase(cell);
        }
        if(std::any_of(
                   free.begin(), free.end(),
                   [&](const auto & value) {
                       return !is_representable_voxel(value, geometry_);
                   })
           || std::any_of(
                   occupied.begin(), occupied.end(),
                   [&](const auto & value) {
                       return !is_representable_voxel(value, geometry_);
                   })) {
            return {ApplyStatus::Rejected, 0, "cell outside deterministic backend range"};
        }

        std::size_t changed = 0;
        for(const auto & cell : free) {
            if(cells_.find(cell) == cells_.end() || cells_.at(cell) != OccupancyState::Free) {
                ++changed;
            }
            cells_[cell] = OccupancyState::Free;
        }
        for(const auto & cell : occupied) {
            if(cells_.find(cell) == cells_.end() || cells_.at(cell) != OccupancyState::Occupied) {
                ++changed;
            }
            cells_[cell] = OccupancyState::Occupied;
        }
        return {ApplyStatus::Applied, changed, {}};
    }

}// namespace PerceptionLocalMap::Test
