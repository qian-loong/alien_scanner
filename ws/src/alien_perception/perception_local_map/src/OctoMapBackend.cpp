#include "perception_local_map/OctoMapBackend.hpp"

#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace PerceptionLocalMap {

    struct OctoMapBackend::Impl {
        explicit Impl(const MapGeometry & value)
            : geometry(value)
            , tree(std::make_unique<octomap::OcTree>(value.resolution_m))
        {}

        MapGeometry                    geometry;
        std::unique_ptr<octomap::OcTree> tree;
        std::set<VoxelIndex>           known_cells;
    };

    namespace {

        octomap::point3d to_octomap_point(const MapPoint & point)
        {
            return {static_cast<float>(point.x), static_cast<float>(point.y),
                    static_cast<float>(point.z)};
        }

        bool can_represent(const VoxelIndex & index, const MapGeometry & geometry, octomap::OcTree & tree)
        {
            octomap::OcTreeKey key;
            return is_representable_voxel(index, geometry)
                   && tree.coordToKeyChecked(
                           to_octomap_point(voxel_center(index, geometry)), key);
        }

    }// namespace

    OctoMapBackend::OctoMapBackend(const MapGeometry & geometry)
    {
        if(!is_valid_geometry(geometry)) {
            throw std::invalid_argument("OctoMapBackend requires valid geometry");
        }
        impl_ = std::make_unique<Impl>(geometry);
    }

    OctoMapBackend::~OctoMapBackend() = default;

    MapGeometry OctoMapBackend::geometry() const noexcept
    {
        return impl_->geometry;
    }

    BackendCapabilities OctoMapBackend::capabilities() const noexcept
    {
        return {true, true, true, true, false, false, true};
    }

    std::optional<AxisAlignedBounds> OctoMapBackend::known_bounds() const
    {
        if(impl_->known_cells.empty()) {
            return std::nullopt;
        }
        VoxelIndex minimum = *impl_->known_cells.begin();
        VoxelIndex maximum = minimum;
        for(const auto & index : impl_->known_cells) {
            minimum.x = std::min(minimum.x, index.x);
            minimum.y = std::min(minimum.y, index.y);
            minimum.z = std::min(minimum.z, index.z);
            maximum.x = std::max(maximum.x, index.x);
            maximum.y = std::max(maximum.y, index.y);
            maximum.z = std::max(maximum.z, index.z);
        }
        return voxel_bounds(
                minimum,
                {maximum.x + 1, maximum.y + 1, maximum.z + 1}, impl_->geometry);
    }

    MapQueryResult OctoMapBackend::query(const MapPoint & point) const
    {
        const auto index = quantize_point(point, impl_->geometry);
        if(index.status != QueryStatus::Ok) {
            return {index.status, OccupancyState::Unknown};
        }
        octomap::OcTreeKey key;
        if(!impl_->tree->coordToKeyChecked(
                   to_octomap_point(voxel_center(index.index, impl_->geometry)), key)) {
            return {QueryStatus::OutOfRange, OccupancyState::Unknown};
        }
        const auto * node = impl_->tree->search(key, impl_->tree->getTreeDepth());
        if(node == nullptr) {
            return {QueryStatus::Ok, OccupancyState::Unknown};
        }
        return {QueryStatus::Ok,
                impl_->tree->isNodeOccupied(node) ? OccupancyState::Occupied
                                                  : OccupancyState::Free};
    }

    RegionQueryResult OctoMapBackend::query_region(const AxisAlignedBounds & bounds) const
    {
        if(!is_valid_bounds(bounds)) {
            return {QueryStatus::Invalid, {}};
        }
        const auto minimum = quantize_point(bounds.minimum, impl_->geometry);
        const MapPoint maximum_inside {
                std::nextafter(bounds.maximum.x, -std::numeric_limits<double>::infinity()),
                std::nextafter(bounds.maximum.y, -std::numeric_limits<double>::infinity()),
                std::nextafter(bounds.maximum.z, -std::numeric_limits<double>::infinity())};
        const auto maximum = quantize_point(maximum_inside, impl_->geometry);
        if(minimum.status != QueryStatus::Ok || maximum.status != QueryStatus::Ok) {
            return {minimum.status != QueryStatus::Ok ? minimum.status : maximum.status, {}};
        }

        RegionQueryResult result {QueryStatus::Ok, {}};
        for(const auto & index : impl_->known_cells) {
            if(index.x < minimum.index.x || index.x > maximum.index.x
               || index.y < minimum.index.y || index.y > maximum.index.y
               || index.z < minimum.index.z || index.z > maximum.index.z) {
                continue;
            }
            const auto queried = query(voxel_center(index, impl_->geometry));
            if(queried.status != QueryStatus::Ok
               || queried.state == OccupancyState::Unknown) {
                return {QueryStatus::Unavailable, {}};
            }
            result.cells.push_back(
                    {index, voxel_center(index, impl_->geometry), queried.state});
        }
        return result;
    }

    bool OctoMapBackend::for_each_known_cell(
            const std::function<void(OccupancyCell)> & visitor) const
    {
        if(!visitor) {
            return false;
        }
        for(const auto & index : impl_->known_cells) {
            const auto queried = query(voxel_center(index, impl_->geometry));
            if(queried.status != QueryStatus::Ok
               || queried.state == OccupancyState::Unknown) {
                return false;
            }
            visitor({index, voxel_center(index, impl_->geometry), queried.state});
        }
        return true;
    }

    void OctoMapBackend::reset(const MapGeometry & geometry)
    {
        if(!is_valid_geometry(geometry)) {
            throw std::invalid_argument("OctoMapBackend reset requires valid geometry");
        }
        impl_ = std::make_unique<Impl>(geometry);
    }

    ApplyResult OctoMapBackend::apply(const EvidenceBatch & batch)
    {
        if(batch.free_cells.empty() && batch.occupied_cells.empty()) {
            return {ApplyStatus::NoEvidence, 0, {}};
        }

        std::set<VoxelIndex> free_cells(batch.free_cells.begin(), batch.free_cells.end());
        std::set<VoxelIndex> occupied_cells(
                batch.occupied_cells.begin(), batch.occupied_cells.end());
        for(const auto & occupied : occupied_cells) {
            free_cells.erase(occupied);
        }
        for(const auto & cell : free_cells) {
            if(!can_represent(cell, impl_->geometry, *impl_->tree)) {
                return {ApplyStatus::Rejected, 0, "free evidence is outside OctoMap range"};
            }
        }
        for(const auto & cell : occupied_cells) {
            if(!can_represent(cell, impl_->geometry, *impl_->tree)) {
                return {ApplyStatus::Rejected, 0, "occupied evidence is outside OctoMap range"};
            }
        }

        std::size_t changed = 0;
        const auto apply_cell = [&](const VoxelIndex & cell, bool occupied) {
            const auto center = voxel_center(cell, impl_->geometry);
            const auto before = query(center).state;
            impl_->tree->updateNode(to_octomap_point(center), occupied, true);
            impl_->known_cells.insert(cell);
            const auto after = query(center).state;
            if(before != after) {
                ++changed;
            }
        };
        for(const auto & cell : free_cells) {
            apply_cell(cell, false);
        }
        for(const auto & cell : occupied_cells) {
            apply_cell(cell, true);
        }
        impl_->tree->updateInnerOccupancy();
        return {ApplyStatus::Applied, changed, {}};
    }

    const octomap::OcTree & OctoMapBackend::tree_for_snapshot() const
    {
        return *impl_->tree;
    }

}// namespace PerceptionLocalMap
