#ifndef PERCEPTION_LOCAL_MAP_TEST_DETERMINISTIC_VOXEL_BACKEND_HPP
#define PERCEPTION_LOCAL_MAP_TEST_DETERMINISTIC_VOXEL_BACKEND_HPP

#include "perception_local_map/OccupancyBackend.hpp"

#include <map>

namespace PerceptionLocalMap::Test {

    class DeterministicVoxelBackend final : public ILocalOccupancyBackend
    {
    public:
        explicit DeterministicVoxelBackend(const MapGeometry & geometry);

        MapGeometry geometry() const noexcept override;
        BackendCapabilities capabilities() const noexcept override;
        std::optional<AxisAlignedBounds> known_bounds() const override;
        MapQueryResult query(const MapPoint & point) const override;
        RegionQueryResult query_region(const AxisAlignedBounds & bounds) const override;
        bool for_each_known_cell(
                const std::function<void(OccupancyCell)> & visitor) const override;
        void reset(const MapGeometry & geometry) override;
        ApplyResult apply(const EvidenceBatch & batch) override;

    private:
        MapGeometry                         geometry_;
        std::map<VoxelIndex, OccupancyState> cells_;
    };

}// namespace PerceptionLocalMap::Test

#endif// PERCEPTION_LOCAL_MAP_TEST_DETERMINISTIC_VOXEL_BACKEND_HPP
