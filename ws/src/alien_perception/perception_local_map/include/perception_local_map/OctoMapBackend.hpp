#ifndef PERCEPTION_LOCAL_MAP_OCTOMAP_BACKEND_HPP
#define PERCEPTION_LOCAL_MAP_OCTOMAP_BACKEND_HPP

#include "perception_local_map/OccupancyBackend.hpp"

#include <memory>

namespace octomap {
    class OcTree;
}

namespace PerceptionLocalMap {

    class OctoMapSnapshotBridge;

    class OctoMapBackend final : public ILocalOccupancyBackend
    {
    public:
        explicit OctoMapBackend(const MapGeometry & geometry);
        ~OctoMapBackend() override;

        OctoMapBackend(const OctoMapBackend &) = delete;
        OctoMapBackend & operator=(const OctoMapBackend &) = delete;

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
        struct Impl;
        std::unique_ptr<Impl> impl_;

        const octomap::OcTree & tree_for_snapshot() const;
        friend class OctoMapSnapshotBridge;
    };

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_OCTOMAP_BACKEND_HPP
