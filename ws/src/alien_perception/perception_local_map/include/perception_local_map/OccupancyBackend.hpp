#ifndef PERCEPTION_LOCAL_MAP_OCCUPANCY_BACKEND_HPP
#define PERCEPTION_LOCAL_MAP_OCCUPANCY_BACKEND_HPP

#include "perception_local_map/MapTypes.hpp"

#include <functional>
#include <memory>

namespace PerceptionLocalMap {

    class IOccupancyMapView
    {
    public:
        virtual ~IOccupancyMapView() = default;

        virtual MapGeometry geometry() const noexcept = 0;
        virtual BackendCapabilities capabilities() const noexcept = 0;
        virtual std::optional<AxisAlignedBounds> known_bounds() const = 0;
        virtual MapQueryResult query(const MapPoint & point) const = 0;
        virtual RegionQueryResult query_region(const AxisAlignedBounds & bounds) const = 0;
        virtual bool for_each_known_cell(
                const std::function<void(OccupancyCell)> & visitor) const = 0;
    };

    class ILocalOccupancyBackend : public IOccupancyMapView
    {
    public:
        ~ILocalOccupancyBackend() override = default;

        virtual void reset(const MapGeometry & geometry) = 0;
        virtual ApplyResult apply(const EvidenceBatch & batch) = 0;
    };

    using BackendFactory = std::function<std::unique_ptr<ILocalOccupancyBackend>(const MapGeometry &)>;

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_OCCUPANCY_BACKEND_HPP
