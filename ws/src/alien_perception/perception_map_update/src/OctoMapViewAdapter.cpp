#include "perception_map_update/OctoMapViewAdapter.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <cmath>
#include <exception>

namespace PerceptionMapUpdate {

    bool OctoMapViewAdapter::materialize(
            const ReconstructedMap & map,
            octomap_msgs::msg::Octomap & message,
            std::string & diagnostic,
            const MapUpdateLimits & limits)
    {
        const auto geometry = CanonicalCodec::validate_geometry(map.geometry, limits);
        const auto cells = CanonicalCodec::validate_cells(map.cells, limits);
        if(!geometry || !cells) {
            diagnostic = !geometry ? geometry.diagnostic : cells.diagnostic;
            return false;
        }
        try {
            octomap::OcTree tree(map.geometry.resolution_m);
            bool cells_valid = true;
            map.cells.for_each([&](const CanonicalCell & cell) {
                if(!cells_valid) {
                    return;
                }
                const double x = map.geometry.lattice_origin.x
                                 + (static_cast<double>(cell.index.x) + 0.5)
                                           * map.geometry.resolution_m;
                const double y = map.geometry.lattice_origin.y
                                 + (static_cast<double>(cell.index.y) + 0.5)
                                           * map.geometry.resolution_m;
                const double z = map.geometry.lattice_origin.z
                                 + (static_cast<double>(cell.index.z) + 0.5)
                                           * map.geometry.resolution_m;
                if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    diagnostic = "reconstructed voxel center is not finite";
                    cells_valid = false;
                    return;
                }
                const octomap::point3d point(
                        static_cast<float>(x),
                        static_cast<float>(y),
                        static_cast<float>(z));
                octomap::OcTreeKey key;
                if(!tree.coordToKeyChecked(point, key)) {
                    diagnostic = "reconstructed voxel is outside OctoMap key range";
                    cells_valid = false;
                    return;
                }
                tree.updateNode(
                        key, cell.state == CellState::Occupied, true);
            });
            if(!cells_valid) {
                return false;
            }
            tree.updateInnerOccupancy();
            if(!octomap_msgs::binaryMapToMsg(tree, message)) {
                diagnostic = "failed to serialize reconstructed OctoMap view";
                return false;
            }
            diagnostic.clear();
            return true;
        }
        catch(const std::exception & error) {
            diagnostic = error.what();
            return false;
        }
        catch(...) {
            diagnostic = "unknown reconstructed OctoMap materialization failure";
            return false;
        }
    }

}// namespace PerceptionMapUpdate
