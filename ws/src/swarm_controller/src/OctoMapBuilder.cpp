#include "swarm_controller/OctoMapBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace SwarmController {

    namespace {

        octomap::point3d toPoint3d(const Point3f & point)
        {
            return octomap::point3d(point.x, point.y, point.z);
        }

        bool isFinite(const Point3f & point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

    }// namespace

    OctoMapBuilder::OctoMapBuilder(float resolution)
        : tree_(std::max(resolution, 0.01F))
    {
        if(!std::isfinite(resolution) || resolution <= 0.0F) {
            throw std::invalid_argument("OctoMapBuilder: resolution must be positive");
        }
    }

    void OctoMapBuilder::insertScan(const Point3f & origin_map, const std::vector<RayReturn> & returns_map)
    {
        if(!isFinite(origin_map)) {
            return;
        }

        const octomap::point3d origin = toPoint3d(origin_map);
        octomap::KeyRay       ray_keys;
        octomap::KeySet       free_keys;
        octomap::KeySet       occupied_keys;
        for(const RayReturn & ret : returns_map) {
            if(!isFinite(ret.endpoint) || !std::isfinite(ret.range) || ret.range <= 0.0F) {
                continue;
            }

            const octomap::point3d endpoint = toPoint3d(ret.endpoint);
            ray_keys.reset();
            if(tree_.computeRayKeys(origin, endpoint, ray_keys)) {
                for(const octomap::OcTreeKey & key : ray_keys) {
                    free_keys.insert(key);
                }
            }
            if(ret.hit) {
                octomap::OcTreeKey endpoint_key;
                if(tree_.coordToKeyChecked(endpoint, endpoint_key)) {
                    occupied_keys.insert(endpoint_key);
                }
            }
        }

        for(const octomap::OcTreeKey & key : occupied_keys) {
            free_keys.erase(key);
        }
        for(const octomap::OcTreeKey & key : free_keys) {
            tree_.updateNode(key, false, true);
        }
        for(const octomap::OcTreeKey & key : occupied_keys) {
            tree_.updateNode(key, true, true);
        }
        tree_.updateInnerOccupancy();
    }

    CellState OctoMapBuilder::query(float x, float y, float z) const
    {
        const octomap::OcTreeNode * node = tree_.search(x, y, z);
        if(node == nullptr) {
            return CellState::Unknown;
        }
        return tree_.isNodeOccupied(node) ? CellState::Occupied : CellState::Free;
    }

    std::size_t OctoMapBuilder::occupiedCount() const
    {
        std::size_t count = 0;
        for(auto it = tree_.begin_leafs(), end = tree_.end_leafs(); it != end; ++it) {
            if(tree_.isNodeOccupied(*it)) {
                ++count;
            }
        }
        return count;
    }

    std::size_t OctoMapBuilder::knownCount() const
    {
        std::size_t count = 0;
        for(auto it = tree_.begin_leafs(), end = tree_.end_leafs(); it != end; ++it) {
            ++count;
        }
        return count;
    }

    const octomap::OcTree & OctoMapBuilder::tree() const
    {
        return tree_;
    }

}// namespace SwarmController
