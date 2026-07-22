#ifndef SWARM_CONTROLLER_OCTOMAPBUILDER_HPP
#define SWARM_CONTROLLER_OCTOMAPBUILDER_HPP

#include "swarm_controller/RayReturn.hpp"

#include <cstddef>
#include <vector>

#include <octomap/OcTree.h>

namespace SwarmController {

    enum class CellState {
        Unknown,
        Free,
        Occupied,
    };

    class OctoMapBuilder
    {
    public:
        explicit OctoMapBuilder(float resolution);

        void insertScan(const Point3f & origin_map, const std::vector<RayReturn> & returns_map);

        CellState query(float x, float y, float z) const;
        std::size_t occupiedCount() const;
        std::size_t knownCount() const;

        const octomap::OcTree & tree() const;

    private:
        octomap::OcTree tree_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_OCTOMAPBUILDER_HPP
