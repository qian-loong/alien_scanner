#ifndef PERCEPTION_LOCAL_MAP_OCTOMAP_SNAPSHOT_BRIDGE_HPP
#define PERCEPTION_LOCAL_MAP_OCTOMAP_SNAPSHOT_BRIDGE_HPP

#include "octomap_msgs/msg/octomap.hpp"
#include "perception_local_map/LocalObservationMapper.hpp"

namespace PerceptionLocalMap {

    class OctoMapSnapshotBridge
    {
    public:
        static bool materialize(
                const MapReadTransaction & transaction,
                octomap_msgs::msg::Octomap & message);
    };

}// namespace PerceptionLocalMap

#endif// PERCEPTION_LOCAL_MAP_OCTOMAP_SNAPSHOT_BRIDGE_HPP
