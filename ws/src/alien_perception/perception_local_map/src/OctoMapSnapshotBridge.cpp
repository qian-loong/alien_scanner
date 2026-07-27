#include "OctoMapSnapshotBridge.hpp"

#include "perception_local_map/OctoMapBackend.hpp"

#include <octomap_msgs/conversions.h>

namespace PerceptionLocalMap {

    bool OctoMapSnapshotBridge::materialize(
            const MapReadTransaction & transaction,
            octomap_msgs::msg::Octomap & message)
    {
        const auto * backend = transaction.backend_for_snapshot();
        const auto * octomap_backend = dynamic_cast<const OctoMapBackend *>(backend);
        if(octomap_backend == nullptr) {
            return false;
        }
        return octomap_msgs::binaryMapToMsg(
                octomap_backend->tree_for_snapshot(), message);
    }

}// namespace PerceptionLocalMap
