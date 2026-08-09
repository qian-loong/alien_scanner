#ifndef PERCEPTION_MAP_UPDATE_OCTOMAP_VIEW_ADAPTER_HPP
#define PERCEPTION_MAP_UPDATE_OCTOMAP_VIEW_ADAPTER_HPP

#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateLimits.hpp"

#include "octomap_msgs/msg/octomap.hpp"

#include <string>

namespace PerceptionMapUpdate {

    class OctoMapViewAdapter
    {
    public:
        static bool materialize(
                const ReconstructedMap & map,
                octomap_msgs::msg::Octomap & message,
                std::string & diagnostic,
                const MapUpdateLimits & limits = {});
    };

}// namespace PerceptionMapUpdate

#endif// PERCEPTION_MAP_UPDATE_OCTOMAP_VIEW_ADAPTER_HPP
