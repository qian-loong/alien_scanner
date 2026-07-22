#ifndef SWARM_CONTROLLER_RAYRETURN_HPP
#define SWARM_CONTROLLER_RAYRETURN_HPP

#include "swarm_controller/Point3f.hpp"

namespace SwarmController {

    /// map 坐标系下的一束 LiDAR return。hit=false 表示 endpoint 只是 max_range free-ray 终点。
    struct RayReturn {
        Point3f endpoint {};
        float   range {};
        bool    hit {false};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_RAYRETURN_HPP
