#ifndef SWARM_CONTROLLER_POSE3D_HPP
#define SWARM_CONTROLLER_POSE3D_HPP

#include "swarm_controller/Point3f.hpp"

namespace SwarmController {

    /// map 坐标系位姿；yaw 为绕 +Z 的弧度角。
    struct Pose3D {
        Point3f position {};
        float   yaw {};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_POSE3D_HPP
