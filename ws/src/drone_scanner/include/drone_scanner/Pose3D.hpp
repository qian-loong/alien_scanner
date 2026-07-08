#ifndef DRONE_SCANNER_POSE3D_HPP
#define DRONE_SCANNER_POSE3D_HPP

namespace DroneScanner {

    /// 平面飞行常用位姿：位置 + 绕 z 的 yaw（rad，从 +x 逆时针）。
    struct Pose3D {
        float x {0.0F};
        float y {0.0F};
        float z {0.0F};
        float yaw {0.0F};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_POSE3D_HPP
