#ifndef DRONE_SCANNER_RIGIDTRANSFORM_HPP
#define DRONE_SCANNER_RIGIDTRANSFORM_HPP

#include "drone_scanner/Point3f.hpp"

namespace DroneScanner {

    /// 刚体变换：平移 + 四元数 (x, y, z, w)。调用方应提供单位四元数。
    struct RigidTransform {
        float tx {0.0F};
        float ty {0.0F};
        float tz {0.0F};
        float qx {0.0F};
        float qy {0.0F};
        float qz {0.0F};
        float qw {1.0F};
    };

    Point3f transformPoint(const Point3f & point, const RigidTransform & transform);

}// namespace DroneScanner

#endif// DRONE_SCANNER_RIGIDTRANSFORM_HPP
