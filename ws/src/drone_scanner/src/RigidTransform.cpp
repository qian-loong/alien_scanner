#include "drone_scanner/RigidTransform.hpp"

namespace DroneScanner {

    Point3f transformPoint(const Point3f & point, const RigidTransform & transform)
    {
        const float qx = transform.qx;
        const float qy = transform.qy;
        const float qz = transform.qz;
        const float qw = transform.qw;

        const float ix = qw * point.x + qy * point.z - qz * point.y;
        const float iy = qw * point.y + qz * point.x - qx * point.z;
        const float iz = qw * point.z + qx * point.y - qy * point.x;
        const float iw = -qx * point.x - qy * point.y - qz * point.z;

        Point3f out;
        out.x = ix * qw + iw * -qx + iy * -qz - iz * -qy + transform.tx;
        out.y = iy * qw + iw * -qy + iz * -qx - ix * -qz + transform.ty;
        out.z = iz * qw + iw * -qz + ix * -qy - iy * -qx + transform.tz;
        return out;
    }

}// namespace DroneScanner
