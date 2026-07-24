#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_SENSOR_DESCRIPTOR_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_SENSOR_DESCRIPTOR_HPP

#include "perception_core/types/identity.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace Perception {

    enum class SensorType
    {
        LIDAR_2D,
        LIDAR_3D
    };

    /// Field of view specification
    struct FieldOfView {
        double horizontal_min_rad;// Horizontal FOV minimum (radians)
        double horizontal_max_rad;// Horizontal FOV maximum (radians)
        double vertical_min_rad;  // Vertical FOV minimum (0 for 2D)
        double vertical_max_rad;  // Vertical FOV maximum (0 for 2D)

        bool operator==(const FieldOfView & other) const
        {
            return horizontal_min_rad == other.horizontal_min_rad && horizontal_max_rad == other.horizontal_max_rad && vertical_min_rad == other.vertical_min_rad && vertical_max_rad == other.vertical_max_rad;
        }

        bool operator!=(const FieldOfView & other) const { return !(*this == other); }
    };

    /// Sensor descriptor (frozen within a vehicle session)
    struct SensorDescriptor {
        SensorID    sensor_id;
        SensorType  type;
        std::string frame_id;// TF frame name

        // Mounting pose (relative to base_link)
        Eigen::Vector3d    mounting_position;
        Eigen::Quaterniond mounting_orientation;

        // Performance parameters
        FieldOfView fov;
        double      angular_resolution_rad;// Angular resolution
        double      range_min_m;           // Minimum valid range
        double      range_max_m;           // Maximum valid range

        bool operator==(const SensorDescriptor & other) const
        {
            return sensor_id == other.sensor_id && type == other.type && frame_id == other.frame_id && mounting_position.isApprox(other.mounting_position, 1e-6) && mounting_orientation.isApprox(other.mounting_orientation, 1e-6) && fov == other.fov && std::abs(angular_resolution_rad - other.angular_resolution_rad) < 1e-9 && std::abs(range_min_m - other.range_min_m) < 1e-6 && std::abs(range_max_m - other.range_max_m) < 1e-6;
        }

        bool operator!=(const SensorDescriptor & other) const { return !(*this == other); }
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_SENSOR_DESCRIPTOR_HPP
