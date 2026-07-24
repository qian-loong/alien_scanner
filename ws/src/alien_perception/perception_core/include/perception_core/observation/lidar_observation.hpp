#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_LIDAR_OBSERVATION_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_LIDAR_OBSERVATION_HPP

#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"
#include <string>
#include <variant>
#include <vector>

namespace Perception {

    /// 2D laser scan data
    struct Scan2D {
        double             angle_min_rad;
        double             angle_max_rad;
        double             angle_increment_rad;
        double             range_min_m;
        double             range_max_m;
        std::vector<float> ranges;     // Distance array
        std::vector<float> intensities;// Intensity array (optional)

        size_t point_count() const
        {
            return ranges.size();
        }
    };

    /// 3D point cloud data
    struct Cloud3D {
        struct Point {
            float x, y, z;
            float intensity;// Optional

            Point()
                : x(0)
                , y(0)
                , z(0)
                , intensity(0)
            {}
            Point(float x_, float y_, float z_, float intensity_ = 0)
                : x(x_)
                , y(y_)
                , z(z_)
                , intensity(intensity_)
            {}
        };
        std::vector<Point> points;

        size_t point_count() const
        {
            return points.size();
        }
    };

    /// Unified lidar observation interface
    struct LidarObservation {
        SensorID    sensor_id;
        SessionID   session_id;
        std::string frame_id;
        std::string clock_domain;// Clock domain identifier (e.g., "vehicle_steady_clock", "gps_time")
        Timestamp   origin_stamp;// Observation acquisition timestamp

        // Actual data (interpreted based on descriptor.type)
        std::variant<Scan2D, Cloud3D> data;

        // Get point count
        size_t point_count() const
        {
            return std::visit([](const auto & d) { return d.point_count(); }, data);
        }

        // Check if 2D/3D
        bool is_2d() const { return std::holds_alternative<Scan2D>(data); }
        bool is_3d() const { return std::holds_alternative<Cloud3D>(data); }

        // Get typed data (throws if wrong type)
        const Scan2D &  as_scan_2d() const { return std::get<Scan2D>(data); }
        const Cloud3D & as_cloud_3d() const { return std::get<Cloud3D>(data); }
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_LIDAR_OBSERVATION_HPP
