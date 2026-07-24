#ifndef PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_POINT_CLOUD2_ADAPTER_HPP
#define PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_POINT_CLOUD2_ADAPTER_HPP

#include "perception_core/observation/lidar_observation.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>
#include <optional>

namespace Perception::Adapters {

    /// PointCloud2 to LidarObservation adapter
    class PointCloud2Adapter
    {
    public:
        /// Validation result
        struct ValidationResult {
            bool        valid;
            std::string error_message;

            static ValidationResult success()
            {
                return ValidationResult {true, ""};
            }

            static ValidationResult failure(const std::string & msg)
            {
                return ValidationResult {false, msg};
            }
        };

        /// Convert PointCloud2 to LidarObservation
        LidarObservation convert(
                const sensor_msgs::msg::PointCloud2 & msg,
                const SensorDescriptor &              descriptor,
                const SessionID &                     session_id,
                const std::string &                   clock_domain = "vehicle_steady_clock") const;

        /// Validate message compatibility with descriptor
        ValidationResult validate(
                const sensor_msgs::msg::PointCloud2 & msg,
                const SensorDescriptor &              descriptor) const;

    private:
        /// Extract Cloud3D from PointCloud2
        Cloud3D extract_cloud(const sensor_msgs::msg::PointCloud2 & msg) const;
    };

}// namespace Perception::Adapters

#endif// PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_POINT_CLOUD2_ADAPTER_HPP
