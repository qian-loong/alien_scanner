#ifndef PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_LASER_SCAN_ADAPTER_HPP
#define PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_LASER_SCAN_ADAPTER_HPP

#include "perception_core/observation/lidar_observation.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <memory>
#include <optional>

namespace Perception::Adapters {

    /// LaserScan to LidarObservation adapter
    class LaserScanAdapter
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

        /// Convert LaserScan to LidarObservation
        LidarObservation convert(
                const sensor_msgs::msg::LaserScan & msg,
                const SensorDescriptor &            descriptor,
                const SessionID &                   session_id,
                const std::string &                 clock_domain = "vehicle_steady_clock") const;

        /// Validate message compatibility with descriptor
        ValidationResult validate(
                const sensor_msgs::msg::LaserScan & msg,
                const SensorDescriptor &            descriptor) const;
    };

}// namespace Perception::Adapters

#endif// PERCEPTION_ADAPTERS_INCLUDE_PERCEPTION_ADAPTERS_LASER_SCAN_ADAPTER_HPP
