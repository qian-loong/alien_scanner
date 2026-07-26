#include "perception_adapters/laser_scan_adapter.hpp"
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace Perception::Adapters {

    LidarObservation LaserScanAdapter::convert(
            const sensor_msgs::msg::LaserScan & msg,
            const SensorDescriptor &            descriptor,
            const SessionID &                   session_id,
            const std::string &                 clock_domain) const
    {
        const auto validation = validate(msg, descriptor);
        if(!validation.valid) {
            throw std::invalid_argument(
                    "Invalid LaserScan conversion input: " + validation.error_message);
        }

        LidarObservation obs;

        // Identity and timing
        obs.sensor_id    = descriptor.sensor_id;
        obs.session_id   = session_id;
        obs.frame_id     = msg.header.frame_id;
        obs.clock_domain = clock_domain;
        obs.origin_stamp = Timestamp::from_nanoseconds(
                msg.header.stamp.sec * 1'000'000'000LL + msg.header.stamp.nanosec);
        obs.ray_evidence = descriptor.ray_evidence;

        // Build Scan2D
        Scan2D scan;
        scan.angle_min_rad       = msg.angle_min;
        scan.angle_max_rad       = msg.angle_max;
        scan.angle_increment_rad = msg.angle_increment;
        scan.range_min_m         = msg.range_min;
        scan.range_max_m         = msg.range_max;
        scan.ranges              = msg.ranges;
        scan.intensities         = msg.intensities;

        obs.data = scan;

        return obs;
    }

    LaserScanAdapter::ValidationResult LaserScanAdapter::validate(
            const sensor_msgs::msg::LaserScan & msg,
            const SensorDescriptor &            descriptor) const
    {
        // Check sensor type
        if(descriptor.type != SensorType::LIDAR_2D) {
            return ValidationResult::failure("Descriptor type is not LIDAR_2D");
        }

        if(!is_valid_ray_evidence(descriptor.ray_evidence)) {
            return ValidationResult::failure("Descriptor has unknown ray evidence capability");
        }

        // Check frame_id match
        if(msg.header.frame_id != descriptor.frame_id) {
            std::ostringstream oss;
            oss << "Frame ID mismatch: msg=" << msg.header.frame_id
                << ", descriptor=" << descriptor.frame_id;
            return ValidationResult::failure(oss.str());
        }

        // Check ranges non-empty
        if(msg.ranges.empty()) {
            return ValidationResult::failure("Empty ranges array");
        }
        if(!msg.intensities.empty() && msg.intensities.size() != msg.ranges.size()) {
            return ValidationResult::failure(
                    "Intensity array must be empty or match ranges array length");
        }

        if(!std::isfinite(msg.range_min) || !std::isfinite(msg.range_max)
           || msg.range_min < 0.0F || msg.range_min >= msg.range_max) {
            return ValidationResult::failure(
                    "Invalid range metadata: expected finite 0 <= range_min < range_max");
        }

        if(!std::isfinite(msg.angle_min) || !std::isfinite(msg.angle_max)
           || !std::isfinite(msg.angle_increment) || msg.angle_increment <= 0.0F
           || msg.angle_max < msg.angle_min) {
            return ValidationResult::failure(
                    "Invalid angle metadata: expected finite angle bounds and positive increment");
        }

        if(provides_at_least(descriptor.ray_evidence, RayEvidenceCapability::HitRay)) {
            constexpr double tolerance = 1e-5;
            const auto matches = [](double actual, double expected) {
                return std::isfinite(expected) && std::abs(actual - expected) <= tolerance;
            };

            if(!matches(msg.range_min, descriptor.range_min_m)
               || !matches(msg.range_max, descriptor.range_max_m)) {
                return ValidationResult::failure(
                        "LaserScan range metadata differs from frozen descriptor");
            }
            if(!matches(msg.angle_min, descriptor.fov.horizontal_min_rad)
               || !matches(msg.angle_max, descriptor.fov.horizontal_max_rad)) {
                return ValidationResult::failure(
                        "LaserScan field of view differs from frozen descriptor");
            }
            if(!matches(msg.angle_increment, descriptor.angular_resolution_rad)) {
                return ValidationResult::failure(
                        "LaserScan angular resolution differs from frozen descriptor");
            }
        }

        // Check angle configuration consistency
        double expected_num_rays = std::ceil((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1;
        if(std::abs(static_cast<double>(msg.ranges.size()) - expected_num_rays) > 2) {
            std::ostringstream oss;
            oss << "Inconsistent angle configuration: ranges.size=" << msg.ranges.size()
                << ", expected~" << expected_num_rays;
            return ValidationResult::failure(oss.str());
        }

        return ValidationResult::success();
    }

}// namespace Perception::Adapters
