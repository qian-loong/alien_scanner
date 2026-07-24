#include "perception_adapters/laser_scan_adapter.hpp"
#include <cmath>
#include <sstream>

namespace Perception::Adapters {

    LidarObservation LaserScanAdapter::convert(
            const sensor_msgs::msg::LaserScan & msg,
            const SensorDescriptor &            descriptor,
            const SessionID &                   session_id,
            const std::string &                 clock_domain) const
    {
        LidarObservation obs;

        // Identity and timing
        obs.sensor_id    = descriptor.sensor_id;
        obs.session_id   = session_id;
        obs.frame_id     = msg.header.frame_id;
        obs.clock_domain = clock_domain;
        obs.origin_stamp = Timestamp::from_nanoseconds(
                msg.header.stamp.sec * 1'000'000'000LL + msg.header.stamp.nanosec);

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
