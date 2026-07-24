#include "perception_adapters/point_cloud2_adapter.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>

namespace Perception::Adapters {

    LidarObservation PointCloud2Adapter::convert(
            const sensor_msgs::msg::PointCloud2 & msg,
            const SensorDescriptor &              descriptor,
            const SessionID &                     session_id,
            const std::string &                   clock_domain) const
    {
        LidarObservation obs;

        // Identity and timing
        obs.sensor_id    = descriptor.sensor_id;
        obs.session_id   = session_id;
        obs.frame_id     = msg.header.frame_id;
        obs.clock_domain = clock_domain;
        obs.origin_stamp = Timestamp::from_nanoseconds(
                msg.header.stamp.sec * 1'000'000'000LL + msg.header.stamp.nanosec);

        // Extract point cloud
        obs.data = extract_cloud(msg);

        return obs;
    }

    PointCloud2Adapter::ValidationResult PointCloud2Adapter::validate(
            const sensor_msgs::msg::PointCloud2 & msg,
            const SensorDescriptor &              descriptor) const
    {
        // Check sensor type
        if(descriptor.type != SensorType::LIDAR_3D) {
            return ValidationResult::failure("Descriptor type is not LIDAR_3D");
        }

        // Check frame_id match
        if(msg.header.frame_id != descriptor.frame_id) {
            std::ostringstream oss;
            oss << "Frame ID mismatch: msg=" << msg.header.frame_id
                << ", descriptor=" << descriptor.frame_id;
            return ValidationResult::failure(oss.str());
        }

        // Check for XYZ fields
        bool has_x = false, has_y = false, has_z = false;
        for(const auto & field : msg.fields) {
            if(field.name == "x") {
                has_x = true;
            }
            if(field.name == "y") {
                has_y = true;
            }
            if(field.name == "z") {
                has_z = true;
            }
        }

        if(!has_x || !has_y || !has_z) {
            return ValidationResult::failure("Missing required XYZ fields");
        }

        // Check point count
        if(msg.width == 0 || msg.height == 0) {
            return ValidationResult::failure("Empty point cloud");
        }

        return ValidationResult::success();
    }

    Cloud3D PointCloud2Adapter::extract_cloud(const sensor_msgs::msg::PointCloud2 & msg) const
    {
        Cloud3D cloud;

        // Reserve space
        size_t num_points = msg.width * msg.height;
        cloud.points.reserve(num_points);

        // Create iterators
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

        // Check if intensity field exists
        bool has_intensity = false;
        for(const auto & field : msg.fields) {
            if(field.name == "intensity") {
                has_intensity = true;
                break;
            }
        }

        if(has_intensity) {
            sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");

            for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
                cloud.points.emplace_back(*iter_x, *iter_y, *iter_z, *iter_intensity);
            }
        }
        else {
            for(; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                cloud.points.emplace_back(*iter_x, *iter_y, *iter_z, 0.0f);
            }
        }

        return cloud;
    }

}// namespace Perception::Adapters
