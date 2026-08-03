#include "perception_adapters/point_cloud2_adapter.hpp"
#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace Perception::Adapters {

    namespace {

        const sensor_msgs::msg::PointField * find_field(
                const sensor_msgs::msg::PointCloud2 & message,
                const std::string &                   name)
        {
            const sensor_msgs::msg::PointField * result = nullptr;
            for(const auto & field : message.fields) {
                if(field.name != name) {
                    continue;
                }
                if(result != nullptr) {
                    return nullptr;
                }
                result = &field;
            }
            return result;
        }

        std::size_t count_fields(
                const sensor_msgs::msg::PointCloud2 & message,
                const std::string &                   name)
        {
            return static_cast<std::size_t>(std::count_if(
                    message.fields.begin(), message.fields.end(),
                    [&](const sensor_msgs::msg::PointField & field) {
                        return field.name == name;
                    }));
        }

        std::string validate_float_field(
                const sensor_msgs::msg::PointField * field,
                const std::string &                  name,
                std::uint32_t                        point_step)
        {
            if(field == nullptr) {
                return "Missing or duplicate required " + name + " field";
            }
            if(field->datatype != sensor_msgs::msg::PointField::FLOAT32 || field->count != 1) {
                return name + " field must contain exactly one FLOAT32 value";
            }
            const auto field_end = static_cast<std::uint64_t>(field->offset) + sizeof(float);
            if(field_end > point_step) {
                return name + " field extends beyond point_step";
            }
            return {};
        }

        bool host_is_big_endian()
        {
            const std::uint16_t value = 0x0102U;
            unsigned char       first_byte = 0;
            std::memcpy(&first_byte, &value, sizeof(first_byte));
            return first_byte == 0x01U;
        }

        std::uint32_t byte_swap_32(std::uint32_t value)
        {
            return ((value & 0x000000FFU) << 24U)
                    | ((value & 0x0000FF00U) << 8U)
                    | ((value & 0x00FF0000U) >> 8U)
                    | ((value & 0xFF000000U) >> 24U);
        }

        float read_float32(const std::uint8_t * data, bool data_is_big_endian)
        {
            std::uint32_t bits = 0;
            std::memcpy(&bits, data, sizeof(bits));
            if(data_is_big_endian != host_is_big_endian()) {
                bits = byte_swap_32(bits);
            }

            float value = 0.0F;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        }

    }// namespace

    LidarObservation PointCloud2Adapter::convert(
            const sensor_msgs::msg::PointCloud2 & msg,
            const SensorDescriptor &              descriptor,
            const SessionID &                     session_id,
            const std::string &                   clock_domain) const
    {
        const auto validation = validate(msg, descriptor);
        if(!validation.valid) {
            throw std::invalid_argument(
                    "Invalid PointCloud2 conversion input: " + validation.error_message);
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

        if(descriptor.ray_evidence != RayEvidenceCapability::HitOnly) {
            return ValidationResult::failure(
                    "PointCloud2Adapter only supports hit_only ray evidence");
        }

        // Check frame_id match
        if(msg.header.frame_id != descriptor.frame_id) {
            std::ostringstream oss;
            oss << "Frame ID mismatch: msg=" << msg.header.frame_id
                << ", descriptor=" << descriptor.frame_id;
            return ValidationResult::failure(oss.str());
        }

        // Check point count
        if(msg.width == 0 || msg.height == 0) {
            return ValidationResult::failure("Empty point cloud");
        }
        if(msg.point_step == 0) {
            return ValidationResult::failure("PointCloud2 point_step must be greater than zero");
        }

        const auto * field_x = find_field(msg, "x");
        const auto * field_y = find_field(msg, "y");
        const auto * field_z = find_field(msg, "z");
        for(const auto & error : {
                    validate_float_field(field_x, "x", msg.point_step),
                    validate_float_field(field_y, "y", msg.point_step),
                    validate_float_field(field_z, "z", msg.point_step)}) {
            if(!error.empty()) {
                return ValidationResult::failure(error);
            }
        }

        const auto * field_intensity = find_field(msg, "intensity");
        if(count_fields(msg, "intensity") > 1) {
            return ValidationResult::failure("Duplicate intensity field");
        }
        if(field_intensity != nullptr) {
            const auto error = validate_float_field(field_intensity, "intensity", msg.point_step);
            if(!error.empty()) {
                return ValidationResult::failure(error);
            }
        }

        const auto minimum_row_step = static_cast<std::uint64_t>(msg.point_step) * msg.width;
        if(msg.row_step < minimum_row_step) {
            return ValidationResult::failure(
                    "PointCloud2 row_step is smaller than width * point_step");
        }
        const auto required_data_size = static_cast<std::uint64_t>(msg.row_step) * msg.height;
        if(required_data_size > std::numeric_limits<std::size_t>::max()
           || msg.data.size() != static_cast<std::size_t>(required_data_size)) {
            return ValidationResult::failure(
                    "PointCloud2 data length must equal row_step * height");
        }

        return ValidationResult::success();
    }

    Cloud3D PointCloud2Adapter::extract_cloud(const sensor_msgs::msg::PointCloud2 & msg) const
    {
        Cloud3D cloud;

        const auto * field_x         = find_field(msg, "x");
        const auto * field_y         = find_field(msg, "y");
        const auto * field_z         = find_field(msg, "z");
        const auto * field_intensity = find_field(msg, "intensity");

        const auto num_points = static_cast<std::size_t>(msg.width)
                * static_cast<std::size_t>(msg.height);
        cloud.points.reserve(num_points);

        for(std::uint32_t row = 0; row < msg.height; ++row) {
            const auto row_offset = static_cast<std::size_t>(row) * msg.row_step;
            for(std::uint32_t column = 0; column < msg.width; ++column) {
                const auto point_offset = row_offset
                        + static_cast<std::size_t>(column) * msg.point_step;
                const auto * point_data = msg.data.data() + point_offset;
                const auto intensity = field_intensity == nullptr
                        ? 0.0F
                        : read_float32(
                                  point_data + field_intensity->offset,
                                  msg.is_bigendian);
                cloud.points.emplace_back(
                        read_float32(point_data + field_x->offset, msg.is_bigendian),
                        read_float32(point_data + field_y->offset, msg.is_bigendian),
                        read_float32(point_data + field_z->offset, msg.is_bigendian),
                        intensity);
            }
        }

        return cloud;
    }

}// namespace Perception::Adapters
