#include "perception_input_node/SensorSessionManager.hpp"
#include <algorithm>
#include <sstream>
#include <utility>

namespace Perception::Input {

    SensorSessionManager::SensorSessionManager(SessionID session_id)
        : session_id_(session_id)
    {}

    SensorSessionManager::RegistrationResult SensorSessionManager::register_sensor(
            const SensorDescriptor & descriptor)
    {
        const auto existing = descriptors_.find(descriptor.sensor_id);
        if(existing != descriptors_.end()) {
            if(existing->second == descriptor) {
                return RegistrationResult {true, "Sensor descriptor already registered"};
            }

            std::ostringstream oss;
            oss << "Sensor '" << descriptor.sensor_id.value
                << "' descriptor mismatch. Registered=" << describe_descriptor(existing->second)
                << ", attempted=" << describe_descriptor(descriptor)
                << ". Start a new vehicle session before changing a descriptor.";
            return RegistrationResult {false, oss.str()};
        }

        if(frozen_) {
            std::ostringstream oss;
            oss << "Descriptor inventory is frozen: cannot add sensor '"
                << descriptor.sensor_id.value << "'. Current sensors=" << inventory_summary()
                << ", attempted=" << describe_descriptor(descriptor)
                << ". Start a new vehicle session to change the inventory.";
            return RegistrationResult {false, oss.str()};
        }

        descriptors_.emplace(descriptor.sensor_id, descriptor);
        return RegistrationResult {true, "Sensor descriptor registered"};
    }

    bool SensorSessionManager::can_reconnect(
            const SensorDescriptor & descriptor,
            std::string *            diagnostic) const
    {
        const auto existing = descriptors_.find(descriptor.sensor_id);
        if(existing == descriptors_.end()) {
            if(diagnostic != nullptr) {
                std::ostringstream oss;
                oss << "Sensor '" << descriptor.sensor_id.value
                    << "' is not part of the registered inventory " << inventory_summary();
                *diagnostic = oss.str();
            }
            return false;
        }

        if(existing->second != descriptor) {
            if(diagnostic != nullptr) {
                std::ostringstream oss;
                oss << "Sensor '" << descriptor.sensor_id.value
                    << "' cannot reconnect with a changed descriptor. Registered="
                    << describe_descriptor(existing->second)
                    << ", attempted=" << describe_descriptor(descriptor);
                *diagnostic = oss.str();
            }
            return false;
        }

        if(diagnostic != nullptr) {
            diagnostic->clear();
        }
        return true;
    }

    const SensorDescriptor * SensorSessionManager::find_descriptor(const SensorID & sensor_id) const
    {
        const auto descriptor = descriptors_.find(sensor_id);
        return descriptor == descriptors_.end() ? nullptr : &descriptor->second;
    }

    std::vector<SensorDescriptor> SensorSessionManager::descriptors() const
    {
        std::vector<SensorDescriptor> result;
        result.reserve(descriptors_.size());
        for(const auto & entry : descriptors_) {
            result.push_back(entry.second);
        }
        std::sort(
                result.begin(), result.end(),
                [](const SensorDescriptor & lhs, const SensorDescriptor & rhs) {
                    return lhs.sensor_id < rhs.sensor_id;
                });
        return result;
    }

    std::string SensorSessionManager::describe_descriptor(const SensorDescriptor & descriptor)
    {
        std::ostringstream oss;
        oss << "{sensor_id=" << descriptor.sensor_id.value
            << ", type=" << (descriptor.type == SensorType::LIDAR_2D ? "2d" : "3d")
            << ", frame_id=" << descriptor.frame_id
            << ", ray_evidence=" << ray_evidence_name(descriptor.ray_evidence)
            << ", range=[" << descriptor.range_min_m << ", " << descriptor.range_max_m << "]}";
        return oss.str();
    }

    std::string SensorSessionManager::inventory_summary() const
    {
        const auto         inventory = descriptors();
        std::ostringstream oss;
        oss << '[';
        for(std::size_t index = 0; index < inventory.size(); ++index) {
            if(index > 0) {
                oss << ", ";
            }
            oss << inventory[index].sensor_id.value;
        }
        oss << ']';
        return oss.str();
    }

}// namespace Perception::Input
