#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_TYPES_IDENTITY_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_TYPES_IDENTITY_HPP

#include <cstdint>
#include <string>

namespace Perception {

    /// Stable sensor identifier (configured, does not change across reboots)
    struct SensorID {
        std::string value;// e.g., "lidar_front", "lidar_rear"

        bool operator==(const SensorID & other) const { return value == other.value; }
        bool operator!=(const SensorID & other) const { return value != other.value; }
        bool operator<(const SensorID & other) const { return value < other.value; }
    };

    /// Session identifier generated at each boot/restart
    struct SessionID {
        uint64_t boot_time_ns; // Boot timestamp (nanoseconds)
        uint32_t random_suffix;// Random suffix (avoid clock rollback collision)

        bool operator==(const SessionID & other) const
        {
            return boot_time_ns == other.boot_time_ns && random_suffix == other.random_suffix;
        }
        bool operator!=(const SessionID & other) const
        {
            return !(*this == other);
        }
        bool operator<(const SessionID & other) const
        {
            if(boot_time_ns != other.boot_time_ns) {
                return boot_time_ns < other.boot_time_ns;
            }
            return random_suffix < other.random_suffix;
        }
    };

    /// Pose source identifier (alias of SensorID)
    using SourceID = SensorID;

}// namespace Perception

// Hash support for std::unordered_map
namespace std {
    template<>
    struct hash<Perception::SensorID> {
        size_t operator()(const Perception::SensorID & id) const
        {
            return hash<string>()(id.value);
        }
    };

    template<>
    struct hash<Perception::SessionID> {
        size_t operator()(const Perception::SessionID & id) const
        {
            return hash<uint64_t>()(id.boot_time_ns) ^ (hash<uint32_t>()(id.random_suffix) << 1);
        }
    };
}// namespace std

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_TYPES_IDENTITY_HPP
