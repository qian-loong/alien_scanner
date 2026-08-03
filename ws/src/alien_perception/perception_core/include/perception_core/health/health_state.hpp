#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_HEALTH_STATE_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_HEALTH_STATE_HPP

#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"
#include <string>

namespace Perception {

    /// Health state enumeration
    enum class HealthState
    {
        Healthy,   // Satisfies minimum viable input
        Degraded,  // Satisfies degraded combination but not minimum
        Unavailable// Does not satisfy any combination
    };

    /// Sensor health status
    enum class SensorHealthStatus
    {
        Active,// Receiving data normally
        Stale, // Data timeout
        Lost   // Connection lost
    };

    /// Sensor health information
    struct SensorHealth {
        SensorID           sensor_id;
        SensorHealthStatus status;
        Timestamp          last_update;

        bool is_healthy() const
        {
            return status == SensorHealthStatus::Active;
        }
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_HEALTH_STATE_HPP
