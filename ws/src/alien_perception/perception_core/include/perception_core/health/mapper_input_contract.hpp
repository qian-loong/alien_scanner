#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_MAPPER_INPUT_CONTRACT_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_MAPPER_INPUT_CONTRACT_HPP

#include "perception_core/observation/sensor_descriptor.hpp"
#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace Perception {

    /// Sensor requirement specification
    struct SensorRequirement {
        SensorType type;
        size_t     min_count;// Minimum number required

        // Optional: specific sensor IDs
        std::optional<std::vector<SensorID>> specific_sensors;
    };

    /// Degraded combination specification
    struct DegradedCombination {
        std::vector<SensorRequirement> requirements;
        std::string                    description;// Degradation description (for logging)
    };

    /// Mapper input contract declaration
    struct MapperInputContract {
        // Minimum requirements
        std::vector<SensorRequirement> minimum_viable;

        // Degraded configurations (sorted by priority)
        std::vector<DegradedCombination> degraded_combinations;

        // Pose requirements
        bool                       requires_pose            = true;
        Duration                   pose_freshness_threshold = Duration::from_seconds(1.0);
        std::optional<std::string> expected_pose_frame;
        double                     minimum_pose_quality = 0.0;

        // Number of consecutive evaluations required before a recovery state
        // transition is committed. Fault transitions remain immediate.
        std::size_t recovery_stability_samples = 3;
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_MAPPER_INPUT_CONTRACT_HPP
