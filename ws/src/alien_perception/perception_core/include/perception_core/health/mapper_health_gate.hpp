#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_MAPPER_HEALTH_GATE_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_MAPPER_HEALTH_GATE_HPP

#include "perception_core/health/health_state.hpp"
#include "perception_core/health/mapper_input_contract.hpp"
#include "perception_core/observation/pose_estimate.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace Perception {

    /// Mapper health gate - evaluates health state based on input
    class MapperHealthGate
    {
    public:
        explicit MapperHealthGate(MapperInputContract contract);

        /// Capability set - current effective capabilities
        struct CapabilitySet {
            bool   has_2d_lidar                = false;
            bool   has_3d_lidar                = false;
            bool   has_pose                    = false;
            bool   has_fresh_pose              = false;
            bool   has_expected_pose_frame     = false;
            bool   has_sufficient_pose_quality = false;
            bool   has_usable_pose             = false;
            bool   has_free_space_hit_rays     = false;
            bool   has_full_no_return_rays     = false;
            size_t active_sensor_count         = 0;
            size_t active_2d_lidar_count       = 0;
            size_t active_3d_lidar_count       = 0;
        };

        /// Evaluate current health state
        HealthState evaluate(
                const std::vector<SensorDescriptor> & descriptors,
                const std::vector<SensorHealth> &     sensor_health,
                const std::optional<PoseEstimate> &   pose);

        /// Get current effective capability set
        CapabilitySet current_capability() const { return current_capability_; }

        /// Get degradation reason (for diagnostics)
        std::string degradation_reason() const { return degradation_reason_; }

        /// Get current health state
        HealthState current_state() const { return current_state_; }

    private:
        MapperInputContract        contract_;
        HealthState                current_state_ = HealthState::Unavailable;
        std::string                degradation_reason_;
        CapabilitySet              current_capability_;
        bool                       has_evaluated_ = false;
        std::optional<HealthState> pending_state_;
        std::size_t                pending_state_count_ = 0;

        /// Check if requirements are satisfied
        bool check_requirements(
                const std::vector<SensorRequirement> & requirements,
                const CapabilitySet &                  capabilities,
                const std::vector<SensorDescriptor> &  active_descriptors) const;

        HealthState commit_candidate(
                HealthState         candidate,
                const std::string & candidate_reason);
    };

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_HEALTH_MAPPER_HEALTH_GATE_HPP
