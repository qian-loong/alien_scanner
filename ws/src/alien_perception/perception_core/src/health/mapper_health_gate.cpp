#include "perception_core/health/mapper_health_gate.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace Perception {

    MapperHealthGate::MapperHealthGate(MapperInputContract contract)
        : contract_(std::move(contract))
    {}

    HealthState MapperHealthGate::evaluate(
            const std::vector<SensorDescriptor> & descriptors,
            const std::vector<SensorHealth> &     sensor_health,
            const std::optional<PoseEstimate> &   pose)
    {
        // Build capability set
        current_capability_ = CapabilitySet {};
        std::vector<SensorDescriptor> active_descriptors;

        for(const auto & desc : descriptors) {
            // Find health status
            auto health_it = std::find_if(
                    sensor_health.begin(), sensor_health.end(),
                    [&](const SensorHealth & h) { return h.sensor_id == desc.sensor_id; });

            if(health_it != sensor_health.end() && health_it->is_healthy()) {
                current_capability_.active_sensor_count++;
                active_descriptors.push_back(desc);
                current_capability_.has_free_space_hit_rays =
                        current_capability_.has_free_space_hit_rays
                        || provides_at_least(desc.ray_evidence, RayEvidenceCapability::HitRay);
                current_capability_.has_full_no_return_rays =
                        current_capability_.has_full_no_return_rays
                        || provides_at_least(desc.ray_evidence, RayEvidenceCapability::FullRay);

                if(desc.type == SensorType::LIDAR_2D) {
                    current_capability_.has_2d_lidar = true;
                    current_capability_.active_2d_lidar_count++;
                }
                else if(desc.type == SensorType::LIDAR_3D) {
                    current_capability_.has_3d_lidar = true;
                    current_capability_.active_3d_lidar_count++;
                }
            }
        }

        // Check the complete pose contract. Freshness, frame and quality stay
        // separate so diagnostics can identify the failed boundary.
        if(contract_.requires_pose) {
            current_capability_.has_pose = pose.has_value();
            if(pose.has_value()) {
                current_capability_.has_fresh_pose =
                        pose->is_fresh(contract_.pose_freshness_threshold);
                current_capability_.has_expected_pose_frame =
                        !contract_.expected_pose_frame.has_value()
                        || pose->frame_id == contract_.expected_pose_frame.value();
                current_capability_.has_sufficient_pose_quality =
                        std::isfinite(pose->quality)
                        && pose->quality >= contract_.minimum_pose_quality;
                current_capability_.has_usable_pose =
                        current_capability_.has_fresh_pose
                        && current_capability_.has_expected_pose_frame
                        && current_capability_.has_sufficient_pose_quality;
            }
        }
        else {
            current_capability_.has_fresh_pose              = true;
            current_capability_.has_expected_pose_frame     = true;
            current_capability_.has_sufficient_pose_quality = true;
            current_capability_.has_usable_pose             = true;
        }

        // Check minimum viable input
        HealthState candidate_state = HealthState::Unavailable;
        std::string candidate_reason;

        if(check_requirements(contract_.minimum_viable, current_capability_, active_descriptors)) {
            candidate_state = HealthState::Healthy;
        }
        else {
            // Check degraded combinations (in priority order)
            for(const auto & degraded : contract_.degraded_combinations) {
                if(check_requirements(degraded.requirements, current_capability_, active_descriptors)) {
                    candidate_state  = HealthState::Degraded;
                    candidate_reason = degraded.description;
                    break;
                }
            }
        }

        if(candidate_state == HealthState::Unavailable) {
            if(contract_.requires_pose && !current_capability_.has_usable_pose) {
                if(!current_capability_.has_pose) {
                    candidate_reason = "Pose missing";
                }
                else if(!current_capability_.has_fresh_pose) {
                    candidate_reason = "Pose stale";
                }
                else if(!current_capability_.has_expected_pose_frame) {
                    std::ostringstream oss;
                    oss << "Pose frame mismatch: expected="
                        << contract_.expected_pose_frame.value_or("<any>")
                        << ", actual=" << pose->frame_id;
                    candidate_reason = oss.str();
                }
                else if(!current_capability_.has_sufficient_pose_quality) {
                    std::ostringstream oss;
                    oss << "Pose quality below threshold: minimum="
                        << contract_.minimum_pose_quality << ", actual=" << pose->quality;
                    candidate_reason = oss.str();
                }
            }
            else {
                std::ostringstream oss;
                oss << "Insufficient input: active=" << current_capability_.active_sensor_count
                    << ", 2d=" << current_capability_.has_2d_lidar
                    << ", 3d=" << current_capability_.has_3d_lidar
                    << ", pose=" << current_capability_.has_usable_pose;
                candidate_reason = oss.str();
            }
        }

        return commit_candidate(candidate_state, candidate_reason);
    }

    bool MapperHealthGate::recovery_stable() const noexcept
    {
        const std::size_t required_samples =
                std::max<std::size_t>(1, contract_.recovery_stability_samples);
        return !pending_state_.has_value() || pending_state_count_ >= required_samples;
    }

    bool MapperHealthGate::check_requirements(
            const std::vector<SensorRequirement> & requirements,
            const CapabilitySet &                  capabilities,
            const std::vector<SensorDescriptor> &  active_descriptors) const
    {
        // Must have a fresh pose that also satisfies frame and quality.
        if(contract_.requires_pose && !capabilities.has_usable_pose) {
            return false;
        }

        for(const auto & req : requirements) {
            size_t available_count = 0;
            for(const auto & descriptor : active_descriptors) {
                if(descriptor.type != req.type) {
                    continue;
                }

                if(!provides_at_least(descriptor.ray_evidence, req.minimum_ray_evidence)) {
                    continue;
                }

                if(req.specific_sensors.has_value()) {
                    const auto & specific_sensors = req.specific_sensors.value();
                    const auto   sensor_it        = std::find(
                            specific_sensors.begin(), specific_sensors.end(), descriptor.sensor_id);
                    if(sensor_it == specific_sensors.end()) {
                        continue;
                    }
                }

                available_count++;
            }

            if(available_count < req.min_count) {
                return false;
            }
        }

        return true;
    }

    HealthState MapperHealthGate::commit_candidate(
            HealthState         candidate,
            const std::string & candidate_reason)
    {
        if(!has_evaluated_) {
            has_evaluated_      = true;
            current_state_      = candidate;
            degradation_reason_ = candidate_reason;
            return current_state_;
        }

        if(candidate == current_state_) {
            pending_state_.reset();
            pending_state_count_ = 0;
            degradation_reason_  = candidate_reason;
            return current_state_;
        }

        const auto state_rank = [](HealthState state) {
            switch(state) {
                case HealthState::Unavailable:
                    return 0;
                case HealthState::Degraded:
                    return 1;
                case HealthState::Healthy:
                    return 2;
            }
            return 0;
        };

        // Fault transitions are immediate so the mapper cannot keep claiming
        // healthy input while a required sensor or pose is stale.
        if(state_rank(candidate) < state_rank(current_state_)) {
            pending_state_.reset();
            pending_state_count_ = 0;
            current_state_       = candidate;
            degradation_reason_  = candidate_reason;
            return current_state_;
        }

        // Recovery must pass through Degraded; this prevents stale map data
        // from being treated as fully recovered after one good sample.
        if(current_state_ == HealthState::Unavailable && candidate == HealthState::Healthy) {
            current_state_       = HealthState::Degraded;
            degradation_reason_  = "Input recovered; awaiting stable healthy input";
            pending_state_       = HealthState::Healthy;
            pending_state_count_ = 1;
            return current_state_;
        }

        if(!pending_state_.has_value() || pending_state_.value() != candidate) {
            pending_state_       = candidate;
            pending_state_count_ = 1;
        }
        else {
            pending_state_count_++;
        }

        const std::size_t required_samples = std::max<std::size_t>(1, contract_.recovery_stability_samples);
        if(pending_state_count_ >= required_samples) {
            current_state_      = candidate;
            degradation_reason_ = candidate_reason;
            pending_state_.reset();
            pending_state_count_ = 0;
        }
        else {
            degradation_reason_ = "Awaiting stable input recovery";
        }

        return current_state_;
    }

}// namespace Perception
