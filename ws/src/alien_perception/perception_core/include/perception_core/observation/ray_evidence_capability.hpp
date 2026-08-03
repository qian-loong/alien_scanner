#ifndef PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_RAY_EVIDENCE_CAPABILITY_HPP
#define PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_RAY_EVIDENCE_CAPABILITY_HPP

#include <cstdint>
#include <string_view>

namespace Perception {

    enum class RayEvidenceCapability : std::uint8_t
    {
        HitOnly = 0,
        HitRay  = 1,
        FullRay = 2
    };

    constexpr bool is_valid_ray_evidence(RayEvidenceCapability capability) noexcept
    {
        switch(capability) {
            case RayEvidenceCapability::HitOnly:
            case RayEvidenceCapability::HitRay:
            case RayEvidenceCapability::FullRay:
                return true;
        }
        return false;
    }

    constexpr bool provides_at_least(
            RayEvidenceCapability actual,
            RayEvidenceCapability required) noexcept
    {
        if(!is_valid_ray_evidence(actual) || !is_valid_ray_evidence(required)) {
            return false;
        }
        return static_cast<std::uint8_t>(actual) >= static_cast<std::uint8_t>(required);
    }

    constexpr std::string_view ray_evidence_name(RayEvidenceCapability capability) noexcept
    {
        switch(capability) {
            case RayEvidenceCapability::HitOnly:
                return "hit_only";
            case RayEvidenceCapability::HitRay:
                return "hit_ray";
            case RayEvidenceCapability::FullRay:
                return "full_ray";
        }
        return "unknown";
    }

}// namespace Perception

#endif// PERCEPTION_CORE_INCLUDE_PERCEPTION_CORE_OBSERVATION_RAY_EVIDENCE_CAPABILITY_HPP
