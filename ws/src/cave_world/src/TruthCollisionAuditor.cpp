#include "cave_world/TruthCollisionAuditor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace CaveWorld {

    TruthCollisionAuditor::TruthCollisionAuditor(
            const ICaveField & field, TruthCollisionAuditConfig config)
        : field_(field), config_(config)
    {
        if(!std::isfinite(config_.robot_radius)
           || !std::isfinite(config_.robot_half_height)
           || !std::isfinite(config_.sample_spacing)
           || !std::isfinite(config_.clearance_max_range)
           || config_.robot_radius < 0.0F || config_.robot_half_height < 0.0F
           || config_.sample_spacing <= 0.0F
           || config_.clearance_direction_count < 8U
           || config_.clearance_max_range <= 0.0F)
        {
            throw std::invalid_argument("invalid truth collision audit config");
        }
    }

    TruthCollisionAuditResult TruthCollisionAuditor::assess(const Point3 & center) const
    {
        TruthCollisionAuditResult result;
        result.minimum_clearance = std::numeric_limits<float>::infinity();
        const int radial_steps = static_cast<int>(
                std::ceil(config_.robot_radius / config_.sample_spacing));
        const int vertical_steps = static_cast<int>(
                std::ceil(config_.robot_half_height / config_.sample_spacing));
        for(int ix = -radial_steps; ix <= radial_steps; ++ix) {
            for(int iy = -radial_steps; iy <= radial_steps; ++iy) {
                const float x = static_cast<float>(ix) * config_.sample_spacing;
                const float y = static_cast<float>(iy) * config_.sample_spacing;
                if(x * x + y * y
                   > config_.robot_radius * config_.robot_radius + 1.0e-6F)
                {
                    continue;
                }
                for(int iz = -vertical_steps; iz <= vertical_steps; ++iz) {
                    const float z = std::clamp(
                            static_cast<float>(iz) * config_.sample_spacing,
                            -config_.robot_half_height, config_.robot_half_height);
                    const Point3 sample {center.x + x, center.y + y, center.z + z};
                    if(field_.isSolid(sample.x, sample.y, sample.z)) {
                        ++result.solid_sample_count;
                        if(!result.first_solid_sample.has_value()) {
                            result.first_solid_sample = sample;
                        }
                    }
                }
            }
        }
        result.collided = result.solid_sample_count > 0U;

        constexpr float PI = 3.14159265358979323846F;
        for(std::size_t index = 0U; index < config_.clearance_direction_count; ++index) {
            const float azimuth = 2.0F * PI * static_cast<float>(index)
                                  / static_cast<float>(config_.clearance_direction_count);
            for(const float dz : {-0.70710678F, 0.0F, 0.70710678F}) {
                const float horizontal = std::sqrt(std::max(0.0F, 1.0F - dz * dz));
                const Point3 direction {
                        horizontal * std::cos(azimuth),
                        horizontal * std::sin(azimuth), dz};
                float hit_distance = config_.clearance_max_range;
                if(field_.raycast(center, direction, config_.clearance_max_range, hit_distance)) {
                    const float envelope_support =
                            config_.robot_radius * horizontal
                            + config_.robot_half_height * std::fabs(dz);
                    result.minimum_clearance = std::min(
                            result.minimum_clearance, hit_distance - envelope_support);
                }
            }
        }
        if(!std::isfinite(result.minimum_clearance)) {
            result.minimum_clearance = config_.clearance_max_range;
        }
        return result;
    }

    const TruthCollisionAuditConfig & TruthCollisionAuditor::config() const
    {
        return config_;
    }

}// namespace CaveWorld
