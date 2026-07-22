#ifndef CAVE_WORLD_TRUTHCOLLISIONAUDITOR_HPP
#define CAVE_WORLD_TRUTHCOLLISIONAUDITOR_HPP

#include "cave_world/ICaveField.hpp"

#include <cstddef>
#include <optional>

namespace CaveWorld {

    struct TruthCollisionAuditConfig {
        float robot_radius {0.25F};
        float robot_half_height {0.15F};
        float sample_spacing {0.05F};
        std::size_t clearance_direction_count {32U};
        float clearance_max_range {10.0F};
    };

    struct TruthCollisionAuditResult {
        bool collided {false};
        std::size_t solid_sample_count {0U};
        std::optional<Point3> first_solid_sample;
        float minimum_clearance {0.0F};
    };

    class TruthCollisionAuditor
    {
    public:
        TruthCollisionAuditor(
                const ICaveField & field, TruthCollisionAuditConfig config = {});

        TruthCollisionAuditResult assess(const Point3 & center) const;
        const TruthCollisionAuditConfig & config() const;

    private:
        const ICaveField & field_;
        TruthCollisionAuditConfig config_;
    };

}// namespace CaveWorld

#endif// CAVE_WORLD_TRUTHCOLLISIONAUDITOR_HPP
