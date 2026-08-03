#ifndef PERCEPTION_FIXTURES_INCLUDE_PERCEPTION_FIXTURES_RAY_EVIDENCE_DEBUG_GEOMETRY_BUILDER_HPP
#define PERCEPTION_FIXTURES_INCLUDE_PERCEPTION_FIXTURES_RAY_EVIDENCE_DEBUG_GEOMETRY_BUILDER_HPP

#include "perception_core/observation/lidar_observation.hpp"
#include <cstddef>
#include <vector>

namespace Perception::Fixtures {

    struct DebugPoint3 {
        double x;
        double y;
        double z;
    };

    struct RayEvidenceDebugGeometry {
        std::vector<DebugPoint3> hit_endpoints;
        std::vector<DebugPoint3> hit_free_segments;
        std::vector<DebugPoint3> no_return_free_segments;
        std::vector<DebugPoint3> invalid_indicators;
        std::size_t invalid_count {0};
    };

    class RayEvidenceDebugGeometryBuilder
    {
    public:
        explicit RayEvidenceDebugGeometryBuilder(double invalid_indicator_length_m = 0.25);

        RayEvidenceDebugGeometry build(
                const LidarObservation & observation,
                std::size_t              beam_stride = 1) const;

    private:
        double invalid_indicator_length_m_;
    };

}// namespace Perception::Fixtures

#endif// PERCEPTION_FIXTURES_INCLUDE_PERCEPTION_FIXTURES_RAY_EVIDENCE_DEBUG_GEOMETRY_BUILDER_HPP
