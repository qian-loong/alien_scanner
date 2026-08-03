#include "perception_fixtures/RayEvidenceDebugGeometryBuilder.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace Perception::Fixtures {

    namespace {

        constexpr DebugPoint3 kOrigin {0.0, 0.0, 0.0};

        void append_segment(
                std::vector<DebugPoint3> & segments,
                const DebugPoint3 &        endpoint)
        {
            segments.push_back(kOrigin);
            segments.push_back(endpoint);
        }

        void validate_scan(const Scan2D & scan)
        {
            if(scan.ranges.empty()) {
                throw std::invalid_argument("Debug Scan2D ranges must not be empty");
            }
            if(!scan.intensities.empty() && scan.intensities.size() != scan.ranges.size()) {
                throw std::invalid_argument(
                        "Debug Scan2D intensities must be empty or match ranges length");
            }
            if(!std::isfinite(scan.range_min_m) || !std::isfinite(scan.range_max_m)
               || scan.range_min_m < 0.0 || scan.range_min_m >= scan.range_max_m) {
                throw std::invalid_argument("Debug Scan2D has invalid range metadata");
            }
            if(!std::isfinite(scan.angle_min_rad) || !std::isfinite(scan.angle_max_rad)
               || !std::isfinite(scan.angle_increment_rad)
               || scan.angle_increment_rad <= 0.0
               || scan.angle_max_rad < scan.angle_min_rad) {
                throw std::invalid_argument("Debug Scan2D has invalid angle metadata");
            }

            const double final_angle = scan.angle_min_rad
                    + scan.angle_increment_rad * static_cast<double>(scan.ranges.size() - 1);
            const double tolerance = std::max(1e-6, std::abs(scan.angle_increment_rad) * 1e-4);
            if(std::abs(final_angle - scan.angle_max_rad) > tolerance) {
                throw std::invalid_argument(
                        "Debug Scan2D angle metadata does not match ranges length");
            }
        }

    }// namespace

    RayEvidenceDebugGeometryBuilder::RayEvidenceDebugGeometryBuilder(
            double invalid_indicator_length_m)
        : invalid_indicator_length_m_(invalid_indicator_length_m)
    {
        if(!std::isfinite(invalid_indicator_length_m_) || invalid_indicator_length_m_ <= 0.0) {
            throw std::invalid_argument(
                    "Invalid indicator length must be finite and greater than zero");
        }
    }

    RayEvidenceDebugGeometry RayEvidenceDebugGeometryBuilder::build(
            const LidarObservation & observation,
            std::size_t              beam_stride) const
    {
        if(beam_stride == 0) {
            throw std::invalid_argument("Debug beam_stride must be at least one");
        }
        if(!is_valid_ray_evidence(observation.ray_evidence)) {
            throw std::invalid_argument("Debug observation has unknown ray evidence capability");
        }

        RayEvidenceDebugGeometry geometry;
        if(observation.is_2d()) {
            const auto & scan = observation.as_scan_2d();
            validate_scan(scan);

            for(std::size_t index = 0; index < scan.ranges.size(); index += beam_stride) {
                const double angle = scan.angle_min_rad
                        + static_cast<double>(index) * scan.angle_increment_rad;
                const double direction_x = std::cos(angle);
                const double direction_y = std::sin(angle);

                switch(scan.return_kind(index)) {
                    case RayReturnKind::Hit: {
                        const double range = static_cast<double>(scan.ranges[index]);
                        const DebugPoint3 endpoint {
                                range * direction_x,
                                range * direction_y,
                                0.0};
                        geometry.hit_endpoints.push_back(endpoint);
                        if(provides_at_least(
                                   observation.ray_evidence,
                                   RayEvidenceCapability::HitRay)) {
                            append_segment(geometry.hit_free_segments, endpoint);
                        }
                        break;
                    }
                    case RayReturnKind::NoReturn:
                        if(observation.ray_evidence == RayEvidenceCapability::FullRay) {
                            append_segment(
                                    geometry.no_return_free_segments,
                                    DebugPoint3 {
                                            scan.range_max_m * direction_x,
                                            scan.range_max_m * direction_y,
                                            0.0});
                        }
                        break;
                    case RayReturnKind::Invalid:
                        ++geometry.invalid_count;
                        append_segment(
                                geometry.invalid_indicators,
                                DebugPoint3 {
                                        invalid_indicator_length_m_ * direction_x,
                                        invalid_indicator_length_m_ * direction_y,
                                        0.0});
                        break;
                }
            }
            return geometry;
        }

        if(observation.ray_evidence != RayEvidenceCapability::HitOnly) {
            throw std::invalid_argument(
                    "Debug Cloud3D only supports hit_only ray evidence");
        }

        const auto & cloud = observation.as_cloud_3d();
        if(cloud.points.empty()) {
            throw std::invalid_argument("Debug Cloud3D points must not be empty");
        }
        geometry.hit_endpoints.reserve((cloud.points.size() + beam_stride - 1) / beam_stride);
        for(std::size_t index = 0; index < cloud.points.size(); index += beam_stride) {
            const auto & point = cloud.points[index];
            if(!std::isfinite(point.x) || !std::isfinite(point.y)
               || !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
                throw std::invalid_argument("Debug Cloud3D contains a non-finite point");
            }
            geometry.hit_endpoints.push_back(
                    DebugPoint3 {point.x, point.y, point.z});
        }
        return geometry;
    }

}// namespace Perception::Fixtures
