#include "perception_fixtures/FixtureScene.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace Perception::Fixtures {

    namespace {

        constexpr double kPi                   = 3.14159265358979323846;
        constexpr std::size_t kDebugBeamCount  = 360;
        constexpr std::size_t kBranchFirstBeam = 255;
        constexpr std::size_t kBranchLastBeam  = 285;
        constexpr std::size_t kNanBeam         = 44;
        constexpr std::size_t kNegativeInfBeam = 136;
        constexpr std::size_t kBelowMinBeam    = 180;
        constexpr std::size_t kAboveMaxBeam    = 316;
        constexpr double kDebugAngleMin        = -kPi;
        constexpr double kDebugAngleIncrement  = 2.0 * kPi / static_cast<double>(kDebugBeamCount);
        constexpr double kDebugAngleMax        = kPi - kDebugAngleIncrement;
        constexpr double kDebugRangeMin        = 0.1;
        constexpr double kDebugRangeMax        = 10.0;
        constexpr double kDebugEllipseAxisX    = 3.0;
        constexpr double kDebugEllipseAxisY    = 4.0;

        bool nearly_equal(double lhs, double rhs)
        {
            return std::abs(lhs - rhs) <= 1e-9;
        }

    }

    const std::vector<float> & FixtureSceneConfig::default_elevation_angles_rad()
    {
        static const std::vector<float> angles = []() {
            std::vector<float> values;
            values.reserve(16);
            for(int degrees = -15; degrees <= 15; degrees += 2) {
                values.push_back(static_cast<float>(static_cast<double>(degrees) * kPi / 180.0));
            }
            return values;
        }();
        return angles;
    }

    double FixtureSceneConfig::scan_angle_increment_rad() const
    {
        if(scan_point_count < 2) {
            throw std::invalid_argument("Fixture scan_point_count must be at least two");
        }
        return (scan_angle_max_rad - scan_angle_min_rad)
                / static_cast<double>(scan_point_count - 1);
    }

    FixtureScene::FixtureScene(FixtureSceneConfig config)
        : config_(std::move(config))
    {
        if(config_.scan_point_count < 2) {
            throw std::invalid_argument("Fixture scan_point_count must be at least two");
        }
        if(!std::isfinite(config_.scan_angle_min_rad)
           || !std::isfinite(config_.scan_angle_max_rad)
           || config_.scan_angle_min_rad >= config_.scan_angle_max_rad) {
            throw std::invalid_argument(
                    "Fixture scan angles must be finite and satisfy min < max");
        }
        if(!std::isfinite(config_.scan_range_min_m)
           || !std::isfinite(config_.scan_range_max_m)
           || config_.scan_range_min_m < 0.0
           || config_.scan_range_min_m >= config_.scan_range_max_m) {
            throw std::invalid_argument(
                    "Fixture scan ranges must be finite and satisfy 0 <= min < max");
        }

        config_.cloud_azimuth_sample_count = std::max<std::size_t>(1, config_.cloud_azimuth_sample_count);
        config_.cloud_range_m              = std::max(0.1F, config_.cloud_range_m);

        if(config_.inject_debug_returns
           && (config_.scan_point_count != kDebugBeamCount
               || !nearly_equal(config_.scan_angle_min_rad, kDebugAngleMin)
               || !nearly_equal(config_.scan_angle_max_rad, kDebugAngleMax)
               || !nearly_equal(config_.scan_angle_increment_rad(), kDebugAngleIncrement)
               || !nearly_equal(config_.scan_range_min_m, kDebugRangeMin)
               || !nearly_equal(config_.scan_range_max_m, kDebugRangeMax))) {
            throw std::invalid_argument(
                    "Debug return injection requires the fixed 360-beam [-pi, pi) layout and [0.1, 10] m range");
        }

        if(config_.elevation_angles_rad.empty()) {
            throw std::invalid_argument("Fixture elevation_angles_rad must not be empty");
        }
        for(const float elevation : config_.elevation_angles_rad) {
            if(!std::isfinite(elevation)
               || static_cast<double>(elevation) < -0.5 * kPi
               || static_cast<double>(elevation) > 0.5 * kPi) {
                throw std::invalid_argument(
                        "Fixture elevation_angles_rad values must be finite and within [-pi/2, pi/2]");
            }
        }
    }

    std::vector<float> FixtureScene::scan_ranges() const
    {
        std::vector<float> ranges(config_.scan_point_count);
        const double angle_increment = config_.scan_angle_increment_rad();
        for(std::size_t index = 0; index < ranges.size(); ++index) {
            const double angle = config_.scan_angle_min_rad
                    + static_cast<double>(index) * angle_increment;
            if(config_.inject_debug_returns) {
                const double cos_angle = std::cos(angle);
                const double sin_angle = std::sin(angle);
                const double inverse_range_squared =
                        cos_angle * cos_angle / (kDebugEllipseAxisX * kDebugEllipseAxisX)
                        + sin_angle * sin_angle / (kDebugEllipseAxisY * kDebugEllipseAxisY);
                ranges[index] = static_cast<float>(1.0 / std::sqrt(inverse_range_squared));
            }
            else {
                // Preserve the original fixture's float arithmetic while allowing
                // callers to configure the scan bounds and sample count.
                const float fraction = static_cast<float>(index)
                        / static_cast<float>(ranges.size() - 1);
                const float legacy_angle = static_cast<float>(config_.scan_angle_min_rad)
                        + fraction * static_cast<float>(
                                config_.scan_angle_max_rad - config_.scan_angle_min_rad);
                ranges[index] = 4.0F
                        + 0.5F * std::cos(legacy_angle)
                        + 0.1F * std::sin(3.0F * legacy_angle);
            }
        }
        if(config_.inject_debug_returns) {
            std::fill(
                    ranges.begin() + static_cast<std::ptrdiff_t>(kBranchFirstBeam),
                    ranges.begin() + static_cast<std::ptrdiff_t>(kBranchLastBeam + 1),
                    std::numeric_limits<float>::infinity());

            const double range_span = config_.scan_range_max_m - config_.scan_range_min_m;
            const double invalid_offset = std::max(0.01, 0.01 * range_span);
            ranges[kNanBeam]             = std::numeric_limits<float>::quiet_NaN();
            ranges[kNegativeInfBeam]     = -std::numeric_limits<float>::infinity();
            ranges[kBelowMinBeam]        = static_cast<float>(config_.scan_range_min_m - invalid_offset);
            ranges[kAboveMaxBeam]        = static_cast<float>(config_.scan_range_max_m + invalid_offset);
        }
        return ranges;
    }

    std::vector<CloudPoint> FixtureScene::cloud_points() const
    {
        std::vector<CloudPoint> points;
        points.reserve(config_.elevation_angles_rad.size() * config_.cloud_azimuth_sample_count);

        std::size_t linear_index = 0;
        for(const float elevation : config_.elevation_angles_rad) {
            const float cos_elevation = std::cos(elevation);
            const float sin_elevation = std::sin(elevation);
            for(std::size_t sample_index = 0;
                sample_index < config_.cloud_azimuth_sample_count;
                ++sample_index, ++linear_index) {
                const float azimuth = 2.0F * kPi * static_cast<float>(sample_index)
                        / static_cast<float>(config_.cloud_azimuth_sample_count);
                const float range = config_.cloud_range_m
                        * (1.0F + 0.05F * std::cos(2.0F * azimuth));
                const float horizontal_range = range * cos_elevation;

                points.push_back(CloudPoint {
                        horizontal_range * std::cos(azimuth),
                        horizontal_range * std::sin(azimuth),
                        range * sin_elevation,
                        static_cast<float>((static_cast<std::uint64_t>(config_.seed) + linear_index) % 256U)});
            }
        }
        return points;
    }

}// namespace Perception::Fixtures
