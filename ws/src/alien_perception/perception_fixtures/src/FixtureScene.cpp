#include "perception_fixtures/FixtureScene.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace Perception::Fixtures {

    namespace {

        constexpr float kPi = 3.14159265358979323846F;

    }

    const std::vector<float> & FixtureSceneConfig::default_elevation_angles_rad()
    {
        static const std::vector<float> angles = []() {
            std::vector<float> values;
            values.reserve(16);
            for(int degrees = -15; degrees <= 15; degrees += 2) {
                values.push_back(static_cast<float>(degrees) * kPi / 180.0F);
            }
            return values;
        }();
        return angles;
    }

    FixtureScene::FixtureScene(FixtureSceneConfig config)
        : config_(std::move(config))
    {
        config_.scan_point_count           = std::max<std::size_t>(2, config_.scan_point_count);
        config_.cloud_azimuth_sample_count = std::max<std::size_t>(1, config_.cloud_azimuth_sample_count);
        config_.cloud_range_m              = std::max(0.1F, config_.cloud_range_m);

        if(config_.elevation_angles_rad.empty()) {
            throw std::invalid_argument("Fixture elevation_angles_rad must not be empty");
        }
        for(const float elevation : config_.elevation_angles_rad) {
            if(!std::isfinite(elevation) || elevation < -0.5F * kPi || elevation > 0.5F * kPi) {
                throw std::invalid_argument(
                        "Fixture elevation_angles_rad values must be finite and within [-pi/2, pi/2]");
            }
        }
    }

    std::vector<float> FixtureScene::scan_ranges() const
    {
        std::vector<float> ranges(config_.scan_point_count);
        for(std::size_t index = 0; index < ranges.size(); ++index) {
            const float fraction = static_cast<float>(index) / static_cast<float>(ranges.size() - 1);
            const float angle    = -0.5F * kPi + fraction * kPi;
            ranges[index]        = 4.0F + 0.5F * std::cos(angle) + 0.1F * std::sin(3.0F * angle);
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
