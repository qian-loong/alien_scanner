#ifndef PERCEPTION_FIXTURES_INCLUDE_PERCEPTION_FIXTURES_FIXTURE_SCENE_HPP
#define PERCEPTION_FIXTURES_INCLUDE_PERCEPTION_FIXTURES_FIXTURE_SCENE_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

namespace Perception::Fixtures {

    struct CloudPoint {
        float x;
        float y;
        float z;
        float intensity;
    };

    struct FixtureSceneConfig {
        static const std::vector<float> & default_elevation_angles_rad();

        std::size_t        scan_point_count              = 181;
        double             scan_angle_min_rad            = -1.5707963267948966;
        double             scan_angle_max_rad            = 1.5707963267948966;
        double             scan_range_min_m              = 0.1;
        double             scan_range_max_m              = 30.0;
        std::size_t        cloud_azimuth_sample_count    = 360;
        float              cloud_range_m                 = 5.0F;
        std::vector<float> elevation_angles_rad          = default_elevation_angles_rad();
        std::uint32_t      seed                          = 17U;
        bool               inject_debug_returns          = false;

        double scan_angle_increment_rad() const;
    };

    class FixtureScene
    {
    public:
        explicit FixtureScene(FixtureSceneConfig config = {});

        std::vector<float>      scan_ranges() const;
        std::vector<CloudPoint> cloud_points() const;

        const FixtureSceneConfig & config() const { return config_; }

    private:
        FixtureSceneConfig config_;
    };

}// namespace Perception::Fixtures

#endif// PERCEPTION_FIXTURES_INCLUDE_PERCEPTION_FIXTURES_FIXTURE_SCENE_HPP
