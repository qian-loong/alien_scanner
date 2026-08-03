#include "perception_fixtures/FixtureScene.hpp"
#include "perception_fixtures/RayEvidenceDebugGeometryBuilder.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace Perception;
using namespace Perception::Fixtures;

namespace {

    constexpr double kPi = 3.14159265358979323846;

    FixtureSceneConfig make_debug_scene_config()
    {
        FixtureSceneConfig config;
        config.scan_point_count       = 360;
        config.scan_angle_min_rad     = -kPi;
        config.scan_angle_max_rad     = kPi - 2.0 * kPi / 360.0;
        config.scan_range_min_m       = 0.1;
        config.scan_range_max_m       = 10.0;
        config.inject_debug_returns   = true;
        return config;
    }

    bool is_debug_invalid_index(std::size_t index)
    {
        constexpr std::array<std::size_t, 4> invalid_indices {44, 136, 180, 316};
        return std::find(invalid_indices.begin(), invalid_indices.end(), index)
                != invalid_indices.end();
    }

    float point_range(const CloudPoint & point)
    {
        return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }

}

TEST(FixtureSceneTest, DefaultScanFixtureRetainsGoldenProfile)
{
    FixtureSceneConfig config;
    config.seed = 123U;
    const FixtureScene first(config);
    const FixtureScene second(config);
    const auto         first_ranges  = first.scan_ranges();
    const auto         second_ranges = second.scan_ranges();

    ASSERT_EQ(first_ranges.size(), 181U);
    EXPECT_EQ(first_ranges, second_ranges);
    EXPECT_NEAR(config.scan_angle_min_rad, -0.5 * kPi, 1e-15);
    EXPECT_NEAR(config.scan_angle_max_rad, 0.5 * kPi, 1e-15);
    EXPECT_NEAR(config.scan_angle_increment_rad(), kPi / 180.0, 1e-15);
    EXPECT_DOUBLE_EQ(config.scan_range_min_m, 0.1);
    EXPECT_DOUBLE_EQ(config.scan_range_max_m, 30.0);

    const std::array<std::pair<std::size_t, float>, 6> golden_ranges {{
            {0, 4.1F},
            {5, 4.14017009735107421875F},
            {45, 4.2828427124746193F},
            {90, 4.5F},
            {135, 4.4242640687119286F},
            {180, 3.9F},
    }};
    for(const auto & [index, expected_range] : golden_ranges) {
        EXPECT_EQ(first_ranges[index], expected_range) << "beam " << index;
    }
}

TEST(FixtureSceneTest, InvalidScanProfilesAreRejected)
{
    FixtureSceneConfig config;
    config.scan_point_count = 1;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = FixtureSceneConfig {};
    config.scan_angle_min_rad = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = FixtureSceneConfig {};
    config.scan_angle_max_rad = std::numeric_limits<double>::infinity();
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = FixtureSceneConfig {};
    config.scan_angle_max_rad = config.scan_angle_min_rad;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = FixtureSceneConfig {};
    config.scan_range_min_m = -0.01;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = FixtureSceneConfig {};
    config.scan_range_max_m = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = FixtureSceneConfig {};
    config.scan_range_max_m = config.scan_range_min_m;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);
}

TEST(FixtureSceneTest, DefaultCloudUsesVendorNeutralSixteenChannelProfile)
{
    const FixtureScene scene;
    const auto &       config = scene.config();
    const auto         points = scene.cloud_points();

    ASSERT_EQ(config.elevation_angles_rad.size(), 16U);
    for(std::size_t channel_index = 0; channel_index < 16U; ++channel_index) {
        const float expected_degrees = -15.0F + 2.0F * static_cast<float>(channel_index);
        EXPECT_NEAR(
                config.elevation_angles_rad[channel_index],
                expected_degrees * kPi / 180.0F,
                1e-7F);
    }
    EXPECT_EQ(config.cloud_azimuth_sample_count, 360U);
    EXPECT_EQ(points.size(), 16U * 360U);
}

TEST(FixtureSceneTest, CloudIsDeterministicAndFollowsRangeAndIntensityFormula)
{
    FixtureSceneConfig config;
    config.cloud_azimuth_sample_count = 4;
    config.cloud_range_m              = 5.0F;
    config.elevation_angles_rad       = {-0.2F, 0.0F, 0.2F};
    config.seed                       = 253U;

    const auto first_points  = FixtureScene(config).cloud_points();
    const auto second_points = FixtureScene(config).cloud_points();

    ASSERT_EQ(first_points.size(), 12U);
    ASSERT_EQ(second_points.size(), first_points.size());
    for(std::size_t index = 0; index < first_points.size(); ++index) {
        EXPECT_FLOAT_EQ(first_points[index].x, second_points[index].x);
        EXPECT_FLOAT_EQ(first_points[index].y, second_points[index].y);
        EXPECT_FLOAT_EQ(first_points[index].z, second_points[index].z);
        EXPECT_FLOAT_EQ(first_points[index].intensity, second_points[index].intensity);

        const std::size_t sample_index = index % config.cloud_azimuth_sample_count;
        const float azimuth = 2.0F * kPi * static_cast<float>(sample_index)
                / static_cast<float>(config.cloud_azimuth_sample_count);
        const float expected_range = config.cloud_range_m
                * (1.0F + 0.05F * std::cos(2.0F * azimuth));
        EXPECT_NEAR(point_range(first_points[index]), expected_range, 1e-5F);
        EXPECT_FLOAT_EQ(
                first_points[index].intensity,
                static_cast<float>((static_cast<std::uint64_t>(config.seed) + index) % 256U));
    }
}

TEST(FixtureSceneTest, ZeroElevationStaysInSensorXyPlaneAndAzimuthOrderIsStandard)
{
    FixtureSceneConfig config;
    config.cloud_azimuth_sample_count = 4;
    config.elevation_angles_rad       = {0.0F};
    const auto points                 = FixtureScene(config).cloud_points();

    ASSERT_EQ(points.size(), 4U);
    EXPECT_GT(points[0].x, 0.0F);
    EXPECT_NEAR(points[0].y, 0.0F, 1e-5F);
    EXPECT_GT(points[1].y, 0.0F);
    EXPECT_NEAR(points[1].x, 0.0F, 1e-5F);
    EXPECT_LT(points[2].x, 0.0F);
    EXPECT_NEAR(points[2].y, 0.0F, 1e-5F);
    EXPECT_LT(points[3].y, 0.0F);
    EXPECT_NEAR(points[3].x, 0.0F, 1e-5F);
    for(const auto & point : points) {
        EXPECT_NEAR(point.z, 0.0F, 1e-6F);
    }
}

TEST(FixtureSceneTest, ElevationChannelsProduceExpectedSignedAnglesInCallerOrder)
{
    FixtureSceneConfig config;
    config.cloud_azimuth_sample_count = 4;
    config.elevation_angles_rad       = {-0.2F, 0.0F, 0.2F};
    const auto points                 = FixtureScene(config).cloud_points();

    ASSERT_EQ(points.size(), 12U);
    for(std::size_t channel_index = 0; channel_index < config.elevation_angles_rad.size(); ++channel_index) {
        for(std::size_t sample_index = 0; sample_index < config.cloud_azimuth_sample_count; ++sample_index) {
            const auto & point = points[channel_index * config.cloud_azimuth_sample_count + sample_index];
            const float measured_elevation = std::atan2(
                    point.z,
                    std::sqrt(point.x * point.x + point.y * point.y));
            EXPECT_NEAR(measured_elevation, config.elevation_angles_rad[channel_index], 1e-6F);
        }
    }
    EXPECT_LT(points.front().z, 0.0F);
    EXPECT_NEAR(points[4].z, 0.0F, 1e-6F);
    EXPECT_GT(points[8].z, 0.0F);
}

TEST(FixtureSceneTest, InvalidElevationProfilesAreRejected)
{
    FixtureSceneConfig config;
    config.elevation_angles_rad.clear();
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config.elevation_angles_rad = {std::numeric_limits<float>::quiet_NaN()};
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config.elevation_angles_rad = {std::numeric_limits<float>::infinity()};
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config.elevation_angles_rad = {0.5F * kPi + 0.01F};
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config.elevation_angles_rad = {-0.5F * kPi - 0.01F};
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);
}

TEST(FixtureSceneTest, DebugTunnelHitsFollowEllipseAcrossAllQuadrants)
{
    const auto config = make_debug_scene_config();
    const auto ranges = FixtureScene(config).scan_ranges();
    std::array<bool, 4> quadrants {false, false, false, false};
    std::size_t hit_count = 0;

    ASSERT_EQ(ranges.size(), 360U);
    for(std::size_t index = 0; index < ranges.size(); ++index) {
        if((index >= 255 && index <= 285) || is_debug_invalid_index(index)) {
            continue;
        }

        const double angle = config.scan_angle_min_rad
                + static_cast<double>(index) * config.scan_angle_increment_rad();
        const double x = static_cast<double>(ranges[index]) * std::cos(angle);
        const double y = static_cast<double>(ranges[index]) * std::sin(angle);
        EXPECT_TRUE(std::isfinite(ranges[index])) << "beam " << index;
        EXPECT_GE(ranges[index], config.scan_range_min_m) << "beam " << index;
        EXPECT_LE(ranges[index], config.scan_range_max_m) << "beam " << index;
        EXPECT_NEAR(x * x / 9.0 + y * y / 16.0, 1.0, 2e-6)
                << "beam " << index;
        if(x > 0.0 && y > 0.0) quadrants[0] = true;
        if(x < 0.0 && y > 0.0) quadrants[1] = true;
        if(x < 0.0 && y < 0.0) quadrants[2] = true;
        if(x > 0.0 && y < 0.0) quadrants[3] = true;
        ++hit_count;
    }

    EXPECT_EQ(hit_count, 325U);
    EXPECT_TRUE(std::all_of(quadrants.begin(), quadrants.end(), [](bool value) {
        return value;
    }));
}

TEST(FixtureSceneTest, DebugTunnelHasExactBranchAndInvalidReturns)
{
    const auto config = make_debug_scene_config();
    const auto ranges = FixtureScene(config).scan_ranges();
    std::vector<std::size_t> positive_infinity_indices;

    for(std::size_t index = 0; index < ranges.size(); ++index) {
        if(std::isinf(ranges[index]) && ranges[index] > 0.0F) {
            positive_infinity_indices.push_back(index);
        }
    }
    ASSERT_EQ(positive_infinity_indices.size(), 31U);
    for(std::size_t offset = 0; offset < positive_infinity_indices.size(); ++offset) {
        EXPECT_EQ(positive_infinity_indices[offset], 255U + offset);
    }

    EXPECT_TRUE(std::isnan(ranges[44]));
    EXPECT_TRUE(std::isinf(ranges[136]));
    EXPECT_LT(ranges[136], 0.0F);
    EXPECT_TRUE(std::isfinite(ranges[180]));
    EXPECT_LT(ranges[180], config.scan_range_min_m);
    EXPECT_TRUE(std::isfinite(ranges[316]));
    EXPECT_GT(ranges[316], config.scan_range_max_m);

    Scan2D scan;
    scan.angle_min_rad       = config.scan_angle_min_rad;
    scan.angle_max_rad       = config.scan_angle_max_rad;
    scan.angle_increment_rad = config.scan_angle_increment_rad();
    scan.range_min_m         = config.scan_range_min_m;
    scan.range_max_m         = config.scan_range_max_m;
    scan.ranges              = ranges;
    EXPECT_EQ(scan.return_kind(255), RayReturnKind::NoReturn);
    EXPECT_EQ(scan.return_kind(285), RayReturnKind::NoReturn);
    EXPECT_EQ(scan.return_kind(44), RayReturnKind::Invalid);
    EXPECT_EQ(scan.return_kind(136), RayReturnKind::Invalid);
    EXPECT_EQ(scan.return_kind(180), RayReturnKind::Invalid);
    EXPECT_EQ(scan.return_kind(316), RayReturnKind::Invalid);
}

TEST(FixtureSceneTest, DebugTunnelDirectionsDoNotRepeatAndLayoutsAreDisjoint)
{
    const auto config = make_debug_scene_config();
    EXPECT_NEAR(config.scan_angle_increment_rad(), 2.0 * kPi / 360.0, 1e-15);
    EXPECT_NEAR(config.scan_angle_max_rad, kPi - config.scan_angle_increment_rad(), 1e-15);

    const double first_x = std::cos(config.scan_angle_min_rad);
    const double first_y = std::sin(config.scan_angle_min_rad);
    const double last_x  = std::cos(config.scan_angle_max_rad);
    const double last_y  = std::sin(config.scan_angle_max_rad);
    EXPECT_GT(std::hypot(first_x - last_x, first_y - last_y), 0.01);

    for(const std::size_t invalid_index : {44U, 136U, 180U, 316U}) {
        EXPECT_FALSE(invalid_index >= 255U && invalid_index <= 285U);
    }
}

TEST(FixtureSceneTest, DebugReturnInjectionRejectsAnyNonFixedLayout)
{
    auto config = make_debug_scene_config();
    config.scan_point_count = 359;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = make_debug_scene_config();
    config.scan_angle_min_rad += 0.01;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = make_debug_scene_config();
    config.scan_angle_max_rad += 0.01;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = make_debug_scene_config();
    config.scan_range_min_m = 0.2;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);

    config = make_debug_scene_config();
    config.scan_range_max_m = 11.0;
    EXPECT_THROW((FixtureScene(config)), std::invalid_argument);
}

namespace {

    LidarObservation make_debug_scan(RayEvidenceCapability capability)
    {
        Scan2D scan;
        scan.angle_min_rad       = 0.0;
        scan.angle_max_rad       = 5.0;
        scan.angle_increment_rad = 1.0;
        scan.range_min_m         = 0.1;
        scan.range_max_m         = 10.0;
        scan.ranges              = {
                2.0F,
                std::numeric_limits<float>::infinity(),
                std::numeric_limits<float>::quiet_NaN(),
                -std::numeric_limits<float>::infinity(),
                0.05F,
                11.0F};

        LidarObservation observation;
        observation.sensor_id    = SensorID {"debug_scan"};
        observation.ray_evidence = capability;
        observation.data         = std::move(scan);
        return observation;
    }

}// namespace

TEST(RayEvidenceDebugGeometryBuilderTest, EnforcesAllThreeCapabilities)
{
    const RayEvidenceDebugGeometryBuilder builder;

    const auto hit_only = builder.build(
            make_debug_scan(RayEvidenceCapability::HitOnly));
    EXPECT_EQ(hit_only.hit_endpoints.size(), 1U);
    EXPECT_TRUE(hit_only.hit_free_segments.empty());
    EXPECT_TRUE(hit_only.no_return_free_segments.empty());
    EXPECT_EQ(hit_only.invalid_indicators.size(), 8U);
    EXPECT_EQ(hit_only.invalid_count, 4U);

    const auto hit_ray = builder.build(
            make_debug_scan(RayEvidenceCapability::HitRay));
    EXPECT_EQ(hit_ray.hit_endpoints.size(), 1U);
    EXPECT_EQ(hit_ray.hit_free_segments.size(), 2U);
    EXPECT_TRUE(hit_ray.no_return_free_segments.empty());

    const auto full_ray = builder.build(
            make_debug_scan(RayEvidenceCapability::FullRay));
    EXPECT_EQ(full_ray.hit_endpoints.size(), 1U);
    EXPECT_EQ(full_ray.hit_free_segments.size(), 2U);
    EXPECT_EQ(full_ray.no_return_free_segments.size(), 2U);
    EXPECT_NEAR(full_ray.no_return_free_segments[1].x, std::cos(1.0) * 10.0, 1e-9);
    EXPECT_NEAR(full_ray.no_return_free_segments[1].y, std::sin(1.0) * 10.0, 1e-9);
}

TEST(RayEvidenceDebugGeometryBuilderTest, BeamStridePreservesSelectedClassification)
{
    const RayEvidenceDebugGeometryBuilder builder;
    const auto geometry = builder.build(
            make_debug_scan(RayEvidenceCapability::FullRay), 2);

    EXPECT_EQ(geometry.hit_endpoints.size(), 1U);
    EXPECT_EQ(geometry.hit_free_segments.size(), 2U);
    EXPECT_TRUE(geometry.no_return_free_segments.empty());
    EXPECT_EQ(geometry.invalid_count, 2U);
    EXPECT_EQ(geometry.invalid_indicators.size(), 4U);
    EXPECT_THROW(
            builder.build(make_debug_scan(RayEvidenceCapability::FullRay), 0),
            std::invalid_argument);
}

TEST(RayEvidenceDebugGeometryBuilderTest, Cloud3DIsHitOnly)
{
    LidarObservation observation;
    observation.sensor_id    = SensorID {"debug_cloud"};
    observation.ray_evidence = RayEvidenceCapability::HitOnly;
    observation.data         = Cloud3D {{
            Cloud3D::Point {1.0F, 2.0F, 3.0F, 4.0F},
            Cloud3D::Point {5.0F, 6.0F, 7.0F, 8.0F}}};

    const RayEvidenceDebugGeometryBuilder builder;
    const auto geometry = builder.build(observation);
    EXPECT_EQ(geometry.hit_endpoints.size(), 2U);
    EXPECT_TRUE(geometry.hit_free_segments.empty());
    EXPECT_TRUE(geometry.no_return_free_segments.empty());
    EXPECT_TRUE(geometry.invalid_indicators.empty());

    observation.ray_evidence = RayEvidenceCapability::HitRay;
    EXPECT_THROW(builder.build(observation), std::invalid_argument);

    observation.ray_evidence = static_cast<RayEvidenceCapability>(255);
    EXPECT_THROW(builder.build(observation), std::invalid_argument);
}
