#include "perception_fixtures/FixtureScene.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace Perception::Fixtures;

namespace {

    constexpr float kPi = 3.14159265358979323846F;

    float point_range(const CloudPoint & point)
    {
        return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    }

}

TEST(FixtureSceneTest, ScanFixtureIsDeterministic)
{
    FixtureSceneConfig config;
    config.scan_point_count = 181;
    config.seed             = 123U;
    const FixtureScene first(config);
    const FixtureScene second(config);
    const auto         first_ranges  = first.scan_ranges();
    const auto         second_ranges = second.scan_ranges();

    ASSERT_EQ(first_ranges.size(), 181U);
    EXPECT_EQ(first_ranges, second_ranges);
    EXPECT_GT(first_ranges.front(), 0.0F);
    EXPECT_GT(first_ranges.back(), 0.0F);
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
