#include "drone_scanner/LaserScanProjection.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

namespace DroneScanner {

    namespace {

        constexpr float kPi = 3.14159265358979323846F;

        std::vector<LidarReturn> make_returns(std::size_t count, float range, bool hit)
        {
            return std::vector<LidarReturn>(count, LidarReturn {0.0F, 0.0F, 0.0F, range, hit});
        }

    }// namespace

    TEST(LaserScanProjectionTest, FreezesHalfOpenMetadataAndRemap)
    {
        const LaserScanProjection projection({360U, 0.1F, 30.0F, 10.0});
        const auto metadata = projection.metadata();

        EXPECT_FLOAT_EQ(metadata.angle_min_rad, -kPi);
        EXPECT_NEAR(metadata.angle_increment_rad, 2.0F * kPi / 360.0F, 1.0e-7F);
        EXPECT_NEAR(
                metadata.angle_max_rad,
                metadata.angle_min_rad + 359.0F * metadata.angle_increment_rad,
                1.0e-7F);
        EXPECT_LT(metadata.angle_max_rad, kPi);
        EXPECT_FLOAT_EQ(metadata.range_min_m, 0.1F);
        EXPECT_FLOAT_EQ(metadata.range_max_m, 30.0F);
        EXPECT_FLOAT_EQ(metadata.scan_time_s, 0.1F);

        EXPECT_EQ(projection.fakeLidarIndex(0U), 180U);
        EXPECT_EQ(projection.fakeLidarIndex(90U), 270U);
        EXPECT_EQ(projection.fakeLidarIndex(180U), 0U);
        EXPECT_EQ(projection.fakeLidarIndex(270U), 90U);
    }

    TEST(LaserScanProjectionTest, MapsEveryDirectionOntoTheVerticalBodyRing)
    {
        const LaserScanProjection projection({360U, 0.1F, 30.0F, 10.0});
        for(std::size_t index = 0U; index < 360U; ++index) {
            const auto direction = projection.bodyDirection(index);
            EXPECT_FLOAT_EQ(direction.x, 0.0F);
            EXPECT_NEAR(
                    std::sqrt(direction.y * direction.y + direction.z * direction.z),
                    1.0F, 1.0e-6F);
        }

        const auto minus_y = projection.bodyDirection(0U);
        const auto minus_z = projection.bodyDirection(90U);
        const auto plus_y = projection.bodyDirection(180U);
        const auto plus_z = projection.bodyDirection(270U);
        EXPECT_NEAR(minus_y.y, -1.0F, 1.0e-6F);
        EXPECT_NEAR(minus_z.z, -1.0F, 1.0e-6F);
        EXPECT_NEAR(plus_y.y, 1.0F, 1.0e-6F);
        EXPECT_NEAR(plus_z.z, 1.0F, 1.0e-6F);
    }

    TEST(LaserScanProjectionTest, EncodesFiniteHitsAndNoReturnInfinity)
    {
        const LaserScanProjection projection({360U, 0.1F, 30.0F, 10.0});
        auto returns = make_returns(360U, 30.0F, false);
        returns[180U] = LidarReturn {0.0F, 2.5F, 0.0F, 2.5F, true};

        std::string diagnostic;
        const auto scan = projection.project(returns, &diagnostic);

        ASSERT_TRUE(scan.has_value()) << diagnostic;
        ASSERT_EQ(scan->ranges.size(), 360U);
        EXPECT_FLOAT_EQ(scan->ranges[0U], 2.5F);
        EXPECT_TRUE(std::isinf(scan->ranges[1U]));
        EXPECT_GT(scan->ranges[1U], 0.0F);
    }

    TEST(LaserScanProjectionTest, RejectsTheWholeFrameForInvalidHits)
    {
        const LaserScanProjection projection({360U, 0.1F, 30.0F, 10.0});
        for(const float invalid : {
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::infinity(),
                    0.09F,
                    30.01F}) {
            auto returns = make_returns(360U, 1.0F, true);
            returns[17U].range = invalid;
            std::string diagnostic;
            EXPECT_FALSE(projection.project(returns, &diagnostic).has_value());
            EXPECT_FALSE(diagnostic.empty());
        }
    }

    TEST(LaserScanProjectionTest, RejectsInvalidConfigurationAndPayloadSize)
    {
        EXPECT_THROW(LaserScanProjection({359U, 0.1F, 30.0F, 10.0}), std::invalid_argument);
        EXPECT_THROW(LaserScanProjection({360U, 30.0F, 30.0F, 10.0}), std::invalid_argument);
        EXPECT_THROW(LaserScanProjection({360U, 0.1F, 30.0F, 0.0}), std::invalid_argument);

        const LaserScanProjection projection({360U, 0.1F, 30.0F, 10.0});
        EXPECT_FALSE(projection.project(make_returns(359U, 1.0F, true)).has_value());
        EXPECT_THROW(projection.bodyDirection(360U), std::out_of_range);
    }

}// namespace DroneScanner
