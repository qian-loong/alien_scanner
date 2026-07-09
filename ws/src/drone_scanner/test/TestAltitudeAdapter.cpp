#include "drone_scanner/AltitudeAdapter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace DroneScanner {

    namespace {

        LidarPoint makeHit(float x, float y, float z)
        {
            return LidarPoint {x, y, z};
        }

        std::vector<LidarPoint> verticalBandHits(float up, float down)
        {
            return {
                    makeHit(0.0F, 0.0F, up),
                    makeHit(0.0F, 0.0F, down),
                    makeHit(0.0F, 1.0F, 0.0F),
            };
        }

    }// namespace

    TEST(AltitudeAdapterTest, EstimatesFloorAndCeilingFromNearestObstacles)
    {
        AltitudeAdaptConfig config;
        config.band_ema_alpha = 1.0F;
        AltitudeAdapter adapter(config);
        const float     z0   = 1.5F;
        const auto      band = adapter.estimateBand(verticalBandHits(2.0F, -1.5F), z0);

        ASSERT_TRUE(band.valid);
        EXPECT_NEAR(band.floor_z, 0.0F, 1e-4F);
        EXPECT_NEAR(band.ceiling_z, 3.5F, 1e-4F);
    }

    TEST(AltitudeAdapterTest, UsesNearestCeilingNotFarthestHit)
    {
        // 通用启发式：近处上障碍优先于更远上命中（复杂凸起/钟乳石完备策略属未来特性）
        AltitudeAdaptConfig config;
        config.band_ema_alpha = 1.0F;
        AltitudeAdapter adapter(config);

        const std::vector<LidarPoint> hits {
                makeHit(0.0F, 0.0F, 2.0F),  // 近处上障碍
                makeHit(0.0F, 0.0F, 4.0F),  // 更远上命中
                makeHit(0.0F, 0.0F, -1.5F), // 底
        };
        const float z0   = 1.5F;
        const auto  band = adapter.estimateBand(hits, z0);
        ASSERT_TRUE(band.valid);
        EXPECT_NEAR(band.ceiling_z, z0 + 2.0F, 1e-3F);
        EXPECT_LT(band.ceiling_z, z0 + 4.0F);
    }

    TEST(AltitudeAdapterTest, TargetsMidBandByDefault)
    {
        AltitudeAdaptConfig config;
        config.target_fraction    = 0.5F;
        config.min_clearance      = 0.35F;
        config.max_vertical_speed = 100.0F;
        config.band_ema_alpha     = 1.0F;
        AltitudeAdapter adapter(config);

        const float adapted = adapter.adaptFromHits(verticalBandHits(2.0F, -1.5F), 0.2F, 0.2F, 1.0F);
        EXPECT_NEAR(adapted, 0.45F, 1e-3F);
    }

    TEST(AltitudeAdapterTest, RespectsMaxVerticalSpeed)
    {
        AltitudeAdaptConfig config;
        config.target_fraction    = 0.5F;
        config.max_vertical_speed = 0.5F;
        config.min_clearance      = 0.0F;
        config.band_ema_alpha     = 1.0F;
        AltitudeAdapter adapter(config);

        const float adapted = adapter.adaptFromHits(verticalBandHits(2.0F, -1.5F), 0.0F, 0.0F, 0.1F);
        EXPECT_NEAR(adapted, 0.05F, 1e-4F);
    }

    TEST(AltitudeAdapterTest, KeepsClearanceFromFloorAndCeiling)
    {
        AltitudeAdaptConfig config;
        config.target_fraction    = 0.0F;
        config.min_clearance      = 0.4F;
        config.max_vertical_speed = 100.0F;
        config.band_ema_alpha     = 1.0F;
        AltitudeAdapter adapter(config);

        const float adapted = adapter.adaptFromHits(verticalBandHits(2.0F, -1.5F), 1.5F, 1.5F, 1.0F);
        EXPECT_NEAR(adapted, 0.4F, 1e-3F);
    }

    TEST(AltitudeAdapterTest, InvalidWhenNoVerticalHits)
    {
        AltitudeAdapter adapter;
        const auto      hits = std::vector<LidarPoint> {makeHit(1.0F, 0.0F, 0.0F), makeHit(0.0F, 1.0F, 0.0F)};
        EXPECT_FALSE(adapter.estimateBand(hits, 1.5F).valid);
        EXPECT_NEAR(adapter.adaptFromHits(hits, 1.5F, 1.5F, 0.05F), 1.5F, 1e-6F);
    }

    TEST(AltitudeAdapterTest, InvalidWhenOnlyCeilingHits)
    {
        AltitudeAdaptConfig config;
        config.band_ema_alpha = 1.0F;
        AltitudeAdapter adapter(config);
        const auto      hits = std::vector<LidarPoint> {makeHit(0.0F, 0.0F, 2.0F)};
        EXPECT_FALSE(adapter.estimateBand(hits, 1.5F).valid);
    }

    TEST(AltitudeAdapterTest, InvalidWhenBandTooThin)
    {
        AltitudeAdaptConfig config;
        config.min_band_height = 1.0F;
        AltitudeAdapter adapter(config);
        EXPECT_FALSE(adapter.estimateBand(verticalBandHits(0.2F, -0.3F), 1.0F).valid);
    }

    TEST(AltitudeAdapterTest, WorksWithPitchedForwardComponent)
    {
        constexpr float kPitch = 0.35F;
        const float     sin_p  = std::sin(kPitch);
        const float     cos_p  = std::cos(kPitch);
        const float     up_r   = 2.0F;
        const float     down_r = 1.5F;
        const std::vector<LidarPoint> hits {
                makeHit(up_r * sin_p, 0.0F, up_r * cos_p),
                makeHit(-down_r * sin_p, 0.0F, -down_r * cos_p),
        };

        AltitudeAdaptConfig config;
        config.band_ema_alpha   = 1.0F;
        config.ring_pitch_rad   = kPitch;
        config.vertical_dot_min = 0.65F;
        AltitudeAdapter adapter(config);
        ASSERT_TRUE(adapter.geometryCompatible());

        const float z0   = 1.5F;
        const auto  band = adapter.estimateBand(hits, z0);
        ASSERT_TRUE(band.valid);
        EXPECT_NEAR(band.ceiling_z, z0 + up_r * cos_p, 0.05F);
        EXPECT_NEAR(band.floor_z, z0 - down_r * cos_p, 0.05F);
    }

    TEST(AltitudeAdapterTest, GeometryIncompatibleWhenPitchTooLarge)
    {
        AltitudeAdaptConfig config;
        config.ring_pitch_rad   = 0.9F; // cos≈0.62
        config.vertical_dot_min = 0.65F;
        AltitudeAdapter adapter(config);
        EXPECT_FALSE(adapter.geometryCompatible());
        EXPECT_FALSE(adapter.estimateBand(verticalBandHits(2.0F, -1.5F), 1.5F).valid);
    }

    TEST(AltitudeAdapterTest, BandEmaSmoothsSuddenJump)
    {
        AltitudeAdaptConfig config;
        config.band_ema_alpha = 0.25F;
        AltitudeAdapter adapter(config);

        const float z0 = 1.5F;
        auto        b1 = adapter.estimateBand(verticalBandHits(2.0F, -1.5F), z0);
        ASSERT_TRUE(b1.valid);
        EXPECT_NEAR(b1.floor_z, 0.0F, 1e-3F);

        // 最近下障碍从 -1.5 变为 -2.5 → raw floor = -1.0
        auto b2 = adapter.estimateBand(verticalBandHits(2.0F, -2.5F), z0);
        ASSERT_TRUE(b2.valid);
        EXPECT_NEAR(b2.floor_z, -0.25F, 1e-3F);
    }

    TEST(AltitudeAdapterTest, ScanOriginZMustMatchHitsNotLaterPose)
    {
        // 若错误地用后续 z=2.5 解释在 z=1.5 采的 hits，ceiling 会整体上漂
        AltitudeAdaptConfig config;
        config.band_ema_alpha = 1.0F;
        AltitudeAdapter adapter(config);

        const auto hits = verticalBandHits(2.0F, -1.5F);
        const auto ok   = adapter.estimateBand(hits, 1.5F);
        ASSERT_TRUE(ok.valid);
        EXPECT_NEAR(ok.ceiling_z, 3.5F, 1e-3F);

        adapter.reset();
        const auto wrong = adapter.estimateBand(hits, 2.5F);
        ASSERT_TRUE(wrong.valid);
        EXPECT_NEAR(wrong.ceiling_z, 4.5F, 1e-3F);
        EXPECT_NE(ok.ceiling_z, wrong.ceiling_z);
    }

}// namespace DroneScanner
