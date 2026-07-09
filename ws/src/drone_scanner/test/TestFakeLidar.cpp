#include "drone_scanner/FakeLidar.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace DroneScanner {

    namespace {

        constexpr float kPi = 3.14159265358979323846F;

        float radialDistance(float x, float y, float z)
        {
            return std::sqrt(x * x + y * y + z * z);
        }

        /// 球形空腔：内部为空气（isSolid=false），外部为岩体。
        class SphereCavityField : public CaveWorld::ICaveField
        {
        public:
            explicit SphereCavityField(float radius)
                : radius_(radius)
            {
            }

            bool isSolid(float x, float y, float z) const override
            {
                return radialDistance(x, y, z) > radius_;
            }

            bool raycast(
                    const CaveWorld::Point3 & origin, const CaveWorld::Point3 & dir, float max_range,
                    float & out_dist) const override
            {
                const float dir_len = radialDistance(dir.x, dir.y, dir.z);
                if(dir_len < 1e-6F || max_range <= 0.0F) {
                    return false;
                }

                const float inv_len = 1.0F / dir_len;
                const float dx      = dir.x * inv_len;
                const float dy      = dir.y * inv_len;
                const float dz      = dir.z * inv_len;
                constexpr float kStep = 0.05F;

                for(float t = kStep; t <= max_range; t += kStep) {
                    if(isSolid(origin.x + dx * t, origin.y + dy * t, origin.z + dz * t)) {
                        out_dist = t;
                        return true;
                    }
                }
                return false;
            }

            std::vector<CaveWorld::Point3> sampleSurface() const override
            {
                return {};
            }

        private:
            float radius_;
        };

        /// 沿 +x 的方洞；x>=step_x 处地面骤降，用于验证垂直环能感知 Δz。
        class StepFloorTunnelField : public CaveWorld::ICaveField
        {
        public:
            StepFloorTunnelField(float half_width, float ceiling_z, float step_x, float drop)
                : half_width_(half_width)
                , ceiling_z_(ceiling_z)
                , step_x_(step_x)
                , drop_(drop)
            {
            }

            bool isSolid(float x, float y, float z) const override
            {
                return !inCavity(x, y, z);
            }

            bool raycast(
                    const CaveWorld::Point3 & origin, const CaveWorld::Point3 & dir, float max_range,
                    float & out_dist) const override
            {
                const float dir_len = radialDistance(dir.x, dir.y, dir.z);
                if(dir_len < 1e-6F || max_range <= 0.0F) {
                    return false;
                }

                const float inv_len = 1.0F / dir_len;
                const float dx      = dir.x * inv_len;
                const float dy      = dir.y * inv_len;
                const float dz      = dir.z * inv_len;
                constexpr float kStep = 0.05F;

                for(float t = kStep; t <= max_range; t += kStep) {
                    if(isSolid(origin.x + dx * t, origin.y + dy * t, origin.z + dz * t)) {
                        out_dist = t;
                        return true;
                    }
                }
                return false;
            }

            std::vector<CaveWorld::Point3> sampleSurface() const override
            {
                return {};
            }

        private:
            float floorAt(float x) const
            {
                return (x < step_x_) ? 0.0F : -drop_;
            }

            bool inCavity(float x, float y, float z) const
            {
                if(std::abs(y) > half_width_) {
                    return false;
                }
                const float floor_z = floorAt(x);
                return z > floor_z && z < ceiling_z_;
            }

            float half_width_;
            float ceiling_z_;
            float step_x_;
            float drop_;
        };

        FakeLidarConfig makeTestConfig(int num_beams)
        {
            FakeLidarConfig config;
            config.num_beams       = num_beams;
            config.max_range       = 30.0F;
            config.noise_seed      = 0U;
            config.range_noise_std = 0.0F;
            config.ring_pitch_rad  = 0.0F;
            return config;
        }

        float beamTheta(const LidarPoint & point)
        {
            return std::atan2(point.z, point.y);
        }

        float angleDelta(float a, float b)
        {
            float d = std::abs(a - b);
            while(d > kPi) {
                d = std::abs(d - 2.0F * kPi);
            }
            return d;
        }

        const LidarPoint * findBeamNearTheta(const std::vector<LidarPoint> & hits, float target_theta)
        {
            const auto it = std::min_element(hits.begin(), hits.end(), [&](const LidarPoint & a, const LidarPoint & b) {
                return angleDelta(beamTheta(a), target_theta) < angleDelta(beamTheta(b), target_theta);
            });
            if(it == hits.end()) {
                return nullptr;
            }
            return &(*it);
        }

    }// namespace

    TEST(FakeLidarTest, VerticalRingHitsSphereAtExpectedRange)
    {
        constexpr float              kRadius = 5.0F;
        const auto                   field   = std::make_shared<SphereCavityField>(kRadius);
        const FakeLidar              lidar(field, makeTestConfig(360));

        Pose3D pose;
        pose.x   = 0.0F;
        pose.y   = 0.0F;
        pose.z   = 0.0F;
        pose.yaw = 0.0F;

        const auto hits = lidar.scan(pose);
        ASSERT_GE(hits.size(), 300U);

        for(const LidarPoint & point : hits) {
            EXPECT_NEAR(point.x, 0.0F, 1e-4F);
            EXPECT_NEAR(radialDistance(point.x, point.y, point.z), kRadius, 0.15F);
        }
    }

    TEST(FakeLidarTest, DownwardBeamDetectsFloorDrop)
    {
        const auto field = std::make_shared<StepFloorTunnelField>(
                /*half_width=*/2.0F, /*ceiling_z=*/3.0F, /*step_x=*/5.0F, /*drop=*/2.0F);
        const FakeLidar lidar(field, makeTestConfig(360));

        Pose3D before_drop;
        before_drop.x   = 3.0F;
        before_drop.y   = 0.0F;
        before_drop.z   = 1.5F;
        before_drop.yaw = 0.0F;

        Pose3D after_drop = before_drop;
        after_drop.x      = 7.0F;

        const auto hits_before = lidar.scan(before_drop);
        const auto hits_after  = lidar.scan(after_drop);
        ASSERT_FALSE(hits_before.empty());
        ASSERT_FALSE(hits_after.empty());

        // θ = 3π/2 → lidar (0, 0, -1)，打向地面
        const LidarPoint * down_before = findBeamNearTheta(hits_before, -0.5F * kPi);
        const LidarPoint * down_after  = findBeamNearTheta(hits_after, -0.5F * kPi);
        ASSERT_NE(down_before, nullptr);
        ASSERT_NE(down_after, nullptr);

        EXPECT_NEAR(down_before->z, -1.5F, 0.12F);
        EXPECT_NEAR(down_after->z, -3.5F, 0.12F);
        EXPECT_LT(down_after->z, down_before->z);
    }

    TEST(FakeLidarTest, RangeNoiseIsDeterministicWithSeed)
    {
        constexpr float kRadius = 5.0F;
        const auto      field   = std::make_shared<SphereCavityField>(kRadius);

        FakeLidarConfig noisy_config = makeTestConfig(36);
        noisy_config.range_noise_std = 0.05F;
        noisy_config.noise_seed      = 42U;

        const FakeLidar lidar_a(field, noisy_config);
        const FakeLidar lidar_b(field, noisy_config);

        Pose3D pose;
        const auto scan_a = lidar_a.scan(pose);
        const auto scan_b = lidar_b.scan(pose);

        ASSERT_EQ(scan_a.size(), scan_b.size());
        for(std::size_t i = 0; i < scan_a.size(); ++i) {
            EXPECT_NEAR(scan_a[i].y, scan_b[i].y, 1e-5F);
            EXPECT_NEAR(scan_a[i].z, scan_b[i].z, 1e-5F);
        }
    }

    TEST(FakeLidarTest, NonZeroYawRotatesBeamDirections)
    {
        constexpr float kRadius = 5.0F;
        const auto      field   = std::make_shared<SphereCavityField>(kRadius);
        const FakeLidar lidar(field, makeTestConfig(360));

        Pose3D pose;
        pose.yaw = 0.5F * kPi;

        const auto hits = lidar.scan(pose);
        ASSERT_FALSE(hits.empty());

        const LidarPoint * forward = findBeamNearTheta(hits, 0.0F);
        ASSERT_NE(forward, nullptr);
        EXPECT_NEAR(forward->x, 0.0F, 1e-4F);
        EXPECT_NEAR(radialDistance(forward->x, forward->y, forward->z), kRadius, 0.15F);
    }

    TEST(FakeLidarTest, RejectsNullField)
    {
        EXPECT_THROW(FakeLidar(nullptr, makeTestConfig(8)), std::invalid_argument);
    }

    TEST(FakeLidarTest, ZeroPitchKeepsHitsInYzPlane)
    {
        constexpr float kRadius = 5.0F;
        const auto      field   = std::make_shared<SphereCavityField>(kRadius);
        FakeLidarConfig config  = makeTestConfig(180);
        config.ring_pitch_rad   = 0.0F;
        const FakeLidar lidar(field, config);

        const auto hits = lidar.scan(Pose3D {});
        ASSERT_FALSE(hits.empty());
        for(const LidarPoint & point : hits) {
            EXPECT_NEAR(point.x, 0.0F, 1e-4F);
        }
    }

    TEST(FakeLidarTest, PositivePitchProducesForwardHits)
    {
        constexpr float kRadius = 5.0F;
        const auto      field   = std::make_shared<SphereCavityField>(kRadius);
        FakeLidarConfig config  = makeTestConfig(360);
        config.ring_pitch_rad   = 0.35F; // ~20°
        const FakeLidar lidar(field, config);

        const auto hits = lidar.scan(Pose3D {});
        ASSERT_GE(hits.size(), 300U);

        float max_x = 0.0F;
        for(const LidarPoint & point : hits) {
            EXPECT_NEAR(radialDistance(point.x, point.y, point.z), kRadius, 0.15F);
            max_x = std::max(max_x, point.x);
        }
        // 上半环前倾：应出现明显 +x 命中（斜前方）
        EXPECT_GT(max_x, 1.0F);
    }

    TEST(FakeLidarTest, PitchedRingSeesForwardWallThatZeroPitchMisses)
    {
        /// 仅 x>=wall_x 为岩体；原点在空腔内朝 +x 看墙。
        class ForwardWallField : public CaveWorld::ICaveField
        {
        public:
            explicit ForwardWallField(float wall_x)
                : wall_x_(wall_x)
            {
            }

            bool isSolid(float x, float /*y*/, float /*z*/) const override
            {
                return x >= wall_x_;
            }

            bool raycast(
                    const CaveWorld::Point3 & origin, const CaveWorld::Point3 & dir, float max_range,
                    float & out_dist) const override
            {
                const float dir_len = radialDistance(dir.x, dir.y, dir.z);
                if(dir_len < 1e-6F || max_range <= 0.0F) {
                    return false;
                }
                const float inv_len = 1.0F / dir_len;
                const float dx      = dir.x * inv_len;
                const float dy      = dir.y * inv_len;
                const float dz      = dir.z * inv_len;
                constexpr float kStep = 0.05F;
                for(float t = kStep; t <= max_range; t += kStep) {
                    if(isSolid(origin.x + dx * t, origin.y + dy * t, origin.z + dz * t)) {
                        out_dist = t;
                        return true;
                    }
                }
                return false;
            }

            std::vector<CaveWorld::Point3> sampleSurface() const override
            {
                return {};
            }

        private:
            float wall_x_;
        };

        const auto field = std::make_shared<ForwardWallField>(2.0F);

        FakeLidarConfig zero_pitch = makeTestConfig(360);
        zero_pitch.ring_pitch_rad  = 0.0F;
        const FakeLidar lidar_flat(field, zero_pitch);

        FakeLidarConfig pitched = makeTestConfig(360);
        pitched.ring_pitch_rad  = 0.35F;
        const FakeLidar lidar_pitched(field, pitched);

        Pose3D pose;
        pose.x = 0.0F;
        pose.y = 0.0F;
        pose.z = 0.0F;

        const auto flat_hits    = lidar_flat.scan(pose);
        const auto pitched_hits = lidar_pitched.scan(pose);

        EXPECT_TRUE(flat_hits.empty()) << "纯 YZ 环不应打到正前方墙";
        ASSERT_FALSE(pitched_hits.empty());
        float max_x = 0.0F;
        for(const LidarPoint & point : pitched_hits) {
            max_x = std::max(max_x, point.x);
        }
        EXPECT_GT(max_x, 1.5F);
    }

}// namespace DroneScanner
