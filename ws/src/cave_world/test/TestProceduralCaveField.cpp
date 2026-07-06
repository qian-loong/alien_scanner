#include "cave_world/ProceduralCaveField.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace CaveWorld {

    namespace {

        ProceduralCaveFieldConfig makeTestConfig()
        {
            ProceduralCaveFieldConfig config;
            config.length        = 20.0F;
            config.base_radius   = 2.0F;
            config.n_segments    = 40;
            config.branch        = false;
            config.chamber_at    = 0.5F;
            config.chamber_scale = 3.0F;
            config.noise_scale   = 0.0F;
            config.density       = 120;
            config.seed          = 42U;
            return config;
        }

        float radialDistance(const Point3 & axis, const Point3 & point)
        {
            const float dx = point.x - axis.x;
            const float dy = point.y - axis.y;
            const float dz = point.z - axis.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

    }// namespace

    TEST(ProceduralCaveFieldTest, SampleSurfaceIsDeterministic)
    {
        const auto                config = makeTestConfig();
        const ProceduralCaveField field_a(config);
        const ProceduralCaveField field_b(config);
        const auto                surface_a = field_a.sampleSurface();
        const auto                surface_b = field_b.sampleSurface();

        ASSERT_FALSE(surface_a.empty());
        EXPECT_EQ(surface_a.size(), surface_b.size());
        EXPECT_NEAR(surface_a.front().x, surface_b.front().x, 1e-4F);
    }

    TEST(ProceduralCaveFieldTest, SampleSurfaceScalesWithDensity)
    {
        auto low     = makeTestConfig();
        low.density  = 80;
        auto high    = makeTestConfig();
        high.density = 200;

        const ProceduralCaveField field_low(low);
        const ProceduralCaveField field_high(high);
        EXPECT_LT(field_low.sampleSurface().size(), field_high.sampleSurface().size());
    }

    TEST(ProceduralCaveFieldTest, SampleSurfaceWithinBoundingBox)
    {
        const ProceduralCaveField field(makeTestConfig());
        const auto                surface = field.sampleSurface();
        ASSERT_FALSE(surface.empty());

        float min_x      = surface.front().x;
        float max_x      = surface.front().x;
        float max_radius = 0.0F;
        for(const auto & point : surface) {
            min_x      = std::min(min_x, point.x);
            max_x      = std::max(max_x, point.x);
            max_radius = std::max(max_radius, radialDistance(Point3 {point.x, 0.0F, 0.0F}, point));
        }

        EXPECT_GE(min_x, -0.5F);
        EXPECT_LE(max_x, 20.5F);
        EXPECT_GE(max_radius, 2.0F);
        EXPECT_LE(max_radius, 8.0F);
    }

    TEST(ProceduralCaveFieldTest, ChamberExpandsRadiusNearCenter)
    {
        auto config          = makeTestConfig();
        config.chamber_scale = 4.0F;
        config.noise_scale   = 0.0F;
        const ProceduralCaveField field(config);
        const auto                surface = field.sampleSurface();
        ASSERT_FALSE(surface.empty());

        float mid_radius = 0.0F;
        float end_radius = 0.0F;
        int   mid_count  = 0;
        int   end_count  = 0;
        for(const auto & point : surface) {
            const float t      = point.x / config.length;
            const float radius = radialDistance(Point3 {point.x, 0.0F, 0.0F}, point);
            if(std::fabs(t - 0.5F) < 0.08F) {
                mid_radius += radius;
                ++mid_count;
            }
            if(t < 0.08F || t > 0.92F) {
                end_radius += radius;
                ++end_count;
            }
        }

        ASSERT_GT(mid_count, 0);
        ASSERT_GT(end_count, 0);
        EXPECT_GT(mid_radius / static_cast<float>(mid_count), end_radius / static_cast<float>(end_count));
    }

    TEST(ProceduralCaveFieldTest, RaycastHitsWallInSimpleTunnel)
    {
        auto config        = makeTestConfig();
        config.noise_scale = 0.0F;
        const ProceduralCaveField field(config);

        float      hit_dist = 0.0F;
        const bool hit      = field.raycast(Point3 {0.0F, 0.0F, 0.0F}, Point3 {0.0F, 1.0F, 0.0F}, 10.0F, hit_dist);
        ASSERT_TRUE(hit);
        EXPECT_NEAR(hit_dist, 2.0F, 0.12F);
    }

    TEST(ProceduralCaveFieldTest, YJunctionAddsTwoBranchArms)
    {
        auto with_y            = makeTestConfig();
        with_y.branch          = true;
        with_y.branch_length   = 12.0F;
        auto trunk_only        = makeTestConfig();
        trunk_only.branch      = false;

        const ProceduralCaveField field_y(with_y);
        const ProceduralCaveField field_trunk(trunk_only);
        EXPECT_GT(field_y.sampleSurface().size(), field_trunk.sampleSurface().size());
    }

}// namespace CaveWorld
