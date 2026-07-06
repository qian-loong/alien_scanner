#include "cave_world/TreeCaveField.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace CaveWorld {

    namespace {

        TreeCaveFieldConfig makeNetworkTestConfig()
        {
            TreeCaveFieldConfig config;
            config.approach_length       = 8.0F;
            config.base_radius           = 2.0F;
            config.n_segments            = 32;
            config.density               = 120;
            config.noise_scale           = 0.0F;
            config.seed                  = 11U;
            config.loop_direct_length    = 8.0F;
            config.loop_bulge            = 5.0F;
            config.exit1_length          = 6.0F;
            config.right_corridor_length = 6.0F;
            config.exit_arm_length       = 6.0F;
            config.asymmetry             = 0.15F;
            return config;
        }

    }// namespace

    TEST(TreeCaveFieldTest, SampleSurfaceIsDeterministic)
    {
        const auto          config = makeNetworkTestConfig();
        const TreeCaveField field_a(config);
        const TreeCaveField field_b(config);
        const auto          surface_a = field_a.sampleSurface();
        const auto          surface_b = field_b.sampleSurface();

        ASSERT_FALSE(surface_a.empty());
        EXPECT_EQ(surface_a.size(), surface_b.size());
    }

    TEST(TreeCaveFieldTest, LongerArmsAddMoreSurface)
    {
        auto large   = makeNetworkTestConfig();
        auto compact = makeNetworkTestConfig();
        compact.loop_direct_length = 4.0F;
        compact.exit1_length       = 3.0F;
        compact.exit_arm_length    = 3.0F;

        const TreeCaveField field_large(large);
        const TreeCaveField field_compact(compact);
        EXPECT_GT(field_large.sampleSurface().size(), field_compact.sampleSurface().size());
    }

    TEST(TreeCaveFieldTest, OriginIsInsideCave)
    {
        const TreeCaveField field(makeNetworkTestConfig());
        EXPECT_FALSE(field.isSolid(0.0F, 0.0F, 0.0F));
    }

    TEST(TreeCaveFieldTest, FirstForkAppearsEarlyAlongPath)
    {
        const auto          config = makeNetworkTestConfig();
        const TreeCaveField field(config);
        const auto          surface = field.sampleSurface();

        float min_x_with_lateral = 1.0e9F;
        for(const Point3 & point : surface) {
            if(std::fabs(point.y) > 1.5F && point.x > 1.0F) {
                min_x_with_lateral = std::min(min_x_with_lateral, point.x);
            }
        }

        ASSERT_LT(min_x_with_lateral, config.approach_length * 1.25F);
    }

    TEST(TreeCaveFieldTest, LoopBulgeAddsMoreSurface)
    {
        auto with_loop    = makeNetworkTestConfig();
        auto without_loop = makeNetworkTestConfig();
        with_loop.loop_bulge    = 6.0F;
        without_loop.loop_bulge = 0.5F;

        const TreeCaveField field_loop(with_loop);
        const TreeCaveField field_plain(without_loop);
        EXPECT_GT(field_loop.sampleSurface().size(), field_plain.sampleSurface().size());
    }

}// namespace CaveWorld
