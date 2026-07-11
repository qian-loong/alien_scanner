#include "swarm_controller/KnownFreePathChecker.hpp"
#include "swarm_controller/OctoMapBuilder.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace SwarmController {

    TEST(YawSweepCoverageTest, ContinuousDefaultRingProvesLocalSweptVolume)
    {
        constexpr float pi          = 3.14159265358979323846F;
        constexpr float pitch       = 0.35F;
        constexpr float yaw_step    = 0.05F;  // 10 Hz scan / 0.5 rad/s yaw
        constexpr int   beam_count  = 360;
        constexpr float ray_length  = 3.0F;
        const Point3f   origin {};
        OctoMapBuilder builder(0.1F);

        for(float yaw = 0.0F; yaw < 2.0F * pi; yaw += yaw_step) {
            std::vector<RayReturn> returns;
            returns.reserve(beam_count);
            for(int beam = 0; beam < beam_count; ++beam) {
                const float theta =
                        2.0F * pi * static_cast<float>(beam) / static_cast<float>(beam_count);
                const float local_x = std::sin(theta) * std::sin(pitch);
                const float local_y = std::cos(theta);
                const float local_z = std::sin(theta) * std::cos(pitch);
                const float map_x   = std::cos(yaw) * local_x - std::sin(yaw) * local_y;
                const float map_y   = std::sin(yaw) * local_x + std::cos(yaw) * local_y;
                returns.push_back(RayReturn {
                        Point3f {
                                ray_length * map_x,
                                ray_length * map_y,
                                ray_length * local_z,
                        },
                        ray_length,
                        false,
                });
            }
            builder.insertScan(origin, returns);
        }

        KnownFreePathChecker checker;
        const PathCheckResult path = checker.checkSegment(
                builder.tree(), Point3f {0.0F, 0.0F, 0.0F},
                Point3f {0.8F, 0.0F, 0.0F});
        EXPECT_EQ(path.status, PathCheckStatus::Safe);
        EXPECT_EQ(builder.query(3.5F, 0.0F, 0.0F), CellState::Unknown);
    }

}// namespace SwarmController
