#include "swarm_controller/OctoMapBuilder.hpp"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace SwarmController {

    TEST(OctoMapBuilderTest, InsertsHitAndMissWithDifferentEndpointSemantics)
    {
        OctoMapBuilder builder(0.1F);
        const Point3f  origin {0.0F, 0.0F, 0.0F};
        const std::vector<RayReturn> returns {
                RayReturn {Point3f {3.0F, 0.0F, 0.0F}, 3.0F, true},
                RayReturn {Point3f {0.0F, 3.0F, 0.0F}, 3.0F, false},
        };

        builder.insertScan(origin, returns);

        EXPECT_EQ(builder.query(3.0F, 0.0F, 0.0F), CellState::Occupied);
        EXPECT_EQ(builder.query(1.0F, 0.0F, 0.0F), CellState::Free);
        EXPECT_EQ(builder.query(0.0F, 1.0F, 0.0F), CellState::Free);
        EXPECT_NE(builder.query(0.0F, 3.0F, 0.0F), CellState::Occupied);
        EXPECT_EQ(builder.query(5.0F, 5.0F, 5.0F), CellState::Unknown);
        EXPECT_GT(builder.knownCount(), 0U);
        EXPECT_EQ(builder.occupiedCount(), 1U);
    }

    TEST(OctoMapBuilderTest, RejectsInvalidResolution)
    {
        EXPECT_THROW(OctoMapBuilder(0.0F), std::invalid_argument);
        EXPECT_THROW(OctoMapBuilder(-0.1F), std::invalid_argument);
    }

}// namespace SwarmController
