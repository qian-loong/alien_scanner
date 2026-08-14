#include "perception_map_update/SpatialChunkLayout.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace PerceptionMapUpdate::Test {

    TEST(SpatialChunkLayout, UsesMathematicalFloorDivisionAcrossZero)
    {
        const auto at_negative_edge = locate_chunk({-8, -16, -24}, 8U);
        ASSERT_TRUE(at_negative_edge) << at_negative_edge.diagnostic;
        EXPECT_EQ(at_negative_edge.address.chunk, (ChunkCoordinate {-1, -2, -3}));
        EXPECT_EQ(at_negative_edge.address.local, (ChunkLocalCoordinate {0U, 0U, 0U}));

        const auto below_zero = locate_chunk({-1, -9, -17}, 8U);
        ASSERT_TRUE(below_zero) << below_zero.diagnostic;
        EXPECT_EQ(below_zero.address.chunk, (ChunkCoordinate {-1, -2, -3}));
        EXPECT_EQ(below_zero.address.local, (ChunkLocalCoordinate {7U, 7U, 7U}));

        const auto above_zero = locate_chunk({7, 8, 15}, 8U);
        ASSERT_TRUE(above_zero) << above_zero.diagnostic;
        EXPECT_EQ(above_zero.address.chunk, (ChunkCoordinate {0, 1, 1}));
        EXPECT_EQ(above_zero.address.local, (ChunkLocalCoordinate {7U, 0U, 7U}));
    }

    TEST(SpatialChunkLayout, SupportsSignedIntegerExtremesWithoutOverflow)
    {
        constexpr auto minimum = std::numeric_limits<std::int64_t>::min();
        constexpr auto maximum = std::numeric_limits<std::int64_t>::max();

        const auto result = locate_chunk({minimum, maximum, -1}, 32U);
        ASSERT_TRUE(result) << result.diagnostic;
        EXPECT_EQ(result.address.chunk.x, minimum / 32);
        EXPECT_EQ(result.address.local.x, 0U);
        EXPECT_EQ(result.address.chunk.y, maximum / 32);
        EXPECT_EQ(result.address.local.y, 31U);
        EXPECT_EQ(result.address.chunk.z, -1);
        EXPECT_EQ(result.address.local.z, 31U);
    }

    TEST(SpatialChunkLayout, RejectsZeroEdgeAndBoundsDeterministicBuckets)
    {
        const auto invalid = locate_chunk({}, 0U);
        EXPECT_FALSE(invalid);
        EXPECT_EQ(invalid.diagnostic, "chunk edge must be positive");

        const ChunkCoordinate coordinate {-17, 23, -41};
        const auto first = chunk_bucket_index(coordinate, 256U);
        const auto second = chunk_bucket_index(coordinate, 256U);
        EXPECT_EQ(first, second);
        EXPECT_LT(first, 256U);
        EXPECT_EQ(chunk_bucket_index(coordinate, 0U), 0U);
    }

}// namespace PerceptionMapUpdate::Test
