#include "drone_scanner/PointCloudAccumulator.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace DroneScanner {

    namespace {
        constexpr float kPi = 3.14159265358979323846F;
    }

    TEST(PointCloudAccumulatorTest, StartsEmpty)
    {
        PointCloudAccumulator accumulator;
        EXPECT_EQ(accumulator.size(), 0U);
        EXPECT_TRUE(accumulator.points().empty());
    }

    TEST(PointCloudAccumulatorTest, AppendPointsInMapFrame)
    {
        PointCloudAccumulator accumulator;
        accumulator.appendPointsInMapFrame({Point3f {1.0F, 0.0F, 0.0F}, Point3f {2.0F, 0.0F, 0.0F}});
        ASSERT_EQ(accumulator.size(), 2U);
        EXPECT_FLOAT_EQ(accumulator.points()[1].x, 2.0F);
    }

    TEST(PointCloudAccumulatorTest, AppendTransformedAppliesTranslation)
    {
        PointCloudAccumulator accumulator;
        RigidTransform transform;
        transform.tx = 5.0F;

        accumulator.appendTransformed({Point3f {1.0F, 2.0F, 3.0F}}, transform);
        ASSERT_EQ(accumulator.size(), 1U);
        EXPECT_FLOAT_EQ(accumulator.points()[0].x, 6.0F);
        EXPECT_FLOAT_EQ(accumulator.points()[0].y, 2.0F);
        EXPECT_FLOAT_EQ(accumulator.points()[0].z, 3.0F);
    }

    TEST(PointCloudAccumulatorTest, MaxPointsDropsOldest)
    {
        PointCloudAccumulator accumulator(3);
        accumulator.appendPointsInMapFrame({Point3f {1.0F, 0.0F, 0.0F}});
        accumulator.appendPointsInMapFrame({Point3f {2.0F, 0.0F, 0.0F}});
        accumulator.appendPointsInMapFrame({Point3f {3.0F, 0.0F, 0.0F}});
        accumulator.appendPointsInMapFrame({Point3f {4.0F, 0.0F, 0.0F}});

        ASSERT_EQ(accumulator.size(), 3U);
        EXPECT_FLOAT_EQ(accumulator.points()[0].x, 2.0F);
        EXPECT_FLOAT_EQ(accumulator.points()[2].x, 4.0F);
    }

    TEST(PointCloudAccumulatorTest, SingleBatchLargerThanCapKeepsNewestOnly)
    {
        PointCloudAccumulator accumulator(3);
        accumulator.appendPointsInMapFrame({
                Point3f {1.0F, 0.0F, 0.0F},
                Point3f {2.0F, 0.0F, 0.0F},
                Point3f {3.0F, 0.0F, 0.0F},
                Point3f {4.0F, 0.0F, 0.0F},
                Point3f {5.0F, 0.0F, 0.0F},
        });

        ASSERT_EQ(accumulator.size(), 3U);
        EXPECT_FLOAT_EQ(accumulator.points()[0].x, 3.0F);
        EXPECT_FLOAT_EQ(accumulator.points()[2].x, 5.0F);
    }

    TEST(PointCloudAccumulatorTest, ZeroCapMeansUnlimited)
    {
        PointCloudAccumulator accumulator(0);
        for(float x = 1.0F; x <= 10.0F; x += 1.0F) {
            accumulator.appendPointsInMapFrame({Point3f {x, 0.0F, 0.0F}});
        }
        EXPECT_EQ(accumulator.size(), 10U);
    }

    TEST(PointCloudAccumulatorTest, ClearThenAppend)
    {
        PointCloudAccumulator accumulator(5);
        accumulator.appendPointsInMapFrame({Point3f {1.0F, 0.0F, 0.0F}});
        accumulator.clear();
        EXPECT_EQ(accumulator.size(), 0U);
        accumulator.appendPointsInMapFrame({Point3f {9.0F, 0.0F, 0.0F}});
        ASSERT_EQ(accumulator.size(), 1U);
        EXPECT_FLOAT_EQ(accumulator.points()[0].x, 9.0F);
    }

    TEST(RigidTransformTest, IdentityLeavesPointUnchanged)
    {
        const Point3f in {3.0F, -1.0F, 2.0F};
        const Point3f out = transformPoint(in, RigidTransform {});
        EXPECT_FLOAT_EQ(out.x, in.x);
        EXPECT_FLOAT_EQ(out.y, in.y);
        EXPECT_FLOAT_EQ(out.z, in.z);
    }

    TEST(RigidTransformTest, Yaw90RotatesPointAroundZ)
    {
        RigidTransform transform;
        transform.qz = std::sin(0.5F * kPi * 0.5F);
        transform.qw = std::cos(0.5F * kPi * 0.5F);

        const Point3f out = transformPoint(Point3f {1.0F, 0.0F, 0.0F}, transform);
        EXPECT_NEAR(out.x, 0.0F, 1e-4F);
        EXPECT_NEAR(out.y, 1.0F, 1e-4F);
        EXPECT_NEAR(out.z, 0.0F, 1e-4F);
    }

    TEST(RigidTransformTest, RotationAndTranslationCombined)
    {
        RigidTransform transform;
        transform.tx = 10.0F;
        transform.ty = -2.0F;
        transform.qz = std::sin(0.5F * kPi * 0.5F);
        transform.qw = std::cos(0.5F * kPi * 0.5F);

        const Point3f out = transformPoint(Point3f {1.0F, 0.0F, 0.0F}, transform);
        EXPECT_NEAR(out.x, 10.0F, 1e-4F);
        EXPECT_NEAR(out.y, -1.0F, 1e-4F);
        EXPECT_NEAR(out.z, 0.0F, 1e-4F);
    }

}// namespace DroneScanner
