#include "drone_scanner/PoseSegmentTrajectory.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace DroneScanner {

    TEST(PoseSegmentTrajectoryTest, InterpolatesPlanarPoseWithinRateLimits)
    {
        PoseSegmentConfig config;
        config.start        = Pose3D {0.0F, 0.0F, 1.5F, 0.0F};
        config.goal         = Pose3D {3.0F, 4.0F, 8.0F, 1.0F};
        config.linear_speed = 2.0F;
        config.yaw_rate     = 0.5F;
        PoseSegmentTrajectory trajectory(config);

        EXPECT_NEAR(trajectory.duration(), 2.5, 1.0e-6);
        const Pose3D middle = trajectory.pose(1.25);
        EXPECT_NEAR(middle.x, 1.5F, 1.0e-6F);
        EXPECT_NEAR(middle.y, 2.0F, 1.0e-6F);
        EXPECT_NEAR(middle.z, 1.5F, 1.0e-6F);
        EXPECT_NEAR(middle.yaw, 0.5F, 1.0e-6F);

        const PoseSegmentVelocity velocity = trajectory.velocity(1.0);
        EXPECT_LE(std::hypot(velocity.x, velocity.y), config.linear_speed);
        EXPECT_LE(std::fabs(velocity.yaw_rate), config.yaw_rate);
    }

    TEST(PoseSegmentTrajectoryTest, PureYawHasZeroLinearVelocity)
    {
        PoseSegmentConfig config;
        config.start    = Pose3D {1.0F, 2.0F, 1.5F, 0.0F};
        config.goal     = Pose3D {1.0F, 2.0F, 3.0F, 1.0F};
        config.yaw_rate = 0.5F;
        PoseSegmentTrajectory trajectory(config);

        EXPECT_NEAR(trajectory.duration(), 2.0, 1.0e-6);
        const auto velocity = trajectory.velocity(1.0);
        EXPECT_FLOAT_EQ(velocity.x, 0.0F);
        EXPECT_FLOAT_EQ(velocity.y, 0.0F);
        EXPECT_FLOAT_EQ(trajectory.pose(1.0).z, config.start.z);
        EXPECT_FALSE(trajectory.isTranslationActive(1.0));
    }

    TEST(PoseSegmentTrajectoryTest, UsesShortestYawAcrossPiBoundary)
    {
        PoseSegmentConfig config;
        config.start    = Pose3D {0.0F, 0.0F, 0.0F, 3.0F};
        config.goal     = Pose3D {0.0F, 0.0F, 0.0F, -3.0F};
        config.yaw_rate = 1.0F;
        PoseSegmentTrajectory trajectory(config);

        EXPECT_LT(trajectory.duration(), 0.3);
        EXPECT_GT(trajectory.velocity(0.1).yaw_rate, 0.0F);
    }

    TEST(PoseSegmentTrajectoryTest, ZeroSegmentIsImmediateHold)
    {
        PoseSegmentConfig config;
        config.start = Pose3D {1.0F, 2.0F, 3.0F, 0.5F};
        config.goal  = config.start;
        PoseSegmentTrajectory trajectory(config);

        EXPECT_DOUBLE_EQ(trajectory.duration(), 0.0);
        const Pose3D pose = trajectory.pose(100.0);
        EXPECT_FLOAT_EQ(pose.x, config.start.x);
        EXPECT_FLOAT_EQ(pose.y, config.start.y);
        EXPECT_FLOAT_EQ(pose.z, config.start.z);
        EXPECT_FLOAT_EQ(pose.yaw, config.start.yaw);
        EXPECT_FLOAT_EQ(trajectory.velocity(0.0).x, 0.0F);
    }

    TEST(PoseSegmentTrajectoryTest, RejectsInvalidConfiguration)
    {
        PoseSegmentConfig config;
        config.linear_speed = 0.0F;
        EXPECT_THROW(PoseSegmentTrajectory {config}, std::invalid_argument);

        config.linear_speed = 0.4F;
        config.goal.x       = std::numeric_limits<float>::quiet_NaN();
        EXPECT_THROW(PoseSegmentTrajectory {config}, std::invalid_argument);
    }

}// namespace DroneScanner
