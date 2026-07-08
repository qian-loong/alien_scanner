#include "drone_scanner/LineTrajectory.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace DroneScanner {

    namespace {

        LineTrajectoryConfig makeDefaultConfig()
        {
            LineTrajectoryConfig config;
            config.start_x          = 0.0F;
            config.start_y          = 0.0F;
            config.start_z          = 1.5F;
            config.end_x            = 11.0F;
            config.end_y            = 0.0F;
            config.end_z            = 1.5F;
            config.duration_seconds = 60.0;
            return config;
        }

    }// namespace

    TEST(LineTrajectoryTest, PoseAtZeroIsStart)
    {
        const LineTrajectory trajectory(makeDefaultConfig());
        const Pose3D         start = trajectory.pose(0.0);

        EXPECT_NEAR(start.x, 0.0F, 1e-5F);
        EXPECT_NEAR(start.y, 0.0F, 1e-5F);
        EXPECT_NEAR(start.z, 1.5F, 1e-5F);
    }

    TEST(LineTrajectoryTest, PoseAtDurationIsEnd)
    {
        const LineTrajectory trajectory(makeDefaultConfig());
        const Pose3D         end = trajectory.pose(trajectory.duration());

        EXPECT_NEAR(end.x, 11.0F, 1e-5F);
        EXPECT_NEAR(end.y, 0.0F, 1e-5F);
        EXPECT_NEAR(end.z, 1.5F, 1e-5F);
    }

    TEST(LineTrajectoryTest, PoseAtHalfTimeIsMidpoint)
    {
        const LineTrajectory trajectory(makeDefaultConfig());
        const Pose3D         mid = trajectory.pose(trajectory.duration() * 0.5);

        EXPECT_NEAR(mid.x, 5.5F, 1e-5F);
        EXPECT_NEAR(mid.y, 0.0F, 1e-5F);
        EXPECT_NEAR(mid.z, 1.5F, 1e-5F);
    }

    TEST(LineTrajectoryTest, YawPointsAlongPlusXIntoCave)
    {
        const LineTrajectory trajectory(makeDefaultConfig());
        const Pose3D         pose = trajectory.pose(1.0);

        EXPECT_NEAR(pose.yaw, 0.0F, 1e-5F);
    }

    TEST(LineTrajectoryTest, SpeedIsPathLengthOverDuration)
    {
        const LineTrajectory trajectory(makeDefaultConfig());

        EXPECT_NEAR(trajectory.speed(), 11.0F / 60.0F, 1e-5F);
    }

    TEST(LineTrajectoryTest, TimeBeyondDurationClampsToEnd)
    {
        const LineTrajectory trajectory(makeDefaultConfig());
        const Pose3D         beyond = trajectory.pose(trajectory.duration() + 10.0);
        const Pose3D         end    = trajectory.pose(trajectory.duration());

        EXPECT_NEAR(beyond.x, end.x, 1e-5F);
        EXPECT_NEAR(beyond.y, end.y, 1e-5F);
        EXPECT_NEAR(beyond.z, end.z, 1e-5F);
    }

    TEST(LineTrajectoryTest, NegativeTimeClampsToStart)
    {
        const LineTrajectory trajectory(makeDefaultConfig());
        const Pose3D         negative = trajectory.pose(-5.0);
        const Pose3D         start    = trajectory.pose(0.0);

        EXPECT_NEAR(negative.x, start.x, 1e-5F);
        EXPECT_NEAR(negative.y, start.y, 1e-5F);
        EXPECT_NEAR(negative.z, start.z, 1e-5F);
    }

    TEST(LineTrajectoryTest, YawFollowsPlusYDirection)
    {
        LineTrajectoryConfig config = makeDefaultConfig();
        config.end_x              = 0.0F;
        config.end_y              = 10.0F;

        const LineTrajectory trajectory(config);
        EXPECT_NEAR(trajectory.pose(0.0).yaw, static_cast<float>(M_PI_2), 1e-5F);
    }

    TEST(LineTrajectoryTest, InvalidDurationIsNormalized)
    {
        LineTrajectoryConfig config = makeDefaultConfig();
        config.duration_seconds   = -5.0;

        const LineTrajectory trajectory(config);
        EXPECT_NEAR(trajectory.duration(), 1.0, 1e-5F);
        EXPECT_NEAR(trajectory.speed(), 11.0F, 1e-5F);
    }

    TEST(LineTrajectoryTest, ZeroLengthPathHasZeroSpeed)
    {
        LineTrajectoryConfig config = makeDefaultConfig();
        config.end_x                = config.start_x;
        config.end_y                = config.start_y;
        config.end_z                = config.start_z;

        const LineTrajectory trajectory(config);
        EXPECT_NEAR(trajectory.speed(), 0.0F, 1e-5F);
        EXPECT_NEAR(trajectory.pose(5.0).x, config.start_x, 1e-5F);
    }

}// namespace DroneScanner
