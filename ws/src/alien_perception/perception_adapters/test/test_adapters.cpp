#include "perception_adapters/laser_scan_adapter.hpp"
#include "perception_adapters/odometry_adapter.hpp"
#include "perception_adapters/point_cloud2_adapter.hpp"
#include <chrono>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>

using namespace Perception;
using namespace Perception::Adapters;

namespace {

    sensor_msgs::msg::PointCloud2 make_valid_xyz_cloud(
            const std::string & frame_id = "lidar_top_link")
    {
        sensor_msgs::msg::PointCloud2 message;
        message.header.frame_id = frame_id;
        sensor_msgs::PointCloud2Modifier modifier(message);
        modifier.setPointCloud2Fields(
                3,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(1);
        return message;
    }

}// namespace

// Test LaserScanAdapter
TEST(LaserScanAdapterTest, BasicConversion)
{
    LaserScanAdapter adapter;

    // Create test message
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id      = "lidar_front_link";
    msg.header.stamp.sec     = 10;
    msg.header.stamp.nanosec = 500000000;
    msg.angle_min            = -M_PI;
    msg.angle_max            = M_PI;
    msg.angle_increment      = M_PI / 2.0;
    msg.range_min            = 0.1;
    msg.range_max            = 30.0;
    msg.ranges               = {1.0, 2.0, 3.0, 4.0, 5.0};
    msg.intensities          = {100.0, 200.0, 150.0, 180.0, 120.0};

    // Create descriptor
    SensorDescriptor descriptor {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, 0.0, 0.0},
            0.01,
            0.1,
            30.0};

    SessionID session {1000000000ULL, 12345};

    // Convert
    auto obs = adapter.convert(msg, descriptor, session);

    // Verify
    EXPECT_EQ(obs.sensor_id.value, "lidar_front");
    EXPECT_EQ(obs.session_id.boot_time_ns, 1000000000ULL);
    EXPECT_EQ(obs.session_id.random_suffix, 12345u);
    EXPECT_EQ(obs.frame_id, "lidar_front_link");
    EXPECT_EQ(obs.clock_domain, "vehicle_steady_clock");
    EXPECT_EQ(obs.origin_stamp.nanoseconds, 10500000000LL);
    EXPECT_EQ(obs.ray_evidence, RayEvidenceCapability::HitOnly);
    EXPECT_TRUE(obs.is_2d());
    EXPECT_EQ(obs.point_count(), 5);

    const auto & scan = obs.as_scan_2d();
    EXPECT_NEAR(scan.angle_min_rad, -M_PI, 1e-5);
    EXPECT_NEAR(scan.angle_max_rad, M_PI, 1e-5);
    EXPECT_EQ(scan.ranges.size(), 5);
    EXPECT_FLOAT_EQ(scan.ranges[0], 1.0);
}

TEST(LaserScanAdapterTest, Validation)
{
    LaserScanAdapter adapter;

    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = "lidar_front_link";
    msg.angle_min       = static_cast<float>(-M_PI);
    msg.angle_max       = static_cast<float>(M_PI);
    msg.angle_increment = static_cast<float>(M_PI * 2 / 3);// 3 rays for 2pi span
    msg.range_min       = 0.1;
    msg.range_max       = 30.0;
    msg.ranges          = {1.0, 2.0, 3.0};

    SensorDescriptor descriptor {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, 0.0, 0.0},
            0.01,
            0.1,
            30.0};

    auto result = adapter.validate(msg, descriptor);
    EXPECT_TRUE(result.valid);

    // Test frame mismatch
    descriptor.frame_id = "wrong_frame";
    result              = adapter.validate(msg, descriptor);
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.error_message.empty());

    // Test wrong type
    descriptor.frame_id = "lidar_front_link";
    descriptor.type     = SensorType::LIDAR_3D;
    result              = adapter.validate(msg, descriptor);
    EXPECT_FALSE(result.valid);
}

TEST(LaserScanAdapterTest, LocksRayMetadataForFreeSpaceCapabilities)
{
    LaserScanAdapter            adapter;
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = "lidar_front_link";
    msg.angle_min       = -1.0F;
    msg.angle_max       = 1.0F;
    msg.angle_increment = 1.0F;
    msg.range_min       = 0.1F;
    msg.range_max       = 30.0F;
    msg.ranges          = {1.0F, 2.0F, 3.0F};

    SensorDescriptor descriptor {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-1.0, 1.0, 0.0, 0.0},
            1.0,
            0.1,
            30.0,
            RayEvidenceCapability::FullRay};

    EXPECT_TRUE(adapter.validate(msg, descriptor).valid);
    EXPECT_EQ(
            adapter.convert(msg, descriptor, SessionID {1, 2}).ray_evidence,
            RayEvidenceCapability::FullRay);

    auto drifted = msg;
    drifted.range_min += 0.01F;
    EXPECT_FALSE(adapter.validate(drifted, descriptor).valid);
    drifted = msg;
    drifted.range_max += 0.01F;
    EXPECT_FALSE(adapter.validate(drifted, descriptor).valid);
    drifted = msg;
    drifted.angle_min += 0.01F;
    EXPECT_FALSE(adapter.validate(drifted, descriptor).valid);
    drifted = msg;
    drifted.angle_max -= 0.01F;
    EXPECT_FALSE(adapter.validate(drifted, descriptor).valid);
    drifted = msg;
    drifted.angle_increment += 0.01F;
    EXPECT_FALSE(adapter.validate(drifted, descriptor).valid);

    descriptor.ray_evidence = RayEvidenceCapability::HitRay;
    EXPECT_TRUE(adapter.validate(msg, descriptor).valid);

    descriptor.ray_evidence          = RayEvidenceCapability::HitOnly;
    descriptor.range_max_m           = 40.0;
    descriptor.fov.horizontal_min_rad = -2.0;
    descriptor.angular_resolution_rad = 0.5;
    EXPECT_TRUE(adapter.validate(msg, descriptor).valid);
}

TEST(LaserScanAdapterTest, RejectsInvalidBaseMetadataAtEveryCapability)
{
    LaserScanAdapter            adapter;
    sensor_msgs::msg::LaserScan msg;
    msg.header.frame_id = "lidar_front_link";
    msg.angle_min       = -1.0F;
    msg.angle_max       = 1.0F;
    msg.angle_increment = 1.0F;
    msg.range_min       = 0.1F;
    msg.range_max       = 30.0F;
    msg.ranges          = {1.0F, 2.0F, 3.0F};

    SensorDescriptor descriptor {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-1.0, 1.0, 0.0, 0.0},
            1.0,
            0.1,
            30.0};

    for(const auto capability : {
                RayEvidenceCapability::HitOnly,
                RayEvidenceCapability::HitRay,
                RayEvidenceCapability::FullRay}) {
        descriptor.ray_evidence = capability;

        auto invalid = msg;
        invalid.range_min = std::numeric_limits<float>::quiet_NaN();
        EXPECT_FALSE(adapter.validate(invalid, descriptor).valid);
        invalid           = msg;
        invalid.range_max = std::numeric_limits<float>::infinity();
        EXPECT_FALSE(adapter.validate(invalid, descriptor).valid);
        invalid                 = msg;
        invalid.angle_increment = 0.0F;
        EXPECT_FALSE(adapter.validate(invalid, descriptor).valid);
        invalid           = msg;
        invalid.angle_min = -std::numeric_limits<float>::infinity();
        EXPECT_FALSE(adapter.validate(invalid, descriptor).valid);
    }
}

TEST(LaserScanAdapterTest, DirectConvertCannotBypassValidation)
{
    LaserScanAdapter            adapter;
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = "lidar_front_link";
    message.angle_min       = -1.0F;
    message.angle_max       = 1.0F;
    message.angle_increment = 1.0F;
    message.range_min       = 0.1F;
    message.range_max       = 30.0F;
    message.ranges          = {1.0F, 2.0F, 3.0F};
    message.intensities     = {1.0F, 1.0F, 1.0F};

    SensorDescriptor descriptor {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-1.0, 1.0, 0.0, 0.0},
            1.0,
            0.1,
            30.0,
            RayEvidenceCapability::FullRay};
    const SessionID session {1, 2};

    auto invalid_message = message;
    invalid_message.header.frame_id = "wrong_frame";
    EXPECT_THROW(adapter.convert(invalid_message, descriptor, session), std::invalid_argument);

    auto invalid_descriptor = descriptor;
    invalid_descriptor.type = SensorType::LIDAR_3D;
    EXPECT_THROW(adapter.convert(message, invalid_descriptor, session), std::invalid_argument);

    invalid_message = message;
    invalid_message.range_max = std::numeric_limits<float>::infinity();
    EXPECT_THROW(adapter.convert(invalid_message, descriptor, session), std::invalid_argument);

    invalid_message = message;
    invalid_message.ranges.clear();
    EXPECT_THROW(adapter.convert(invalid_message, descriptor, session), std::invalid_argument);

    invalid_message = message;
    invalid_message.intensities.pop_back();
    EXPECT_THROW(adapter.convert(invalid_message, descriptor, session), std::invalid_argument);

    invalid_message = message;
    invalid_message.angle_increment = 0.5F;
    EXPECT_THROW(adapter.convert(invalid_message, descriptor, session), std::invalid_argument);

    invalid_descriptor = descriptor;
    invalid_descriptor.ray_evidence = static_cast<RayEvidenceCapability>(3);
    EXPECT_THROW(adapter.convert(message, invalid_descriptor, session), std::invalid_argument);

    invalid_descriptor.ray_evidence = static_cast<RayEvidenceCapability>(255);
    EXPECT_THROW(adapter.convert(message, invalid_descriptor, session), std::invalid_argument);
}

// Test PointCloud2Adapter
TEST(PointCloud2AdapterTest, BasicConversion)
{
    PointCloud2Adapter adapter;

    // Create test message
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id      = "lidar_top_link";
    msg.header.stamp.sec     = 20;
    msg.header.stamp.nanosec = 0;
    msg.height               = 1;
    msg.width                = 3;
    msg.is_dense             = true;
    msg.is_bigendian         = false;

    // Setup fields
    sensor_msgs::msg::PointField field_x, field_y, field_z, field_intensity;
    field_x.name     = "x";
    field_x.offset   = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count    = 1;

    field_y.name     = "y";
    field_y.offset   = 4;
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count    = 1;

    field_z.name     = "z";
    field_z.offset   = 8;
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count    = 1;

    field_intensity.name     = "intensity";
    field_intensity.offset   = 12;
    field_intensity.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_intensity.count    = 1;

    msg.fields     = {field_z, field_x, field_intensity, field_y};
    msg.point_step = 16;
    msg.row_step   = msg.point_step * msg.width;
    msg.data.resize(msg.row_step * msg.height);

    // Fill in points
    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(msg, "intensity");

    *iter_x         = 1.0f;
    *iter_y         = 2.0f;
    *iter_z         = 3.0f;
    *iter_intensity = 100.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    *iter_x         = 4.0f;
    *iter_y         = 5.0f;
    *iter_z         = 6.0f;
    *iter_intensity = 200.0f;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_intensity;
    *iter_x         = 7.0f;
    *iter_y         = 8.0f;
    *iter_z         = 9.0f;
    *iter_intensity = 150.0f;

    // Create descriptor
    SensorDescriptor descriptor {
            SensorID {"lidar_top"},
            SensorType::LIDAR_3D,
            "lidar_top_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, -M_PI / 4, M_PI / 4},
            0.01,
            0.1,
            50.0};

    SessionID session {2000000000ULL, 67890};

    // Convert
    auto obs = adapter.convert(msg, descriptor, session);

    // Verify
    EXPECT_EQ(obs.sensor_id.value, "lidar_top");
    EXPECT_EQ(obs.frame_id, "lidar_top_link");
    EXPECT_EQ(obs.ray_evidence, RayEvidenceCapability::HitOnly);
    EXPECT_TRUE(obs.is_3d());
    EXPECT_EQ(obs.point_count(), 3);

    const auto & cloud = obs.as_cloud_3d();
    EXPECT_EQ(cloud.points.size(), 3);
    EXPECT_FLOAT_EQ(cloud.points[0].x, 1.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].y, 2.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].z, 3.0f);
    EXPECT_FLOAT_EQ(cloud.points[0].intensity, 100.0f);
}

TEST(PointCloud2AdapterTest, Validation)
{
    PointCloud2Adapter adapter;
    const auto msg = make_valid_xyz_cloud();

    SensorDescriptor descriptor {
            SensorID {"lidar_top"},
            SensorType::LIDAR_3D,
            "lidar_top_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, -M_PI / 4, M_PI / 4},
            0.01,
            0.1,
            50.0};

    auto result = adapter.validate(msg, descriptor);
    EXPECT_TRUE(result.valid);

    // Test wrong type
    descriptor.type = SensorType::LIDAR_2D;
    result          = adapter.validate(msg, descriptor);
    EXPECT_FALSE(result.valid);
}

TEST(PointCloud2AdapterTest, FailsClosedForRayEvidenceAboveHitOnly)
{
    PointCloud2Adapter              adapter;
    const auto msg = make_valid_xyz_cloud();

    SensorDescriptor descriptor {
            SensorID {"lidar_top"},
            SensorType::LIDAR_3D,
            "lidar_top_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, -M_PI / 4, M_PI / 4},
            0.01,
            0.1,
            50.0};

    for(const auto capability : {
                RayEvidenceCapability::HitRay,
                RayEvidenceCapability::FullRay,
                static_cast<RayEvidenceCapability>(3),
                static_cast<RayEvidenceCapability>(255)}) {
        descriptor.ray_evidence = capability;
        const auto validation   = adapter.validate(msg, descriptor);
        EXPECT_FALSE(validation.valid);
        EXPECT_NE(validation.error_message.find("hit_only"), std::string::npos);
        EXPECT_THROW(
                adapter.convert(msg, descriptor, SessionID {1, 2}),
                std::invalid_argument);
    }
}

TEST(PointCloud2AdapterTest, DirectConvertCannotBypassLayoutValidation)
{
    PointCloud2Adapter adapter;
    const auto         message = make_valid_xyz_cloud();
    SensorDescriptor descriptor {
            SensorID {"lidar_top"},
            SensorType::LIDAR_3D,
            "lidar_top_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, -M_PI / 4, M_PI / 4},
            0.01,
            0.1,
            50.0};
    const SessionID session {1, 2};

    auto malformed = message;
    malformed.data.clear();
    EXPECT_FALSE(adapter.validate(malformed, descriptor).valid);
    EXPECT_THROW(adapter.convert(malformed, descriptor, session), std::invalid_argument);

    malformed = message;
    malformed.row_step = 0;
    EXPECT_FALSE(adapter.validate(malformed, descriptor).valid);
    EXPECT_THROW(adapter.convert(malformed, descriptor, session), std::invalid_argument);

    malformed = message;
    malformed.fields.front().datatype = sensor_msgs::msg::PointField::FLOAT64;
    EXPECT_FALSE(adapter.validate(malformed, descriptor).valid);
    EXPECT_THROW(adapter.convert(malformed, descriptor, session), std::invalid_argument);

    malformed = message;
    malformed.fields.front().offset = malformed.point_step;
    EXPECT_FALSE(adapter.validate(malformed, descriptor).valid);
    EXPECT_THROW(adapter.convert(malformed, descriptor, session), std::invalid_argument);

    auto wrong_descriptor = descriptor;
    wrong_descriptor.type = SensorType::LIDAR_2D;
    EXPECT_THROW(adapter.convert(message, wrong_descriptor, session), std::invalid_argument);

    malformed = message;
    malformed.header.frame_id = "wrong_frame";
    EXPECT_THROW(adapter.convert(malformed, descriptor, session), std::invalid_argument);
}

// Test OdometryAdapter
TEST(OdometryAdapterTest, BasicConversion)
{
    OdometryAdapter adapter;

    nav_msgs::msg::Odometry msg;
    msg.header.frame_id         = "odom";
    msg.header.stamp.sec        = 30;
    msg.header.stamp.nanosec    = 0;
    msg.pose.pose.position.x    = 1.0;
    msg.pose.pose.position.y    = 2.0;
    msg.pose.pose.position.z    = 3.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;

    // Fill covariance
    for(size_t i = 0; i < 36; ++i) {
        msg.pose.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
    }

    SourceID  source {"odom"};
    SessionID session {3000000000ULL, 11111};

    auto pose = adapter.convert(msg, source, session);

    EXPECT_EQ(pose.source_id.value, "odom");
    EXPECT_EQ(pose.frame_id, "odom");
    EXPECT_DOUBLE_EQ(pose.position.x(), 1.0);
    EXPECT_DOUBLE_EQ(pose.position.y(), 2.0);
    EXPECT_DOUBLE_EQ(pose.position.z(), 3.0);
    EXPECT_TRUE(pose.covariance.has_value());
    EXPECT_GT(pose.quality, 0.0);
    EXPECT_LE(pose.quality, 1.0);
    EXPECT_EQ(pose.reset_epoch, 0);
}

TEST(OdometryAdapterTest, TfConversion)
{
    OdometryAdapter adapter;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id         = "map";
    transform.header.stamp.sec        = 40;
    transform.header.stamp.nanosec    = 250000000;
    transform.child_frame_id          = "base_link";
    transform.transform.translation.x = 4.0;
    transform.transform.translation.y = 5.0;
    transform.transform.translation.z = 6.0;
    transform.transform.rotation.w    = 1.0;

    SourceID  source {"tf_localization"};
    SessionID session {4000000000ULL, 22222};

    const auto pose = adapter.from_tf(transform, source, session, "ros_time");

    EXPECT_EQ(pose.source_id.value, "tf_localization");
    EXPECT_EQ(pose.session_id, session);
    EXPECT_EQ(pose.frame_id, "map");
    EXPECT_EQ(pose.clock_domain, "ros_time");
    EXPECT_EQ(pose.stamp.nanoseconds, 40'250'000'000LL);
    EXPECT_DOUBLE_EQ(pose.position.x(), 4.0);
    EXPECT_DOUBLE_EQ(pose.position.y(), 5.0);
    EXPECT_DOUBLE_EQ(pose.position.z(), 6.0);
    EXPECT_DOUBLE_EQ(pose.quality, 1.0);
    EXPECT_EQ(pose.reset_epoch, 0);
}

TEST(OdometryAdapterTest, ResetEpochDetection)
{
    OdometryAdapter::Config config;
    config.position_jump_threshold_m   = 5.0;
    config.position_jump_time_window_s = 1.0;

    OdometryAdapter adapter(config);

    // First pose
    nav_msgs::msg::Odometry msg1;
    msg1.header.frame_id         = "odom";
    msg1.header.stamp.sec        = 10;
    msg1.header.stamp.nanosec    = 0;
    msg1.pose.pose.position.x    = 0.0;
    msg1.pose.pose.position.y    = 0.0;
    msg1.pose.pose.position.z    = 0.0;
    msg1.pose.pose.orientation.w = 1.0;

    SourceID  source {"odom"};
    SessionID session {1ULL, 1};

    auto pose1 = adapter.convert(msg1, source, session);
    EXPECT_EQ(pose1.reset_epoch, 0);

    // Second pose with position jump (> 5m in < 1s)
    nav_msgs::msg::Odometry msg2 = msg1;
    msg2.header.stamp.sec        = 10;
    msg2.header.stamp.nanosec    = 500000000;// 0.5s later
    msg2.pose.pose.position.x    = 10.0;     // Jumped 10m

    auto pose2 = adapter.convert(msg2, source, session);
    EXPECT_EQ(pose2.reset_epoch, 1);// Should increment

    // Third pose with frame change
    nav_msgs::msg::Odometry msg3 = msg2;
    msg3.header.frame_id         = "map";
    msg3.header.stamp.sec        = 11;

    auto pose3 = adapter.convert(msg3, source, session);
    EXPECT_EQ(pose3.reset_epoch, 2);// Should increment again

    // Fourth pose with time going backward
    nav_msgs::msg::Odometry msg4 = msg3;
    msg4.header.stamp.sec        = 9;// Earlier than previous

    auto pose4 = adapter.convert(msg4, source, session);
    EXPECT_EQ(pose4.reset_epoch, 3);// Should increment
}

TEST(PerceptionAdapterPerformanceTest, ReportsLaserScanConversionBaseline)
{
    constexpr std::size_t       sample_count = 1000;
    LaserScanAdapter            adapter;
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = "lidar_front_link";
    message.angle_min       = -3.14F;
    message.angle_max       = 3.14F;
    message.angle_increment = 6.28F / 180.0F;
    message.range_min       = 0.1F;
    message.range_max       = 30.0F;
    message.ranges.assign(181, 5.0F);
    message.intensities.assign(181, 1.0F);

    SensorDescriptor descriptor {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-3.14, 3.14, 0.0, 0.0},
            6.28 / 180.0,
            0.1,
            30.0};
    const SessionID session {1ULL, 1};

    std::size_t converted_points = 0;
    const auto  start            = std::chrono::steady_clock::now();
    for(std::size_t index = 0; index < sample_count; ++index) {
        converted_points += adapter.convert(message, descriptor, session).point_count();
    }
    const auto elapsed    = std::chrono::steady_clock::now() - start;
    const auto average_us = std::chrono::duration<double, std::micro>(elapsed).count()
                            / static_cast<double>(sample_count);

    std::cout << "[PERF] laser_scan_conversion samples=" << sample_count
              << " average_us=" << average_us << std::endl;
    EXPECT_EQ(converted_points, sample_count * message.ranges.size());
    EXPECT_LT(average_us, 5000.0);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
