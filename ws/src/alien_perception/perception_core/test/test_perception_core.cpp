#include "perception_core/health/health_state.hpp"
#include "perception_core/health/mapper_health_gate.hpp"
#include "perception_core/health/mapper_input_contract.hpp"
#include "perception_core/observation/lidar_observation.hpp"
#include "perception_core/observation/pose_estimate.hpp"
#include "perception_core/observation/sensor_descriptor.hpp"
#include "perception_core/types/identity.hpp"
#include "perception_core/types/timestamp.hpp"
#include <chrono>
#include <gtest/gtest.h>
#include <iostream>
#include <limits>

using namespace Perception;

namespace {

    SensorDescriptor make_descriptor(const char * sensor_id, SensorType type)
    {
        return SensorDescriptor {
                SensorID {sensor_id},
                type,
                std::string(sensor_id) + "_link",
                Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(),
                FieldOfView {-M_PI, M_PI, 0.0, 0.0},
                0.01,
                0.1,
                30.0};
    }

    SensorHealth make_health(const char * sensor_id, SensorHealthStatus status)
    {
        return SensorHealth {SensorID {sensor_id}, status, Timestamp::from_seconds(1.0)};
    }

    PoseEstimate make_pose(double freshness_seconds)
    {
        return PoseEstimate {
                SourceID {"odom"},
                SessionID {1ULL, 1},
                "odom",
                "vehicle_steady_clock",
                Timestamp::from_seconds(1.0),
                Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(),
                std::nullopt,
                1.0,
                Duration::from_seconds(freshness_seconds),
                0};
    }

}// namespace

// Test SensorID and SessionID
TEST(IdentityTest, SensorIDEquality)
{
    SensorID id1 {"lidar_front"};
    SensorID id2 {"lidar_front"};
    SensorID id3 {"lidar_rear"};

    EXPECT_EQ(id1, id2);
    EXPECT_NE(id1, id3);
}

TEST(IdentityTest, SessionIDEquality)
{
    SessionID session1 {1000000000ULL, 12345};
    SessionID session2 {1000000000ULL, 12345};
    SessionID session3 {1000000000ULL, 67890};

    EXPECT_EQ(session1, session2);
    EXPECT_NE(session1, session3);
}

// Test Timestamp
TEST(TimestampTest, Conversion)
{
    auto ts = Timestamp::from_seconds(1.5);
    EXPECT_EQ(ts.nanoseconds, 1'500'000'000);
    EXPECT_DOUBLE_EQ(ts.to_seconds(), 1.5);
}

TEST(TimestampTest, Comparison)
{
    auto t1 = Timestamp::from_seconds(1.0);
    auto t2 = Timestamp::from_seconds(2.0);

    EXPECT_LT(t1, t2);
    EXPECT_GT(t2, t1);
    EXPECT_EQ(t1, t1);
}

TEST(TimestampTest, Arithmetic)
{
    auto t1   = Timestamp::from_seconds(1.0);
    auto t2   = Timestamp::from_seconds(2.0);
    auto diff = t2 - t1;

    EXPECT_EQ(diff.nanoseconds, 1'000'000'000);
}

// Test Duration
TEST(DurationTest, Conversion)
{
    auto dur = Duration::from_seconds(0.5);
    EXPECT_EQ(dur.nanoseconds, 500'000'000);
    EXPECT_DOUBLE_EQ(dur.to_seconds(), 0.5);
}

TEST(DurationTest, Comparison)
{
    auto d1 = Duration::from_seconds(1.0);
    auto d2 = Duration::from_seconds(2.0);

    EXPECT_LT(d1, d2);
    EXPECT_GT(d2, d1);
}

// Test SensorDescriptor
TEST(SensorDescriptorTest, Equality)
{
    SensorDescriptor desc1 {
            SensorID {"lidar_front"},
            SensorType::LIDAR_2D,
            "lidar_front_link",
            Eigen::Vector3d(1.0, 0.0, 0.5),
            Eigen::Quaterniond::Identity(),
            FieldOfView {-M_PI, M_PI, 0.0, 0.0},
            0.01,
            0.1,
            30.0};

    SensorDescriptor desc2 = desc1;

    EXPECT_EQ(desc1, desc2);

    desc2.range_max_m = 20.0;
    EXPECT_NE(desc1, desc2);
}

// Test LidarObservation with Scan2D
TEST(LidarObservationTest, Scan2DConstruction)
{
    Scan2D scan;
    scan.angle_min_rad       = -M_PI;
    scan.angle_max_rad       = M_PI;
    scan.angle_increment_rad = 0.01;
    scan.range_min_m         = 0.1;
    scan.range_max_m         = 30.0;
    scan.ranges              = {1.0, 2.0, 3.0, 4.0, 5.0};
    scan.intensities         = {100.0, 200.0, 150.0, 180.0, 120.0};

    LidarObservation obs {
            SensorID {"lidar_front"},
            SessionID {1000000000ULL, 12345},
            "lidar_front_link",
            "vehicle_steady_clock",
            Timestamp::from_seconds(1.5),
            scan};

    EXPECT_TRUE(obs.is_2d());
    EXPECT_FALSE(obs.is_3d());
    EXPECT_EQ(obs.point_count(), 5);

    const auto & scan_data = obs.as_scan_2d();
    EXPECT_EQ(scan_data.ranges.size(), 5);
    EXPECT_DOUBLE_EQ(scan_data.ranges[0], 1.0);
}

// Test LidarObservation with Cloud3D
TEST(LidarObservationTest, Cloud3DConstruction)
{
    Cloud3D cloud;
    cloud.points.emplace_back(1.0f, 2.0f, 3.0f, 100.0f);
    cloud.points.emplace_back(4.0f, 5.0f, 6.0f, 200.0f);

    LidarObservation obs {
            SensorID {"lidar_top"},
            SessionID {2000000000ULL, 67890},
            "lidar_top_link",
            "vehicle_steady_clock",
            Timestamp::from_seconds(2.0),
            cloud};

    EXPECT_TRUE(obs.is_3d());
    EXPECT_FALSE(obs.is_2d());
    EXPECT_EQ(obs.point_count(), 2);

    const auto & cloud_data = obs.as_cloud_3d();
    EXPECT_EQ(cloud_data.points.size(), 2);
    EXPECT_FLOAT_EQ(cloud_data.points[0].x, 1.0f);
}

// Test PoseEstimate
TEST(PoseEstimateTest, Construction)
{
    PoseEstimate pose {
            SourceID {"odom"},
            SessionID {3000000000ULL, 11111},
            "odom",
            "vehicle_steady_clock",
            Timestamp::from_seconds(5.0),
            Eigen::Vector3d(1.0, 2.0, 3.0),
            Eigen::Quaterniond::Identity(),
            std::nullopt,
            0.95,
            Duration::from_seconds(0.1),
            0};

    EXPECT_EQ(pose.source_id.value, "odom");
    EXPECT_DOUBLE_EQ(pose.quality, 0.95);
    EXPECT_TRUE(pose.is_fresh(Duration::from_seconds(1.0)));
    EXPECT_FALSE(pose.is_fresh(Duration::from_seconds(0.05)));
}

// Test HealthState
TEST(HealthStateTest, SensorHealth)
{
    SensorHealth health {
            SensorID {"lidar_front"},
            SensorHealthStatus::Active,
            Timestamp::from_seconds(10.0)};

    EXPECT_TRUE(health.is_healthy());

    health.status = SensorHealthStatus::Stale;
    EXPECT_FALSE(health.is_healthy());
}

// Test MapperInputContract
TEST(MapperInputContractTest, DefaultContract)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_3D, 1, {}}};
    contract.requires_pose            = true;
    contract.pose_freshness_threshold = Duration::from_seconds(1.0);

    EXPECT_EQ(contract.minimum_viable.size(), 1);
    EXPECT_TRUE(contract.requires_pose);
}

// Test MapperHealthGate
TEST(MapperHealthGateTest, HealthyState)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_2D, 1, {}}};
    contract.requires_pose            = true;
    contract.pose_freshness_threshold = Duration::from_seconds(1.0);

    MapperHealthGate gate(contract);

    // Setup descriptors
    std::vector<SensorDescriptor> descriptors = {
            SensorDescriptor {
                    SensorID {"lidar_front"},
                    SensorType::LIDAR_2D,
                    "lidar_front_link",
                    Eigen::Vector3d::Zero(),
                    Eigen::Quaterniond::Identity(),
                    FieldOfView {-M_PI, M_PI, 0.0, 0.0},
                    0.01,
                    0.1,
                    30.0}};

    // Setup sensor health
    std::vector<SensorHealth> sensor_health = {
            SensorHealth {
                    SensorID {"lidar_front"},
                    SensorHealthStatus::Active,
                    Timestamp::from_seconds(1.0)}};

    // Setup pose
    PoseEstimate pose {
            SourceID {"odom"},
            SessionID {1ULL, 1},
            "odom",
            "vehicle_steady_clock",
            Timestamp::from_seconds(1.0),
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            std::nullopt,
            1.0,
            Duration::from_seconds(0.1),
            0};

    auto state = gate.evaluate(descriptors, sensor_health, pose);

    EXPECT_EQ(state, HealthState::Healthy);
    EXPECT_TRUE(gate.current_capability().has_2d_lidar);
    EXPECT_TRUE(gate.current_capability().has_fresh_pose);
    EXPECT_EQ(gate.current_capability().active_sensor_count, 1);
}

TEST(MapperHealthGateTest, RejectsUnexpectedPoseFrame)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_2D, 1, {}}};
    contract.requires_pose        = true;
    contract.expected_pose_frame  = "map";
    contract.minimum_pose_quality = 0.5;

    MapperHealthGate                    gate(contract);
    const std::vector<SensorDescriptor> descriptors = {
            make_descriptor("lidar_front", SensorType::LIDAR_2D)};
    const std::vector<SensorHealth> sensor_health = {
            make_health("lidar_front", SensorHealthStatus::Active)};
    auto pose     = make_pose(0.0);
    pose.frame_id = "odom";

    EXPECT_EQ(gate.evaluate(descriptors, sensor_health, pose), HealthState::Unavailable);
    EXPECT_TRUE(gate.current_capability().has_fresh_pose);
    EXPECT_FALSE(gate.current_capability().has_expected_pose_frame);
    EXPECT_FALSE(gate.current_capability().has_usable_pose);
    EXPECT_NE(gate.degradation_reason().find("Pose frame mismatch"), std::string::npos);
}

TEST(MapperHealthGateTest, RejectsLowOrNonFinitePoseQuality)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_2D, 1, {}}};
    contract.requires_pose        = true;
    contract.expected_pose_frame  = "odom";
    contract.minimum_pose_quality = 0.75;

    const std::vector<SensorDescriptor> descriptors = {
            make_descriptor("lidar_front", SensorType::LIDAR_2D)};
    const std::vector<SensorHealth> sensor_health = {
            make_health("lidar_front", SensorHealthStatus::Active)};

    auto low_quality_pose    = make_pose(0.0);
    low_quality_pose.quality = 0.5;
    MapperHealthGate low_quality_gate(contract);
    EXPECT_EQ(
            low_quality_gate.evaluate(descriptors, sensor_health, low_quality_pose),
            HealthState::Unavailable);
    EXPECT_FALSE(low_quality_gate.current_capability().has_sufficient_pose_quality);
    EXPECT_NE(
            low_quality_gate.degradation_reason().find("Pose quality below threshold"),
            std::string::npos);

    auto invalid_pose    = make_pose(0.0);
    invalid_pose.quality = std::numeric_limits<double>::quiet_NaN();
    MapperHealthGate invalid_gate(contract);
    EXPECT_EQ(
            invalid_gate.evaluate(descriptors, sensor_health, invalid_pose),
            HealthState::Unavailable);
    EXPECT_FALSE(invalid_gate.current_capability().has_sufficient_pose_quality);
}

TEST(MapperHealthGateTest, UnavailableState)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_3D, 1, {}}};
    contract.requires_pose = true;

    MapperHealthGate gate(contract);

    std::vector<SensorDescriptor> descriptors;
    std::vector<SensorHealth>     sensor_health;
    std::optional<PoseEstimate>   pose = std::nullopt;

    auto state = gate.evaluate(descriptors, sensor_health, pose);

    EXPECT_EQ(state, HealthState::Unavailable);
    EXPECT_FALSE(gate.degradation_reason().empty());
}

TEST(MapperHealthGateTest, DegradedState)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_3D, 1, {}}};
    contract.degraded_combinations = {
            DegradedCombination {
                    {SensorRequirement {SensorType::LIDAR_2D, 1, {}}},
                    "Only 2D lidar available"}};
    contract.requires_pose            = true;
    contract.pose_freshness_threshold = Duration::from_seconds(1.0);

    MapperHealthGate gate(contract);

    // Only 2D lidar available (degraded)
    std::vector<SensorDescriptor> descriptors = {
            SensorDescriptor {
                    SensorID {"lidar_2d"},
                    SensorType::LIDAR_2D,
                    "lidar_2d_link",
                    Eigen::Vector3d::Zero(),
                    Eigen::Quaterniond::Identity(),
                    FieldOfView {-M_PI, M_PI, 0.0, 0.0},
                    0.01,
                    0.1,
                    30.0}};

    std::vector<SensorHealth> sensor_health = {
            SensorHealth {
                    SensorID {"lidar_2d"},
                    SensorHealthStatus::Active,
                    Timestamp::from_seconds(1.0)}};

    PoseEstimate pose {
            SourceID {"odom"},
            SessionID {1ULL, 1},
            "odom",
            "vehicle_steady_clock",
            Timestamp::from_seconds(1.0),
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            std::nullopt,
            1.0,
            Duration::from_seconds(0.1),
            0};

    auto state = gate.evaluate(descriptors, sensor_health, pose);

    EXPECT_EQ(state, HealthState::Degraded);
    EXPECT_EQ(gate.degradation_reason(), "Only 2D lidar available");
}

TEST(MapperHealthGateTest, HonorsSensorCountsAndSpecificSensorRequirements)
{
    MapperInputContract count_contract;
    count_contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_2D, 2, {}}};
    count_contract.requires_pose = false;

    MapperHealthGate                    count_gate(count_contract);
    const std::vector<SensorDescriptor> descriptors = {
            make_descriptor("front", SensorType::LIDAR_2D),
            make_descriptor("rear", SensorType::LIDAR_2D),
            make_descriptor("top", SensorType::LIDAR_3D)};
    const std::vector<SensorHealth> healthy_sensors = {
            make_health("front", SensorHealthStatus::Active),
            make_health("rear", SensorHealthStatus::Active),
            make_health("top", SensorHealthStatus::Stale)};

    EXPECT_EQ(count_gate.evaluate(descriptors, healthy_sensors, std::nullopt), HealthState::Healthy);
    EXPECT_EQ(count_gate.current_capability().active_2d_lidar_count, 2);
    EXPECT_EQ(count_gate.current_capability().active_3d_lidar_count, 0);

    MapperInputContract specific_contract;
    specific_contract.minimum_viable = {
            SensorRequirement {
                    SensorType::LIDAR_2D,
                    1,
                    std::vector<SensorID> {SensorID {"rear"}}}};
    specific_contract.requires_pose = false;

    MapperHealthGate                front_only_gate(specific_contract);
    const std::vector<SensorHealth> front_only = {
            make_health("front", SensorHealthStatus::Active),
            make_health("rear", SensorHealthStatus::Stale)};
    EXPECT_EQ(front_only_gate.evaluate(descriptors, front_only, std::nullopt), HealthState::Unavailable);

    MapperHealthGate                rear_only_gate(specific_contract);
    const std::vector<SensorHealth> rear_only = {
            make_health("front", SensorHealthStatus::Stale),
            make_health("rear", SensorHealthStatus::Active)};
    EXPECT_EQ(rear_only_gate.evaluate(descriptors, rear_only, std::nullopt), HealthState::Healthy);
}

TEST(MapperHealthGateTest, AppliesCompleteTransitionAndRecoveryPath)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_3D, 1, {}}};
    contract.degraded_combinations = {
            DegradedCombination {
                    {SensorRequirement {SensorType::LIDAR_2D, 1, {}}},
                    "Only 2D lidar available"}};
    contract.requires_pose              = false;
    contract.recovery_stability_samples = 3;

    MapperHealthGate                    gate(contract);
    const std::vector<SensorDescriptor> descriptors = {
            make_descriptor("front", SensorType::LIDAR_2D),
            make_descriptor("top", SensorType::LIDAR_3D)};

    const std::vector<SensorHealth> healthy = {
            make_health("front", SensorHealthStatus::Active),
            make_health("top", SensorHealthStatus::Active)};
    EXPECT_EQ(gate.evaluate(descriptors, healthy, std::nullopt), HealthState::Healthy);

    const std::vector<SensorHealth> degraded = {
            make_health("front", SensorHealthStatus::Active),
            make_health("top", SensorHealthStatus::Stale)};
    EXPECT_EQ(gate.evaluate(descriptors, degraded, std::nullopt), HealthState::Degraded);

    const std::vector<SensorHealth> unavailable = {
            make_health("front", SensorHealthStatus::Stale),
            make_health("top", SensorHealthStatus::Stale)};
    EXPECT_EQ(gate.evaluate(descriptors, unavailable, std::nullopt), HealthState::Unavailable);

    // Recovery to Degraded takes three consecutive degraded samples.
    EXPECT_EQ(gate.evaluate(descriptors, degraded, std::nullopt), HealthState::Unavailable);
    EXPECT_EQ(gate.evaluate(descriptors, degraded, std::nullopt), HealthState::Unavailable);
    EXPECT_EQ(gate.evaluate(descriptors, degraded, std::nullopt), HealthState::Degraded);

    // Recovery to Healthy also requires three samples and cannot skip Degraded.
    EXPECT_EQ(gate.evaluate(descriptors, healthy, std::nullopt), HealthState::Degraded);
    EXPECT_EQ(gate.evaluate(descriptors, healthy, std::nullopt), HealthState::Degraded);
    EXPECT_EQ(gate.evaluate(descriptors, healthy, std::nullopt), HealthState::Healthy);
}

TEST(MapperHealthGateTest, PoseTimeoutCausesImmediateUnavailableAndStableRecovery)
{
    MapperInputContract contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_2D, 1, {}}};
    contract.requires_pose              = true;
    contract.pose_freshness_threshold   = Duration::from_seconds(1.0);
    contract.recovery_stability_samples = 3;

    MapperHealthGate                    gate(contract);
    const std::vector<SensorDescriptor> descriptors = {
            make_descriptor("front", SensorType::LIDAR_2D)};
    const std::vector<SensorHealth> sensor_health = {
            make_health("front", SensorHealthStatus::Active)};

    EXPECT_EQ(gate.evaluate(descriptors, sensor_health, make_pose(0.5)), HealthState::Healthy);
    EXPECT_EQ(gate.evaluate(descriptors, sensor_health, make_pose(1.0)), HealthState::Unavailable);

    // The first fresh pose after timeout enters the recovery bridge state.
    EXPECT_EQ(gate.evaluate(descriptors, sensor_health, make_pose(0.1)), HealthState::Degraded);
    EXPECT_EQ(gate.evaluate(descriptors, sensor_health, make_pose(0.1)), HealthState::Degraded);
    EXPECT_EQ(gate.evaluate(descriptors, sensor_health, make_pose(0.1)), HealthState::Healthy);
}

TEST(MapperHealthGatePerformanceTest, ReportsHealthEvaluationBaseline)
{
    constexpr std::size_t sample_count = 5000;
    MapperInputContract   contract;
    contract.minimum_viable = {
            SensorRequirement {SensorType::LIDAR_2D, 1, {}}};
    contract.requires_pose              = true;
    contract.expected_pose_frame        = "odom";
    contract.minimum_pose_quality       = 0.5;
    contract.recovery_stability_samples = 1;

    MapperHealthGate                    gate(contract);
    const std::vector<SensorDescriptor> descriptors = {
            make_descriptor("lidar_front", SensorType::LIDAR_2D)};
    const std::vector<SensorHealth> sensor_health = {
            make_health("lidar_front", SensorHealthStatus::Active)};
    const auto pose = make_pose(0.0);

    std::size_t healthy_count = 0;
    const auto  start         = std::chrono::steady_clock::now();
    for(std::size_t index = 0; index < sample_count; ++index) {
        if(gate.evaluate(descriptors, sensor_health, pose) == HealthState::Healthy) {
            ++healthy_count;
        }
    }
    const auto elapsed    = std::chrono::steady_clock::now() - start;
    const auto average_us = std::chrono::duration<double, std::micro>(elapsed).count()
                            / static_cast<double>(sample_count);

    std::cout << "[PERF] mapper_health_evaluation samples=" << sample_count
              << " average_us=" << average_us << std::endl;
    EXPECT_EQ(healthy_count, sample_count);
    EXPECT_LT(average_us, 1000.0);
}

int main(int argc, char ** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
