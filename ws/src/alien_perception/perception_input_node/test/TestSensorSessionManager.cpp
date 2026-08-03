#include "perception_input_node/SensorSessionManager.hpp"
#include <gtest/gtest.h>

using namespace Perception;
using namespace Perception::Input;

namespace {

    SensorDescriptor make_descriptor(const char * sensor_id, SensorType type, const char * frame_id)
    {
        return SensorDescriptor {
                SensorID {sensor_id},
                type,
                frame_id,
                Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(),
                FieldOfView {-3.14, 3.14, 0.0, 0.0},
                0.01,
                0.1,
                30.0};
    }

}// namespace

TEST(SensorSessionManagerTest, FreezesInventoryAndAllowsMatchingReconnect)
{
    SensorSessionManager manager(SessionID {10, 20});
    const auto           front = make_descriptor("front", SensorType::LIDAR_2D, "front_link");
    const auto           top   = make_descriptor("top", SensorType::LIDAR_3D, "top_link");

    EXPECT_TRUE(manager.register_sensor(front).accepted);
    EXPECT_TRUE(manager.register_sensor(top).accepted);
    manager.freeze();

    EXPECT_TRUE(manager.is_frozen());
    EXPECT_TRUE(manager.can_reconnect(front));
    ASSERT_EQ(manager.descriptors().size(), 2);
    EXPECT_EQ(manager.descriptors()[0].sensor_id.value, "front");
    EXPECT_EQ(manager.session_id(), (SessionID {10, 20}));
}

TEST(SensorSessionManagerTest, RejectsNewSensorAfterFreezeWithActionableDiagnostic)
{
    SensorSessionManager manager(SessionID {10, 20});
    EXPECT_TRUE(manager.register_sensor(
                               make_descriptor("front", SensorType::LIDAR_2D, "front_link"))
                        .accepted);
    manager.freeze();

    const auto result = manager.register_sensor(
            make_descriptor("rear", SensorType::LIDAR_2D, "rear_link"));
    EXPECT_FALSE(result.accepted);
    EXPECT_NE(result.diagnostic.find("rear"), std::string::npos);
    EXPECT_NE(result.diagnostic.find("front"), std::string::npos);
    EXPECT_NE(result.diagnostic.find("new vehicle session"), std::string::npos);
}

TEST(SensorSessionManagerTest, RejectsChangedDescriptorBeforeAndAfterFreeze)
{
    SensorSessionManager manager(SessionID {10, 20});
    const auto           original = make_descriptor("front", SensorType::LIDAR_2D, "front_link");
    auto                 changed  = original;
    changed.range_max_m           = 50.0;

    EXPECT_TRUE(manager.register_sensor(original).accepted);
    const auto before_freeze = manager.register_sensor(changed);
    EXPECT_FALSE(before_freeze.accepted);
    EXPECT_NE(before_freeze.diagnostic.find("Registered="), std::string::npos);
    EXPECT_NE(before_freeze.diagnostic.find("attempted="), std::string::npos);

    manager.freeze();
    std::string diagnostic;
    EXPECT_FALSE(manager.can_reconnect(changed, &diagnostic));
    EXPECT_NE(diagnostic.find("changed descriptor"), std::string::npos);
}

TEST(SensorSessionManagerTest, TreatsRayEvidenceAsFrozenDescriptorState)
{
    SensorSessionManager manager(SessionID {10, 20});
    auto original         = make_descriptor("front", SensorType::LIDAR_2D, "front_link");
    original.ray_evidence = RayEvidenceCapability::HitOnly;
    auto elevated         = original;
    elevated.ray_evidence = RayEvidenceCapability::FullRay;

    EXPECT_TRUE(manager.register_sensor(original).accepted);
    manager.freeze();

    const auto result = manager.register_sensor(elevated);
    EXPECT_FALSE(result.accepted);
    EXPECT_NE(result.diagnostic.find("ray_evidence=hit_only"), std::string::npos);
    EXPECT_NE(result.diagnostic.find("ray_evidence=full_ray"), std::string::npos);
    EXPECT_FALSE(manager.can_reconnect(elevated));
}
