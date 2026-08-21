#include "swarm_data_plane/ros/TopologyConversions.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>

namespace SwarmDataPlane::Test {

    namespace {

        VehicleIdentity vehicle(
                const char * id,
                std::uint64_t boot_time_ns)
        {
            return {"fleet-a", id, {boot_time_ns, 1U}};
        }

        TopologySnapshot snapshot_fixture()
        {
            const auto source = vehicle("drone-a", 100U);
            const auto target = vehicle("drone-b", 200U);
            TopologySnapshot snapshot;
            snapshot.fleet_id = "fleet-a";
            snapshot.topology_epoch = 9U;
            for(const auto & current : {source, target}) {
                VehicleRegistration registration;
                registration.identity = current;
                registration.registration_generation = 1U;
                registration.components = {{"mapper", {300U, 1U}}};
                SensorDescriptorIdentity sensor;
                sensor.sensor_id = "lidar";
                sensor.descriptor_hash[0] = 1U;
                registration.sensors = {sensor};
                snapshot.members.push_back({
                        registration,
                        MembershipState::Ready,
                        MemberAvailability::Live,
                        {true, true, true, true}});
            }
            const LinkDescriptor link {
                    "link-control", 7U, source, target, LinkHealth::Up,
                    1'000U, 1'000'000U, 100U};
            snapshot.links = {link};
            snapshot.edges = {{
                    LogicalGraphKind::Control,
                    link.link_id,
                    link.link_epoch,
                    source,
                    target}};
            snapshot.routes = {{
                    "route-control",
                    LogicalGraphKind::Control,
                    source,
                    target,
                    snapshot.topology_epoch,
                    4U,
                    2U,
                    10'000U,
                    {{link.link_id, link.link_epoch}}}};
            std::sort(snapshot.members.begin(), snapshot.members.end());
            return snapshot;
        }

    }// namespace

    TEST(TopologyConversionsTest, RoundTripPreservesCanonicalSnapshot)
    {
        const auto original = snapshot_fixture();
        swarm_data_interfaces::msg::TopologySnapshot message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_topology_snapshot(original, message, diagnostic))
                << diagnostic;

        const auto decoded = Ros::decode_topology_snapshot(message);
        ASSERT_TRUE(decoded.success) << decoded.diagnostic;
        ASSERT_TRUE(decoded.snapshot.has_value());
        EXPECT_EQ(*decoded.snapshot, original);
        EXPECT_EQ(message.members.size(), 2U);
        EXPECT_EQ(message.routes.front().route_epoch, 4U);
    }

    TEST(TopologyConversionsTest, RejectsUnknownEnumAndOversizedCollections)
    {
        const auto original = snapshot_fixture();
        swarm_data_interfaces::msg::TopologySnapshot message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_topology_snapshot(original, message, diagnostic));

        message.members.front().state = 99U;
        EXPECT_FALSE(Ros::decode_topology_snapshot(message).success);

        ASSERT_TRUE(Ros::encode_topology_snapshot(original, message, diagnostic));
        TopologyLimits limits;
        limits.max_members = 1U;
        EXPECT_FALSE(Ros::decode_topology_snapshot(message, limits).success);
    }

}// namespace SwarmDataPlane::Test
