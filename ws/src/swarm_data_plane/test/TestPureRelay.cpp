#include "swarm_data_plane/PureRelay.hpp"

#include "TestFixtures.hpp"

#include "perception_map_update/MapUpdateProducer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>

namespace SwarmDataPlane::Test {

    namespace {

        VehicleIdentity vehicle(const char * id, std::uint64_t boot_time_ns)
        {
            return {"fleet-a", id, {boot_time_ns, 1U}};
        }

        MemberRecord ready_member(const VehicleIdentity & identity)
        {
            VehicleRegistration registration;
            registration.identity = identity;
            registration.registration_generation = 1U;
            registration.components = {
                    {"runtime", {identity.session.boot_time_ns + 1U, 2U}}};
            SensorDescriptorIdentity sensor;
            sensor.sensor_id = "lidar";
            sensor.descriptor_hash[0] = 0xA5U;
            registration.sensors = {sensor};
            return {registration, MembershipState::Ready,
                    availability_for(MembershipState::Ready),
                    {true, true, true, true}};
        }

        TopologySnapshot relay_topology(
                const VehicleIdentity & source,
                const VehicleIdentity & relay,
                const VehicleIdentity & receiver,
                std::uint64_t route_epoch = 3U)
        {
            TopologySnapshot topology;
            topology.fleet_id = source.fleet_id;
            topology.topology_epoch = 5U;
            topology.members = {
                    ready_member(source), ready_member(relay), ready_member(receiver)};
            topology.links = {
                    {"link-source-relay", 1U, source, relay, LinkHealth::Up,
                     1'000U, 1'000'000U, 100U},
                    {"link-relay-receiver", 1U, relay, receiver, LinkHealth::Up,
                     1'000U, 1'000'000U, 100U}};
            for(const auto graph : {
                        LogicalGraphKind::Communication,
                        LogicalGraphKind::Map}) {
                topology.edges.push_back(
                        {graph, "link-source-relay", 1U, source, relay});
                topology.edges.push_back(
                        {graph, "link-relay-receiver", 1U, relay, receiver});
            }
            topology.routes = {
                    {"route-map", LogicalGraphKind::Map, source, receiver,
                     topology.topology_epoch, route_epoch, 3U, 1'000'000'000U,
                     {{"link-source-relay", 1U},
                      {"link-relay-receiver", 1U}}}};
            std::sort(topology.members.begin(), topology.members.end());
            std::sort(topology.links.begin(), topology.links.end());
            std::sort(topology.edges.begin(), topology.edges.end());
            return topology;
        }

        ServiceBudget relay_budget()
        {
            return {4'096U, 8'192U, 1'000'000U, 0U, 2U};
        }

        RoleSnapshot relay_roles(
                const VehicleIdentity & relay,
                std::uint64_t topology_epoch,
                PrimaryRole role = PrimaryRole::Relay)
        {
            RoleSnapshot snapshot;
            snapshot.fleet_id = relay.fleet_id;
            snapshot.topology_epoch = topology_epoch;
            snapshot.role_epoch = 2U;
            snapshot.assignments = {{
                    relay,
                    1U,
                    role,
                    RoleLifecycle::Active,
                    {{ServiceKind::Relay, ServiceLifecycle::Active,
                      relay_budget()}}}};
            return snapshot;
        }

        CapabilityEvidence relay_evidence(const VehicleIdentity & relay)
        {
            CapabilityEvidence evidence;
            evidence.identity = relay;
            evidence.evidence_revision = 1U;
            evidence.effective_capabilities = {CapabilityKind::RelayForwarding};
            evidence.vehicle_health = VehicleHealth::Healthy;
            evidence.resource_health = ResourceHealth::Healthy;
            evidence.service_health = {
                    {ServiceKind::Relay, ResourceHealth::Healthy}};
            return evidence;
        }

        RuntimeSnapshotCache relay_cache(
                const VehicleIdentity & relay,
                const TopologySnapshot & topology,
                PrimaryRole role = PrimaryRole::Relay)
        {
            RuntimeSnapshotCache cache(relay);
            EXPECT_TRUE(cache.apply_topology(topology));
            EXPECT_TRUE(cache.apply_role(
                    relay_roles(relay, topology.topology_epoch, role)));
            auto evidence = relay_evidence(relay);
            if(role == PrimaryRole::Explorer) {
                evidence.effective_capabilities.insert(
                        evidence.effective_capabilities.begin(),
                        CapabilityKind::Exploration);
            }
            EXPECT_TRUE(cache.apply_evidence(std::move(evidence)));
            return cache;
        }

        RoutedMapUpdate map_message(std::uint64_t route_epoch = 3U)
        {
            PerceptionMapUpdate::MapUpdateProducer producer;
            const auto prepared = producer.prepare(snapshot(
                    1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free}}));
            EXPECT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
            auto message = routed(shared_update(prepared), 1U, "message-1", route_epoch);
            message.route.ttl_hops = 3U;
            return message;
        }

    }// namespace

    TEST(PureRelayTest, ForwardsWithoutChangingPayloadIdentity)
    {
        const auto source = vehicle("explorer-0", 100U);
        const auto relay = vehicle("relay-0", 200U);
        const auto receiver = vehicle("receiver", 300U);
        const auto topology = relay_topology(source, relay, receiver);
        const auto cache = relay_cache(relay, topology);
        PureRelay runtime(relay, "route-map", 0U);
        const auto input = map_message();

        const auto result = runtime.forward(cache, input, 100U);
        ASSERT_TRUE(result) << result.diagnostic;
        ASSERT_TRUE(result.message.has_value());
        EXPECT_EQ(result.message->route.hop_count, 1U);
        EXPECT_EQ(result.message->accumulated_forwarding_ns, 110U);
        EXPECT_EQ(result.message->update, input.update);
        EXPECT_EQ(result.message->producer, input.producer);
        EXPECT_EQ(result.message->origin.domain, input.origin.domain);
        EXPECT_EQ(result.message->sequence, input.sequence);
        EXPECT_EQ(result.message->payload_hash, input.payload_hash);
        EXPECT_EQ(result.message->payload_bytes, input.payload_bytes);
        EXPECT_EQ(runtime.counters().forwarded, 1U);
    }

    TEST(PureRelayTest, SupportsExplorerWithRelayService)
    {
        const auto source = vehicle("explorer-0", 100U);
        const auto relay = vehicle("explorer-1", 200U);
        const auto receiver = vehicle("receiver", 300U);
        const auto topology = relay_topology(source, relay, receiver);
        const auto cache = relay_cache(relay, topology, PrimaryRole::Explorer);
        PureRelay runtime(relay, "route-map", 0U);

        EXPECT_TRUE(runtime.forward(cache, map_message(), 100U));
    }

    TEST(PureRelayTest, SupportsLocalReceiverAfterTerminalFleetHop)
    {
        const auto source = vehicle("explorer-0", 100U);
        const auto relay = vehicle("relay-0", 200U);
        auto topology = relay_topology(source, relay, vehicle("receiver", 300U));
        topology.routes.front().target = relay;
        topology.routes.front().hops.resize(1U);
        topology.routes.front().ttl_hops = 2U;
        topology.links.erase(
                std::remove_if(
                        topology.links.begin(), topology.links.end(),
                        [&](const LinkDescriptor & link) {
                            return link.source == relay;
                        }),
                topology.links.end());
        topology.edges.erase(
                std::remove_if(
                        topology.edges.begin(), topology.edges.end(),
                        [&](const GraphEdge & edge) {
                            return edge.source == relay;
                        }),
                topology.edges.end());
        const auto cache = relay_cache(relay, topology);
        PureRelay runtime(relay, "route-map", 0U, true);
        auto message = map_message();
        message.route.ttl_hops = 2U;

        EXPECT_TRUE(runtime.forward(cache, message, 100U));
        EXPECT_TRUE(runtime.terminal_output());
    }

    TEST(PureRelayTest, RejectsOldRouteAndWrongHopWithoutForwarding)
    {
        const auto source = vehicle("explorer-0", 100U);
        const auto relay = vehicle("relay-0", 200U);
        const auto receiver = vehicle("receiver", 300U);
        const auto topology = relay_topology(source, relay, receiver);
        const auto cache = relay_cache(relay, topology);
        PureRelay runtime(relay, "route-map", 0U);

        EXPECT_EQ(
                runtime.forward(cache, map_message(2U), 100U).status,
                PureRelayStatus::RejectedRoute);
        auto wrong_hop = map_message();
        wrong_hop.route.hop_count = 1U;
        EXPECT_EQ(
                runtime.forward(cache, wrong_hop, 100U).status,
                PureRelayStatus::RejectedRoute);
        EXPECT_EQ(runtime.counters().forwarded, 0U);
        EXPECT_EQ(runtime.counters().rejected_route, 2U);
    }

    TEST(PureRelayTest, RejectsRoleTransitionBeforeQuiesceAck)
    {
        const auto source = vehicle("explorer-0", 100U);
        const auto relay = vehicle("relay-0", 200U);
        const auto receiver = vehicle("receiver", 300U);
        const auto topology = relay_topology(source, relay, receiver);
        auto cache = relay_cache(relay, topology);
        auto pending = RoleTransition {
                "transition-1",
                RoleTransitionState::Prepared,
                2U,
                topology.topology_epoch,
                relay_roles(relay, topology.topology_epoch).assignments,
                {relay},
                {}};
        pending.target_assignments.front().lifecycle = RoleLifecycle::Draining;
        ASSERT_TRUE(cache.apply_transition(std::move(pending)));
        PureRelay runtime(relay, "route-map", 0U);

        const auto result = runtime.forward(cache, map_message(), 100U);
        EXPECT_EQ(result.status, PureRelayStatus::RejectedRole);
        EXPECT_EQ(result.admission, WorkAdmissionStatus::TransitionBlocked);
    }

    TEST(PureRelayTest, RejectsExpiredEnvelopeAndCountsItWithoutHistory)
    {
        const auto source = vehicle("explorer-0", 100U);
        const auto relay = vehicle("relay-0", 200U);
        const auto receiver = vehicle("receiver", 300U);
        const auto topology = relay_topology(source, relay, receiver);
        const auto cache = relay_cache(relay, topology);
        PureRelay runtime(relay, "route-map", 0U);
        auto message = map_message();
        message.accumulated_forwarding_ns = message.validity_budget_ns - 1U;

        EXPECT_EQ(
                runtime.forward(cache, message, 1U).status,
                PureRelayStatus::RejectedExpired);
        EXPECT_EQ(runtime.counters().rejected_expired, 1U);
    }

}// namespace SwarmDataPlane::Test
