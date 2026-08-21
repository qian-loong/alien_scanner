#include "swarm_data_plane/RuntimeAuthority.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        struct Fixture {
            VehicleIdentity explorer_0 {"fleet-a", "explorer-0", {100U, 1U}};
            VehicleIdentity explorer_1 {"fleet-a", "explorer-1", {110U, 1U}};
            VehicleIdentity relay_0 {"fleet-a", "relay-0", {200U, 1U}};
            VehicleIdentity relay_1 {"fleet-a", "relay-1", {210U, 1U}};
        };

        VehicleRegistration registration(const VehicleIdentity & identity)
        {
            SensorDescriptorIdentity sensor;
            sensor.sensor_id = "lidar";
            sensor.descriptor_hash[0] = 0xA5U;
            return {
                    identity,
                    1U,
                    {{"runtime", {identity.session.boot_time_ns + 1U, 2U}}},
                    {sensor}};
        }

        void make_ready(TopologyState & state, const VehicleIdentity & identity)
        {
            ASSERT_TRUE(state.register_vehicle(registration(identity)));
            ASSERT_TRUE(state.transition_member(identity, MembershipState::Resyncing));
            ASSERT_TRUE(state.set_prerequisites(identity, {true, true, true, true}));
            ASSERT_TRUE(state.transition_member(identity, MembershipState::Ready));
        }

        LinkDescriptor link(
                std::string id,
                const VehicleIdentity & source,
                const VehicleIdentity & target)
        {
            return {std::move(id), 1U, source, target, LinkHealth::Up,
                    1'000U, 1'000'000U, 100U};
        }

        TopologyState topology_state(const Fixture & fixture)
        {
            TopologyState state(
                    "fleet-a",
                    {{fixture.explorer_0.vehicle_id, 8U, 8U},
                     {fixture.explorer_1.vehicle_id, 8U, 8U},
                     {fixture.relay_0.vehicle_id, 8U, 8U},
                     {fixture.relay_1.vehicle_id, 8U, 8U}});
            make_ready(state, fixture.explorer_0);
            make_ready(state, fixture.explorer_1);
            make_ready(state, fixture.relay_0);
            make_ready(state, fixture.relay_1);

            const auto map_e0_r0 = link(
                    "map-e0-r0", fixture.explorer_0, fixture.relay_0);
            const auto map_e0_r1 = link(
                    "map-e0-r1", fixture.explorer_0, fixture.relay_1);
            const auto map_e1_r0 = link(
                    "map-e1-r0", fixture.explorer_1, fixture.relay_0);
            const auto map_e1_r1 = link(
                    "map-e1-r1", fixture.explorer_1, fixture.relay_1);
            const auto control_e0_r0 = link(
                    "control-e0-r0", fixture.explorer_0, fixture.relay_0);
            const auto control_r0_e1 = link(
                    "control-r0-e1", fixture.relay_0, fixture.explorer_1);
            const auto control_e1_r1 = link(
                    "control-e1-r1", fixture.explorer_1, fixture.relay_1);
            const auto control_r1_e0 = link(
                    "control-r1-e0", fixture.relay_1, fixture.explorer_0);

            TopologyCandidate candidate;
            candidate.base_topology_epoch = state.snapshot().topology_epoch;
            candidate.links = {
                    map_e0_r0, map_e0_r1, map_e1_r0, map_e1_r1,
                    control_e0_r0, control_r0_e1, control_e1_r1,
                    control_r1_e0};
            for(const auto & value : candidate.links) {
                candidate.edges.push_back(
                        {LogicalGraphKind::Communication,
                         value.link_id, value.link_epoch,
                         value.source, value.target});
            }
            candidate.edges.push_back(
                    {LogicalGraphKind::Map, map_e0_r0.link_id,
                     map_e0_r0.link_epoch, map_e0_r0.source, map_e0_r0.target});
            candidate.edges.push_back(
                    {LogicalGraphKind::Map, map_e0_r1.link_id,
                     map_e0_r1.link_epoch, map_e0_r1.source, map_e0_r1.target});
            candidate.edges.push_back(
                    {LogicalGraphKind::Map, map_e1_r0.link_id,
                     map_e1_r0.link_epoch, map_e1_r0.source, map_e1_r0.target});
            candidate.edges.push_back(
                    {LogicalGraphKind::Map, map_e1_r1.link_id,
                     map_e1_r1.link_epoch, map_e1_r1.source, map_e1_r1.target});
            for(const auto & value : {
                        control_e0_r0, control_r0_e1,
                        control_e1_r1, control_r1_e0}) {
                candidate.edges.push_back(
                        {LogicalGraphKind::Control,
                         value.link_id, value.link_epoch,
                         value.source, value.target});
            }
            candidate.routes = {
                    {"route-map-0", LogicalGraphKind::Map,
                     fixture.explorer_0, fixture.relay_0,
                     state.snapshot().topology_epoch, 1U, 2U,
                     1'000'000'000U,
                     {{map_e0_r0.link_id, map_e0_r0.link_epoch}}},
                    {"route-map-1", LogicalGraphKind::Map,
                     fixture.explorer_1, fixture.relay_0,
                     state.snapshot().topology_epoch, 1U, 2U,
                     1'000'000'000U,
                     {{map_e1_r0.link_id, map_e1_r0.link_epoch}}},
                    {"route-control-0", LogicalGraphKind::Control,
                     fixture.explorer_0, fixture.explorer_1,
                     state.snapshot().topology_epoch, 1U, 3U,
                     1'000'000'000U,
                     {{control_e0_r0.link_id, control_e0_r0.link_epoch},
                      {control_r0_e1.link_id, control_r0_e1.link_epoch}}},
                    {"route-control-1", LogicalGraphKind::Control,
                     fixture.explorer_1, fixture.explorer_0,
                     state.snapshot().topology_epoch, 1U, 3U,
                     1'000'000'000U,
                     {{control_e1_r1.link_id, control_e1_r1.link_epoch},
                      {control_r1_e0.link_id, control_r1_e0.link_epoch}}}};
            EXPECT_TRUE(state.replace_topology(std::move(candidate)));
            return state;
        }

        CapabilityRegistration capability_registration(
                const VehicleIdentity & identity,
                CapabilityKind capability)
        {
            return {identity, 1U, {capability}};
        }

        CapabilityEvidence evidence(
                const VehicleIdentity & identity,
                CapabilityKind capability,
                std::uint64_t revision = 1U,
                bool effective = true)
        {
            CapabilityEvidence value;
            value.identity = identity;
            value.evidence_revision = revision;
            if(effective) {
                value.effective_capabilities = {capability};
            }
            value.vehicle_health = VehicleHealth::Healthy;
            value.resource_health = ResourceHealth::Healthy;
            if(capability == CapabilityKind::RelayForwarding) {
                value.service_health = {
                        {ServiceKind::Relay, ResourceHealth::Healthy}};
            }
            return value;
        }

        ServiceBudget relay_budget()
        {
            return {4'096U, 8'192U, 1'000'000U, 0U, 2U};
        }

        RoleAssignment explorer_assignment(const VehicleIdentity & identity)
        {
            return {identity, 1U, PrimaryRole::Explorer,
                    RoleLifecycle::Active, {}};
        }

        RoleAssignment relay_assignment(const VehicleIdentity & identity)
        {
            return {
                    identity,
                    1U,
                    PrimaryRole::Relay,
                    RoleLifecycle::Active,
                    {{ServiceKind::Relay, ServiceLifecycle::Active,
                      relay_budget()}}};
        }

        RoleState role_state(
                const Fixture & fixture,
                const TopologySnapshot & topology)
        {
            RoleState state(topology);
            EXPECT_TRUE(state.register_capabilities(
                    capability_registration(
                            fixture.explorer_0, CapabilityKind::Exploration),
                    topology));
            EXPECT_TRUE(state.register_capabilities(
                    capability_registration(
                            fixture.explorer_1, CapabilityKind::Exploration),
                    topology));
            EXPECT_TRUE(state.register_capabilities(
                    capability_registration(
                            fixture.relay_0, CapabilityKind::RelayForwarding),
                    topology));
            EXPECT_TRUE(state.register_capabilities(
                    capability_registration(
                            fixture.relay_1, CapabilityKind::RelayForwarding),
                    topology));
            EXPECT_TRUE(state.update_capability_evidence(
                    evidence(fixture.explorer_0, CapabilityKind::Exploration),
                    topology));
            EXPECT_TRUE(state.update_capability_evidence(
                    evidence(fixture.explorer_1, CapabilityKind::Exploration),
                    topology));
            EXPECT_TRUE(state.update_capability_evidence(
                    evidence(fixture.relay_0, CapabilityKind::RelayForwarding),
                    topology));
            EXPECT_TRUE(state.update_capability_evidence(
                    evidence(fixture.relay_1, CapabilityKind::RelayForwarding),
                    topology));

            RoleCandidate initial;
            initial.base_role_epoch = state.snapshot().role_epoch;
            initial.topology_epoch = topology.topology_epoch;
            initial.assignments = {
                    explorer_assignment(fixture.explorer_0),
                    explorer_assignment(fixture.explorer_1),
                    relay_assignment(fixture.relay_0)};
            EXPECT_TRUE(state.begin_transition("initial", std::move(initial), topology));
            EXPECT_TRUE(state.commit_transition("initial", topology));
            return state;
        }

        RuntimeAuthority authority(const Fixture & fixture)
        {
            auto topology = topology_state(fixture);
            auto roles = role_state(fixture, topology.snapshot());
            const auto map_e0_r1 = std::find_if(
                    topology.snapshot().links.begin(),
                    topology.snapshot().links.end(),
                    [](const LinkDescriptor & value) {
                        return value.link_id == "map-e0-r1";
                    });
            const auto map_e1_r1 = std::find_if(
                    topology.snapshot().links.begin(),
                    topology.snapshot().links.end(),
                    [](const LinkDescriptor & value) {
                        return value.link_id == "map-e1-r1";
                    });
            EXPECT_NE(map_e0_r1, topology.snapshot().links.end());
            EXPECT_NE(map_e1_r1, topology.snapshot().links.end());
            RuntimeAuthorityConfig config;
            config.active_relay = fixture.relay_0;
            config.standby_relay = fixture.relay_1;
            config.route_failovers = {
                    {"route-map-0",
                     {"route-map-0", LogicalGraphKind::Map,
                      fixture.explorer_0, fixture.relay_1,
                      topology.snapshot().topology_epoch, 2U, 2U,
                      1'000'000'000U,
                      {{map_e0_r1->link_id, map_e0_r1->link_epoch}}}},
                    {"route-map-1",
                     {"route-map-1", LogicalGraphKind::Map,
                      fixture.explorer_1, fixture.relay_1,
                      topology.snapshot().topology_epoch, 2U, 2U,
                      1'000'000'000U,
                      {{map_e1_r1->link_id, map_e1_r1->link_epoch}}}}};
            config.failover_assignment = relay_assignment(fixture.relay_1);
            config.heartbeat_timeout_ns = 1'000U;
            config.initial_time_ns = 10'000U;
            return RuntimeAuthority(
                    std::move(topology), std::move(roles), std::move(config));
        }

        const RoleAssignment * find_assignment(
                const RoleSnapshot & snapshot,
                const VehicleIdentity & identity)
        {
            const auto found = std::find_if(
                    snapshot.assignments.begin(), snapshot.assignments.end(),
                    [&](const RoleAssignment & value) {
                        return value.identity == identity;
                    });
            return found == snapshot.assignments.end() ? nullptr : &*found;
        }

    }// namespace

    TEST(RuntimeAuthorityTest, CommitsFailoverAfterHeartbeatTimeout)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        const auto initial_topology_epoch = runtime.topology().topology_epoch;
        const auto initial_role_epoch = runtime.roles().role_epoch;

        EXPECT_EQ(
                runtime.tick(11'000U).status,
                RuntimeAuthorityStatus::NoChange);
        const auto result = runtime.tick(11'001U);
        ASSERT_EQ(result.status, RuntimeAuthorityStatus::FailoverCommitted)
                << result.diagnostic;
        EXPECT_TRUE(runtime.failed_over());
        EXPECT_GT(runtime.topology().topology_epoch, initial_topology_epoch);
        EXPECT_EQ(runtime.roles().role_epoch, initial_role_epoch + 1U);
        for(const auto & route_id : {"route-map-0", "route-map-1"}) {
            const auto route = std::find_if(
                    runtime.topology().routes.begin(),
                    runtime.topology().routes.end(),
                    [&](const RouteDescriptor & value) {
                        return value.route_id == route_id;
                    });
            ASSERT_NE(route, runtime.topology().routes.end());
            EXPECT_EQ(route->target, fixture.relay_1);
            EXPECT_EQ(route->route_epoch, 2U);
        }
        EXPECT_EQ(find_assignment(runtime.roles(), fixture.relay_0), nullptr);
        ASSERT_NE(find_assignment(runtime.roles(), fixture.relay_1), nullptr);
    }

    TEST(RuntimeAuthorityTest, DuplicateEvidenceDoesNotRefreshHeartbeat)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        auto next = evidence(
                fixture.relay_0, CapabilityKind::RelayForwarding, 2U);
        ASSERT_EQ(
                runtime.observe_evidence(next, 10'500U).status,
                RuntimeAuthorityStatus::EvidenceApplied);
        EXPECT_EQ(
                runtime.observe_evidence(std::move(next), 11'200U).status,
                RuntimeAuthorityStatus::EvidenceIgnored);
        EXPECT_EQ(
                runtime.tick(11'500U).status,
                RuntimeAuthorityStatus::NoChange);
        EXPECT_EQ(
                runtime.tick(11'501U).status,
                RuntimeAuthorityStatus::FailoverCommitted);
    }

    TEST(RuntimeAuthorityTest, KeepsCommittedSnapshotsWhenStandbyIsIneligible)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        const auto topology_before = runtime.topology();
        const auto roles_before = runtime.roles();
        ASSERT_EQ(
                runtime.observe_evidence(
                               evidence(
                                       fixture.relay_1,
                                       CapabilityKind::RelayForwarding,
                                       2U,
                                       false),
                               10'500U)
                        .status,
                RuntimeAuthorityStatus::EvidenceApplied);

        const auto rejected = runtime.tick(11'001U);
        EXPECT_EQ(rejected.status, RuntimeAuthorityStatus::RejectedFailover);
        EXPECT_EQ(runtime.topology(), topology_before);
        EXPECT_EQ(runtime.roles(), roles_before);
        EXPECT_FALSE(runtime.failed_over());

        ASSERT_EQ(
                runtime.observe_evidence(
                               evidence(
                                       fixture.relay_1,
                                       CapabilityKind::RelayForwarding,
                                       3U),
                               11'100U)
                        .status,
                RuntimeAuthorityStatus::EvidenceApplied);
        EXPECT_EQ(
                runtime.tick(11'101U).status,
                RuntimeAuthorityStatus::FailoverCommitted);
    }

    TEST(RuntimeAuthorityTest, RejectsRegressedReceiptTime)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        EXPECT_EQ(
                runtime.observe_evidence(
                               evidence(
                                       fixture.relay_0,
                                       CapabilityKind::RelayForwarding,
                                       2U),
                               9'999U)
                        .status,
                RuntimeAuthorityStatus::RejectedClock);
    }

    TEST(RuntimeAuthorityTest, AppliesNonRelayEvidenceWithoutRefreshingRelayHeartbeat)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        EXPECT_EQ(
                runtime.observe_evidence(
                               evidence(
                                       fixture.explorer_0,
                                       CapabilityKind::Exploration,
                                       2U),
                               10'500U)
                        .status,
                RuntimeAuthorityStatus::EvidenceApplied);
        EXPECT_EQ(
                runtime.tick(11'001U).status,
                RuntimeAuthorityStatus::FailoverCommitted);
    }

    TEST(RuntimeAuthorityTest, RunsExplorerTransitionThroughCommit)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        RoleCandidate candidate;
        candidate.base_role_epoch = runtime.roles().role_epoch;
        candidate.topology_epoch = runtime.topology().topology_epoch;
        candidate.assignments = runtime.roles().assignments;
        const auto target = std::find_if(
                candidate.assignments.begin(), candidate.assignments.end(),
                [&](const RoleAssignment & value) {
                    return value.identity == fixture.explorer_0;
                });
        ASSERT_NE(target, candidate.assignments.end());
        target->lifecycle = RoleLifecycle::Draining;

        ASSERT_TRUE(runtime.begin_role_transition(
                "explorer-quiesce-test", std::move(candidate)));
        ASSERT_NE(runtime.active_transition(), nullptr);
        EXPECT_EQ(
                runtime.active_transition()->state,
                RoleTransitionState::Prepared);
        ASSERT_TRUE(runtime.acknowledge_role_transition(
                "explorer-quiesce-test", fixture.explorer_0,
                RoleTransitionAckKind::Quiesced));
        EXPECT_EQ(
                runtime.active_transition()->state,
                RoleTransitionState::Quiescing);
        ASSERT_TRUE(runtime.acknowledge_role_transition(
                "explorer-quiesce-test", fixture.explorer_0,
                RoleTransitionAckKind::HandoffReady));
        EXPECT_EQ(
                runtime.active_transition()->state,
                RoleTransitionState::HandoffReady);
        ASSERT_TRUE(runtime.commit_role_transition("explorer-quiesce-test"));
        EXPECT_EQ(runtime.active_transition(), nullptr);
        ASSERT_NE(runtime.last_transition(), nullptr);
        EXPECT_EQ(runtime.last_transition()->state, RoleTransitionState::Committed);
        ASSERT_NE(find_assignment(runtime.roles(), fixture.explorer_0), nullptr);
        EXPECT_EQ(
                find_assignment(runtime.roles(), fixture.explorer_0)->lifecycle,
                RoleLifecycle::Draining);
    }

    TEST(RuntimeAuthorityTest, RollsBackExplorerTransitionWithoutRoleChange)
    {
        const Fixture fixture;
        auto runtime = authority(fixture);
        const auto role_epoch = runtime.roles().role_epoch;
        RoleCandidate candidate;
        candidate.base_role_epoch = role_epoch;
        candidate.topology_epoch = runtime.topology().topology_epoch;
        candidate.assignments = runtime.roles().assignments;
        const auto target = std::find_if(
                candidate.assignments.begin(), candidate.assignments.end(),
                [&](const RoleAssignment & value) {
                    return value.identity == fixture.explorer_0;
                });
        ASSERT_NE(target, candidate.assignments.end());
        target->lifecycle = RoleLifecycle::Draining;

        ASSERT_TRUE(runtime.begin_role_transition(
                "explorer-rollback-test", std::move(candidate)));
        ASSERT_TRUE(runtime.rollback_role_transition("explorer-rollback-test"));
        EXPECT_EQ(runtime.roles().role_epoch, role_epoch);
        EXPECT_EQ(runtime.active_transition(), nullptr);
        ASSERT_NE(runtime.last_transition(), nullptr);
        EXPECT_EQ(runtime.last_transition()->state, RoleTransitionState::RolledBack);
    }

}// namespace SwarmDataPlane::Test
