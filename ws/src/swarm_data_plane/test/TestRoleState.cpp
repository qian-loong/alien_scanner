#include "swarm_data_plane/RoleState.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        VehicleIdentity vehicle(const char * id, std::uint64_t boot_time_ns)
        {
            return {"fleet-a", id, {boot_time_ns, 1U}};
        }

        MemberRecord member(
                const VehicleIdentity & identity,
                MembershipState state = MembershipState::Ready)
        {
            VehicleRegistration registration;
            registration.identity = identity;
            registration.registration_generation = 1U;
            registration.components = {{"mapper", {identity.session.boot_time_ns + 1U, 2U}}};
            SensorDescriptorIdentity sensor;
            sensor.sensor_id = "lidar";
            sensor.descriptor_hash[0] = 0xA5U;
            registration.sensors = {sensor};
            return {
                    registration,
                    state,
                    availability_for(state),
                    state == MembershipState::Ready
                            ? ResyncPrerequisites {true, true, true, true}
                            : ResyncPrerequisites {}};
        }

        TopologySnapshot topology_fixture()
        {
            const auto explorer = vehicle("drone-a", 100U);
            const auto relay = vehicle("drone-b", 200U);
            const auto aggregator = vehicle("drone-c", 300U);

            TopologySnapshot topology;
            topology.fleet_id = "fleet-a";
            topology.topology_epoch = 7U;
            topology.members = {
                    member(explorer), member(relay), member(aggregator)};
            topology.links = {
                    {"link-ab", 1U, explorer, relay, LinkHealth::Up,
                     1'000U, 1'000'000U, 100U},
                    {"link-bc", 1U, relay, aggregator, LinkHealth::Up,
                     1'000U, 1'000'000U, 100U}};
            for(const auto graph : {
                        LogicalGraphKind::Communication,
                        LogicalGraphKind::Control,
                        LogicalGraphKind::Map}) {
                topology.edges.push_back(
                        {graph, "link-ab", 1U, explorer, relay});
                topology.edges.push_back(
                        {graph, "link-bc", 1U, relay, aggregator});
            }
            topology.routes = {
                    {"route-control", LogicalGraphKind::Control,
                     explorer, aggregator, topology.topology_epoch, 1U, 3U,
                     10'000U, {{"link-ab", 1U}, {"link-bc", 1U}}},
                    {"route-map", LogicalGraphKind::Map,
                     explorer, aggregator, topology.topology_epoch, 1U, 3U,
                     10'000U, {{"link-ab", 1U}, {"link-bc", 1U}}}};
            std::sort(topology.members.begin(), topology.members.end());
            std::sort(topology.links.begin(), topology.links.end());
            std::sort(topology.edges.begin(), topology.edges.end());
            std::sort(topology.routes.begin(), topology.routes.end());
            EXPECT_TRUE(validate_topology_snapshot(topology));
            return topology;
        }

        CapabilityRegistration registration(
                const VehicleIdentity & identity,
                std::vector<CapabilityKind> capabilities,
                std::uint64_t generation = 1U)
        {
            return {identity, generation, std::move(capabilities)};
        }

        CapabilityEvidence evidence(
                const VehicleIdentity & identity,
                std::vector<CapabilityKind> capabilities,
                std::vector<ServiceHealthEvidence> services = {},
                std::uint64_t revision = 1U,
                VehicleHealth vehicle_health = VehicleHealth::Healthy,
                ResourceHealth resource_health = ResourceHealth::Healthy)
        {
            return {
                    identity,
                    revision,
                    std::move(capabilities),
                    vehicle_health,
                    resource_health,
                    std::move(services)};
        }

        ServiceBudget relay_budget()
        {
            return {4'096U, 8'192U, 1'000'000U, 0U, 2U};
        }

        ServiceBudget aggregation_budget()
        {
            return {8'192U, 65'536U, 2'000'000U, 8U, 2U};
        }

        RoleAssignment explorer_assignment(
                const VehicleIdentity & identity,
                std::uint64_t revision = 1U)
        {
            return {identity, revision, PrimaryRole::Explorer,
                    RoleLifecycle::Active, {}};
        }

        RoleAssignment relay_assignment(
                const VehicleIdentity & identity,
                std::uint64_t revision = 1U,
                PrimaryRole role = PrimaryRole::Relay)
        {
            return {
                    identity,
                    revision,
                    role,
                    RoleLifecycle::Active,
                    {{ServiceKind::Relay, ServiceLifecycle::Active,
                      relay_budget()}}};
        }

        RoleAssignment aggregator_assignment(
                const VehicleIdentity & identity,
                std::uint64_t revision = 1U)
        {
            return {
                    identity,
                    revision,
                    PrimaryRole::EdgeAggregator,
                    RoleLifecycle::Active,
                    {{ServiceKind::Aggregation, ServiceLifecycle::Active,
                      aggregation_budget()}}};
        }

        void register_healthy_fixture(
                RoleState & state,
                const TopologySnapshot & topology)
        {
            const auto & explorer = topology.members[0].registration.identity;
            const auto & relay = topology.members[1].registration.identity;
            const auto & aggregator = topology.members[2].registration.identity;
            ASSERT_TRUE(state.register_capabilities(
                    registration(explorer, {CapabilityKind::Exploration}), topology));
            ASSERT_TRUE(state.register_capabilities(
                    registration(
                            relay,
                            {CapabilityKind::RelayForwarding,
                             CapabilityKind::Exploration}),
                    topology));
            ASSERT_TRUE(state.register_capabilities(
                    registration(aggregator, {CapabilityKind::MapAggregation}),
                    topology));
            ASSERT_TRUE(state.update_capability_evidence(
                    evidence(explorer, {CapabilityKind::Exploration}), topology));
            ASSERT_TRUE(state.update_capability_evidence(
                    evidence(
                            relay,
                            {CapabilityKind::RelayForwarding,
                             CapabilityKind::Exploration},
                            {{ServiceKind::Relay, ResourceHealth::Healthy}}),
                    topology));
            ASSERT_TRUE(state.update_capability_evidence(
                    evidence(
                            aggregator,
                            {CapabilityKind::MapAggregation},
                            {{ServiceKind::Aggregation, ResourceHealth::Healthy}}),
                    topology));
        }

        void commit_initial_roles(
                RoleState & state,
                const TopologySnapshot & topology)
        {
            RoleCandidate candidate;
            candidate.base_role_epoch = state.snapshot().role_epoch;
            candidate.topology_epoch = topology.topology_epoch;
            candidate.assignments = {
                    explorer_assignment(topology.members[0].registration.identity),
                    relay_assignment(topology.members[1].registration.identity),
                    aggregator_assignment(topology.members[2].registration.identity)};
            ASSERT_TRUE(state.begin_transition("initial", std::move(candidate), topology));
            ASSERT_TRUE(state.commit_transition("initial", topology));
        }

    }// namespace

    TEST(RoleStateTest, FreezesDeclaredCapabilitiesAndRejectsStaleEvidence)
    {
        const auto topology = topology_fixture();
        const auto relay = topology.members[1].registration.identity;
        RoleState state(topology);

        ASSERT_TRUE(state.register_capabilities(
                registration(
                        relay,
                        {CapabilityKind::Exploration,
                         CapabilityKind::RelayForwarding}),
                topology));
        EXPECT_EQ(
                state.register_capabilities(
                             registration(relay, {CapabilityKind::RelayForwarding}, 2U),
                             topology)
                        .status,
                RoleStatus::RejectedConflict);
        ASSERT_TRUE(state.update_capability_evidence(
                evidence(relay, {CapabilityKind::RelayForwarding}), topology));
        EXPECT_EQ(
                state.update_capability_evidence(
                             evidence(
                                     relay,
                                     {CapabilityKind::RelayForwarding}, {}, 1U,
                                     VehicleHealth::LowPower),
                             topology)
                        .status,
                RoleStatus::RejectedConflict);
        EXPECT_EQ(
                state.update_capability_evidence(
                             evidence(relay, {CapabilityKind::MapAggregation}, {}, 2U),
                             topology)
                        .status,
                RoleStatus::RejectedCapability);
    }

    TEST(RoleStateTest, RegistersCapabilitiesBeforeReadyButRejectsRoleAndOldSession)
    {
        const auto joining_identity = vehicle("drone-j", 500U);
        TopologySnapshot joining;
        joining.fleet_id = "fleet-a";
        joining.topology_epoch = 1U;
        joining.members = {member(joining_identity, MembershipState::Joining)};
        ASSERT_TRUE(validate_topology_snapshot(joining));
        RoleState state(joining);
        ASSERT_TRUE(state.register_capabilities(
                registration(joining_identity, {CapabilityKind::Exploration}),
                joining));
        ASSERT_TRUE(state.update_capability_evidence(
                evidence(joining_identity, {CapabilityKind::Exploration}),
                joining));

        RoleCandidate candidate;
        candidate.base_role_epoch = state.snapshot().role_epoch;
        candidate.topology_epoch = joining.topology_epoch;
        candidate.assignments = {explorer_assignment(joining_identity)};
        EXPECT_EQ(
                state.begin_transition("joining-role", candidate, joining).status,
                RoleStatus::RejectedAdmission);

        auto resyncing = joining;
        resyncing.members = {
                member(joining_identity, MembershipState::Resyncing)};
        ASSERT_TRUE(validate_topology_snapshot(resyncing));
        ASSERT_TRUE(state.register_capabilities(
                registration(
                        joining_identity, {CapabilityKind::Exploration}, 2U),
                resyncing));
        ASSERT_TRUE(state.update_capability_evidence(
                evidence(
                        joining_identity, {CapabilityKind::Exploration}, {}, 2U),
                resyncing));
        EXPECT_EQ(
                state.begin_transition(
                             "resyncing-role", candidate, resyncing)
                        .status,
                RoleStatus::RejectedAdmission);

        const auto current_topology = topology_fixture();
        const auto old_session = vehicle("drone-b", 150U);
        RoleState current_state(current_topology);
        EXPECT_EQ(
                current_state.register_capabilities(
                                     registration(
                                             old_session,
                                             {CapabilityKind::RelayForwarding}),
                                     current_topology)
                        .status,
                RoleStatus::RejectedAdmission);
    }

    TEST(RoleStateTest, EnforcesRoleServiceCombinationAndBudgetLimits)
    {
        RoleSnapshot snapshot;
        snapshot.fleet_id = "fleet-a";
        snapshot.topology_epoch = 7U;
        snapshot.role_epoch = 2U;
        snapshot.assignments = {
                relay_assignment(vehicle("drone-b", 200U), 1U, PrimaryRole::Explorer)};
        EXPECT_TRUE(validate_role_snapshot(snapshot));

        snapshot.assignments.front().primary_role = PrimaryRole::Reserve;
        EXPECT_EQ(
                validate_role_snapshot(snapshot).status,
                RoleStatus::RejectedCombination);

        snapshot.assignments = {{
                vehicle("drone-d", 400U),
                1U,
                PrimaryRole::Reserve,
                RoleLifecycle::Active,
                {}}};
        EXPECT_TRUE(validate_role_snapshot(snapshot));

        snapshot.assignments = {aggregator_assignment(vehicle("drone-c", 300U))};
        snapshot.assignments.front().services.front().budget.memory_bytes =
                RoleLimits {}.max_memory_bytes + 1U;
        EXPECT_EQ(
                validate_role_snapshot(snapshot).status,
                RoleStatus::RejectedResourceLimit);
    }

    TEST(RoleStateTest, RejectsNonReadyMemberAndMissingGraphPrerequisitesAtomically)
    {
        auto topology = topology_fixture();
        const auto relay = topology.members[1].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);

        RoleCandidate relay_candidate;
        relay_candidate.base_role_epoch = state.snapshot().role_epoch;
        relay_candidate.topology_epoch = topology.topology_epoch;
        relay_candidate.assignments = {relay_assignment(relay)};

        auto no_communication = topology;
        no_communication.edges.erase(
                std::remove_if(
                        no_communication.edges.begin(), no_communication.edges.end(),
                        [](const GraphEdge & edge) {
                            return edge.graph == LogicalGraphKind::Communication;
                        }),
                no_communication.edges.end());
        EXPECT_TRUE(validate_topology_snapshot(no_communication));
        const auto before = state.snapshot();
        EXPECT_EQ(
                state.begin_transition(
                             "missing-comm", relay_candidate, no_communication)
                        .status,
                RoleStatus::RejectedPrerequisite);
        EXPECT_EQ(state.snapshot(), before);

        RoleCandidate aggregation_candidate;
        aggregation_candidate.base_role_epoch = state.snapshot().role_epoch;
        aggregation_candidate.topology_epoch = topology.topology_epoch;
        aggregation_candidate.assignments = {
                aggregator_assignment(
                        topology.members[2].registration.identity)};
        auto no_map_route = topology;
        no_map_route.routes.erase(
                std::remove_if(
                        no_map_route.routes.begin(), no_map_route.routes.end(),
                        [](const RouteDescriptor & route) {
                            return route.graph == LogicalGraphKind::Map;
                        }),
                no_map_route.routes.end());
        EXPECT_TRUE(validate_topology_snapshot(no_map_route));
        EXPECT_EQ(
                state.begin_transition(
                             "missing-map-route", aggregation_candidate,
                             no_map_route)
                        .status,
                RoleStatus::RejectedPrerequisite);
        EXPECT_EQ(state.snapshot(), before);

        auto not_ready = topology;
        not_ready.links.clear();
        not_ready.edges.clear();
        not_ready.routes.clear();
        not_ready.members[1] = member(relay, MembershipState::Joining);
        EXPECT_TRUE(validate_topology_snapshot(not_ready));
        EXPECT_EQ(
                state.begin_transition("not-ready", relay_candidate, not_ready).status,
                RoleStatus::RejectedAdmission);
        EXPECT_EQ(state.snapshot(), before);

        auto stale_topology = topology;
        stale_topology.topology_epoch = topology.topology_epoch - 1U;
        for(auto & route : stale_topology.routes) {
            route.topology_epoch = stale_topology.topology_epoch;
        }
        ASSERT_TRUE(validate_topology_snapshot(stale_topology));
        relay_candidate.topology_epoch = stale_topology.topology_epoch;
        EXPECT_EQ(
                state.begin_transition(
                             "stale-topology", relay_candidate, stale_topology)
                        .status,
                RoleStatus::RejectedStale);
        EXPECT_EQ(state.snapshot(), before);
    }

    TEST(RoleStateTest, CommitsMultiMemberHandoffAfterIdempotentBarrier)
    {
        const auto topology = topology_fixture();
        const auto explorer = topology.members[0].registration.identity;
        const auto relay = topology.members[1].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);
        commit_initial_roles(state, topology);
        const auto old_epoch = state.snapshot().role_epoch;

        RoleCandidate candidate;
        candidate.base_role_epoch = old_epoch;
        candidate.topology_epoch = topology.topology_epoch;
        candidate.assignments = {
                relay_assignment(relay, 1U, PrimaryRole::Explorer),
                aggregator_assignment(topology.members[2].registration.identity)};
        ASSERT_TRUE(state.begin_transition("handoff", std::move(candidate), topology));
        EXPECT_EQ(
                state.commit_transition("handoff", topology).status,
                RoleStatus::RejectedPrerequisite);

        for(const auto & identity : {explorer, relay}) {
            ASSERT_TRUE(state.acknowledge_transition(
                    "handoff", identity, RoleTransitionAckKind::Quiesced, topology));
            EXPECT_FALSE(state.can_accept_new_work(identity));
            EXPECT_EQ(
                    state.acknowledge_transition(
                                 "handoff", identity,
                                 RoleTransitionAckKind::Quiesced, topology)
                            .status,
                    RoleStatus::IgnoredDuplicate);
            ASSERT_TRUE(state.acknowledge_transition(
                    "handoff", identity, RoleTransitionAckKind::HandoffReady,
                    topology));
        }
        ASSERT_TRUE(state.commit_transition("handoff", topology));
        EXPECT_EQ(state.snapshot().role_epoch, old_epoch + 1U);
        EXPECT_EQ(state.snapshot().assignments.size(), 2U);
        EXPECT_EQ(state.snapshot().assignments.front().identity, relay);
        EXPECT_EQ(
                state.snapshot().assignments.front().primary_role,
                PrimaryRole::Explorer);
        EXPECT_EQ(state.last_transition()->state, RoleTransitionState::Committed);
    }

    TEST(RoleStateTest, RollbackRestoresQuiescedAssignmentWithoutEpochChange)
    {
        const auto topology = topology_fixture();
        const auto explorer = topology.members[0].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);
        commit_initial_roles(state, topology);
        const auto committed = state.snapshot();

        RoleCandidate candidate;
        candidate.base_role_epoch = committed.role_epoch;
        candidate.topology_epoch = topology.topology_epoch;
        candidate.assignments = {
                relay_assignment(topology.members[1].registration.identity),
                aggregator_assignment(topology.members[2].registration.identity)};
        ASSERT_TRUE(state.begin_transition("rollback", std::move(candidate), topology));
        ASSERT_TRUE(state.acknowledge_transition(
                "rollback", explorer, RoleTransitionAckKind::Quiesced, topology));
        EXPECT_FALSE(state.can_accept_new_work(explorer));
        ASSERT_TRUE(state.rollback_transition("rollback"));
        EXPECT_EQ(state.snapshot(), committed);
        EXPECT_TRUE(state.can_accept_new_work(explorer));
        EXPECT_EQ(state.last_transition()->state, RoleTransitionState::RolledBack);
    }

    TEST(RoleStateTest, HealthEvidenceImmediatelyShrinksEligibilityWithoutRoleCommit)
    {
        const auto topology = topology_fixture();
        const auto relay = topology.members[1].registration.identity;
        const auto aggregator = topology.members[2].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);
        commit_initial_roles(state, topology);
        const auto committed = state.snapshot();

        ASSERT_TRUE(state.update_capability_evidence(
                evidence(
                        relay, {}, {{ServiceKind::Relay, ResourceHealth::Healthy}},
                        2U),
                topology));
        EXPECT_EQ(state.snapshot(), committed);
        EXPECT_FALSE(state.can_accept_new_work(relay));
        EXPECT_FALSE(state.can_service_accept_new_work(relay, ServiceKind::Relay));

        ASSERT_TRUE(state.update_capability_evidence(
                evidence(
                        relay, {CapabilityKind::RelayForwarding},
                        {{ServiceKind::Relay, ResourceHealth::LinkDegraded}}, 3U),
                topology));
        EXPECT_TRUE(state.can_accept_new_work(relay));
        EXPECT_FALSE(state.can_service_accept_new_work(relay, ServiceKind::Relay));
        EXPECT_TRUE(state.can_service_accept_new_work(
                aggregator, ServiceKind::Aggregation));
        EXPECT_EQ(state.snapshot(), committed);
    }

    TEST(RoleStateTest, RejectsUnsafeVehicleAndResourceHealthForNewWork)
    {
        const auto topology = topology_fixture();
        const auto relay = topology.members[1].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);
        commit_initial_roles(state, topology);

        const auto update_and_expect_blocked = [&](
                                                   std::uint64_t revision,
                                                   VehicleHealth vehicle_health,
                                                   ResourceHealth resource_health) {
            ASSERT_TRUE(state.update_capability_evidence(
                    evidence(
                            relay, {CapabilityKind::RelayForwarding},
                            {{ServiceKind::Relay, ResourceHealth::Healthy}},
                            revision, vehicle_health, resource_health),
                    topology));
            EXPECT_FALSE(state.can_accept_new_work(relay));
            EXPECT_FALSE(state.can_service_accept_new_work(
                    relay, ServiceKind::Relay));
        };
        update_and_expect_blocked(
                2U, VehicleHealth::LowPower, ResourceHealth::Healthy);
        update_and_expect_blocked(
                3U, VehicleHealth::Failsafe, ResourceHealth::Healthy);
        update_and_expect_blocked(
                4U, VehicleHealth::Healthy,
                ResourceHealth::ComputeOverBudget);
        update_and_expect_blocked(
                5U, VehicleHealth::Unknown, ResourceHealth::Unknown);

        RoleState fresh(topology);
        ASSERT_TRUE(fresh.register_capabilities(
                registration(relay, {CapabilityKind::RelayForwarding}),
                topology));
        ASSERT_TRUE(fresh.update_capability_evidence(
                evidence(
                        relay, {CapabilityKind::RelayForwarding},
                        {{ServiceKind::Relay, ResourceHealth::Healthy}}, 1U,
                        VehicleHealth::LowPower),
                topology));
        RoleCandidate unsafe_assignment;
        unsafe_assignment.base_role_epoch = fresh.snapshot().role_epoch;
        unsafe_assignment.topology_epoch = topology.topology_epoch;
        unsafe_assignment.assignments = {relay_assignment(relay)};
        EXPECT_EQ(
                fresh.begin_transition(
                             "unsafe-assignment", std::move(unsafe_assignment),
                             topology)
                        .status,
                RoleStatus::RejectedHealth);
    }

    TEST(RoleStateTest, LostOldOwnerDoesNotBlockRevocationCommit)
    {
        const auto explorer = vehicle("drone-a", 100U);
        TopologySnapshot ready;
        ready.fleet_id = "fleet-a";
        ready.topology_epoch = 1U;
        ready.members = {member(explorer)};
        RoleState state(ready);
        ASSERT_TRUE(state.register_capabilities(
                registration(explorer, {CapabilityKind::Exploration}), ready));
        ASSERT_TRUE(state.update_capability_evidence(
                evidence(explorer, {CapabilityKind::Exploration}), ready));

        RoleCandidate initial;
        initial.base_role_epoch = state.snapshot().role_epoch;
        initial.topology_epoch = ready.topology_epoch;
        initial.assignments = {explorer_assignment(explorer)};
        ASSERT_TRUE(state.begin_transition("assign", std::move(initial), ready));
        ASSERT_TRUE(state.commit_transition("assign", ready));

        auto lost = ready;
        lost.topology_epoch = 2U;
        lost.members = {member(explorer, MembershipState::Lost)};
        ASSERT_TRUE(validate_topology_snapshot(lost));
        RoleCandidate revoke;
        revoke.base_role_epoch = state.snapshot().role_epoch;
        revoke.topology_epoch = lost.topology_epoch;
        ASSERT_TRUE(state.begin_transition("revoke-lost", std::move(revoke), lost));
        EXPECT_TRUE(state.commit_transition("revoke-lost", lost));
        EXPECT_TRUE(state.snapshot().assignments.empty());
    }

    TEST(RoleStateTest, UnchangedAssignmentsSurviveNewerHealthEvidence)
    {
        const auto topology = topology_fixture();
        const auto explorer = topology.members[0].registration.identity;
        const auto relay = topology.members[1].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);
        commit_initial_roles(state, topology);

        ASSERT_TRUE(state.update_capability_evidence(
                evidence(explorer, {CapabilityKind::Exploration}, {}, 2U),
                topology));
        RoleCandidate candidate;
        candidate.base_role_epoch = state.snapshot().role_epoch;
        candidate.topology_epoch = topology.topology_epoch;
        candidate.assignments = {
                state.snapshot().assignments[0],
                aggregator_assignment(topology.members[2].registration.identity)};
        ASSERT_TRUE(state.begin_transition(
                "remove-relay", std::move(candidate), topology));
        ASSERT_TRUE(state.acknowledge_transition(
                "remove-relay", relay, RoleTransitionAckKind::Quiesced, topology));
        ASSERT_TRUE(state.acknowledge_transition(
                "remove-relay", relay, RoleTransitionAckKind::HandoffReady,
                topology));
        EXPECT_TRUE(state.commit_transition("remove-relay", topology));
        EXPECT_EQ(state.snapshot().assignments.front().identity, explorer);
        EXPECT_EQ(state.snapshot().assignments.front().capability_revision, 1U);
        EXPECT_TRUE(state.can_accept_new_work(explorer));
    }

    TEST(RoleStateTest, RejectsUnknownAckAndRecentlyReusedTransitionId)
    {
        const auto topology = topology_fixture();
        const auto explorer = topology.members[0].registration.identity;
        RoleState state(topology);
        register_healthy_fixture(state, topology);
        commit_initial_roles(state, topology);

        RoleCandidate remove_explorer;
        remove_explorer.base_role_epoch = state.snapshot().role_epoch;
        remove_explorer.topology_epoch = topology.topology_epoch;
        remove_explorer.assignments = {
                relay_assignment(topology.members[1].registration.identity),
                aggregator_assignment(topology.members[2].registration.identity)};
        ASSERT_TRUE(state.begin_transition(
                "second", std::move(remove_explorer), topology));
        EXPECT_EQ(
                state.acknowledge_transition(
                             "second", explorer,
                             static_cast<RoleTransitionAckKind>(99U), topology)
                        .status,
                RoleStatus::RejectedInvalid);
        ASSERT_TRUE(state.rollback_transition("second"));

        RoleCandidate reused;
        reused.base_role_epoch = state.snapshot().role_epoch;
        reused.topology_epoch = topology.topology_epoch;
        reused.assignments = {relay_assignment(
                topology.members[1].registration.identity)};
        EXPECT_EQ(
                state.begin_transition("initial", std::move(reused), topology).status,
                RoleStatus::RejectedConflict);
    }

}// namespace SwarmDataPlane::Test
