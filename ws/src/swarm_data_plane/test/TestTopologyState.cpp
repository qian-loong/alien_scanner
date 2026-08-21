#include "swarm_data_plane/TopologyState.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        VehicleIdentity identity(
                const std::string & vehicle_id,
                std::uint64_t boot_time_ns,
                std::uint32_t random_suffix = 1U)
        {
            return {"fleet-a", vehicle_id, {boot_time_ns, random_suffix}};
        }

        VehicleRegistration registration(
                const VehicleIdentity & vehicle,
                std::uint64_t generation = 1U,
                std::string sensor_id = "lidar")
        {
            SensorDescriptorIdentity sensor;
            sensor.sensor_id = std::move(sensor_id);
            sensor.descriptor_hash[0] = 0xA5U;
            return {
                    vehicle,
                    generation,
                    {{"mapper", {vehicle.session.boot_time_ns + 10U, 2U}},
                     {"lidar", {vehicle.session.boot_time_ns + 20U, 3U}}},
                    {sensor}};
        }

        void make_ready(TopologyState & state, const VehicleIdentity & vehicle)
        {
            ASSERT_TRUE(state.register_vehicle(registration(vehicle)));
            ASSERT_TRUE(state.transition_member(vehicle, MembershipState::Resyncing));
            ASSERT_TRUE(state.set_prerequisites(vehicle, {true, true, true, true}));
            ASSERT_TRUE(state.transition_member(vehicle, MembershipState::Ready));
        }

        TopologyCandidate two_vehicle_topology(
                const TopologySnapshot & snapshot,
                const VehicleIdentity & source,
                const VehicleIdentity & target)
        {
            const LinkDescriptor comm {
                    "link-comm", 1U, source, target, LinkHealth::Up,
                    1'000U, 1'000'000U, 100U};
            const LinkDescriptor control {
                    "link-control", 2U, source, target, LinkHealth::Up,
                    1'000U, 1'000'000U, 100U};
            const LinkDescriptor map {
                    "link-map", 3U, source, target, LinkHealth::Degraded,
                    2'000U, 500'000U, 500U};

            TopologyCandidate candidate;
            candidate.base_topology_epoch = snapshot.topology_epoch;
            candidate.links = {map, control, comm};
            candidate.edges = {
                    {LogicalGraphKind::Map, map.link_id, map.link_epoch,
                     source, target},
                    {LogicalGraphKind::Control, control.link_id, control.link_epoch,
                     source, target},
                    {LogicalGraphKind::Communication, comm.link_id, comm.link_epoch,
                     source, target}};
            candidate.routes = {
                    {"route-map", LogicalGraphKind::Map, source, target,
                     snapshot.topology_epoch, 1U, 2U, 10'000U,
                     {{map.link_id, map.link_epoch}}},
                    {"route-control", LogicalGraphKind::Control, source, target,
                     snapshot.topology_epoch, 1U, 2U, 10'000U,
                     {{control.link_id, control.link_epoch}}}};
            return candidate;
        }

    }// namespace

    TEST(TopologyStateTest, RegistrationAndReadyBarrierAreAtomicAndDeterministic)
    {
        TopologyState state(
                "fleet-a",
                {{"drone-b", 4U, 4U}, {"drone-a", 4U, 4U}});
        const auto drone_a = identity("drone-a", 100U);

        auto registration_value = registration(drone_a);
        std::reverse(
                registration_value.components.begin(), registration_value.components.end());
        const auto accepted = state.register_vehicle(registration_value);
        ASSERT_TRUE(accepted);
        ASSERT_EQ(state.snapshot().members.size(), 1U);
        EXPECT_EQ(
                state.snapshot().members.front().state,
                MembershipState::Joining);
        EXPECT_EQ(
                state.snapshot().members.front().registration.components.front().component_id,
                "lidar");

        const auto before_ready = state.snapshot();
        EXPECT_EQ(
                state.transition_member(drone_a, MembershipState::Ready).status,
                TopologyStatus::RejectedTransition);
        EXPECT_EQ(state.snapshot(), before_ready);

        ASSERT_TRUE(state.transition_member(drone_a, MembershipState::Resyncing));
        EXPECT_EQ(
                state.transition_member(drone_a, MembershipState::Ready).status,
                TopologyStatus::RejectedPrerequisite);
        EXPECT_TRUE(state.set_prerequisites(drone_a, {true, true, true, true}));
        ASSERT_TRUE(state.transition_member(drone_a, MembershipState::Ready));
        EXPECT_EQ(
                state.snapshot().members.front().availability,
                MemberAvailability::Live);
    }

    TEST(TopologyStateTest, RejectsUnknownAndDuplicateIdentityWithoutMutation)
    {
        TopologyState state("fleet-a", {{"drone-a", 4U, 4U}});
        const auto initial = state.snapshot();

        EXPECT_EQ(
                state.register_vehicle(registration(identity("drone-x", 100U))).status,
                TopologyStatus::RejectedAdmission);
        EXPECT_EQ(state.snapshot(), initial);

        auto duplicate = registration(identity("drone-a", 100U));
        duplicate.components.push_back(duplicate.components.front());
        EXPECT_EQ(
                state.register_vehicle(std::move(duplicate)).status,
                TopologyStatus::RejectedInvalid);
        EXPECT_EQ(state.snapshot(), initial);

        auto invalid_session = registration(identity("drone-a", 0U));
        EXPECT_EQ(
                state.register_vehicle(std::move(invalid_session)).status,
                TopologyStatus::RejectedAdmission);
        EXPECT_EQ(state.snapshot(), initial);
    }

    TEST(TopologyStateTest, AllowsComponentSessionRestartWithinFrozenInventory)
    {
        TopologyState state("fleet-a", {{"drone-a", 4U, 4U}});
        const auto vehicle = identity("drone-a", 100U);
        ASSERT_TRUE(state.register_vehicle(registration(vehicle)));

        auto restarted = registration(vehicle, 2U);
        restarted.components.front().session.boot_time_ns += 1'000U;
        ASSERT_TRUE(state.register_vehicle(std::move(restarted)));
        const auto * member = state.find_member(vehicle);
        ASSERT_NE(member, nullptr);
        const auto component = std::find_if(
                member->registration.components.begin(),
                member->registration.components.end(),
                [](const auto & value) { return value.component_id == "mapper"; });
        ASSERT_NE(component, member->registration.components.end());
        EXPECT_EQ(
                state.find_member(vehicle)->registration.registration_generation,
                2U);
        EXPECT_EQ(
                component->session.boot_time_ns,
                vehicle.session.boot_time_ns + 10U + 1'000U);
    }

    TEST(TopologyStateTest, RejectsDescriptorDriftAndFencesRetiredSession)
    {
        TopologyState state("fleet-a", {{"drone-a", 4U, 4U}});
        const auto old_session = identity("drone-a", 100U);
        make_ready(state, old_session);

        const auto before_drift = state.snapshot();
        auto drift = registration(old_session, 2U, "camera");
        EXPECT_EQ(
                state.register_vehicle(std::move(drift)).status,
                TopologyStatus::RejectedConflict);
        EXPECT_EQ(state.snapshot(), before_drift);

        EXPECT_EQ(
                state.register_vehicle(registration(identity("drone-a", 200U))).status,
                TopologyStatus::RejectedAdmission);

        ASSERT_TRUE(state.transition_member(old_session, MembershipState::Lost));
        const auto new_session = identity("drone-a", 200U);
        ASSERT_TRUE(state.register_vehicle(registration(new_session)));
        EXPECT_EQ(
                state.register_vehicle(registration(old_session)).status,
                TopologyStatus::RejectedStale);
        EXPECT_EQ(state.find_member("drone-a")->registration.identity, new_session);
    }

    TEST(TopologyStateTest, GracefulLeaveFreezesThenRemovesMember)
    {
        TopologyState state("fleet-a", {{"drone-a", 4U, 4U}});
        const auto first_session = identity("drone-a", 100U);
        make_ready(state, first_session);

        ASSERT_TRUE(state.transition_member(first_session, MembershipState::Draining));
        ASSERT_EQ(state.find_member(first_session)->availability, MemberAvailability::Frozen);
        ASSERT_TRUE(state.transition_member(first_session, MembershipState::Absent));
        ASSERT_EQ(state.find_member(first_session)->availability, MemberAvailability::Removed);

        const auto rejoined_session = identity("drone-a", 200U);
        ASSERT_TRUE(state.register_vehicle(registration(rejoined_session)));
        EXPECT_EQ(
                state.find_member("drone-a")->state,
                MembershipState::Joining);
    }

    TEST(TopologyStateTest, ThreeGraphsAndRouteEpochsCommitAsOneSnapshot)
    {
        TopologyState state(
                "fleet-a",
                {{"drone-a", 4U, 4U}, {"drone-b", 4U, 4U}});
        const auto drone_a = identity("drone-a", 100U);
        const auto drone_b = identity("drone-b", 200U);
        make_ready(state, drone_a);
        make_ready(state, drone_b);

        const auto old_snapshot = state.snapshot();
        auto candidate = two_vehicle_topology(old_snapshot, drone_a, drone_b);
        ASSERT_TRUE(state.replace_topology(std::move(candidate)));
        ASSERT_EQ(state.snapshot().links.size(), 3U);
        ASSERT_EQ(state.snapshot().routes.size(), 2U);
        EXPECT_LT(
                state.snapshot().routes[0].route_id,
                state.snapshot().routes[1].route_id);
        for(const auto & route : state.snapshot().routes) {
            EXPECT_EQ(route.topology_epoch, state.snapshot().topology_epoch);
            EXPECT_TRUE(state.validate_route(route, 0U, 1U));
            EXPECT_EQ(
                    state.validate_route(route, 1U, 1U).status,
                    TopologyStatus::RejectedRoute);
            EXPECT_EQ(
                    state.validate_route(route, 0U, route.validity_budget_ns).status,
                    TopologyStatus::RejectedRoute);
        }

        const auto committed = state.snapshot();
        TopologyCandidate stale;
        stale.base_topology_epoch = old_snapshot.topology_epoch;
        stale.links = committed.links;
        EXPECT_EQ(
                state.replace_topology(std::move(stale)).status,
                TopologyStatus::RejectedStale);
        EXPECT_EQ(state.snapshot(), committed);

        auto invalid = two_vehicle_topology(committed, drone_a, drone_b);
        invalid.routes.front().hops.front().link_epoch += 1U;
        EXPECT_EQ(
                state.replace_topology(std::move(invalid)).status,
                TopologyStatus::RejectedConflict);
        EXPECT_EQ(state.snapshot(), committed);

        auto stale_route = two_vehicle_topology(committed, drone_a, drone_b);
        stale_route.routes.front().route_epoch = 0U;
        EXPECT_EQ(
                state.replace_topology(std::move(stale_route)).status,
                TopologyStatus::RejectedStale);
        EXPECT_EQ(state.snapshot(), committed);
    }

    TEST(TopologyStateTest, RejectsRetiredLinkAndRouteEpochReplay)
    {
        TopologyState state(
                "fleet-a",
                {{"drone-a", 4U, 4U}, {"drone-b", 4U, 4U}});
        const auto drone_a = identity("drone-a", 100U);
        const auto drone_b = identity("drone-b", 200U);
        make_ready(state, drone_a);
        make_ready(state, drone_b);

        ASSERT_TRUE(state.replace_topology(
                two_vehicle_topology(state.snapshot(), drone_a, drone_b)));
        TopologyCandidate routes_removed;
        routes_removed.base_topology_epoch = state.snapshot().topology_epoch;
        routes_removed.links = state.snapshot().links;
        routes_removed.edges = state.snapshot().edges;
        ASSERT_TRUE(state.replace_topology(std::move(routes_removed)));
        const auto without_routes = state.snapshot();

        EXPECT_EQ(
                state.replace_topology(
                             two_vehicle_topology(state.snapshot(), drone_a, drone_b))
                        .status,
                TopologyStatus::RejectedStale);
        EXPECT_EQ(state.snapshot(), without_routes);

        TopologyCandidate links_removed;
        links_removed.base_topology_epoch = state.snapshot().topology_epoch;
        ASSERT_TRUE(state.replace_topology(std::move(links_removed)));
        const auto without_links = state.snapshot();
        EXPECT_EQ(
                state.replace_topology(
                             two_vehicle_topology(state.snapshot(), drone_a, drone_b))
                        .status,
                TopologyStatus::RejectedStale);
        EXPECT_EQ(state.snapshot(), without_links);
    }

    TEST(TopologyStateTest, RejectsPendingEndpointsAndCandidateResourceOverflow)
    {
        TopologyLimits limits;
        limits.max_links = 3U;
        TopologyState state(
                "fleet-a",
                {{"drone-a", 4U, 4U}, {"drone-b", 4U, 4U}},
                limits);
        const auto drone_a = identity("drone-a", 100U);
        const auto drone_b = identity("drone-b", 200U);
        make_ready(state, drone_a);
        ASSERT_TRUE(state.register_vehicle(registration(drone_b)));

        const auto before_pending_route = state.snapshot();
        EXPECT_EQ(
                state.replace_topology(
                             two_vehicle_topology(state.snapshot(), drone_a, drone_b))
                        .status,
                TopologyStatus::RejectedInvalid);
        EXPECT_EQ(state.snapshot(), before_pending_route);

        ASSERT_TRUE(state.transition_member(drone_b, MembershipState::Resyncing));
        ASSERT_TRUE(state.set_prerequisites(drone_b, {true, true, true, true}));
        ASSERT_TRUE(state.transition_member(drone_b, MembershipState::Ready));

        auto overflow = two_vehicle_topology(state.snapshot(), drone_a, drone_b);
        overflow.links.push_back({
                "link-extra", 1U, drone_a, drone_b, LinkHealth::Up,
                1'000U, 1'000'000U, 100U});
        const auto before_overflow = state.snapshot();
        EXPECT_EQ(
                state.replace_topology(std::move(overflow)).status,
                TopologyStatus::RejectedResourceLimit);
        EXPECT_EQ(state.snapshot(), before_overflow);

        ASSERT_TRUE(state.replace_topology(
                two_vehicle_topology(state.snapshot(), drone_a, drone_b)));
    }

    TEST(TopologyStateTest, LosingIntermediateMemberPrunesDependentRoutes)
    {
        TopologyState state(
                "fleet-a",
                {{"drone-a", 4U, 4U},
                 {"drone-b", 4U, 4U},
                 {"drone-c", 4U, 4U}});
        const auto drone_a = identity("drone-a", 100U);
        const auto drone_b = identity("drone-b", 200U);
        const auto drone_c = identity("drone-c", 300U);
        make_ready(state, drone_a);
        make_ready(state, drone_b);
        make_ready(state, drone_c);

        const LinkDescriptor first{
                "link-control-ab", 1U, drone_a, drone_b, LinkHealth::Up,
                1'000U, 1'000'000U, 100U};
        const LinkDescriptor second{
                "link-control-bc", 1U, drone_b, drone_c, LinkHealth::Up,
                1'000U, 1'000'000U, 100U};
        TopologyCandidate candidate;
        candidate.base_topology_epoch = state.snapshot().topology_epoch;
        candidate.links = {first, second};
        candidate.edges = {
                {LogicalGraphKind::Control, first.link_id, first.link_epoch,
                 drone_a, drone_b},
                {LogicalGraphKind::Control, second.link_id, second.link_epoch,
                 drone_b, drone_c}};
        candidate.routes = {{
                "route-control-via-b", LogicalGraphKind::Control,
                drone_a, drone_c, state.snapshot().topology_epoch, 1U, 3U,
                10'000U,
                {{first.link_id, first.link_epoch},
                 {second.link_id, second.link_epoch}}}};
        ASSERT_TRUE(state.replace_topology(std::move(candidate)));

        ASSERT_TRUE(state.transition_member(drone_b, MembershipState::Lost));
        const auto * lost_member = state.find_member(drone_b);
        ASSERT_NE(lost_member, nullptr);
        EXPECT_EQ(lost_member->state, MembershipState::Lost);
        EXPECT_TRUE(state.snapshot().links.empty());
        EXPECT_TRUE(state.snapshot().edges.empty());
        EXPECT_TRUE(state.snapshot().routes.empty());
    }

    TEST(TopologyStateTest, SwitchesControlRouteWithoutChangingMapRoute)
    {
        TopologyState state(
                "fleet-a",
                {{"drone-a", 4U, 4U}, {"drone-b", 4U, 4U}});
        const auto drone_a = identity("drone-a", 100U);
        const auto drone_b = identity("drone-b", 200U);
        make_ready(state, drone_a);
        make_ready(state, drone_b);
        ASSERT_TRUE(state.replace_topology(
                two_vehicle_topology(state.snapshot(), drone_a, drone_b)));

        const auto previous = state.snapshot();
        auto rerouted = two_vehicle_topology(previous, drone_a, drone_b);
        auto control_link = std::find_if(
                rerouted.links.begin(), rerouted.links.end(),
                [](const LinkDescriptor & link) {
                    return link.link_id == "link-control";
                });
        ASSERT_NE(control_link, rerouted.links.end());
        control_link->link_id = "link-control-alt";
        control_link->link_epoch = 1U;
        auto control_edge = std::find_if(
                rerouted.edges.begin(), rerouted.edges.end(),
                [](const GraphEdge & edge) {
                    return edge.graph == LogicalGraphKind::Control;
                });
        ASSERT_NE(control_edge, rerouted.edges.end());
        control_edge->link_id = control_link->link_id;
        control_edge->link_epoch = control_link->link_epoch;
        auto control_route = std::find_if(
                rerouted.routes.begin(), rerouted.routes.end(),
                [](const RouteDescriptor & route) {
                    return route.graph == LogicalGraphKind::Control;
                });
        ASSERT_NE(control_route, rerouted.routes.end());
        control_route->route_epoch = 2U;
        control_route->hops = {{control_link->link_id, control_link->link_epoch}};

        ASSERT_TRUE(state.replace_topology(std::move(rerouted)));
        const auto current_control = std::find_if(
                state.snapshot().routes.begin(), state.snapshot().routes.end(),
                [](const RouteDescriptor & route) {
                    return route.graph == LogicalGraphKind::Control;
                });
        const auto current_map = std::find_if(
                state.snapshot().routes.begin(), state.snapshot().routes.end(),
                [](const RouteDescriptor & route) {
                    return route.graph == LogicalGraphKind::Map;
                });
        const auto previous_map = std::find_if(
                previous.routes.begin(), previous.routes.end(),
                [](const RouteDescriptor & route) {
                    return route.graph == LogicalGraphKind::Map;
                });
        ASSERT_NE(current_control, state.snapshot().routes.end());
        ASSERT_NE(current_map, state.snapshot().routes.end());
        ASSERT_NE(previous_map, previous.routes.end());
        EXPECT_EQ(current_control->route_epoch, 2U);
        ASSERT_EQ(current_control->hops.size(), 1U);
        EXPECT_EQ(current_control->hops.front().link_id, "link-control-alt");
        EXPECT_EQ(current_map->route_epoch, previous_map->route_epoch);
        EXPECT_EQ(current_map->hops, previous_map->hops);
    }

}// namespace SwarmDataPlane::Test
