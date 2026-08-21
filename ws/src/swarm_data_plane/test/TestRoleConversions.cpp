#include "swarm_data_plane/ros/RoleConversions.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>

namespace SwarmDataPlane::Test {

    namespace {

        VehicleIdentity vehicle(const char * id, std::uint64_t boot_time_ns)
        {
            return {"fleet-a", id, {boot_time_ns, 1U}};
        }

        ServiceBudget relay_budget()
        {
            return {4'096U, 8'192U, 1'000'000U, 0U, 2U};
        }

        RoleAssignment relay_assignment(const VehicleIdentity & identity)
        {
            return {
                    identity,
                    3U,
                    PrimaryRole::Relay,
                    RoleLifecycle::Active,
                    {{ServiceKind::Relay, ServiceLifecycle::Active,
                      relay_budget()}}};
        }

        RoleSnapshot snapshot_fixture()
        {
            RoleSnapshot snapshot;
            snapshot.fleet_id = "fleet-a";
            snapshot.topology_epoch = 9U;
            snapshot.role_epoch = 4U;
            snapshot.assignments = {
                    {vehicle("drone-a", 100U), 2U, PrimaryRole::Explorer,
                     RoleLifecycle::Active, {}},
                    relay_assignment(vehicle("drone-b", 200U))};
            return snapshot;
        }

        RoleTransition transition_fixture()
        {
            const auto source = vehicle("drone-a", 100U);
            RoleTransition transition;
            transition.transition_id = "handoff-5";
            transition.state = RoleTransitionState::HandoffReady;
            transition.base_role_epoch = 4U;
            transition.topology_epoch = 9U;
            transition.target_assignments = {
                    relay_assignment(vehicle("drone-b", 200U))};
            transition.changed_members = {source};
            transition.acknowledgements = {
                    {source, RoleTransitionAckKind::Quiesced},
                    {source, RoleTransitionAckKind::HandoffReady}};
            return transition;
        }

    }// namespace

    TEST(RoleConversionsTest, CapabilityMessagesRoundTripCanonicalContent)
    {
        const auto identity = vehicle("drone-b", 200U);
        const CapabilityRegistration registration {
                identity,
                2U,
                {CapabilityKind::Exploration,
                 CapabilityKind::RelayForwarding}};
        swarm_data_interfaces::msg::CapabilityRegistration registration_message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_capability_registration(
                registration, registration_message, diagnostic))
                << diagnostic;
        const auto decoded_registration =
                Ros::decode_capability_registration(registration_message);
        ASSERT_TRUE(decoded_registration.success)
                << decoded_registration.diagnostic;
        ASSERT_TRUE(decoded_registration.registration.has_value());
        EXPECT_EQ(*decoded_registration.registration, registration);

        const CapabilityEvidence evidence {
                identity,
                3U,
                {CapabilityKind::Exploration,
                 CapabilityKind::RelayForwarding},
                VehicleHealth::Healthy,
                ResourceHealth::Healthy,
                {{ServiceKind::Relay, ResourceHealth::LinkDegraded}}};
        swarm_data_interfaces::msg::CapabilityEvidence evidence_message;
        ASSERT_TRUE(Ros::encode_capability_evidence(
                evidence, evidence_message, diagnostic))
                << diagnostic;
        const auto decoded_evidence =
                Ros::decode_capability_evidence(evidence_message);
        ASSERT_TRUE(decoded_evidence.success) << decoded_evidence.diagnostic;
        ASSERT_TRUE(decoded_evidence.evidence.has_value());
        EXPECT_EQ(*decoded_evidence.evidence, evidence);
    }

    TEST(RoleConversionsTest, SnapshotAndTransitionRoundTripCanonicalContent)
    {
        const auto snapshot = snapshot_fixture();
        swarm_data_interfaces::msg::RoleSnapshot snapshot_message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_role_snapshot(
                snapshot, snapshot_message, diagnostic))
                << diagnostic;
        const auto decoded_snapshot = Ros::decode_role_snapshot(snapshot_message);
        ASSERT_TRUE(decoded_snapshot.success) << decoded_snapshot.diagnostic;
        ASSERT_TRUE(decoded_snapshot.snapshot.has_value());
        EXPECT_EQ(*decoded_snapshot.snapshot, snapshot);

        const auto transition = transition_fixture();
        swarm_data_interfaces::msg::RoleTransitionDescriptor transition_message;
        ASSERT_TRUE(Ros::encode_role_transition(
                transition, transition_message, diagnostic))
                << diagnostic;
        const auto decoded_transition =
                Ros::decode_role_transition(transition_message);
        ASSERT_TRUE(decoded_transition.success)
                << decoded_transition.diagnostic;
        ASSERT_TRUE(decoded_transition.transition.has_value());
        EXPECT_EQ(*decoded_transition.transition, transition);
    }

    TEST(RoleConversionsTest, RejectsUnsupportedEnumsVersionsAndOrdering)
    {
        std::string diagnostic;
        swarm_data_interfaces::msg::CapabilityRegistration registration_message;
        const CapabilityRegistration registration {
                vehicle("drone-b", 200U),
                1U,
                {CapabilityKind::Exploration,
                 CapabilityKind::RelayForwarding}};
        ASSERT_TRUE(Ros::encode_capability_registration(
                registration, registration_message, diagnostic));
        std::reverse(
                registration_message.declared_capabilities.begin(),
                registration_message.declared_capabilities.end());
        EXPECT_FALSE(
                Ros::decode_capability_registration(registration_message).success);
        registration_message.protocol_version = 99U;
        EXPECT_FALSE(
                Ros::decode_capability_registration(registration_message).success);

        swarm_data_interfaces::msg::RoleSnapshot snapshot_message;
        ASSERT_TRUE(Ros::encode_role_snapshot(
                snapshot_fixture(), snapshot_message, diagnostic));
        snapshot_message.assignments.front().primary_role = 99U;
        EXPECT_FALSE(Ros::decode_role_snapshot(snapshot_message).success);

        ASSERT_TRUE(Ros::encode_role_snapshot(
                snapshot_fixture(), snapshot_message, diagnostic));
        std::reverse(
                snapshot_message.assignments.begin(),
                snapshot_message.assignments.end());
        EXPECT_FALSE(Ros::decode_role_snapshot(snapshot_message).success);

        swarm_data_interfaces::msg::RoleTransitionDescriptor transition_message;
        ASSERT_TRUE(Ros::encode_role_transition(
                transition_fixture(), transition_message, diagnostic));
        transition_message.acknowledgements.front().kind = 99U;
        EXPECT_FALSE(Ros::decode_role_transition(transition_message).success);
    }

    TEST(RoleConversionsTest, AppliesRuntimeLimitsBeforeNestedDecode)
    {
        std::string diagnostic;
        swarm_data_interfaces::msg::RoleSnapshot message;
        ASSERT_TRUE(Ros::encode_role_snapshot(
                snapshot_fixture(), message, diagnostic));

        RoleLimits limits;
        limits.max_assignments = 1U;
        EXPECT_FALSE(Ros::decode_role_snapshot(message, limits).success);
        EXPECT_FALSE(Ros::encode_role_snapshot(
                snapshot_fixture(), message, diagnostic, limits));

        ASSERT_TRUE(Ros::encode_role_snapshot(
                snapshot_fixture(), message, diagnostic));
        limits = RoleLimits {};
        limits.max_identity_bytes = 3U;
        EXPECT_FALSE(Ros::decode_role_snapshot(message, limits).success);

        CapabilityRegistration oversized_registration;
        oversized_registration.identity = {
                std::string(129U, 'f'), "drone-a", {100U, 1U}};
        oversized_registration.registration_generation = 1U;
        limits = RoleLimits {};
        limits.max_identity_bytes = 256U;
        swarm_data_interfaces::msg::CapabilityRegistration registration_message;
        EXPECT_FALSE(Ros::encode_capability_registration(
                oversized_registration, registration_message, diagnostic, limits));
    }

}// namespace SwarmDataPlane::Test
