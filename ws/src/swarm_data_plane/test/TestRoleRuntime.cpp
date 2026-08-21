#include "swarm_data_plane/RuntimeSnapshotCache.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

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

        TopologySnapshot topology(const VehicleIdentity & identity, std::uint64_t epoch = 1U)
        {
            TopologySnapshot value;
            value.fleet_id = identity.fleet_id;
            value.topology_epoch = epoch;
            value.members = {ready_member(identity)};
            return value;
        }

        ServiceBudget relay_budget()
        {
            return {4'096U, 8'192U, 1'000'000U, 0U, 2U};
        }

        RoleAssignment explorer_assignment(
                const VehicleIdentity & identity,
                bool with_relay = false)
        {
            RoleAssignment assignment;
            assignment.identity = identity;
            assignment.capability_revision = 1U;
            assignment.primary_role = PrimaryRole::Explorer;
            assignment.lifecycle = RoleLifecycle::Active;
            if(with_relay) {
                assignment.services = {
                        {ServiceKind::Relay, ServiceLifecycle::Active,
                         relay_budget()}};
            }
            return assignment;
        }

        RoleSnapshot roles(
                const VehicleIdentity & identity,
                bool with_relay = false,
                std::uint64_t topology_epoch = 1U,
                std::uint64_t role_epoch = 2U)
        {
            RoleSnapshot snapshot;
            snapshot.fleet_id = identity.fleet_id;
            snapshot.topology_epoch = topology_epoch;
            snapshot.role_epoch = role_epoch;
            snapshot.assignments = {explorer_assignment(identity, with_relay)};
            return snapshot;
        }

        CapabilityEvidence healthy_evidence(
                const VehicleIdentity & identity,
                bool with_relay = false,
                std::uint64_t revision = 1U)
        {
            CapabilityEvidence evidence;
            evidence.identity = identity;
            evidence.evidence_revision = revision;
            evidence.effective_capabilities = {CapabilityKind::Exploration};
            evidence.vehicle_health = VehicleHealth::Healthy;
            evidence.resource_health = ResourceHealth::Healthy;
            if(with_relay) {
                evidence.effective_capabilities.push_back(
                        CapabilityKind::RelayForwarding);
                evidence.service_health = {
                        {ServiceKind::Relay, ResourceHealth::Healthy}};
            }
            std::sort(
                    evidence.effective_capabilities.begin(),
                    evidence.effective_capabilities.end());
            return evidence;
        }

        RoleTransition transition(
                const VehicleIdentity & identity,
                RoleTransitionState state = RoleTransitionState::Prepared)
        {
            RoleTransition value;
            value.transition_id = "transition-1";
            value.state = state;
            value.base_role_epoch = 2U;
            value.topology_epoch = 1U;
            value.target_assignments = {explorer_assignment(identity)};
            value.target_assignments.front().lifecycle = RoleLifecycle::Draining;
            value.changed_members = {identity};
            return value;
        }

    }// namespace

    TEST(RoleRuntimePolicyTest, AppliesRoleHealthCapabilityAndPrimaryRoleGates)
    {
        const auto identity = vehicle("explorer-0", 100U);
        auto evidence = healthy_evidence(identity);
        const auto snapshot = roles(identity);

        EXPECT_TRUE(evaluate_role_work_admission(
                snapshot, &evidence, nullptr, identity, PrimaryRole::Explorer));
        EXPECT_EQ(
                evaluate_role_work_admission(
                        snapshot, &evidence, nullptr, identity,
                        PrimaryRole::Relay)
                        .status,
                WorkAdmissionStatus::WrongPrimaryRole);

        evidence.vehicle_health = VehicleHealth::LowPower;
        EXPECT_EQ(
                evaluate_role_work_admission(
                        snapshot, &evidence, nullptr, identity)
                        .status,
                WorkAdmissionStatus::HealthBlocked);
        evidence.vehicle_health = VehicleHealth::Healthy;
        evidence.effective_capabilities.clear();
        EXPECT_EQ(
                evaluate_role_work_admission(
                        snapshot, &evidence, nullptr, identity)
                        .status,
                WorkAdmissionStatus::CapabilityMissing);
    }

    TEST(RoleRuntimePolicyTest, DistinguishesAuthorityAndLocalTransitionGates)
    {
        const auto identity = vehicle("explorer-0", 100U);
        const auto evidence = healthy_evidence(identity);
        const auto snapshot = roles(identity);
        auto pending = transition(identity);

        EXPECT_TRUE(evaluate_role_work_admission(
                snapshot, &evidence, &pending, identity));
        EXPECT_EQ(
                evaluate_role_work_admission(
                        snapshot, &evidence, &pending, identity, std::nullopt,
                        TransitionAdmissionMode::ChangedMember)
                        .status,
                WorkAdmissionStatus::TransitionBlocked);

        pending.state = RoleTransitionState::Quiescing;
        pending.acknowledgements = {
                {identity, RoleTransitionAckKind::Quiesced}};
        EXPECT_EQ(
                evaluate_role_work_admission(
                        snapshot, &evidence, &pending, identity)
                        .status,
                WorkAdmissionStatus::TransitionBlocked);
    }

    TEST(RoleRuntimePolicyTest, GatesRelayServiceIndependentlyFromExplorerRole)
    {
        const auto identity = vehicle("explorer-0", 100U);
        auto evidence = healthy_evidence(identity, true);
        const auto snapshot = roles(identity, true);

        EXPECT_TRUE(evaluate_service_work_admission(
                snapshot, &evidence, nullptr, identity, ServiceKind::Relay));
        evidence.service_health.front().health = ResourceHealth::LinkDegraded;
        EXPECT_EQ(
                evaluate_service_work_admission(
                        snapshot, &evidence, nullptr, identity,
                        ServiceKind::Relay)
                        .status,
                WorkAdmissionStatus::ServiceHealthBlocked);
    }

    TEST(RuntimeSnapshotCacheTest, AppliesMonotonicSnapshotsAndRejectsConflicts)
    {
        const auto identity = vehicle("explorer-0", 100U);
        RuntimeSnapshotCache cache(identity);

        EXPECT_TRUE(cache.apply_topology(topology(identity)));
        EXPECT_TRUE(cache.apply_role(roles(identity)));
        EXPECT_TRUE(cache.apply_evidence(healthy_evidence(identity)));
        EXPECT_TRUE(cache.aligned());
        EXPECT_TRUE(cache.role_admission(PrimaryRole::Explorer));

        EXPECT_EQ(
                cache.apply_role(roles(identity)).status,
                RuntimeSnapshotStatus::IgnoredDuplicate);
        auto conflicting = roles(identity);
        conflicting.assignments.front().lifecycle = RoleLifecycle::Draining;
        EXPECT_EQ(
                cache.apply_role(std::move(conflicting)).status,
                RuntimeSnapshotStatus::RejectedConflict);

        auto stale_evidence = healthy_evidence(identity, false, 0U);
        EXPECT_EQ(
                cache.apply_evidence(std::move(stale_evidence)).status,
                RuntimeSnapshotStatus::RejectedInvalid);
    }

    TEST(RuntimeSnapshotCacheTest, AcceptsAckProgressAndRejectsTransitionRegression)
    {
        const auto identity = vehicle("explorer-0", 100U);
        RuntimeSnapshotCache cache(identity);
        ASSERT_TRUE(cache.apply_topology(topology(identity)));
        ASSERT_TRUE(cache.apply_role(roles(identity)));
        ASSERT_TRUE(cache.apply_evidence(healthy_evidence(identity)));

        auto pending = transition(identity);
        ASSERT_TRUE(cache.apply_transition(pending));
        EXPECT_EQ(
                cache.role_admission(PrimaryRole::Explorer).status,
                WorkAdmissionStatus::TransitionBlocked);

        auto quiesced = pending;
        quiesced.state = RoleTransitionState::Quiescing;
        quiesced.acknowledgements = {
                {identity, RoleTransitionAckKind::Quiesced}};
        ASSERT_TRUE(cache.apply_transition(quiesced));
        EXPECT_EQ(
                cache.apply_transition(std::move(pending)).status,
                RuntimeSnapshotStatus::RejectedStale);
    }

    TEST(RuntimeSnapshotCacheTest, RejectsEvidenceForAnotherSession)
    {
        const auto identity = vehicle("explorer-0", 100U);
        RuntimeSnapshotCache cache(identity);
        EXPECT_EQ(
                cache.apply_evidence(
                             healthy_evidence(vehicle("explorer-0", 101U)))
                        .status,
                RuntimeSnapshotStatus::RejectedIdentity);
    }

}// namespace SwarmDataPlane::Test
