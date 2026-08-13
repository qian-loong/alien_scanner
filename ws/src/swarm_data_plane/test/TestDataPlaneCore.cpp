#include "swarm_data_plane/AggregateContract.hpp"
#include "swarm_data_plane/MapUpdateIngress.hpp"
#include "swarm_data_plane/RoutedMapValidator.hpp"
#include "swarm_data_plane/RoutedResync.hpp"
#include "swarm_data_plane/TrustValidator.hpp"

#include "TestFixtures.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        PerceptionMapUpdate::PreparedUpdate keyframe(
                PerceptionMapUpdate::MapUpdateProducer & producer)
        {
            return producer.prepare(snapshot(
                    1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free}}));
        }

        AggregateManifest manifest_for(
                const PerceptionMapUpdate::MapUpdate & update,
                std::vector<ContributorRevision> contributors)
        {
            AggregateManifest manifest;
            manifest.aggregate = {
                    update.source.vehicle_id, update.source.mapper_session};
            manifest.aggregate_revision = update.new_revision;
            manifest.contributors = std::move(contributors);
            const auto hash = compute_manifest_hash(manifest);
            EXPECT_TRUE(hash.success) << hash.diagnostic;
            manifest.manifest_hash = hash.hash;
            return manifest;
        }

    }// namespace

    TEST(RoutedMapValidatorTest, ForwardingPreservesPayloadAndEnforcesBudgets)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
        const auto message = routed(shared_update(prepared), 1U, "message-1");

        const auto validation = validate_routed_map_update(message);
        ASSERT_TRUE(validation) << validation.diagnostic;

        const auto forwarded = forward_routed_map_update(message, 100U);
        ASSERT_TRUE(forwarded) << forwarded.diagnostic;
        ASSERT_TRUE(forwarded.message.has_value());
        EXPECT_EQ(forwarded.message->route.hop_count, 1U);
        EXPECT_EQ(forwarded.message->accumulated_forwarding_ns, 110U);
        EXPECT_EQ(forwarded.message->update, message.update);
        EXPECT_EQ(forwarded.message->payload_hash, message.payload_hash);

        auto expired = message;
        expired.accumulated_forwarding_ns = expired.validity_budget_ns;
        EXPECT_EQ(
                validate_routed_map_update(expired).status,
                EnvelopeValidationStatus::RejectedExpired);

        auto final_hop = message;
        final_hop.route.hop_count = 7U;
        EXPECT_FALSE(forward_routed_map_update(final_hop, 1U));
    }

    TEST(MapUpdateIngressTest, DuplicateDoesNotRefreshFreshnessAndConflictRequiresResync)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
        ASSERT_TRUE(producer.commit_published(prepared));

        MapUpdateIngress ingress;
        ASSERT_TRUE(ingress.admit_producer({"mapper_endpoint", {300U, 11U}}));
        ASSERT_TRUE(ingress.admit_source(prepared.update->source));
        const auto first = routed(shared_update(prepared), 1U, "message-1");
        const auto applied = ingress.receive(first, 100U);
        EXPECT_EQ(applied.status, IngressStatus::AppliedKeyframe);
        EXPECT_TRUE(applied.freshness_refreshed);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 100U);

        const auto duplicate = ingress.receive(first, 200U);
        EXPECT_EQ(duplicate.status, IngressStatus::IgnoredDuplicate);
        EXPECT_FALSE(duplicate.freshness_refreshed);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 100U);

        const auto delta = producer.prepare(snapshot(
                2U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied}}));
        ASSERT_TRUE(delta.update.has_value()) << delta.diagnostic;
        const auto conflicting_identity = routed(shared_update(delta), 2U, "message-1");
        const auto conflict = ingress.receive(conflicting_identity, 300U);
        EXPECT_EQ(conflict.status, IngressStatus::RejectedConflict);
        EXPECT_TRUE(conflict.resync_required);
        ASSERT_TRUE(ingress.map_applier().reconstructed_map().has_value());
        EXPECT_EQ(ingress.map_applier().reconstructed_map()->revision, 1U);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 100U);
    }

    TEST(MapUpdateIngressTest, SequenceGapRejectsDeltaAndCorrelatedKeyframeRecovers)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
        ASSERT_TRUE(producer.commit_published(prepared));

        MapUpdateIngress ingress;
        ASSERT_TRUE(ingress.admit_producer({"mapper_endpoint", {300U, 11U}}));
        ASSERT_TRUE(ingress.admit_source(prepared.update->source));
        ASSERT_EQ(
                ingress.receive(routed(shared_update(prepared), 1U, "message-1"), 100U).status,
                IngressStatus::AppliedKeyframe);

        const auto target = snapshot(
                2U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
        const auto delta = producer.prepare(target);
        ASSERT_TRUE(delta.update.has_value()) << delta.diagnostic;
        const auto gap = ingress.receive(
                routed(shared_update(delta), 3U, "message-3"), 200U);
        EXPECT_EQ(gap.status, IngressStatus::RejectedGap);
        EXPECT_TRUE(gap.resync_required);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 100U);

        ASSERT_TRUE(producer.request_keyframe("resync-1"));
        const auto recovery = producer.prepare(target);
        ASSERT_TRUE(recovery.update.has_value()) << recovery.diagnostic;
        ASSERT_EQ(recovery.update->kind, PerceptionMapUpdate::UpdateKind::Keyframe);
        ASSERT_EQ(recovery.update->correlation_id, "resync-1");
        EXPECT_EQ(
                ingress.receive(
                        routed(shared_update(recovery), 4U, "message-4"), 250U)
                        .status,
                IngressStatus::RejectedGap);
        ASSERT_TRUE(ingress.expect_resync("resync-1"));

        const auto delta_while_waiting = ingress.receive(
                routed(shared_update(delta), 5U, "message-5"), 275U);
        EXPECT_EQ(delta_while_waiting.status, IngressStatus::RejectedGap);
        EXPECT_TRUE(delta_while_waiting.resync_required);
        const auto recovered = ingress.receive(
                routed(shared_update(recovery), 6U, "message-6"), 300U);
        EXPECT_EQ(recovered.status, IngressStatus::AppliedKeyframe);
        EXPECT_FALSE(recovered.resync_required);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 300U);
        ASSERT_TRUE(ingress.map_applier().reconstructed_map().has_value());
        EXPECT_EQ(ingress.map_applier().reconstructed_map()->revision, 2U);
    }

    TEST(MapUpdateIngressTest, RejectsRetiredProducerAndOldRouteWithoutMutation)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value());

        MapUpdateIngress ingress;
        const ProducerIdentity original {"mapper_endpoint", {300U, 11U}};
        ASSERT_TRUE(ingress.admit_producer(original));
        ASSERT_TRUE(ingress.admit_source(prepared.update->source));
        ASSERT_EQ(
                ingress.receive(
                        routed(shared_update(prepared), 1U, "message-1", 2U),
                        100U)
                        .status,
                IngressStatus::AppliedKeyframe);

        auto old_route = routed(shared_update(prepared), 2U, "message-2", 1U);
        EXPECT_EQ(ingress.receive(old_route, 200U).status, IngressStatus::RejectedRoute);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 100U);

        ASSERT_TRUE(ingress.admit_producer({"mapper_endpoint", {301U, 1U}}));
        EXPECT_FALSE(ingress.admit_producer(original));
        EXPECT_EQ(
                ingress.receive(old_route, 300U).status,
                IngressStatus::RejectedAdmission);
    }

    TEST(RoutedResyncLedgerTest, ReusesCorrelationAndRejectsConflictingRequestIdentity)
    {
        const ProducerIdentity producer {"mapper_endpoint", {300U, 11U}};
        const auto source = snapshot(1U, {}).source;
        RoutedResyncLedger ledger(producer, 4U);

        RoutedResyncIntent intent;
        intent.target_producer = producer;
        intent.route_epoch = 4U;
        intent.request.requester = {"receiver", {500U, 3U}};
        intent.request.client_request_id = "request-1";
        intent.request.expected_source = source;
        intent.request.receiver_revision = 1U;
        intent.request.reason = PerceptionMapUpdate::ResyncReason::Gap;

        const auto first = ledger.accept(intent, source, 2U);
        ASSERT_TRUE(first.accepted) << first.diagnostic;
        EXPECT_FALSE(first.correlation_id.empty());
        const auto duplicate = ledger.accept(intent, source, 2U);
        EXPECT_TRUE(duplicate.accepted);
        EXPECT_EQ(duplicate.correlation_id, first.correlation_id);
        EXPECT_EQ(ledger.size(), 1U);

        auto conflict = intent;
        conflict.request.receiver_revision = 0U;
        const auto rejected = ledger.accept(conflict, source, 2U);
        EXPECT_FALSE(rejected.accepted);
        EXPECT_TRUE(rejected.correlation_id.empty());
        EXPECT_EQ(ledger.size(), 1U);

        auto stale_route = intent;
        stale_route.request.client_request_id = "request-2";
        stale_route.route_epoch = 3U;
        EXPECT_FALSE(ledger.accept(stale_route, source, 2U).accepted);
        EXPECT_EQ(ledger.size(), 1U);
    }

    TEST(AggregateIngressTest, CommitsManifestAtomicallyAcrossDeleteAndEpochRejoin)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto first = keyframe(producer);
        ASSERT_TRUE(first.update.has_value()) << first.diagnostic;
        ASSERT_TRUE(producer.commit_published(first));

        const PerceptionMapUpdate::SourceIdentity contributor_v1 {
                "drone_a", {10U, 1U}, 1U};
        AggregateMapUpdate aggregate_first {
                routed(shared_update(first), 1U, "aggregate-1"),
                manifest_for(
                        *first.update,
                        {{contributor_v1, 5U, first.update->content_hash, true}})};

        AggregateIngress ingress;
        ASSERT_TRUE(ingress.admit_producer({"mapper_endpoint", {300U, 11U}}));
        ASSERT_TRUE(ingress.admit_source(first.update->source));
        const auto applied_first = ingress.receive(aggregate_first, 100U);
        EXPECT_EQ(applied_first.map_result.status, IngressStatus::AppliedKeyframe);
        EXPECT_TRUE(applied_first.manifest_changed);
        ASSERT_TRUE(ingress.current_manifest().has_value());
        EXPECT_TRUE(ingress.current_manifest()->contributors.front().active);

        const auto second_snapshot = snapshot(
                2U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
        const auto second = producer.prepare(second_snapshot);
        ASSERT_TRUE(second.update.has_value()) << second.diagnostic;
        ASSERT_TRUE(producer.commit_published(second));
        AggregateMapUpdate aggregate_second {
                routed(shared_update(second), 2U, "aggregate-2"),
                manifest_for(
                        *second.update,
                        {{contributor_v1, 5U, first.update->content_hash, false}})};
        const auto applied_second = ingress.receive(aggregate_second, 200U);
        EXPECT_EQ(applied_second.map_result.status, IngressStatus::AppliedDelta);
        EXPECT_TRUE(applied_second.manifest_changed);
        ASSERT_TRUE(ingress.current_manifest().has_value());
        EXPECT_FALSE(ingress.current_manifest()->contributors.front().active);

        ASSERT_TRUE(producer.request_keyframe("aggregate-rejoin"));
        const auto third = producer.prepare(snapshot(
                3U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                 {{1, 0, 0}, PerceptionMapUpdate::CellState::Free}}));
        ASSERT_TRUE(third.update.has_value()) << third.diagnostic;
        const PerceptionMapUpdate::SourceIdentity contributor_v2 {
                "drone_a", {11U, 2U}, 2U};
        AggregateMapUpdate aggregate_third {
                routed(shared_update(third), 3U, "aggregate-3"),
                manifest_for(
                        *third.update,
                        {{contributor_v2, 1U, third.update->content_hash, true}})};
        const auto applied_third = ingress.receive(aggregate_third, 300U);
        EXPECT_EQ(applied_third.map_result.status, IngressStatus::AppliedKeyframe);
        EXPECT_TRUE(applied_third.manifest_changed);
        ASSERT_TRUE(ingress.current_manifest().has_value());
        EXPECT_EQ(
                ingress.current_manifest()->contributors.front().source,
                contributor_v2);
        EXPECT_TRUE(ingress.current_manifest()->contributors.front().active);
    }

    TEST(AggregateIngressTest, ConflictingManifestCannotPartiallyCommit)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value());
        const PerceptionMapUpdate::SourceIdentity contributor {
                "drone_a", {10U, 1U}, 1U};
        AggregateMapUpdate accepted {
                routed(shared_update(prepared), 1U, "aggregate-1"),
                manifest_for(
                        *prepared.update,
                        {{contributor, 5U, prepared.update->content_hash, true}})};

        AggregateIngress ingress;
        ASSERT_TRUE(ingress.admit_producer({"mapper_endpoint", {300U, 11U}}));
        ASSERT_TRUE(ingress.admit_source(prepared.update->source));
        ASSERT_TRUE(ingress.receive(accepted, 100U).manifest_changed);
        const auto committed = ingress.current_manifest();
        ASSERT_TRUE(committed.has_value());

        auto conflict = accepted;
        conflict.aggregate_update.message_id = "aggregate-2";
        conflict.aggregate_update.sequence = 2U;
        conflict.manifest.contributors.front().active = false;
        const auto hash = compute_manifest_hash(conflict.manifest);
        ASSERT_TRUE(hash.success);
        conflict.manifest.manifest_hash = hash.hash;
        const auto rejected = ingress.receive(conflict, 200U);
        EXPECT_EQ(rejected.map_result.status, IngressStatus::RejectedConflict);
        EXPECT_FALSE(rejected.manifest_changed);
        ASSERT_TRUE(ingress.current_manifest().has_value());
        EXPECT_EQ(*ingress.current_manifest(), *committed);
        ASSERT_TRUE(ingress.map_ingress().map_applier().reconstructed_map().has_value());
        EXPECT_EQ(ingress.map_ingress().map_applier().reconstructed_map()->revision, 1U);
    }

    TEST(StaticTrustValidatorTest, RejectsSpoofReplayAuthorityAndCredentialFaults)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value());
        const auto message = routed(shared_update(prepared), 1U, "message-1");
        StaticTrustValidator validator({{message.producer, 7U}});
        TrustEvidence valid {message.producer, CredentialState::Valid, 7U};

        EXPECT_TRUE(validator.validate(message, valid).accepted);
        EXPECT_EQ(
                validator.validate(message, valid).reason,
                TrustRejectionReason::Replay);

        auto next = message;
        next.sequence = 2U;
        auto spoof = valid;
        spoof.authenticated_producer = {"other", {300U, 11U}};
        EXPECT_EQ(
                validator.validate(next, spoof).reason,
                TrustRejectionReason::SpoofedProducer);
        auto expired = valid;
        expired.credential_state = CredentialState::Expired;
        EXPECT_EQ(
                validator.validate(next, expired).reason,
                TrustRejectionReason::ExpiredCredential);
        auto revoked = valid;
        revoked.credential_state = CredentialState::Revoked;
        EXPECT_EQ(
                validator.validate(next, revoked).reason,
                TrustRejectionReason::RevokedCredential);
        auto old_authority = valid;
        old_authority.authority_epoch = 6U;
        EXPECT_EQ(
                validator.validate(next, old_authority).reason,
                TrustRejectionReason::OldAuthorityEpoch);
        auto future_authority = valid;
        future_authority.authority_epoch = 8U;
        EXPECT_EQ(
                validator.validate(next, future_authority).reason,
                TrustRejectionReason::UnknownAuthorityEpoch);

        auto unknown = next;
        unknown.producer = {"unknown", {900U, 1U}};
        TrustEvidence unknown_evidence {
                unknown.producer, CredentialState::Valid, 7U};
        EXPECT_EQ(
                validator.validate(unknown, unknown_evidence).reason,
                TrustRejectionReason::UnknownProducer);
    }

    TEST(StaticTrustValidatorTest, RejectionLeavesDomainFingerprintUnchanged)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = keyframe(producer);
        ASSERT_TRUE(prepared.update.has_value());
        ASSERT_TRUE(producer.commit_published(prepared));
        const auto first = routed(shared_update(prepared), 1U, "message-1");

        MapUpdateIngress ingress;
        ASSERT_TRUE(ingress.admit_producer(first.producer));
        ASSERT_TRUE(ingress.admit_source(prepared.update->source));
        ASSERT_EQ(ingress.receive(first, 100U).status, IngressStatus::AppliedKeyframe);
        ASSERT_TRUE(ingress.map_applier().reconstructed_map().has_value());
        const auto fingerprint = ingress.map_applier().reconstructed_map()->content_hash;

        const auto delta = producer.prepare(snapshot(
                2U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied}}));
        ASSERT_TRUE(delta.update.has_value());
        const auto second = routed(shared_update(delta), 2U, "message-2");
        StaticTrustValidator validator({{second.producer, 7U}});
        TrustEvidence revoked {second.producer, CredentialState::Revoked, 7U};
        EXPECT_FALSE(validator.validate(second, revoked).accepted);

        ASSERT_TRUE(ingress.map_applier().reconstructed_map().has_value());
        EXPECT_EQ(ingress.map_applier().reconstructed_map()->revision, 1U);
        EXPECT_EQ(ingress.map_applier().reconstructed_map()->content_hash, fingerprint);
        EXPECT_EQ(ingress.last_fresh_receive_monotonic_ns(), 100U);
    }

}// namespace SwarmDataPlane::Test
