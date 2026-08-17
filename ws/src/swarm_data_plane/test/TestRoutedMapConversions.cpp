#include "swarm_data_plane/ros/AggregateConversions.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"
#include "swarm_data_plane/ros/RoutedResyncConversions.hpp"

#include "TestFixtures.hpp"

#include <gtest/gtest.h>

#include <rmw/types.h>

#include <string>

namespace SwarmDataPlane::Test {

    TEST(RoutedMapConversionsTest, RoundTripPreservesC3AndRoutingContracts)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot(
                1U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free}}));
        ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
        const auto original = routed(shared_update(prepared), 1U, "message-1");

        swarm_data_interfaces::msg::RoutedMapUpdate ros_message;
        ros_message.map_update.header.stamp.sec = 42;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_routed_map_update(
                original, ros_message, diagnostic)) << diagnostic;
        EXPECT_EQ(ros_message.map_update.header.stamp.sec, 42);

        const auto decoded = Ros::decode_routed_map_update(ros_message);
        ASSERT_TRUE(decoded.success) << decoded.diagnostic;
        ASSERT_TRUE(decoded.message.has_value());
        EXPECT_EQ(decoded.message->message_id, original.message_id);
        EXPECT_EQ(decoded.message->producer, original.producer);
        EXPECT_EQ(decoded.message->route.route_epoch, original.route.route_epoch);
        EXPECT_EQ(decoded.message->payload_hash, original.payload_hash);
        ASSERT_NE(decoded.message->update, nullptr);
        EXPECT_EQ(decoded.message->update->source, original.update->source);
        EXPECT_EQ(decoded.message->update->new_revision, original.update->new_revision);
        EXPECT_EQ(decoded.message->update->update_hash, original.update->update_hash);
        EXPECT_EQ(decoded.message->update->payload, original.update->payload);
    }

    TEST(RoutedMapConversionsTest, RejectsEnvelopeDriftBeforeAdmission)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot(1U, {}));
        ASSERT_TRUE(prepared.update.has_value());
        const auto original = routed(shared_update(prepared), 1U, "message-1");
        swarm_data_interfaces::msg::RoutedMapUpdate ros_message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_routed_map_update(
                original, ros_message, diagnostic));

        ros_message.payload_hash[0] ^= 0xFFU;
        EXPECT_FALSE(Ros::decode_routed_map_update(ros_message).success);
        ros_message.payload_hash[0] ^= 0xFFU;
        ros_message.correlation_id = "different";
        EXPECT_FALSE(Ros::decode_routed_map_update(ros_message).success);
    }

    TEST(QosProfilesTest, UsesOnlyStandardBoundedRmwPolicies)
    {
        const auto map = Ros::map_update_qos(4U).get_rmw_qos_profile();
        EXPECT_EQ(map.history, RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        EXPECT_EQ(map.depth, 4U);
        EXPECT_EQ(map.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(map.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);

        const auto state = Ros::state_health_qos(
                100'000'000U, 500'000'000U).get_rmw_qos_profile();
        EXPECT_EQ(state.depth, 1U);
        EXPECT_EQ(state.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        EXPECT_EQ(state.deadline.sec, 0U);
        EXPECT_EQ(state.deadline.nsec, 100'000'000U);
        EXPECT_EQ(state.lifespan.nsec, 500'000'000U);

        const auto diagnostic_qos = Ros::diagnostic_qos(16U).get_rmw_qos_profile();
        EXPECT_EQ(
                diagnostic_qos.reliability,
                RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        EXPECT_EQ(diagnostic_qos.depth, 16U);
    }

    TEST(RoutedResyncConversionsTest, RoundTripPreservesIdempotencyIdentity)
    {
        RoutedResyncIntent intent;
        intent.target_producer = {"mapper_endpoint", {300U, 11U}};
        intent.route_epoch = 4U;
        intent.request.requester = {"receiver", {500U, 3U}};
        intent.request.client_request_id = "request-1";
        intent.request.expected_source = snapshot(1U, {}).source;
        intent.request.receiver_revision = 1U;
        intent.request.reason = PerceptionMapUpdate::ResyncReason::Gap;

        swarm_data_interfaces::msg::ResyncIntent ros_intent;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_resync_intent(
                intent, ros_intent, diagnostic)) << diagnostic;
        const auto decoded_intent = Ros::decode_resync_intent(ros_intent);
        ASSERT_TRUE(decoded_intent.success) << decoded_intent.diagnostic;
        ASSERT_TRUE(decoded_intent.intent.has_value());
        EXPECT_EQ(decoded_intent.intent->target_producer, intent.target_producer);
        EXPECT_EQ(decoded_intent.intent->route_epoch, intent.route_epoch);
        EXPECT_EQ(decoded_intent.intent->request, intent.request);

        RoutedResyncLedger ledger(intent.target_producer, intent.route_epoch);
        PerceptionMapUpdate::VersionedContentDigest current_identity;
        current_identity.digest[0] = 42U;
        const auto ack = ledger.accept(
                intent, *intent.request.expected_source, 2U, current_identity);
        ASSERT_TRUE(ack.accepted) << ack.diagnostic;
        swarm_data_interfaces::msg::ResyncAck ros_ack;
        ASSERT_TRUE(Ros::encode_resync_ack(ack, ros_ack, diagnostic)) << diagnostic;
        const auto decoded_ack = Ros::decode_resync_ack(ros_ack);
        ASSERT_TRUE(decoded_ack.success) << decoded_ack.diagnostic;
        ASSERT_TRUE(decoded_ack.ack.has_value());
        EXPECT_EQ(decoded_ack.ack->correlation_id, ack.correlation_id);
        EXPECT_EQ(decoded_ack.ack->target_producer, ack.target_producer);
        EXPECT_EQ(decoded_ack.ack->current_source, ack.current_source);
        EXPECT_EQ(decoded_ack.ack->current_revision, ack.current_revision);
        EXPECT_EQ(decoded_ack.ack->current_content_identity, current_identity);
    }

    TEST(AggregateConversionsTest, RoundTripKeepsManifestAtomicWithMapRevision)
    {
        PerceptionMapUpdate::MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot(
                1U,
                {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free}}));
        ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
        AggregateManifest manifest;
        manifest.aggregate = {
                prepared.update->source.vehicle_id,
                prepared.update->source.mapper_session};
        manifest.aggregate_revision = prepared.update->new_revision;
        manifest.contributors = {{
                {"drone_a", {10U, 1U}, 1U},
                5U,
                prepared.update->content_hash,
                true}};
        const auto hash = compute_manifest_hash(manifest);
        ASSERT_TRUE(hash.success) << hash.diagnostic;
        manifest.manifest_hash = hash.hash;
        AggregateMapUpdate aggregate {
                routed(shared_update(prepared), 1U, "aggregate-1"), manifest};

        swarm_data_interfaces::msg::AggregateMapUpdate ros_message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_aggregate_map_update(
                aggregate, ros_message, diagnostic)) << diagnostic;
        const auto decoded = Ros::decode_aggregate_map_update(ros_message);
        ASSERT_TRUE(decoded.success) << decoded.diagnostic;
        ASSERT_TRUE(decoded.update.has_value());
        EXPECT_EQ(decoded.update->manifest, aggregate.manifest);
        EXPECT_EQ(
                decoded.update->aggregate_update.update->update_hash,
                aggregate.aggregate_update.update->update_hash);
        EXPECT_EQ(
                decoded.update->manifest.aggregate_revision,
                decoded.update->aggregate_update.update->new_revision);
    }

}// namespace SwarmDataPlane::Test
