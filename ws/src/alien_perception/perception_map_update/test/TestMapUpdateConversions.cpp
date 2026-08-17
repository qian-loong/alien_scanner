#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "perception_map_update/OctoMapViewAdapter.hpp"
#include "perception_map_update/ros/MapUpdateConversions.hpp"

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>

#include <gtest/gtest.h>

#include <memory>

namespace PerceptionMapUpdate::Test {

    namespace {

        CanonicalSnapshot snapshot()
        {
            CanonicalSnapshot result;
            result.source = {"drone_0", {100U, 7U}, 1U};
            result.geometry = {0.1, {0.0, 0.0, 0.0}, "drone_0/map"};
            result.revision = 1U;
            result.latest_commit = {
                    "lidar", {200U, 9U}, Perception::Timestamp {3'000'000U}, "sim", 1U};
            result.cells = {{{0, 0, 0}, CellState::Free}};
            result.geometry_fingerprint = ContentHasher::geometry_fingerprint(result.geometry);
            result.content_hash = ContentHasher::content_hash(
                    result.source, result.geometry_fingerprint, result.cells);
            return result;
        }

    }// namespace

    TEST(MapUpdateConversionsTest, MapUpdateRoundTripPreservesCoreContract)
    {
        MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot());
        ASSERT_TRUE(prepared.update.has_value()) << prepared.diagnostic;

        perception_interfaces::msg::MapUpdate message;
        message.header.stamp.sec = 42;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_map_update(*prepared.update, message, diagnostic)) << diagnostic;
        EXPECT_EQ(message.header.stamp.sec, 42);
        EXPECT_EQ(message.header.frame_id, "drone_0/map");

        const auto decoded = Ros::decode_map_update(message);
        ASSERT_TRUE(decoded.success) << decoded.diagnostic;
        ASSERT_TRUE(decoded.update.has_value());
        EXPECT_EQ(decoded.update->update_hash, prepared.update->update_hash);
        EXPECT_EQ(decoded.update->protocol_version, kProtocolVersion);
        EXPECT_EQ(decoded.update->content_identity, ContentIdentityDescriptor {});
        EXPECT_EQ(decoded.update->payload, prepared.update->payload);
        EXPECT_EQ(decoded.update->latest_commit, prepared.update->latest_commit);
    }

    TEST(MapUpdateConversionsTest, RejectsFrameLengthAndTimestampBoundaryFaults)
    {
        MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot());
        ASSERT_TRUE(prepared.update.has_value());
        perception_interfaces::msg::MapUpdate message;
        std::string diagnostic;
        ASSERT_TRUE(Ros::encode_map_update(*prepared.update, message, diagnostic));

        message.header.frame_id = "other";
        EXPECT_FALSE(Ros::decode_map_update(message).success);
        message.header.frame_id = message.source_map_frame;
        ++message.canonical_payload_bytes;
        EXPECT_FALSE(Ros::decode_map_update(message).success);
        --message.canonical_payload_bytes;
        message.content_identity.chunk_edge = 8U;
        EXPECT_FALSE(Ros::decode_map_update(message).success);
        message.content_identity.chunk_edge = kMerkleChunkEdge;
        message.latest_observation_stamp.nanosec = 1'000'000'000U;
        EXPECT_FALSE(Ros::decode_map_update(message).success);
    }

    TEST(MapUpdateConversionsTest, ResyncRequestDistinguishesBootstrapAndExactModes)
    {
        perception_interfaces::srv::RequestMapResync::Request message;
        message.requester_id = "receiver";
        message.requester_session_boot_time_ns = 500U;
        message.requester_session_random_suffix = 2U;
        message.client_request_id = "client-1";
        message.bootstrap_latest = true;
        message.reason = message.REASON_INITIAL_BASELINE;
        message.receiver_content_identity.scheme = static_cast<std::uint16_t>(
                ContentIdentityScheme::MerklePatriciaSha256V2);
        message.receiver_content_identity.chunk_edge = kMerkleChunkEdge;
        message.receiver_content_identity.coordinate_key_version =
                kMerkleCoordinateKeyVersion;
        message.receiver_content_identity.node_encoding_version =
                kMerkleNodeEncodingVersion;
        ResyncRequest request;
        std::string diagnostic;
        ASSERT_TRUE(Ros::decode_resync_request(message, request, {}, diagnostic)) << diagnostic;
        EXPECT_FALSE(request.expected_source.has_value());

        message.bootstrap_latest = false;
        message.expected_vehicle_id = "drone_0";
        message.expected_mapper_session_boot_time_ns = 100U;
        message.expected_mapper_session_random_suffix = 7U;
        message.expected_map_epoch = 1U;
        message.reason = message.REASON_GAP;
        ASSERT_TRUE(Ros::decode_resync_request(message, request, {}, diagnostic)) << diagnostic;
        ASSERT_TRUE(request.expected_source.has_value());
        EXPECT_EQ(request.expected_source->map_epoch, 1U);

        message.bootstrap_latest = true;
        message.expected_vehicle_id.clear();
        message.expected_mapper_session_boot_time_ns = 0U;
        message.expected_mapper_session_random_suffix = 0U;
        message.expected_map_epoch = 0U;
        message.reason = message.REASON_INITIAL_BASELINE;
        ASSERT_TRUE(Ros::decode_resync_request(message, request, {}, diagnostic)) << diagnostic;
        EXPECT_FALSE(request.expected_source.has_value());

        message.receiver_content_identity.chunk_edge = 8U;
        EXPECT_FALSE(Ros::decode_resync_request(message, request, {}, diagnostic));
        message.receiver_content_identity.chunk_edge = kMerkleChunkEdge;

        message.requester_id = std::string("\xC0\xAF", 2U);
        EXPECT_FALSE(Ros::decode_resync_request(message, request, {}, diagnostic));
    }

    TEST(OctoMapViewAdapterTest, MaterializesCanonicalStatesAndEnforcesCellLimit)
    {
        ReconstructedMap map;
        map.source = snapshot().source;
        map.geometry = snapshot().geometry;
        map.revision = 2U;
        map.cells = {
                {{0, 0, 0}, CellState::Free},
                {{1, 0, 0}, CellState::Occupied}};
        map.content_identity = {};

        octomap_msgs::msg::Octomap message;
        std::string diagnostic;
        ASSERT_TRUE(OctoMapViewAdapter::materialize(map, message, diagnostic)) << diagnostic;
        EXPECT_TRUE(message.binary);
        EXPECT_EQ(message.id, "OcTree");
        EXPECT_EQ(message.resolution, map.geometry.resolution_m);

        std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(message));
        ASSERT_NE(abstract, nullptr);
        const auto * tree = dynamic_cast<const octomap::OcTree *>(abstract.get());
        ASSERT_NE(tree, nullptr);
        const auto * free_node = tree->search(0.05F, 0.05F, 0.05F);
        const auto * occupied_node = tree->search(0.15F, 0.05F, 0.05F);
        ASSERT_NE(free_node, nullptr);
        ASSERT_NE(occupied_node, nullptr);
        EXPECT_FALSE(tree->isNodeOccupied(free_node));
        EXPECT_TRUE(tree->isNodeOccupied(occupied_node));

        MapUpdateLimits limits;
        limits.max_known_cells = 1U;
        EXPECT_FALSE(OctoMapViewAdapter::materialize(map, message, diagnostic, limits));
        EXPECT_FALSE(diagnostic.empty());
    }

}// namespace PerceptionMapUpdate::Test
