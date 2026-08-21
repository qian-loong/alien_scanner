#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "swarm_data_plane/EdgeAggregator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

namespace SwarmDataPlane::Test {

    namespace {

        using namespace PerceptionMapUpdate;

        SourceIdentity source(std::string vehicle, std::uint64_t session)
        {
            return {std::move(vehicle), {session, 1U}, 1U};
        }

        CanonicalSnapshot snapshot(
                const SourceIdentity & identity,
                std::uint64_t revision,
                std::vector<CanonicalCell> cells,
                std::string frame = "map")
        {
            CanonicalSnapshot result;
            result.source = identity;
            result.geometry = {0.1, {0.0, 0.0, 0.0}, std::move(frame)};
            result.revision = revision;
            result.latest_commit = {
                    "lidar",
                    {identity.mapper_session.boot_time_ns + 100U, 2U},
                    Perception::Timestamp {static_cast<std::int64_t>(revision)},
                    "steady-sim",
                    static_cast<std::uint32_t>(cells.size())};
            result.cells = std::move(cells);
            result.geometry_fingerprint = ContentHasher::geometry_fingerprint(result.geometry);
            result.content_hash = ContentHasher::content_hash(
                    result.source, result.geometry_fingerprint, result.cells);
            return result;
        }

        RoutedMapUpdate wrap(
                const PreparedUpdate & prepared,
                const ProducerIdentity & producer,
                std::uint64_t sequence,
                std::uint64_t route_epoch = 1U)
        {
            RoutedMapUpdate result;
            result.message_id = producer.producer_id + "-" + std::to_string(sequence);
            result.producer = producer;
            result.sequence = sequence;
            result.correlation_id = prepared.update->correlation_id;
            result.priority = prepared.update->kind == UpdateKind::Delta
                                      ? LogicalPriority::MapDelta
                                      : LogicalPriority::MapKeyframe;
            result.origin = {"steady-sim", {900U, 1U}, sequence};
            result.validity_budget_ns = 1'000'000'000U;
            result.route = {route_epoch, 0U, 8U};
            result.payload_bytes = prepared.update->canonical_payload_bytes;
            result.payload_hash = prepared.update->update_hash;
            result.update = std::make_shared<const MapUpdate>(*prepared.update);
            return result;
        }

        EdgeAggregator make_aggregator(std::size_t max_aggregate_cells = 3'000'000U)
        {
            EdgeAggregatorConfig config;
            config.aggregate_source = {"edge-aggregator", {800U, 1U}, 1U};
            config.aggregate_producer = {"edge-aggregator", {810U, 1U}};
            config.origin_clock_session = {820U, 1U};
            config.sensor_session = {830U, 1U};
            EdgeAggregatorLimits limits;
            limits.max_aggregate_cells = max_aggregate_cells;
            return EdgeAggregator(std::move(config), std::move(limits));
        }

    }// namespace

    TEST(EdgeAggregatorTest, MergesContributorsAndPublishesSortedManifest)
    {
        const auto source_a = source("explorer-a", 100U);
        const auto source_b = source("explorer-b", 110U);
        ProducerIdentity producer_a {"mapper-a", {200U, 1U}};
        ProducerIdentity producer_b {"mapper-b", {210U, 1U}};
        MapUpdateProducer producer_a_core;
        MapUpdateProducer producer_b_core;
        const auto prepared_a = producer_a_core.prepare(snapshot(
                source_a,
                1U,
                {{{0, 0, 0}, CellState::Occupied}, {{2, 0, 0}, CellState::Free}}));
        const auto prepared_b = producer_b_core.prepare(snapshot(
                source_b,
                1U,
                {{{0, 0, 0}, CellState::Free}, {{1, 0, 0}, CellState::Occupied}}));
        ASSERT_TRUE(prepared_a.update.has_value());
        ASSERT_TRUE(prepared_b.update.has_value());
        ASSERT_TRUE(producer_a_core.commit_published(prepared_a));
        ASSERT_TRUE(producer_b_core.commit_published(prepared_b));

        auto aggregator = make_aggregator();
        const auto first = aggregator.receive(wrap(prepared_a, producer_a, 1U), 1'000U);
        ASSERT_TRUE(first) << first.diagnostic;
        const auto second = aggregator.receive(wrap(prepared_b, producer_b, 1U), 2'000U);
        ASSERT_TRUE(second) << second.diagnostic;
        ASSERT_EQ(second.update->manifest.contributors.size(), 2U);
        EXPECT_EQ(second.update->manifest.contributors[0].source.vehicle_id, "explorer-a");
        EXPECT_EQ(second.update->manifest.contributors[1].source.vehicle_id, "explorer-b");
        EXPECT_EQ(second.update->aggregate_update.update->source.vehicle_id, "edge-aggregator");
        EXPECT_EQ(second.update->manifest.aggregate_revision, 2U);

        AggregateIngress receiver;
        ASSERT_TRUE(receiver.admit_producer(aggregator.config().aggregate_producer));
        ASSERT_TRUE(receiver.admit_source(aggregator.config().aggregate_source));
        const auto first_received = receiver.receive(*first.update, 2'500U);
        EXPECT_EQ(first_received.map_result.status, IngressStatus::AppliedKeyframe)
                << first_received.map_result.diagnostic;
        const auto received = receiver.receive(*second.update, 3'000U);
        EXPECT_TRUE(received.map_result.status == IngressStatus::AppliedKeyframe
                    || received.map_result.status == IngressStatus::AppliedDelta)
                << received.map_result.diagnostic;
        ASSERT_EQ(aggregator.contributors().size(), 2U);
        EXPECT_EQ(aggregator.contributors()[0].cell_count, 2U);
        EXPECT_EQ(aggregator.contributors()[1].cell_count, 2U);
    }

    TEST(EdgeAggregatorTest, OccupiedWinsDeterministicallyOnVoxelConflict)
    {
        const auto source_a = source("a", 100U);
        const auto source_b = source("b", 110U);
        MapUpdateProducer producer_a;
        MapUpdateProducer producer_b;
        const auto a = producer_a.prepare(snapshot(
                source_a, 1U, {{{4, 5, 6}, CellState::Free}}));
        const auto b = producer_b.prepare(snapshot(
                source_b, 1U, {{{4, 5, 6}, CellState::Occupied}}));
        ASSERT_TRUE(a.update.has_value());
        ASSERT_TRUE(b.update.has_value());
        ASSERT_TRUE(producer_a.commit_published(a));
        ASSERT_TRUE(producer_b.commit_published(b));
        auto aggregator = make_aggregator();
        const auto first = aggregator.receive(
                wrap(a, {"mapper-a", {200U, 1U}}, 1U), 1U);
        ASSERT_TRUE(first) << first.diagnostic;
        const auto result = aggregator.receive(
                wrap(b, {"mapper-b", {210U, 1U}}, 1U), 2U);
        ASSERT_TRUE(result) << result.diagnostic;
        const auto & update = *result.update->aggregate_update.update;
        MapUpdateApplier applier;
        ASSERT_TRUE(applier.admit_source(update.source));
        ASSERT_EQ(
                applier.apply(*first.update->aggregate_update.update).status,
                ApplyUpdateStatus::AppliedKeyframe);
        const auto applied = applier.apply(update);
        EXPECT_TRUE(applied.status == ApplyUpdateStatus::AppliedKeyframe
                    || applied.status == ApplyUpdateStatus::AppliedDelta)
                << applied.diagnostic;
        ASSERT_TRUE(applier.reconstructed_map().has_value());
        const auto cells = applier.reconstructed_map()->cells.materialize();
        ASSERT_EQ(cells.size(), 1U);
        EXPECT_EQ(cells.front().state, CellState::Occupied);
    }

    TEST(EdgeAggregatorTest, GeometryMismatchDoesNotAdvanceAggregate)
    {
        const auto source_a = source("a", 100U);
        const auto source_b = source("b", 110U);
        MapUpdateProducer producer_a;
        MapUpdateProducer producer_b;
        const auto a = producer_a.prepare(snapshot(
                source_a, 1U, {{{0, 0, 0}, CellState::Occupied}}, "map-a"));
        const auto b = producer_b.prepare(snapshot(
                source_b, 1U, {{{1, 0, 0}, CellState::Occupied}}, "map-b"));
        ASSERT_TRUE(a.update.has_value());
        ASSERT_TRUE(b.update.has_value());
        ASSERT_TRUE(producer_a.commit_published(a));
        ASSERT_TRUE(producer_b.commit_published(b));
        auto aggregator = make_aggregator();
        ASSERT_TRUE(aggregator.receive(wrap(a, {"mapper-a", {200U, 1U}}, 1U), 1U));
        const auto result = aggregator.receive(
                wrap(b, {"mapper-b", {210U, 1U}}, 1U), 2U);
        EXPECT_EQ(result.status, EdgeAggregatorStatus::RejectedGeometry);
        EXPECT_EQ(aggregator.aggregate_revision(), 1U);
        ASSERT_TRUE(aggregator.last_update().has_value());
        EXPECT_EQ(aggregator.last_update()->manifest.aggregate_revision, 1U);
    }

    TEST(EdgeAggregatorTest, RemoveCreatesInactiveContributorManifestEntry)
    {
        const auto source_a = source("a", 100U);
        const ProducerIdentity identity {"mapper-a", {200U, 1U}};
        MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot(
                source_a,
                1U,
                {{{0, 0, 0}, CellState::Occupied}, {{1, 0, 0}, CellState::Free}}));
        ASSERT_TRUE(prepared.update.has_value());
        ASSERT_TRUE(producer.commit_published(prepared));

        auto remove = *prepared.update;
        remove.kind = UpdateKind::Remove;
        remove.base_revision = prepared.update->new_revision;
        remove.new_revision = prepared.update->new_revision + 1U;
        remove.revision_span = 1U;
        remove.base_content_hash = prepared.update->content_hash;
        remove.content_hash = {};
        remove.known_cell_count = 0U;
        remove.operation_count = 0U;
        remove.canonical_payload_bytes = 0U;
        remove.payload.clear();
        remove.latest_commit.observation_stamp = Perception::Timestamp {2};
        remove.latest_commit.changed_cell_count = 0U;
        remove.update_hash = ContentHasher::update_hash(remove);
        PreparedUpdate prepared_remove;
        prepared_remove.status = ProduceStatus::ProducedDelta;
        prepared_remove.update = remove;

        auto aggregator = make_aggregator();
        ASSERT_TRUE(aggregator.receive(wrap(prepared, identity, 1U), 1U));
        const auto result = aggregator.receive(wrap(prepared_remove, identity, 2U), 2U);
        ASSERT_TRUE(result) << result.diagnostic;
        ASSERT_EQ(result.update->manifest.contributors.size(), 1U);
        EXPECT_FALSE(result.update->manifest.contributors.front().active);
        EXPECT_EQ(result.update->manifest.contributors.front().revision, 2U);
        EXPECT_EQ(result.update->aggregate_update.update->new_revision, 2U);
    }

    TEST(EdgeAggregatorTest, DuplicateAndStaleSessionAreRejectedWithoutOutput)
    {
        const auto source_a = source("a", 100U);
        MapUpdateProducer producer;
        const auto prepared = producer.prepare(snapshot(
                source_a, 1U, {{{0, 0, 0}, CellState::Occupied}}));
        ASSERT_TRUE(prepared.update.has_value());
        ASSERT_TRUE(producer.commit_published(prepared));
        const ProducerIdentity identity {"mapper-a", {200U, 1U}};
        auto aggregator = make_aggregator();
        const auto first = aggregator.receive(wrap(prepared, identity, 1U), 1U);
        ASSERT_TRUE(first);
        const auto duplicate = aggregator.receive(wrap(prepared, identity, 1U), 2U);
        EXPECT_EQ(duplicate.status, EdgeAggregatorStatus::IgnoredDuplicate);
        EXPECT_EQ(aggregator.aggregate_revision(), 1U);

        MapUpdateProducer stale_producer;
        const auto stale = stale_producer.prepare(snapshot(
                source("a", 90U), 1U, {{{1, 0, 0}, CellState::Occupied}}));
        ASSERT_TRUE(stale.update.has_value());
        EXPECT_EQ(
                aggregator.receive(wrap(stale, identity, 2U), 3U).status,
                EdgeAggregatorStatus::RejectedContributor);
        EXPECT_EQ(aggregator.aggregate_revision(), 1U);
    }

    TEST(EdgeAggregatorTest, AggregateLimitRejectsWithoutMutatingContributorOrOutput)
    {
        const auto identity = source("a", 100U);
        const ProducerIdentity producer_identity {"mapper-a", {200U, 1U}};
        MapUpdateProducer producer;
        const auto first = producer.prepare(snapshot(
                identity, 1U, {{{0, 0, 0}, CellState::Occupied}}));
        ASSERT_TRUE(first.update.has_value());
        ASSERT_TRUE(producer.commit_published(first));
        const auto second = producer.prepare(snapshot(
                identity,
                2U,
                {{{0, 0, 0}, CellState::Occupied}, {{1, 0, 0}, CellState::Free}}));
        ASSERT_TRUE(second.update.has_value());

        auto aggregator = make_aggregator(1U);
        const auto accepted = aggregator.receive(wrap(first, producer_identity, 1U), 1U);
        ASSERT_TRUE(accepted) << accepted.diagnostic;
        const auto before = *aggregator.last_update();
        const auto rejected = aggregator.receive(wrap(second, producer_identity, 2U), 2U);
        EXPECT_EQ(rejected.status, EdgeAggregatorStatus::RejectedResource);
        EXPECT_FALSE(rejected.state_changed);
        EXPECT_EQ(aggregator.aggregate_revision(), 1U);
        ASSERT_TRUE(aggregator.last_update().has_value());
        EXPECT_EQ(aggregator.last_update()->manifest, before.manifest);
        ASSERT_EQ(aggregator.contributors().size(), 1U);
        EXPECT_EQ(aggregator.contributors().front().revision, 1U);
        EXPECT_EQ(aggregator.contributors().front().cell_count, 1U);
    }

    TEST(EdgeAggregatorTest, RouteEpochBarrierRecoversWithCorrelatedKeyframe)
    {
        const auto identity = source("a", 100U);
        const ProducerIdentity producer_identity {"mapper-a", {200U, 1U}};
        MapUpdateProducer producer;
        const auto first = producer.prepare(snapshot(
                identity, 1U, {{{0, 0, 0}, CellState::Occupied}}));
        ASSERT_TRUE(first.update.has_value());
        ASSERT_TRUE(producer.commit_published(first));
        const auto second_snapshot = snapshot(
                identity,
                2U,
                {{{0, 0, 0}, CellState::Occupied}, {{1, 0, 0}, CellState::Free}});
        ASSERT_TRUE(producer.request_keyframe("upstream-route-recovery"));
        const auto upstream_keyframe = producer.prepare(second_snapshot);
        ASSERT_TRUE(upstream_keyframe.update.has_value());
        ASSERT_EQ(upstream_keyframe.update->kind, UpdateKind::Keyframe);
        ASSERT_TRUE(producer.commit_published(upstream_keyframe));

        auto aggregator = make_aggregator();
        ASSERT_TRUE(aggregator.receive(wrap(first, producer_identity, 1U), 1U));
        const auto barrier = aggregator.receive(
                wrap(upstream_keyframe, producer_identity, 2U, 2U), 2U);
        EXPECT_EQ(barrier.status, EdgeAggregatorStatus::RejectedResync);
        EXPECT_EQ(aggregator.aggregate_revision(), 1U);
        ASSERT_EQ(aggregator.contributors().size(), 1U);
        EXPECT_TRUE(aggregator.contributors().front().resync_required);

        ASSERT_TRUE(producer.request_keyframe("route-2-recovery"));
        const auto recovery = producer.prepare(second_snapshot);
        ASSERT_TRUE(recovery.update.has_value());
        ASSERT_TRUE(producer.commit_published(recovery));
        ASSERT_TRUE(aggregator.expect_resync(identity, "route-2-recovery"));
        const auto recovered = aggregator.receive(
                wrap(recovery, producer_identity, 3U, 2U), 3U);
        ASSERT_TRUE(recovered) << recovered.diagnostic;
        EXPECT_EQ(aggregator.aggregate_revision(), 2U);
        ASSERT_EQ(aggregator.contributors().size(), 1U);
        EXPECT_EQ(aggregator.contributors().front().revision, 2U);
        EXPECT_FALSE(aggregator.contributors().front().resync_required);
    }

}// namespace SwarmDataPlane::Test
