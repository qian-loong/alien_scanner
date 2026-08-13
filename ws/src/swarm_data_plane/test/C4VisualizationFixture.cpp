#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "swarm_data_plane/AggregateContract.hpp"
#include "swarm_data_plane/MapUpdateIngress.hpp"
#include "swarm_data_plane/RoutedMapValidator.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        using CanonicalCell = PerceptionMapUpdate::CanonicalCell;
        using CanonicalSnapshot = PerceptionMapUpdate::CanonicalSnapshot;
        using MapUpdate = PerceptionMapUpdate::MapUpdate;
        using Marker = visualization_msgs::msg::Marker;
        using MarkerArray = visualization_msgs::msg::MarkerArray;

        struct Color {
            float red = 1.0F;
            float green = 1.0F;
            float blue = 1.0F;
            float alpha = 1.0F;
        };

        constexpr Color kGreen {0.2F, 0.9F, 0.3F, 1.0F};
        constexpr Color kBlue {0.2F, 0.55F, 1.0F, 1.0F};
        constexpr Color kYellow {1.0F, 0.75F, 0.15F, 1.0F};
        constexpr Color kRed {1.0F, 0.15F, 0.12F, 1.0F};
        constexpr Color kPurple {0.75F, 0.35F, 1.0F, 1.0F};
        constexpr Color kCyan {0.15F, 0.9F, 0.9F, 1.0F};

        struct ContributorView {
            std::string id;
            std::vector<CanonicalCell> cells;
            Color color;
            std::uint64_t revision = 0U;
        };

        struct StageData {
            std::string title;
            std::string detail;
            std::vector<CanonicalCell> aggregate_cells;
            std::vector<ContributorView> contributors;
            bool fault = false;
        };

        struct Scene {
            std::string prefix;
            double origin_x = 0.0;
            std::vector<StageData> stages;
        };

        void require(bool condition, const std::string & diagnostic)
        {
            if(!condition) {
                throw std::runtime_error(diagnostic);
            }
        }

        CanonicalSnapshot snapshot(
                std::string vehicle_id,
                Perception::SessionID mapper_session,
                std::uint64_t map_epoch,
                std::uint64_t revision,
                std::vector<CanonicalCell> cells)
        {
            CanonicalSnapshot result;
            result.source = {std::move(vehicle_id), mapper_session, map_epoch};
            result.geometry = {
                    0.8,
                    {0.0, 0.0, 0.0},
                    result.source.vehicle_id + "/map"};
            result.revision = revision;
            result.latest_commit = {
                    "c4_visualization",
                    {900U, 1U},
                    Perception::Timestamp {7'000'000U
                                            + static_cast<std::int64_t>(revision)},
                    "c4-sim-clock",
                    static_cast<std::uint32_t>(cells.size())};
            result.cells = std::move(cells);
            result.geometry_fingerprint = PerceptionMapUpdate::ContentHasher::geometry_fingerprint(
                    result.geometry);
            result.content_hash = PerceptionMapUpdate::ContentHasher::content_hash(
                    result.source, result.geometry_fingerprint, result.cells);
            return result;
        }

        std::shared_ptr<const MapUpdate> shared_update(
                const PerceptionMapUpdate::PreparedUpdate & prepared)
        {
            require(prepared.update.has_value(), prepared.diagnostic);
            return std::make_shared<const MapUpdate>(*prepared.update);
        }

        LogicalPriority priority_for(PerceptionMapUpdate::UpdateKind kind)
        {
            switch(kind) {
                case PerceptionMapUpdate::UpdateKind::Keyframe:
                case PerceptionMapUpdate::UpdateKind::Remove:
                    return LogicalPriority::MapKeyframe;
                case PerceptionMapUpdate::UpdateKind::Delta:
                    return LogicalPriority::MapDelta;
                case PerceptionMapUpdate::UpdateKind::Summary:
                    return LogicalPriority::Summary;
            }
            return LogicalPriority::Diagnostic;
        }

        RoutedMapUpdate routed(
                std::shared_ptr<const MapUpdate> update,
                ProducerIdentity producer,
                std::uint64_t sequence,
                std::string message_id,
                std::uint16_t ttl_hops = 8U)
        {
            RoutedMapUpdate message;
            message.message_id = std::move(message_id);
            message.producer = std::move(producer);
            message.sequence = sequence;
            message.correlation_id = update->correlation_id;
            message.priority = priority_for(update->kind);
            message.origin = {"c4-sim-clock", {901U, 1U}, 8'000'000U};
            message.validity_budget_ns = 1'000'000'000U;
            message.accumulated_forwarding_ns = 10U;
            message.route = {1U, 0U, ttl_hops};
            message.payload_bytes = update->canonical_payload_bytes;
            message.payload_hash = update->update_hash;
            message.update = std::move(update);
            return message;
        }

        AggregateManifest manifest_for(
                const MapUpdate & update,
                std::vector<ContributorRevision> contributors)
        {
            AggregateManifest manifest;
            manifest.aggregate = {
                    update.source.vehicle_id,
                    update.source.mapper_session};
            manifest.aggregate_revision = update.new_revision;
            manifest.contributors = std::move(contributors);
            const auto hash = compute_manifest_hash(manifest);
            require(hash.success, hash.diagnostic);
            manifest.manifest_hash = hash.hash;
            return manifest;
        }

        Marker base_marker(
                const std::string & frame,
                const std::string & ns,
                std::int32_t id,
                std::int32_t type,
                Color color)
        {
            Marker marker;
            marker.header.frame_id = frame;
            marker.ns = ns;
            marker.id = id;
            marker.type = type;
            marker.action = Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.color.r = color.red;
            marker.color.g = color.green;
            marker.color.b = color.blue;
            marker.color.a = color.alpha;
            return marker;
        }

        void append_status(
                MarkerArray & output,
                const std::string & frame,
                const std::string & prefix,
                double x,
                const StageData & stage)
        {
            const Color color = stage.fault
                                        ? kRed
                                : stage.title.find("RESYNC") != std::string::npos
                                          && stage.title.find("RECOVERED") == std::string::npos
                                        ? kYellow
                                        : kGreen;
            auto marker = base_marker(
                    frame, prefix + "/status", 0, Marker::TEXT_VIEW_FACING, color);
            marker.pose.position.x = x;
            marker.pose.position.y = 5.0;
            marker.pose.position.z = 1.8;
            marker.scale.z = 0.48;
            marker.text = stage.title + "\n" + stage.detail;
            output.markers.push_back(std::move(marker));
        }

        void append_map(
                MarkerArray & output,
                const std::string & frame,
                const std::string & ns,
                double x,
                double y,
                const std::vector<CanonicalCell> & cells,
                Color color)
        {
            auto marker = base_marker(frame, ns, 0, Marker::CUBE_LIST, color);
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.scale.x = 0.72;
            marker.scale.y = 0.72;
            marker.scale.z = 0.72;
            for(const auto & cell : cells) {
                geometry_msgs::msg::Point point;
                point.x = static_cast<double>(cell.index.x) * 1.1;
                point.y = static_cast<double>(cell.index.y) * 1.1;
                point.z = cell.state == PerceptionMapUpdate::CellState::Occupied
                                  ? 0.65
                                  : 0.15;
                marker.points.push_back(point);
                std_msgs::msg::ColorRGBA point_color;
                point_color.r = color.red;
                point_color.g = color.green;
                point_color.b = color.blue;
                point_color.a = cell.state == PerceptionMapUpdate::CellState::Occupied
                                        ? color.alpha
                                        : 0.35F;
                marker.colors.push_back(point_color);
            }
            output.markers.push_back(std::move(marker));
        }

        void append_timeline(
                MarkerArray & output,
                const std::string & frame,
                const std::string & prefix,
                double x,
                std::size_t current_stage,
                const std::vector<StageData> & stages)
        {
            auto line = base_marker(frame, prefix + "/timeline", 0, Marker::LINE_STRIP, kCyan);
            line.scale.x = 0.08;
            for(std::size_t index = 0U; index < stages.size(); ++index) {
                geometry_msgs::msg::Point point;
                point.x = x - 4.5 + static_cast<double>(index) * 3.0;
                point.y = 0.0;
                point.z = 0.1;
                line.points.push_back(point);
            }
            output.markers.push_back(std::move(line));

            for(std::size_t index = 0U; index < stages.size(); ++index) {
                const auto & stage = stages[index];
                const bool active = index == current_stage;
                const Color color = stage.fault
                                            ? kRed
                                    : active && stage.title.find("RESYNC") != std::string::npos
                                                  && stage.title.find("RECOVERED") == std::string::npos
                                            ? kYellow
                                            : index <= current_stage
                                                    ? kGreen
                                                    : Color {0.35F, 0.4F, 0.45F, 1.0F};
                auto marker = base_marker(
                        frame, prefix + "/events", static_cast<std::int32_t>(index),
                        Marker::SPHERE, color);
                marker.pose.position.x = x - 4.5 + static_cast<double>(index) * 3.0;
                marker.pose.position.y = 0.0;
                marker.pose.position.z = 0.1;
                marker.scale.x = active ? 0.8 : 0.55;
                marker.scale.y = marker.scale.x;
                marker.scale.z = marker.scale.x;
                output.markers.push_back(std::move(marker));

                auto label = base_marker(
                        frame, prefix + "/labels", static_cast<std::int32_t>(index),
                        Marker::TEXT_VIEW_FACING, color);
                label.pose.position.x = x - 4.5 + static_cast<double>(index) * 3.0;
                label.pose.position.y = -0.8;
                label.pose.position.z = 0.1;
                label.scale.z = 0.28;
                label.text = stage.title;
                output.markers.push_back(std::move(label));
            }
        }

        MarkerArray render_stage(
                const Scene & scene,
                std::size_t stage_index,
                const std::string & frame)
        {
            const auto & stage = scene.stages.at(stage_index);
            MarkerArray output;
            append_status(output, frame, scene.prefix, scene.origin_x, stage);
            append_timeline(
                    output,
                    frame,
                    scene.prefix,
                    scene.origin_x,
                    stage_index,
                    scene.stages);
            if(!stage.aggregate_cells.empty()) {
                append_map(
                        output,
                        frame,
                        scene.prefix + "/aggregate_map",
                        scene.origin_x,
                        2.0,
                        stage.aggregate_cells,
                        kBlue);
            }
            double contributor_x = scene.origin_x - 2.4;
            for(const auto & contributor : stage.contributors) {
                append_map(
                        output,
                        frame,
                        scene.prefix + "/" + contributor.id,
                        contributor_x,
                        -2.0,
                        contributor.cells,
                        contributor.color);
                auto label = base_marker(
                        frame,
                        scene.prefix + "/contributor_labels",
                        static_cast<std::int32_t>(contributor_x * 10.0),
                        Marker::TEXT_VIEW_FACING,
                        contributor.color);
                label.pose.position.x = contributor_x;
                label.pose.position.y = -3.2;
                label.pose.position.z = 0.4;
                label.scale.z = 0.3;
                label.text = contributor.id + " rev "
                             + std::to_string(contributor.revision);
                output.markers.push_back(std::move(label));
                contributor_x += 4.8;
            }
            return output;
        }

        Scene build_edge_recovery()
        {
            const ProducerIdentity producer {"drone_a_mapper", {300U, 11U}};
            const auto baseline_snapshot = snapshot(
                    "drone_a", {100U, 7U}, 1U, 1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
            const auto delta_snapshot = snapshot(
                    "drone_a", {100U, 7U}, 1U, 2U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{2, 0, 0}, PerceptionMapUpdate::CellState::Free}});
            const auto final_snapshot = snapshot(
                    "drone_a", {100U, 7U}, 1U, 3U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});

            PerceptionMapUpdate::MapUpdateProducer map_producer;
            const auto first = map_producer.prepare(baseline_snapshot);
            require(first.update.has_value(), first.diagnostic);
            require(map_producer.commit_published(first), "edge baseline commit failed");
            const auto second = map_producer.prepare(delta_snapshot);
            require(second.update.has_value(), second.diagnostic);
            require(map_producer.commit_published(second), "edge delta commit failed");
            const auto third = map_producer.prepare(final_snapshot);
            require(third.update.has_value(), third.diagnostic);
            require(map_producer.commit_published(third), "edge final delta commit failed");

            MapUpdateIngress ingress;
            require(ingress.admit_producer(producer), "edge producer admission failed");
            require(ingress.admit_source(first.update->source), "edge source admission failed");
            const auto first_message = routed(
                    shared_update(first), producer, 1U, "edge-seq-1");
            require(
                    ingress.receive(first_message, 100U).status
                            == IngressStatus::AppliedKeyframe,
                    "edge baseline was not applied");
            const auto gap_result = ingress.receive(
                    routed(shared_update(third), producer, 3U, "edge-seq-3"), 200U);
            require(
                    gap_result.status == IngressStatus::RejectedGap
                            && gap_result.resync_required,
                    "edge gap was not rejected");

            require(
                    map_producer.request_keyframe("edge-recovery"),
                    "edge keyframe request failed");
            const auto recovery = map_producer.prepare(final_snapshot);
            require(recovery.update.has_value(), recovery.diagnostic);
            require(
                    recovery.update->kind == PerceptionMapUpdate::UpdateKind::Keyframe,
                    "edge recovery was not a keyframe");
            require(ingress.expect_resync("edge-recovery"), "edge correlation was not expected");
            const auto recovered = ingress.receive(
                    routed(shared_update(recovery), producer, 4U, "edge-seq-4"), 300U);
            require(
                    recovered.status == IngressStatus::AppliedKeyframe
                            && !recovered.resync_required,
                    "edge recovery was not applied");

            Scene scene {"c4_edge_recovery", 0.0, {}};
            scene.stages = {
                    {"READY BASELINE", "revision 1 | sequence 1 | state READY",
                     baseline_snapshot.cells, {}, false},
                    {"DELTA DROPPED", "revision 1 | dropped sequence 2 | map held",
                     baseline_snapshot.cells, {}, true},
                    {"GAP REJECTED", "revision 1 | expected sequence 2 | received sequence 3 | RESYNC_REQUIRED",
                     baseline_snapshot.cells, {}, true},
                    {"RESYNC RECOVERED", "revision 3 | sequence 4 | correlation edge-recovery | state READY",
                     final_snapshot.cells, {}, false}};
            return scene;
        }

        std::vector<ContributorView> contributor_views(std::uint64_t a2_revision)
        {
            return {
                    {"A1", {{{-2, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                              {{-1, 0, 0}, PerceptionMapUpdate::CellState::Free}},
                     kGreen, 2U},
                    {"A2", {{{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                              {{3, 0, 0}, PerceptionMapUpdate::CellState::Free}},
                     kPurple, a2_revision}};
        }

        Scene build_edge_aggregation()
        {
            const ProducerIdentity producer {"edge_b_aggregate", {600U, 1U}};
            const auto first_snapshot = snapshot(
                    "edge_b", {700U, 1U}, 1U, 1U,
                    {{{-2, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{-1, 0, 0}, PerceptionMapUpdate::CellState::Free},
                     {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
            const auto second_snapshot = snapshot(
                    "edge_b", {700U, 1U}, 1U, 2U,
                    {{{-2, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{-1, 0, 0}, PerceptionMapUpdate::CellState::Free},
                     {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{3, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
            const PerceptionMapUpdate::SourceIdentity a1 {
                    "drone_a1", {101U, 1U}, 1U};
            const PerceptionMapUpdate::SourceIdentity a2 {
                    "drone_a2", {102U, 1U}, 1U};

            PerceptionMapUpdate::MapUpdateProducer map_producer;
            const auto first = map_producer.prepare(first_snapshot);
            require(first.update.has_value(), first.diagnostic);
            require(map_producer.commit_published(first), "aggregate baseline commit failed");
            const auto second = map_producer.prepare(second_snapshot);
            require(second.update.has_value(), second.diagnostic);
            require(map_producer.commit_published(second), "aggregate delta commit failed");

            auto make_update = [&](const PerceptionMapUpdate::PreparedUpdate & prepared,
                                   std::uint64_t a2_revision) {
                AggregateMapUpdate result;
                result.aggregate_update = routed(
                        shared_update(prepared), producer, prepared.update->new_revision == 1U
                                                                     ? 1U
                                                                     : 3U,
                        "aggregate-seq-" + std::to_string(prepared.update->new_revision));
                result.manifest = manifest_for(
                        *prepared.update,
                        {{a1, 2U, prepared.update->content_hash, true},
                         {a2, a2_revision, prepared.update->content_hash, true}});
                return result;
            };

            const auto aggregate_first = make_update(first, 1U);
            const auto aggregate_delta = make_update(second, 2U);
            AggregateIngress ingress;
            require(ingress.admit_producer(producer), "aggregate producer admission failed");
            require(ingress.admit_source(first.update->source), "aggregate source admission failed");
            require(
                    ingress.receive(aggregate_first, 100U).map_result.status
                            == IngressStatus::AppliedKeyframe,
                    "aggregate baseline was not applied");
            const auto gap = ingress.receive(aggregate_delta, 200U);
            require(
                    gap.map_result.status == IngressStatus::RejectedGap
                            && gap.map_result.resync_required,
                    "aggregate gap was not rejected");
            require(map_producer.request_keyframe("aggregate-recovery"), "aggregate request failed");
            const auto recovery = map_producer.prepare(second_snapshot);
            require(recovery.update.has_value(), recovery.diagnostic);
            AggregateMapUpdate aggregate_recovery {
                    routed(shared_update(recovery), producer, 4U, "aggregate-seq-4"),
                    manifest_for(*recovery.update, {{a1, 2U, recovery.update->content_hash, true},
                                                    {a2, 2U, recovery.update->content_hash, true}})};
            require(ingress.expect_resync("aggregate-recovery"), "aggregate correlation failed");
            const auto recovered = ingress.receive(aggregate_recovery, 300U);
            require(
                    recovered.map_result.status == IngressStatus::AppliedKeyframe
                            && recovered.manifest_changed,
                    "aggregate recovery was not committed");

            const auto views_before = contributor_views(1U);
            const auto views_after = contributor_views(2U);
            Scene scene {"c4_edge_aggregation", 20.0, {}};
            scene.stages = {
                    {"CONTRIBUTORS READY", "A1 rev 2 | A2 rev 1 | aggregate revision 1",
                     first_snapshot.cells, views_before, false},
                    {"A2 GAP", "A2 missing delta | aggregate held at revision 1",
                     first_snapshot.cells, views_before, true},
                    {"A2 RESYNC", "correlation aggregate-recovery | aggregate barrier",
                     first_snapshot.cells, views_before, false},
                    {"AGGREGATE COMMITTED", "manifest revision 2 | A1 rev 2 | A2 rev 2",
                     second_snapshot.cells, views_after, false}};
            return scene;
        }

        Scene build_upstream_recovery()
        {
            const ProducerIdentity producer {"edge_b_aggregate", {610U, 1U}};
            const auto first_snapshot = snapshot(
                    "edge_b", {710U, 1U}, 1U, 1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Free}});
            const auto second_snapshot = snapshot(
                    "edge_b", {710U, 1U}, 1U, 2U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{2, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
            const PerceptionMapUpdate::SourceIdentity a1 {
                    "drone_a1", {111U, 1U}, 1U};
            const PerceptionMapUpdate::SourceIdentity a2 {
                    "drone_a2", {112U, 1U}, 1U};

            PerceptionMapUpdate::MapUpdateProducer producer_state;
            const auto first = producer_state.prepare(first_snapshot);
            require(first.update.has_value(), first.diagnostic);
            require(producer_state.commit_published(first), "upstream baseline commit failed");
            const auto second = producer_state.prepare(second_snapshot);
            require(second.update.has_value(), second.diagnostic);
            require(producer_state.commit_published(second), "upstream delta commit failed");

            const auto make_manifest = [&](const MapUpdate & update) {
                return manifest_for(
                        update,
                        {{a1, update.new_revision, update.content_hash, true},
                         {a2, update.new_revision, update.content_hash, true}});
            };
            AggregateMapUpdate aggregate_first {
                    routed(shared_update(first), producer, 1U, "upstream-seq-1"),
                    make_manifest(*first.update)};
            AggregateMapUpdate aggregate_delta {
                    routed(shared_update(second), producer, 3U, "upstream-seq-3"),
                    make_manifest(*second.update)};

            AggregateIngress central;
            require(central.admit_producer(producer), "central producer admission failed");
            require(central.admit_source(first.update->source), "central source admission failed");
            require(
                    central.receive(aggregate_first, 100U).map_result.status
                            == IngressStatus::AppliedKeyframe,
                    "central baseline was not applied");
            const auto gap = central.receive(aggregate_delta, 200U);
            require(
                    gap.map_result.status == IngressStatus::RejectedGap
                            && gap.map_result.resync_required,
                    "central upstream gap was not rejected");
            require(producer_state.request_keyframe("central-recovery"), "central request failed");
            const auto recovery = producer_state.prepare(second_snapshot);
            require(recovery.update.has_value(), recovery.diagnostic);
            AggregateMapUpdate aggregate_recovery {
                    routed(shared_update(recovery), producer, 4U, "upstream-seq-4"),
                    make_manifest(*recovery.update)};
            require(central.expect_resync("central-recovery"), "central correlation failed");
            const auto recovered = central.receive(aggregate_recovery, 300U);
            require(
                    recovered.map_result.status == IngressStatus::AppliedKeyframe
                            && recovered.manifest_changed,
                    "central recovery was not applied");

            Scene scene {"c4_upstream_recovery", 40.0, {}};
            scene.stages = {
                    {"AGGREGATE BASELINE", "B revision 1 -> C revision 1 | manifest ready",
                     first_snapshot.cells, {}, false},
                    {"UPSTREAM GAP", "C holds revision 1 | received sequence 3 | gap",
                     first_snapshot.cells, {}, true},
                    {"CENTRAL RESYNC", "correlation central-recovery | C barrier active",
                     first_snapshot.cells, {}, false},
                    {"CENTRAL RECOVERED", "C revision 2 | aggregate manifest atomic | READY",
                     second_snapshot.cells, {}, false}};
            return scene;
        }

        Scene build_multihop_ttl()
        {
            const ProducerIdentity producer {"edge_b_aggregate", {620U, 1U}};
            const auto map_snapshot = snapshot(
                    "edge_b", {720U, 1U}, 1U, 1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Occupied},
                     {{1, 0, 0}, PerceptionMapUpdate::CellState::Occupied}});
            PerceptionMapUpdate::MapUpdateProducer producer_state;
            const auto prepared = producer_state.prepare(map_snapshot);
            require(prepared.update.has_value(), prepared.diagnostic);
            const auto original = routed(
                    shared_update(prepared), producer, 1U, "ttl-seq-1", 2U);
            const auto first_hop = forward_routed_map_update(original, 100U);
            require(first_hop.message.has_value(), first_hop.diagnostic);
            const auto ttl_rejection = forward_routed_map_update(*first_hop.message, 100U);
            require(
                    ttl_rejection.status == EnvelopeValidationStatus::RejectedRoute,
                    "TTL exhaustion was not rejected");

            Scene scene {"c4_multihop_ttl", 60.0, {}};
            scene.stages = {
                    {"SOURCE HOP 0", "payload hash stable | route hop 0 | TTL 2",
                     map_snapshot.cells, {}, false},
                    {"RELAY B1 HOP 1", "payload hash stable | route hop 1 | TTL remaining 1",
                     map_snapshot.cells, {}, false},
                    {"B2 TTL REJECTED", "route fault | next hop would exhaust TTL",
                     map_snapshot.cells, {}, true},
                    {"ROUTE SAFE", "map unchanged | no expired message committed",
                     map_snapshot.cells, {}, false}};
            return scene;
        }

    }// namespace

    class C4VisualizationFixture final : public rclcpp::Node
    {
    public:
        C4VisualizationFixture() : Node("c4_visualization_fixture")
        {
            const auto publish_rate_hz = declare_parameter<double>(
                    "publish_rate_hz", 10.0);
            const auto scenario_step_period_s = declare_parameter<double>(
                    "scenario_step_period_s", 0.5);
            if(!std::isfinite(publish_rate_hz) || publish_rate_hz <= 0.0
               || !std::isfinite(scenario_step_period_s) || scenario_step_period_s <= 0.0) {
                throw std::invalid_argument(
                        "visualization publish rate and scenario step period must be positive");
            }
            const auto publish_period_ms = std::max(
                    1LL,
                    static_cast<long long>(std::llround(1000.0 / publish_rate_hz)));
            const auto step_period_ms = std::max(
                    1LL,
                    static_cast<long long>(std::llround(1000.0 * scenario_step_period_s)));

            scenes_ = {
                    build_edge_recovery(),
                    build_edge_aggregation(),
                    build_upstream_recovery(),
                    build_multihop_ttl()};
            for(const auto & scene : scenes_) {
                publishers_.push_back(create_publisher<MarkerArray>(
                        "/c4/visualization/" + scene.prefix.substr(3),
                        rclcpp::QoS(1).reliable().transient_local()));
            }
            publish_timer_ = create_wall_timer(
                    std::chrono::milliseconds(publish_period_ms),
                    [this]() { publish_current(); });
            stage_timer_ = create_wall_timer(
                    std::chrono::milliseconds(step_period_ms),
                    [this]() {
                        stage_index_ = (stage_index_ + 1U) % 4U;
                    });
        }

    private:
        void publish_current()
        {
            const auto stamp = now();
            for(std::size_t index = 0U; index < scenes_.size(); ++index) {
                auto message = render_stage(
                        scenes_[index], stage_index_, "c4_visualization_world");
                for(auto & marker : message.markers) {
                    marker.header.stamp = stamp;
                }
                publishers_[index]->publish(message);
            }
        }

        std::array<Scene, 4U> scenes_;
        std::vector<rclcpp::Publisher<MarkerArray>::SharedPtr> publishers_;
        std::size_t stage_index_ = 0U;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr stage_timer_;
    };

}// namespace SwarmDataPlane::Test

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::Test::C4VisualizationFixture>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("c4_visualization_fixture"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
