#include "perception_map_update/ContentHasher.hpp"
#include "perception_map_update/MapUpdateProducer.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"

#include "swarm_data_interfaces/msg/routed_map_update.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        using PerceptionMapUpdate::CanonicalCell;
        using PerceptionMapUpdate::CanonicalSnapshot;
        using PerceptionMapUpdate::CellState;
        using PerceptionMapUpdate::ContentHasher;
        using PerceptionMapUpdate::Hash256;
        using PerceptionMapUpdate::MapGeometry;
        using PerceptionMapUpdate::MapUpdate;
        using PerceptionMapUpdate::MapUpdateProducer;
        using PerceptionMapUpdate::RevisionProvenance;
        using PerceptionMapUpdate::SourceIdentity;

        constexpr std::uint64_t kBoundedProfileMaxDeltaChainLength = 0U;

        enum class WorkloadMode
        {
            Bounded,
            Expanding,
            KeyframeReplacement
        };

        WorkloadMode parse_workload_mode(const std::string & value)
        {
            if(value == "bounded") {
                return WorkloadMode::Bounded;
            }
            if(value == "expanding") {
                return WorkloadMode::Expanding;
            }
            if(value == "keyframe-replacement") {
                return WorkloadMode::KeyframeReplacement;
            }
            throw std::invalid_argument(
                    "workload_mode must be bounded, expanding, or keyframe-replacement");
        }

        const char * workload_mode_name(WorkloadMode mode) noexcept
        {
            switch(mode) {
                case WorkloadMode::Bounded:
                    return "bounded";
                case WorkloadMode::Expanding:
                    return "expanding";
                case WorkloadMode::KeyframeReplacement:
                    return "keyframe-replacement";
            }
            return "unknown";
        }

        std::uint64_t steady_now_ns()
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
        }

        std::size_t positive_size_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(name, default_value);
            if(value <= 0) {
                throw std::invalid_argument(name + " must be positive");
            }
            return static_cast<std::size_t>(value);
        }

        double positive_rate_parameter(
                rclcpp::Node & node,
                const std::string & name,
                double default_value)
        {
            const auto value = node.declare_parameter<double>(name, default_value);
            if(!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument(name + " must be finite and positive");
            }
            return value;
        }

        void write_value(std::ofstream & output, const char * key, std::uint64_t value)
        {
            output << key << '\t' << value << '\n';
        }

        struct SourceState {
            std::size_t index = 0U;
            WorkloadMode mode = WorkloadMode::Bounded;
            SourceIdentity source;
            ProducerIdentity producer;
            MapGeometry geometry;
            Hash256 current_content_hash {};
            MapUpdate initial_keyframe;
            std::unique_ptr<MapUpdateProducer> map_producer;
            std::vector<CanonicalCell> current_cells;
            std::uint64_t target_known_cells = 0U;
            std::uint64_t current_known_cells = 0U;
            std::uint64_t delta_operations = 0U;
            std::uint64_t revision = 0U;
            std::uint64_t sequence = 0U;
            std::uint64_t messages_sent = 0U;
            std::uint64_t payload_bytes_sent = 0U;
        };

        RevisionProvenance make_provenance(
                const SourceState & state,
                std::uint64_t now_ns,
                std::uint64_t changed_cells);

        std::vector<CanonicalCell> make_cells(std::size_t count)
        {
            std::vector<CanonicalCell> cells;
            cells.reserve(count);
            for(std::size_t cell = 0U; cell < count; ++cell) {
                cells.push_back({
                        {static_cast<std::int64_t>(cell), 0, 0},
                        (cell % 2U) == 0U ? CellState::Free : CellState::Occupied});
            }
            return cells;
        }

        CanonicalSnapshot make_snapshot(
                const SourceIdentity & source,
                const MapGeometry & geometry,
                std::vector<CanonicalCell> cells,
                std::uint64_t revision = 1U)
        {
            CanonicalSnapshot snapshot;
            snapshot.source = source;
            snapshot.geometry = geometry;
            snapshot.revision = revision;
            snapshot.latest_commit = {
                    "profile-lidar",
                    {2'000U + source.mapper_session.random_suffix, 17U},
                    Perception::Timestamp {1},
                    "steady-profile",
                    static_cast<std::uint32_t>(std::min<std::size_t>(
                            cells.size(), std::numeric_limits<std::uint32_t>::max()))};
            snapshot.cells = std::move(cells);
            snapshot.geometry_fingerprint = ContentHasher::geometry_fingerprint(geometry);
            return snapshot;
        }

        MapUpdate prepare_target(
                SourceState & state,
                std::vector<CanonicalCell> target_cells,
                std::uint64_t now_ns)
        {
            auto target = std::make_shared<CanonicalSnapshot>(make_snapshot(
                    state.source,
                    state.geometry,
                    std::move(target_cells),
                    state.revision + 1U));
            target->latest_commit = make_provenance(
                    state,
                    now_ns,
                    state.delta_operations);
            auto prepared = state.map_producer->prepare(std::move(target), 1U);
            if(!prepared.update.has_value()
               || !state.map_producer->commit_published(prepared)) {
                throw std::runtime_error(
                        "cannot prepare deterministic v2 profile update: "
                        + prepared.diagnostic);
            }
            state.current_cells = prepared.target_snapshot->cells;
            state.current_known_cells = state.current_cells.size();
            state.current_content_hash = prepared.update->content_hash;
            return std::move(*prepared.update);
        }

        SourceState make_source_state(
                std::size_t index,
                std::size_t known_cells,
                std::size_t initial_cells,
                std::size_t delta_operations,
                WorkloadMode mode)
        {
            SourceState state;
            state.index = index;
            state.mode = mode;
            state.source = {
                    "profile-source-" + std::to_string(index),
                    {10'000U + index, static_cast<std::uint32_t>(100U + index)},
                    1U};
            state.producer = {
                    "profile-producer-" + std::to_string(index),
                    {20'000U + index, static_cast<std::uint32_t>(200U + index)}};
            state.geometry = {
                    0.1,
                    {0.0, static_cast<double>(index) * 100.0, 0.0},
                    state.source.vehicle_id + "/map"};

            const auto first_cell_count = mode == WorkloadMode::Expanding
                                                  ? std::min(initial_cells, known_cells)
                                                  : known_cells;
            auto base_cells = make_cells(first_cell_count);

            PerceptionMapUpdate::MapUpdateLimits producer_limits;
            if(mode == WorkloadMode::Bounded) {
                // Keep V1/V2 bounded A/B runs at the same initial-only keyframe cadence.
                // The production default remains bounded by its normal chain limit.
                producer_limits.max_delta_chain_length =
                        kBoundedProfileMaxDeltaChainLength;
            }
            else if(mode == WorkloadMode::KeyframeReplacement) {
                producer_limits.periodic_keyframe_revision_interval = 1U;
            }
            state.map_producer =
                    std::make_unique<MapUpdateProducer>(std::move(producer_limits));
            state.current_cells = base_cells;
            auto snapshot = make_snapshot(
                    state.source,
                    state.geometry,
                    std::move(base_cells));
            auto prepared = state.map_producer->prepare(std::move(snapshot));
            if(!prepared.update.has_value()
               || !state.map_producer->commit_published(prepared)) {
                throw std::runtime_error("cannot build deterministic profile keyframe");
            }
            state.initial_keyframe = std::move(*prepared.update);
            state.current_content_hash = state.initial_keyframe.content_hash;
            state.target_known_cells = static_cast<std::uint64_t>(known_cells);
            state.current_known_cells = static_cast<std::uint64_t>(first_cell_count);
            state.delta_operations = static_cast<std::uint64_t>(delta_operations);

            state.current_cells = prepared.target_snapshot->cells;
            return state;
        }

        RevisionProvenance make_provenance(
                const SourceState & state,
                std::uint64_t now_ns,
                std::uint64_t changed_cells)
        {
            return {
                    "profile-lidar",
                    {2'000U + state.index, 17U},
                    Perception::Timestamp {static_cast<std::int64_t>(now_ns)},
                    "steady-profile",
                    static_cast<std::uint32_t>(std::min<std::uint64_t>(
                            changed_cells,
                            std::numeric_limits<std::uint32_t>::max()))};
        }

        MapUpdate make_alternating_delta(SourceState & state, std::uint64_t now_ns)
        {
            auto target_cells = state.current_cells;
            const auto operation_count = std::min<std::size_t>(
                    static_cast<std::size_t>(state.delta_operations), target_cells.size());
            for(std::size_t cell = 0U; cell < operation_count; ++cell) {
                target_cells[cell].state = target_cells[cell].state == CellState::Free
                                                   ? CellState::Occupied
                                                   : CellState::Free;
            }
            return prepare_target(state, std::move(target_cells), now_ns);
        }

        MapUpdate make_expanding_delta(SourceState & state, std::uint64_t now_ns)
        {
            const auto remaining = state.target_known_cells - state.current_known_cells;
            if(remaining == 0U) {
                return make_alternating_delta(state, now_ns);
            }
            const auto added = std::min(remaining, state.delta_operations);
            auto target_cells = state.current_cells;
            target_cells.reserve(target_cells.size() + static_cast<std::size_t>(added));
            for(std::uint64_t offset = 0U; offset < added; ++offset) {
                const auto cell = state.current_known_cells + offset;
                const auto cell_state = (cell % 2U) == 0U
                                                ? CellState::Free
                                                : CellState::Occupied;
                const CanonicalCell canonical {
                        {static_cast<std::int64_t>(cell), 0, 0}, cell_state};
                target_cells.push_back(canonical);
            }
            return prepare_target(state, std::move(target_cells), now_ns);
        }

        MapUpdate make_replacement_keyframe(SourceState & state, std::uint64_t now_ns)
        {
            auto target_cells = state.current_cells;
            const auto operation_count = std::min<std::size_t>(
                    static_cast<std::size_t>(state.delta_operations), target_cells.size());
            for(std::size_t cell = 0U; cell < operation_count; ++cell) {
                target_cells[cell].state = target_cells[cell].state == CellState::Free
                                                   ? CellState::Occupied
                                                   : CellState::Free;
            }
            return prepare_target(state, std::move(target_cells), now_ns);
        }

    }// namespace

    class C4ResourceProfileSource final : public rclcpp::Node
    {
    public:
        C4ResourceProfileSource() : Node("c4_resource_profile_source")
        {
            workload_mode_ = parse_workload_mode(
                    declare_parameter<std::string>("workload_mode", "bounded"));
            const auto source_count = positive_size_parameter(*this, "source_count", 2);
            const auto cells_per_source = positive_size_parameter(
                    *this, "cells_per_source", 10'000);
            const auto initial_cells_per_source = positive_size_parameter(
                    *this, "initial_cells_per_source", 10'000);
            const auto delta_operations = positive_size_parameter(
                    *this, "delta_operations", 256);
            if(delta_operations > cells_per_source) {
                throw std::invalid_argument(
                        "delta_operations must not exceed cells_per_source");
            }
            if(workload_mode_ == WorkloadMode::Expanding
               && initial_cells_per_source > cells_per_source) {
                throw std::invalid_argument(
                        "initial_cells_per_source must not exceed cells_per_source");
            }
            if(workload_mode_ == WorkloadMode::Expanding
               && delta_operations > initial_cells_per_source) {
                throw std::invalid_argument(
                        "expanding delta_operations must not exceed initial_cells_per_source");
            }
            const auto rate_hz = positive_rate_parameter(*this, "rate_hz_per_source", 10.0);
            const auto qos_depth = positive_size_parameter(*this, "qos_depth", 4);
            summary_path_ = declare_parameter<std::string>("summary_path", "");
            per_source_path_ = declare_parameter<std::string>("per_source_path", "");
            const auto topic = declare_parameter<std::string>(
                    "output_topic", "/c4/profile/routed_updates");

            if(source_count > 64U) {
                throw std::invalid_argument("source_count exceeds profile limit 64");
            }
            sources_.reserve(source_count);
            for(std::size_t index = 0U; index < source_count; ++index) {
                sources_.push_back(make_source_state(
                        index,
                        cells_per_source,
                        initial_cells_per_source,
                        delta_operations,
                        workload_mode_));
            }

            publisher_ = create_publisher<swarm_data_interfaces::msg::RoutedMapUpdate>(
                    topic, Ros::map_update_qos(qos_depth));
            const double total_rate_hz = rate_hz * static_cast<double>(source_count);
            const auto period_ns = static_cast<std::int64_t>(1.0e9 / total_rate_hz);
            if(period_ns <= 0) {
                throw std::invalid_argument("aggregate profile publication rate is too high");
            }
            timer_ = create_wall_timer(
                    std::chrono::nanoseconds(period_ns), [this]() { publish_next(); });
        }

        ~C4ResourceProfileSource() override { write_summary(); }

    private:
        void publish_next()
        {
            const auto subscription_count = publisher_->get_subscription_count();
            if(subscription_count == 0U || sources_.empty()) {
                return;
            }
            if(subscription_count != 1U) {
                ++endpoint_count_anomalies_;
            }
            auto & source = sources_[next_source_];
            const auto now_ns = steady_now_ns();
            MapUpdate update;
            if(source.sequence == 0U) {
                update = source.initial_keyframe;
            } else {
                switch(workload_mode_) {
                    case WorkloadMode::Bounded:
                        update = make_alternating_delta(source, now_ns);
                        break;
                    case WorkloadMode::Expanding:
                        update = make_expanding_delta(source, now_ns);
                        break;
                    case WorkloadMode::KeyframeReplacement:
                        update = make_replacement_keyframe(source, now_ns);
                        break;
                }
            }
            const auto sequence = source.sequence + 1U;
            RoutedMapUpdate routed;
            routed.message_id = "profile-" + std::to_string(source.index) + '-'
                                + std::to_string(sequence);
            routed.producer = source.producer;
            routed.sequence = sequence;
            routed.priority = update.kind == PerceptionMapUpdate::UpdateKind::Keyframe
                                      ? LogicalPriority::MapKeyframe
                                      : LogicalPriority::MapDelta;
            routed.origin = {
                    "steady-profile", {30'000U + source.index, 301U}, now_ns};
            routed.validity_budget_ns = 5'000'000'000U;
            routed.route = {1U, 0U, 8U};
            routed.payload_bytes = update.canonical_payload_bytes;
            routed.payload_hash = update.update_hash;
            routed.update = std::make_shared<const MapUpdate>(std::move(update));

            swarm_data_interfaces::msg::RoutedMapUpdate output;
            std::string diagnostic;
            if(!Ros::encode_routed_map_update(routed, output, diagnostic)) {
                ++conversion_failures_;
                RCLCPP_ERROR_THROTTLE(
                        get_logger(),
                        *get_clock(),
                        5000,
                        "profile conversion failed: %s",
                        diagnostic.c_str());
                return;
            }
            try {
                rclcpp::SerializedMessage serialized;
                serializer_.serialize_message(&output, &serialized);
                application_serialized_bytes_ += serialized.size();
            }
            catch(const std::exception &) {
                ++serialization_failures_;
            }
            publisher_->publish(output);
            source.sequence = sequence;
            source.revision = routed.update->new_revision;
            ++source.messages_sent;
            source.payload_bytes_sent += routed.payload_bytes;
            ++messages_sent_;
            if(routed.update->kind == PerceptionMapUpdate::UpdateKind::Keyframe) {
                ++keyframes_sent_;
            } else if(routed.update->kind == PerceptionMapUpdate::UpdateKind::Delta) {
                ++deltas_sent_;
            }
            payload_bytes_sent_ += routed.payload_bytes;
            next_source_ = (next_source_ + 1U) % sources_.size();
        }

        void write_summary() const
        {
            if(summary_path_.empty() || per_source_path_.empty()) {
                return;
            }
            std::ofstream summary(summary_path_, std::ios::trunc);
            std::ofstream per_source(per_source_path_, std::ios::trunc);
            if(!summary || !per_source) {
                return;
            }
            write_value(summary, "schema_version", 1U);
            summary << "workload_mode\t" << workload_mode_name(workload_mode_) << '\n';
            summary << "keyframe_policy\t"
                    << (workload_mode_ == WorkloadMode::Bounded
                                ? "initial-only"
                                : workload_mode_ == WorkloadMode::KeyframeReplacement
                                      ? "every-revision"
                                      : "producer-default")
                    << '\n';
            write_value(summary, "source_count", sources_.size());
            write_value(summary, "messages_sent", messages_sent_);
            write_value(summary, "keyframes_sent", keyframes_sent_);
            write_value(summary, "deltas_sent", deltas_sent_);
            write_value(summary, "payload_bytes_sent", payload_bytes_sent_);
            write_value(summary, "application_serialized_bytes", application_serialized_bytes_);
            write_value(summary, "conversion_failures", conversion_failures_);
            write_value(summary, "serialization_failures", serialization_failures_);
            write_value(summary, "endpoint_count_anomalies", endpoint_count_anomalies_);
            summary << "normal_completion\t1\n";

            per_source << "source\tmessages_sent\tfinal_revision\tfinal_cells\t"
                          "final_content_hash\tpayload_bytes_sent\n";
            for(const auto & source : sources_) {
                per_source << source.source.vehicle_id << '\t' << source.messages_sent << '\t'
                           << source.revision << '\t' << source.current_known_cells << '\t'
                           << PerceptionMapUpdate::hash_to_hex(source.current_content_hash)
                           << '\t' << source.payload_bytes_sent << '\n';
            }
        }

        WorkloadMode workload_mode_ = WorkloadMode::Bounded;
        std::vector<SourceState> sources_;
        std::size_t next_source_ = 0U;
        std::uint64_t messages_sent_ = 0U;
        std::uint64_t keyframes_sent_ = 0U;
        std::uint64_t deltas_sent_ = 0U;
        std::uint64_t payload_bytes_sent_ = 0U;
        std::uint64_t application_serialized_bytes_ = 0U;
        std::uint64_t conversion_failures_ = 0U;
        std::uint64_t serialization_failures_ = 0U;
        std::uint64_t endpoint_count_anomalies_ = 0U;
        std::string summary_path_;
        std::string per_source_path_;
        rclcpp::Publisher<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr publisher_;
        rclcpp::Serialization<swarm_data_interfaces::msg::RoutedMapUpdate> serializer_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}// namespace SwarmDataPlane::Test

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::Test::C4ResourceProfileSource>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("c4_resource_profile_source"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
