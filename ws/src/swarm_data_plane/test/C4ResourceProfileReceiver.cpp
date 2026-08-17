#include "swarm_data_plane/MapUpdateIngress.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"

#include "swarm_data_interfaces/msg/routed_map_update.hpp"

#include <rclcpp/rclcpp.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace SwarmDataPlane::Test {

    namespace {

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

        void write_value(std::ofstream & output, const char * key, std::uint64_t value)
        {
            output << key << '\t' << value << '\n';
        }

        class BoundedHistogram
        {
        public:
            void observe(std::uint64_t value_ns) noexcept
            {
                std::size_t bucket = 0U;
                auto value = value_ns;
                while(value > 1U && bucket + 1U < buckets_.size()) {
                    value >>= 1U;
                    ++bucket;
                }
                ++buckets_[bucket];
                ++count_;
                total_ns_ += value_ns;
                max_ns_ = std::max(max_ns_, value_ns);
            }

            std::uint64_t percentile_upper_bound(double fraction) const noexcept
            {
                if(count_ == 0U) {
                    return 0U;
                }
                const auto rank = static_cast<std::uint64_t>(
                        std::ceil(fraction * static_cast<double>(count_)));
                std::uint64_t cumulative = 0U;
                for(std::size_t bucket = 0U; bucket < buckets_.size(); ++bucket) {
                    cumulative += buckets_[bucket];
                    if(cumulative >= rank) {
                        if(bucket >= 63U) {
                            return std::numeric_limits<std::uint64_t>::max();
                        }
                        return (std::uint64_t {1U} << (bucket + 1U)) - 1U;
                    }
                }
                return max_ns_;
            }

            std::uint64_t count() const noexcept { return count_; }
            std::uint64_t total_ns() const noexcept { return total_ns_; }
            std::uint64_t max_ns() const noexcept { return max_ns_; }

        private:
            std::array<std::uint64_t, 64U> buckets_ {};
            std::uint64_t count_ = 0U;
            std::uint64_t total_ns_ = 0U;
            std::uint64_t max_ns_ = 0U;
        };

        bool is_applied(IngressStatus status) noexcept
        {
            return status == IngressStatus::AppliedKeyframe
                   || status == IngressStatus::AppliedDelta
                   || status == IngressStatus::AppliedRemove;
        }

        struct SourceState {
            SourceState() = default;

            MapUpdateIngress ingress;
            std::uint64_t messages_received = 0U;
            std::uint64_t messages_applied = 0U;
            std::uint64_t payload_bytes = 0U;
            std::uint64_t last_sequence = 0U;
            std::uint64_t last_receive_ns = 0U;
            std::uint64_t max_interarrival_ns = 0U;
        };

    }// namespace

    class C4ResourceProfileReceiver final : public rclcpp::Node
    {
    public:
        C4ResourceProfileReceiver() : Node("c4_resource_profile_receiver")
        {
            expected_sources_ = positive_size_parameter(*this, "expected_sources", 2);
            if(expected_sources_ > 64U) {
                throw std::invalid_argument("expected_sources exceeds profile limit 64");
            }
            qos_depth_ = positive_size_parameter(*this, "qos_depth", 4);
            target_cells_per_source_ = positive_size_parameter(
                    *this, "target_cells_per_source", 10'000);
            summary_path_ = declare_parameter<std::string>("summary_path", "");
            per_source_path_ = declare_parameter<std::string>("per_source_path", "");
            ready_path_ = declare_parameter<std::string>("ready_path", "");
            const auto topic = declare_parameter<std::string>(
                    "input_topic", "/c4/profile/routed_updates");
            subscription_ = create_subscription<swarm_data_interfaces::msg::RoutedMapUpdate>(
                    topic,
                    Ros::map_update_qos(qos_depth_),
                    [this](swarm_data_interfaces::msg::RoutedMapUpdate::ConstSharedPtr message) {
                        receive(*message);
                    });
        }

        ~C4ResourceProfileReceiver() override { write_summary(); }

    private:
        void receive(const swarm_data_interfaces::msg::RoutedMapUpdate & wire)
        {
            const auto callback_start = steady_now_ns();
            ++messages_received_;
            if(subscription_->get_publisher_count() != 1U) {
                ++endpoint_count_anomalies_;
            }
            const auto decode_start = steady_now_ns();
            auto decoded = Ros::decode_routed_map_update(wire);
            const auto decode_end = steady_now_ns();
            decode_latency_.observe(decode_end - decode_start);
            if(!decoded.success || !decoded.message.has_value()) {
                ++messages_rejected_;
                callback_latency_.observe(steady_now_ns() - callback_start);
                return;
            }

            auto & routed = *decoded.message;
            const auto & source_id = routed.update->source.vehicle_id;
            auto found = sources_.find(source_id);
            if(found == sources_.end()) {
                if(sources_.size() >= expected_sources_) {
                    ++messages_rejected_;
                    callback_latency_.observe(steady_now_ns() - callback_start);
                    return;
                }
                auto state = std::make_unique<SourceState>();
                if(!state->ingress.admit_producer(routed.producer)
                   || !state->ingress.admit_source(routed.update->source)) {
                    ++messages_rejected_;
                    callback_latency_.observe(steady_now_ns() - callback_start);
                    return;
                }
                found = sources_.emplace(source_id, std::move(state)).first;
            }

            auto & source = *found->second;
            ++source.messages_received;
            source.payload_bytes += routed.payload_bytes;
            payload_bytes_received_ += routed.payload_bytes;
            if(source.last_receive_ns != 0U) {
                source.max_interarrival_ns = std::max(
                        source.max_interarrival_ns, callback_start - source.last_receive_ns);
            }
            source.last_receive_ns = callback_start;
            if(routed.origin.time_ns <= callback_start) {
                origin_age_.observe(callback_start - routed.origin.time_ns);
            } else {
                ++origin_clock_anomalies_;
            }

            const auto apply_start = steady_now_ns();
            const auto result = source.ingress.receive(routed, callback_start);
            const auto apply_end = steady_now_ns();
            apply_latency_.observe(apply_end - apply_start);
            if(result.status == IngressStatus::AppliedKeyframe
               || result.status == IngressStatus::AppliedDelta) {
                const auto & timing = source.ingress.map_applier().last_apply_timing();
                payload_decode_latency_.observe(timing.payload_decode_duration_ns);
                candidate_build_latency_.observe(timing.candidate_build_duration_ns);
                merkle_latency_.observe(timing.merkle_duration_ns);
                commit_latency_.observe(timing.commit_duration_ns);
            }
            if(is_applied(result.status)) {
                ++messages_applied_;
                ++source.messages_applied;
                source.last_sequence = routed.sequence;
                if(result.status == IngressStatus::AppliedKeyframe) {
                    ++keyframes_applied_;
                } else if(result.status == IngressStatus::AppliedDelta) {
                    ++deltas_applied_;
                    const auto & metrics = source.ingress.map_applier().storage_metrics();
                    delta_touched_chunks_.observe(metrics.touched_chunks);
                    delta_shared_chunks_.observe(metrics.shared_chunks);
                    delta_copied_cells_.observe(metrics.copied_cells);
                    delta_copied_bucket_entries_.observe(metrics.copied_bucket_entries);
                    delta_candidate_owned_bytes_.observe(metrics.candidate_owned_bytes);
                }
                write_ready_if_complete();
            } else if(result.status == IngressStatus::IgnoredDuplicate) {
                ++messages_duplicate_;
            } else {
                ++messages_rejected_;
            }
            callback_latency_.observe(steady_now_ns() - callback_start);
        }

        void write_ready_if_complete()
        {
            if(ready_written_ || ready_path_.empty()
               || sources_.size() != expected_sources_) {
                return;
            }
            for(const auto & [name, state] : sources_) {
                (void) name;
                const auto & map = state->ingress.map_applier().reconstructed_map();
                if(!map.has_value() || map->cells.size() != target_cells_per_source_) {
                    return;
                }
            }
            std::ofstream ready(ready_path_, std::ios::trunc);
            if(!ready) {
                return;
            }
            ready << "schema_version\t1\n"
                  << "sources_ready\t" << sources_.size() << '\n'
                  << "cells_per_source\t" << target_cells_per_source_ << '\n'
                  << "ready_monotonic_ns\t" << steady_now_ns() << '\n';
            ready.flush();
            ready_written_ = static_cast<bool>(ready);
        }

        void write_histogram(
                std::ofstream & output,
                const char * prefix,
                const BoundedHistogram & histogram) const
        {
            output << prefix << "_count\t" << histogram.count() << '\n';
            output << prefix << "_total_ns\t" << histogram.total_ns() << '\n';
            output << prefix << "_p50_upper_ns\t"
                   << histogram.percentile_upper_bound(0.50) << '\n';
            output << prefix << "_p95_upper_ns\t"
                   << histogram.percentile_upper_bound(0.95) << '\n';
            output << prefix << "_p99_upper_ns\t"
                   << histogram.percentile_upper_bound(0.99) << '\n';
            output << prefix << "_max_ns\t" << histogram.max_ns() << '\n';
        }

        void write_distribution(
                std::ofstream & output,
                const char * prefix,
                const BoundedHistogram & histogram) const
        {
            output << prefix << "_count\t" << histogram.count() << '\n';
            output << prefix << "_total\t" << histogram.total_ns() << '\n';
            output << prefix << "_p50_upper\t"
                   << histogram.percentile_upper_bound(0.50) << '\n';
            output << prefix << "_p95_upper\t"
                   << histogram.percentile_upper_bound(0.95) << '\n';
            output << prefix << "_max\t" << histogram.max_ns() << '\n';
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
            std::uint64_t final_cells = 0U;
            for(const auto & [name, state] : sources_) {
                (void) name;
                const auto & map = state->ingress.map_applier().reconstructed_map();
                if(map.has_value()) {
                    final_cells += map->cells.size();
                }
            }
            write_value(summary, "schema_version", 1U);
            summary << "content_identity_scheme\tmerkle-patricia-sha256-v2\n"
                    << "storage_mode\tchunked\n";
            write_value(summary, "chunk_edge", PerceptionMapUpdate::kMerkleChunkEdge);
            write_value(
                    summary,
                    "chunk_bucket_count",
                    PerceptionMapUpdate::kMerkleChunkBucketCount);
            write_value(summary, "expected_sources", expected_sources_);
            write_value(summary, "sources_seen", sources_.size());
            write_value(summary, "qos_history_depth", qos_depth_);
            write_value(summary, "application_queue_peak", 0U);
            write_value(
                    summary,
                    "max_recent_messages_per_source",
                    DataPlaneLimits {}.max_recent_messages);
            write_value(
                    summary,
                    "max_recent_messages_total",
                    DataPlaneLimits {}.max_recent_messages * expected_sources_);
            write_value(summary, "messages_received", messages_received_);
            write_value(summary, "messages_applied", messages_applied_);
            write_value(summary, "keyframes_applied", keyframes_applied_);
            write_value(summary, "deltas_applied", deltas_applied_);
            write_value(summary, "messages_duplicate", messages_duplicate_);
            write_value(summary, "messages_rejected", messages_rejected_);
            write_value(summary, "origin_clock_anomalies", origin_clock_anomalies_);
            write_value(summary, "endpoint_count_anomalies", endpoint_count_anomalies_);
            write_value(summary, "payload_bytes_received", payload_bytes_received_);
            write_value(summary, "final_cells_total", final_cells);
            write_histogram(summary, "decode", decode_latency_);
            write_histogram(summary, "apply", apply_latency_);
            write_histogram(summary, "payload_decode", payload_decode_latency_);
            write_histogram(summary, "candidate_build", candidate_build_latency_);
            write_histogram(summary, "merkle", merkle_latency_);
            write_histogram(summary, "commit", commit_latency_);
            write_histogram(summary, "callback", callback_latency_);
            write_histogram(summary, "origin_age", origin_age_);
            write_distribution(summary, "delta_touched_chunks", delta_touched_chunks_);
            write_distribution(summary, "delta_shared_chunks", delta_shared_chunks_);
            write_distribution(summary, "delta_copied_cells", delta_copied_cells_);
            write_distribution(
                    summary,
                    "delta_copied_bucket_entries",
                    delta_copied_bucket_entries_);
            write_distribution(
                    summary,
                    "delta_candidate_owned_bytes",
                    delta_candidate_owned_bytes_);
            summary << "normal_completion\t1\n";

            per_source << "source\tmessages_received\tmessages_applied\tlast_sequence\t"
                          "final_revision\tfinal_cells\tfinal_content_hash\t"
                          "payload_bytes_received\tmax_interarrival_ns\n";
            for(const auto & [name, state] : sources_) {
                const auto & map = state->ingress.map_applier().reconstructed_map();
                per_source << name << '\t' << state->messages_received << '\t'
                           << state->messages_applied << '\t' << state->last_sequence << '\t';
                if(map.has_value()) {
                    per_source << map->revision << '\t' << map->cells.size() << '\t'
                               << PerceptionMapUpdate::hash_to_hex(
                                          map->content_identity.digest);
                } else {
                    per_source << "0\t0\t";
                }
                per_source << '\t' << state->payload_bytes << '\t'
                           << state->max_interarrival_ns << '\n';
            }
        }

        std::size_t expected_sources_ = 0U;
        std::size_t qos_depth_ = 0U;
        std::size_t target_cells_per_source_ = 0U;
        std::string summary_path_;
        std::string per_source_path_;
        std::string ready_path_;
        bool ready_written_ = false;
        std::map<std::string, std::unique_ptr<SourceState>> sources_;
        std::uint64_t messages_received_ = 0U;
        std::uint64_t messages_applied_ = 0U;
        std::uint64_t keyframes_applied_ = 0U;
        std::uint64_t deltas_applied_ = 0U;
        std::uint64_t messages_duplicate_ = 0U;
        std::uint64_t messages_rejected_ = 0U;
        std::uint64_t origin_clock_anomalies_ = 0U;
        std::uint64_t endpoint_count_anomalies_ = 0U;
        std::uint64_t payload_bytes_received_ = 0U;
        BoundedHistogram decode_latency_;
        BoundedHistogram apply_latency_;
        BoundedHistogram payload_decode_latency_;
        BoundedHistogram candidate_build_latency_;
        BoundedHistogram merkle_latency_;
        BoundedHistogram commit_latency_;
        BoundedHistogram callback_latency_;
        BoundedHistogram origin_age_;
        BoundedHistogram delta_touched_chunks_;
        BoundedHistogram delta_shared_chunks_;
        BoundedHistogram delta_copied_cells_;
        BoundedHistogram delta_copied_bucket_entries_;
        BoundedHistogram delta_candidate_owned_bytes_;
        rclcpp::Subscription<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                subscription_;
    };

}// namespace SwarmDataPlane::Test

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::Test::C4ResourceProfileReceiver>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("c4_resource_profile_receiver"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
