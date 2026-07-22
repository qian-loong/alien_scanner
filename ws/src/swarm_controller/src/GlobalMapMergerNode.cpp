#include "swarm_controller/OctoMapMerger.hpp"
#include "swarm_controller/MapMergeDiagnostics.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>

namespace SwarmController {

    namespace {

        using SteadyClock = std::chrono::steady_clock;

        double steadySeconds(
                const SteadyClock::time_point start,
                const SteadyClock::time_point finish)
        {
            return std::chrono::duration<double>(finish - start).count();
        }

        std::string sourceOutcome(const SourceUpdateResult & result)
        {
            if(result.status == SourceUpdateStatus::AcceptedChanged) {
                return "accepted_changed";
            }
            if(result.status == SourceUpdateStatus::AcceptedUnchanged) {
                return "accepted_unchanged";
            }
            switch(result.failure_stage) {
            case SourceUpdateFailureStage::None:
            case SourceUpdateFailureStage::InvalidInput:
                return "delta_preflight_rejected";
            case SourceUpdateFailureStage::Normalize:
                return "normalize_rejected";
            case SourceUpdateFailureStage::Resource:
                return "resource_rejected";
            case SourceUpdateFailureStage::DeltaPreflight:
                return "delta_preflight_rejected";
            case SourceUpdateFailureStage::Commit:
                return "commit_exception";
            }
            return "delta_preflight_rejected";
        }

        diagnostic_msgs::msg::KeyValue value(
                const std::string & key, const std::string & content)
        {
            diagnostic_msgs::msg::KeyValue result;
            result.key   = key;
            result.value = content;
            return result;
        }

        template<typename T>
        diagnostic_msgs::msg::KeyValue numericValue(const std::string & key, const T content)
        {
            return value(key, std::to_string(content));
        }

        bool checkedMultiply(
                const std::size_t lhs, const std::size_t rhs, std::size_t & result)
        {
            if(lhs != 0U && rhs > std::numeric_limits<std::size_t>::max() / lhs) {
                return false;
            }
            result = lhs * rhs;
            return true;
        }

    }// namespace

    class GlobalMapMergerNode : public rclcpp::Node
    {
    public:
        GlobalMapMergerNode()
            : rclcpp::Node("global_map_merger")
        {
            declareParameters();
            loadConfiguration();
            initSources();
            initPublishers();
            initTimer();

            RCLCPP_INFO(
                    get_logger(),
                    "global_map_merger: %zu sources -> %s at %.2f Hz, resolution %.3f m",
                    sources_.size(), output_topic_.c_str(), merge_rate_hz_, resolution_);
        }

    private:
        struct SourceCycleRecord {
            bool valid {false};
            std::uint64_t cycle_id {};
            std::size_t source_index {};
            std::string topic;
            std::string outcome;
            std::string reason;
            std::uint64_t admission_sequence {};
            std::int64_t stamp_ns {};
            std::int64_t receive_ros_ns {};
            std::int64_t claim_ros_ns {};
            std::uint64_t receive_ros_epoch {};
            std::uint64_t claim_ros_epoch {};
            std::size_t coalesced_before_claim {};
            std::uint64_t source_revision_before {};
            std::uint64_t source_revision_after {};
            std::uint64_t global_revision_before {};
            std::uint64_t global_revision_after {};
            std::size_t added_keys {};
            std::size_t removed_keys {};
            std::size_t flipped_keys {};
            bool global_changed {false};
            bool merge_applied {false};
            double receive_to_claim_seconds {-1.0};
            double decode_seconds {};
            double decode_cleanup_seconds {};
            SourceUpdateStageTiming timing {};
            MergeHeaderAge receive_header_age {};
            MergeHeaderAge claim_header_age {};
        };

        struct SourceTrack {
            std::string topic;
            rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription;
            octomap_msgs::msg::Octomap::SharedPtr pending;
            std::int64_t pending_stamp_ns {0};
            std::uint64_t pending_admission_sequence {};
            SteadyClock::time_point pending_received_steady {};
            std::int64_t pending_received_ros_ns {};
            std::uint64_t pending_received_ros_epoch {};
            std::int64_t latest_enqueued_stamp_ns {0};
            std::int64_t last_accepted_stamp_ns {0};
            std::uint64_t last_accepted_ros_epoch {};
            SteadyClock::time_point last_received_steady {};
            SteadyClock::time_point last_accepted_steady {};
            std::size_t received_count {0U};
            std::size_t coalesced_count {0U};
            std::size_t pending_coalesced_count {0U};
            std::size_t accepted_count {0U};
            std::size_t unchanged_count {0U};
            std::size_t rejected_count {0U};
            std::size_t last_message_bytes {0U};
            std::size_t voxel_count {0U};
            std::string last_rejection;
            SourceCycleRecord last_record;
        };

        struct PendingMap {
            std::size_t index {0U};
            std::string topic;
            octomap_msgs::msg::Octomap::SharedPtr message;
            std::int64_t stamp_ns {0};
            std::uint64_t admission_sequence {};
            SteadyClock::time_point received_steady {};
            std::int64_t received_ros_ns {};
            std::uint64_t received_ros_epoch {};
            std::size_t coalesced_before_claim {};
            SteadyClock::time_point claim_steady {};
            std::int64_t claim_ros_ns {};
            std::uint64_t claim_ros_epoch {};
        };

        struct OutputPublishResult {
            bool serialize_attempted {false};
            bool serialize_succeeded {false};
            bool publish_attempted {false};
            bool publish_succeeded {false};
            std::int64_t output_stamp_ns {};
            std::uint64_t output_stamp_epoch {};
            std::uint64_t output_publish_sequence {};
            double serialize_seconds {};
            double output_prepare_seconds {};
            double publish_seconds {};
            MergeHeaderAge publish_header_age {};
        };

        struct MergeCycleRecord {
            bool valid {false};
            std::uint64_t cycle_id {};
            std::string outcome;
            std::string claim_source_order;
            std::string admission_sequence_order;
            std::uint64_t source_revision_before {};
            std::uint64_t source_revision_after {};
            std::uint64_t global_revision_before {};
            std::uint64_t global_revision_after {};
            std::size_t claimed_sources {};
            std::size_t changed_sources {};
            std::size_t unchanged_sources {};
            std::size_t rejected_sources {};
            bool merge_applied {false};
            OutputPublishResult output;
            double receive_to_claim_max_seconds {-1.0};
            MergeCycleStageDurations stages;
        };

        struct RosClockSample {
            std::int64_t now_ns {};
            std::uint64_t epoch {};
        };

        void declareParameters()
        {
            declare_parameter("map_frame", "map");
            declare_parameter<std::vector<std::string>>(
                    "source_topics", std::vector<std::string> {});
            declare_parameter("output_topic", "global_map");
            declare_parameter("diagnostics_topic", "global_map_diagnostics");
            declare_parameter("resolution", 0.1);
            declare_parameter("merge_rate", 1.0);
            declare_parameter("source_stale_timeout", 5.0);
            declare_parameter("max_serialized_bytes_per_source", std::int64_t {67'108'864});
            declare_parameter("max_voxels_per_source", std::int64_t {5'000'000});
            declare_parameter("max_global_voxels", std::int64_t {10'000'000});
            // Test-only hook; the default keeps the production path unchanged.
            declare_parameter("test_serialization_failures", std::int64_t {0});
        }

        static std::size_t positiveSize(
                const std::int64_t raw, const char * parameter_name)
        {
            if(raw <= 0) {
                throw std::invalid_argument(
                        std::string(parameter_name) + " must be positive");
            }
            if(static_cast<std::uint64_t>(raw)
               > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
            {
                throw std::invalid_argument(
                        std::string(parameter_name) + " exceeds size_t range");
            }
            return static_cast<std::size_t>(raw);
        }

        void loadConfiguration()
        {
            map_frame_         = get_parameter("map_frame").as_string();
            source_topics_     = get_parameter("source_topics").as_string_array();
            output_topic_      = get_parameter("output_topic").as_string();
            diagnostics_topic_ = get_parameter("diagnostics_topic").as_string();
            resolution_        = get_parameter("resolution").as_double();
            merge_rate_hz_     = get_parameter("merge_rate").as_double();
            stale_timeout_s_   = get_parameter("source_stale_timeout").as_double();
            max_serialized_bytes_per_source_ = positiveSize(
                    get_parameter("max_serialized_bytes_per_source").as_int(),
                    "max_serialized_bytes_per_source");
            max_voxels_per_source_ = positiveSize(
                    get_parameter("max_voxels_per_source").as_int(),
                    "max_voxels_per_source");
            max_global_voxels_ = positiveSize(
                    get_parameter("max_global_voxels").as_int(),
                    "max_global_voxels");
            const std::int64_t serialization_failures =
                    get_parameter("test_serialization_failures").as_int();
            if(serialization_failures < 0) {
                throw std::invalid_argument(
                        "test_serialization_failures must not be negative");
            }
            test_serialization_failures_ = static_cast<std::size_t>(serialization_failures);

            if(map_frame_.empty()) {
                throw std::invalid_argument("map_frame must not be empty");
            }
            if(output_topic_.empty() || diagnostics_topic_.empty()) {
                throw std::invalid_argument("output topics must not be empty");
            }
            if(source_topics_.empty()) {
                throw std::invalid_argument("source_topics must not be empty");
            }
            if(!std::isfinite(resolution_) || resolution_ <= 0.0) {
                throw std::invalid_argument("resolution must be positive and finite");
            }
            if(!std::isfinite(merge_rate_hz_) || merge_rate_hz_ <= 0.0) {
                throw std::invalid_argument("merge_rate must be positive and finite");
            }
            if(!std::isfinite(stale_timeout_s_) || stale_timeout_s_ <= 0.0) {
                throw std::invalid_argument("source_stale_timeout must be positive and finite");
            }

            std::size_t total_source_limit = 0U;
            if(!checkedMultiply(
                       source_topics_.size(), max_voxels_per_source_, total_source_limit))
            {
                throw std::invalid_argument("source_count * max_voxels_per_source overflows");
            }

            OctoMapMergerConfig merger_config;
            merger_config.resolution            = resolution_;
            merger_config.max_voxels_per_source = max_voxels_per_source_;
            merger_config.max_global_voxels     = max_global_voxels_;
            merger_ = std::make_unique<OctoMapMerger>(merger_config);
        }

        void initSources()
        {
            std::vector<std::string> resolved;
            resolved.reserve(source_topics_.size());
            for(const std::string & topic : source_topics_) {
                if(topic.empty()) {
                    throw std::invalid_argument("source_topics contains an empty topic");
                }
                resolved.push_back(
                        get_node_topics_interface()->resolve_topic_name(topic, false));
            }
            std::sort(resolved.begin(), resolved.end());
            const auto duplicate = std::adjacent_find(resolved.begin(), resolved.end());
            if(duplicate != resolved.end()) {
                throw std::invalid_argument(
                        "source_topics contains duplicate resolved topic '" + *duplicate + "'");
            }

            sources_.reserve(resolved.size());
            for(const std::string & topic : resolved) {
                SourceTrack track;
                track.topic = topic;
                sources_.push_back(std::move(track));
            }

            const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            for(std::size_t i = 0U; i < sources_.size(); ++i) {
                sources_[i].subscription = create_subscription<octomap_msgs::msg::Octomap>(
                        sources_[i].topic, qos,
                        [this, i](const octomap_msgs::msg::Octomap::SharedPtr message) {
                            onSourceMap(i, message);
                        });
            }
        }

        void initPublishers()
        {
            const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
            map_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(output_topic_, qos);
            diagnostics_publisher_ =
                    create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                            diagnostics_topic_, qos);
        }

        void initTimer()
        {
            const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / merge_rate_hz_));
            timer_ = create_wall_timer(period, [this]() {
                onMergeTimer();
            });
        }

        RosClockSample observeRosClockLocked(const std::int64_t now_ns)
        {
            if(ros_clock_initialized_ && now_ns < last_ros_now_ns_) {
                ++ros_clock_epoch_;
            }
            ros_clock_initialized_ = true;
            last_ros_now_ns_ = now_ns;
            return RosClockSample {now_ns, ros_clock_epoch_};
        }

        void rejectEnvelope(
                const std::size_t index, SourceTrack & source,
                const std::int64_t stamp_ns, const std::int64_t receive_ros_ns,
                const std::uint64_t receive_ros_epoch, const std::string & reason,
                const bool throttle_log)
        {
            ++source.rejected_count;
            source.last_rejection = reason;
            SourceCycleRecord record;
            record.valid = true;
            record.source_index = index;
            record.topic = source.topic;
            record.outcome = "envelope_rejected";
            record.reason = reason;
            record.stamp_ns = stamp_ns;
            record.receive_ros_ns = receive_ros_ns;
            record.receive_ros_epoch = receive_ros_epoch;
            record.receive_header_age = mergeHeaderAge(
                    stamp_ns, receive_ros_ns, receive_ros_epoch, receive_ros_epoch);
            source.last_record = std::move(record);
            if(throttle_log) {
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s: %s",
                        source.topic.c_str(), reason.c_str());
            }
        }

        void onSourceMap(
                const std::size_t index,
                const octomap_msgs::msg::Octomap::SharedPtr message)
        {
            const auto now = SteadyClock::now();
            const std::int64_t receive_ros_ns = now_ros().nanoseconds();
            const std::int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();

            std::lock_guard<std::mutex> lock(mutex_);
            const RosClockSample receive_ros = observeRosClockLocked(receive_ros_ns);
            SourceTrack & source = sources_.at(index);
            ++source.received_count;
            source.last_received_steady = now;

            if(message->header.frame_id != map_frame_) {
                rejectEnvelope(
                        index, source, stamp_ns, receive_ros.now_ns, receive_ros.epoch,
                        "frame_id does not match map_frame", true);
                return;
            }
            if(message->binary) {
                rejectEnvelope(
                        index, source, stamp_ns, receive_ros.now_ns, receive_ros.epoch,
                        "binary Octomap is not accepted; expected full map", true);
                return;
            }
            if(message->data.size() > max_serialized_bytes_per_source_) {
                rejectEnvelope(
                        index, source, stamp_ns, receive_ros.now_ns, receive_ros.epoch,
                        "serialized map exceeds byte limit", true);
                return;
            }
            if(stamp_ns <= 0) {
                rejectEnvelope(
                        index, source, stamp_ns, receive_ros.now_ns, receive_ros.epoch,
                        "observation stamp must be nonzero", true);
                return;
            }
            if(stamp_ns <= source.latest_enqueued_stamp_ns) {
                rejectEnvelope(
                        index, source, stamp_ns, receive_ros.now_ns, receive_ros.epoch,
                        "observation stamp is not strictly increasing", false);
                return;
            }

            if(source.pending) {
                ++source.coalesced_count;
                ++source.pending_coalesced_count;
            } else {
                source.pending_coalesced_count = 0U;
            }
            source.pending          = message;
            source.pending_stamp_ns = stamp_ns;
            source.pending_admission_sequence = next_admission_sequence_++;
            source.pending_received_steady = now;
            source.pending_received_ros_ns = receive_ros.now_ns;
            source.pending_received_ros_epoch = receive_ros.epoch;
            source.latest_enqueued_stamp_ns = stamp_ns;
        }

        void recordProcessedResult(
                const std::size_t index, const PendingMap & pending,
                const SourceUpdateResult & result, SourceCycleRecord record,
                const SteadyClock::time_point now)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            SourceTrack & source = sources_.at(index);
            if(!result.accepted()) {
                ++source.rejected_count;
                source.last_rejection = result.reason;
                source.last_record = std::move(record);
                // A deeply invalid snapshot must not permanently raise the admission
                // watermark. Preserve it only when a newer snapshot arrived while this
                // one was being processed.
                if(source.latest_enqueued_stamp_ns == pending.stamp_ns) {
                    source.latest_enqueued_stamp_ns = std::max(
                            source.last_accepted_stamp_ns, source.pending_stamp_ns);
                }
                RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s: %s",
                        source.topic.c_str(), result.reason.c_str());
                return;
            }

            ++source.accepted_count;
            if(result.status == SourceUpdateStatus::AcceptedUnchanged) {
                ++source.unchanged_count;
            }
            source.last_accepted_stamp_ns = pending.stamp_ns;
            source.last_accepted_ros_epoch = pending.received_ros_epoch;
            source.last_accepted_steady   = now;
            source.last_message_bytes     = pending.message->data.size();
            source.voxel_count            = merger_->sourceVoxelCount(source.topic);
            source.last_rejection.clear();
            source.last_record = std::move(record);
        }

        void onMergeTimer()
        {
            const auto merge_start = SteadyClock::now();
            const auto claim_start = SteadyClock::now();
            const std::int64_t claim_ros_ns = now_ros().nanoseconds();
            std::vector<PendingMap> pending_maps;
            RosClockSample claim_ros;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                claim_ros = observeRosClockLocked(claim_ros_ns);
                pending_maps.reserve(sources_.size());
                for(std::size_t i = 0U; i < sources_.size(); ++i) {
                    if(!sources_[i].pending) {
                        continue;
                    }
                    pending_maps.push_back(PendingMap {
                            i,
                            sources_[i].topic,
                            std::move(sources_[i].pending),
                            sources_[i].pending_stamp_ns,
                            sources_[i].pending_admission_sequence,
                            sources_[i].pending_received_steady,
                            sources_[i].pending_received_ros_ns,
                            sources_[i].pending_received_ros_epoch,
                            sources_[i].pending_coalesced_count,
                            claim_start,
                            claim_ros.now_ns,
                            claim_ros.epoch,
                    });
                    sources_[i].pending_stamp_ns = 0;
                    sources_[i].pending_admission_sequence = 0;
                    sources_[i].pending_received_steady = {};
                    sources_[i].pending_received_ros_ns = 0;
                    sources_[i].pending_received_ros_epoch = 0;
                    sources_[i].pending_coalesced_count = 0U;
                }
            }

            const double claim_seconds = steadySeconds(claim_start, SteadyClock::now());
            if(pending_maps.empty()) {
                const double duration_ms = std::chrono::duration<double, std::milli>(
                                                   SteadyClock::now() - merge_start)
                                                   .count();
                last_merge_duration_ms_ = duration_ms;
                max_merge_duration_ms_  = std::max(max_merge_duration_ms_, duration_ms);
                publishDiagnostics();
                return;
            }

            MergeCycleRecord cycle;
            cycle.valid = true;
            cycle.cycle_id = ++merge_cycle_id_;
            cycle.claimed_sources = pending_maps.size();
            cycle.source_revision_before = merger_->sourceRevision();
            cycle.global_revision_before = merger_->globalRevision();
            cycle.stages.claim_seconds = claim_seconds;
            cycle.receive_to_claim_max_seconds = 0.0;

            std::size_t accepted_this_cycle = 0U;
            std::size_t rejected_this_cycle = 0U;
            for(const PendingMap & pending : pending_maps) {
                SourceCycleRecord record;
                record.valid = true;
                record.cycle_id = cycle.cycle_id;
                record.source_index = pending.index;
                record.topic = pending.topic;
                record.admission_sequence = pending.admission_sequence;
                record.stamp_ns = pending.stamp_ns;
                record.receive_ros_ns = pending.received_ros_ns;
                record.claim_ros_ns = pending.claim_ros_ns;
                record.receive_ros_epoch = pending.received_ros_epoch;
                record.claim_ros_epoch = pending.claim_ros_epoch;
                record.coalesced_before_claim = pending.coalesced_before_claim;
                record.receive_to_claim_seconds = steadySeconds(
                        pending.received_steady, pending.claim_steady);
                record.receive_header_age = mergeHeaderAge(
                        pending.stamp_ns, pending.received_ros_ns,
                        pending.received_ros_epoch, pending.received_ros_epoch);
                record.claim_header_age = mergeHeaderAge(
                        pending.stamp_ns, pending.claim_ros_ns,
                        pending.received_ros_epoch, pending.claim_ros_epoch);
                cycle.receive_to_claim_max_seconds = std::max(
                        cycle.receive_to_claim_max_seconds,
                        std::max(0.0, record.receive_to_claim_seconds));

                record.source_revision_before = merger_->sourceRevision();
                record.global_revision_before = merger_->globalRevision();
                SourceUpdateResult result;
                bool decode_failure = false;

                const auto decode_start = SteadyClock::now();
                std::unique_ptr<octomap::AbstractOcTree> abstract;
                octomap::OcTree * source_tree = nullptr;
                try {
                    abstract.reset(octomap_msgs::msgToMap(*pending.message));
                    source_tree = dynamic_cast<octomap::OcTree *>(abstract.get());
                    if(source_tree == nullptr) {
                        decode_failure = true;
                        result.reason = "message does not contain an OcTree";
                    }
                } catch(const std::exception & exception) {
                    decode_failure = true;
                    result.reason = std::string("Octomap deserialization failed: ")
                                    + exception.what();
                } catch(...) {
                    decode_failure = true;
                    result.reason = "Octomap deserialization failed with unknown exception";
                }
                record.decode_seconds = steadySeconds(decode_start, SteadyClock::now());
                cycle.stages.decode_seconds += record.decode_seconds;

                if(!decode_failure) {
                    // Commit-stage allocation failures are process-fatal by contract;
                    // do not continue with a possibly partially mutated third-party tree.
                    result = merger_->updateSource(pending.topic, *source_tree);
                }

                const auto decode_cleanup_start = SteadyClock::now();
                abstract.reset();
                record.decode_cleanup_seconds = steadySeconds(
                        decode_cleanup_start, SteadyClock::now());
                cycle.stages.decode_cleanup_seconds += record.decode_cleanup_seconds;

                if(decode_failure) {
                    result.status = SourceUpdateStatus::Invalid;
                    result.failure_stage = SourceUpdateFailureStage::InvalidInput;
                    result.source_revision = merger_->sourceRevision();
                    result.global_revision = merger_->globalRevision();
                }

                record.source_revision_after = result.source_revision;
                record.global_revision_after = result.global_revision;
                record.added_keys = result.added_keys;
                record.removed_keys = result.removed_keys;
                record.flipped_keys = result.flipped_keys;
                record.global_changed = result.global_changed;
                record.merge_applied = result.accepted();
                record.timing = result.timing;
                if(decode_failure) {
                    record.outcome = "decode_failure";
                } else {
                    record.outcome = sourceOutcome(result);
                }
                record.reason = result.reason;

                cycle.stages.normalize_seconds += result.timing.normalize_seconds;
                cycle.stages.snapshot_compare_seconds += result.timing.snapshot_compare_seconds;
                cycle.stages.delta_preflight_seconds += result.timing.delta_preflight_seconds;
                cycle.stages.contribution_tree_apply_seconds +=
                        result.timing.contribution_tree_apply_seconds;
                cycle.stages.update_inner_occupancy_seconds +=
                        result.timing.update_inner_occupancy_seconds;
                cycle.stages.source_commit_seconds += result.timing.source_commit_seconds;

                const auto bookkeeping_start = SteadyClock::now();
                recordProcessedResult(
                        pending.index, pending, result, std::move(record), SteadyClock::now());
                cycle.stages.bookkeeping_seconds += steadySeconds(
                        bookkeeping_start, SteadyClock::now());

                if(result.accepted()) {
                    ++accepted_this_cycle;
                    if(result.status == SourceUpdateStatus::AcceptedChanged) {
                        ++cycle.changed_sources;
                    } else {
                        ++cycle.unchanged_sources;
                    }
                } else {
                    ++rejected_this_cycle;
                    ++cycle.rejected_sources;
                }

                if(!cycle.claim_source_order.empty()) {
                    cycle.claim_source_order += ',';
                    cycle.admission_sequence_order += ',';
                }
                cycle.claim_source_order += pending.topic;
                cycle.admission_sequence_order +=
                        std::to_string(pending.admission_sequence);
            }

            if(accepted_this_cycle > 0U) {
                cycle.output = publishMap();
                cycle.stages.serialize_seconds = cycle.output.serialize_seconds;
                cycle.stages.output_prepare_seconds = cycle.output.output_prepare_seconds;
                cycle.stages.publish_seconds = cycle.output.publish_seconds;
            }

            cycle.source_revision_after = merger_->sourceRevision();
            cycle.global_revision_after = merger_->globalRevision();
            const MergeCycleCompletion completion = completeMergeCycle(
                    accepted_this_cycle, rejected_this_cycle,
                    cycle.output.serialize_succeeded, cycle.output.publish_succeeded);
            cycle.merge_applied = completion.merge_applied;
            cycle.outcome = completion.outcome;

            cycle.stages.finishAccounting(steadySeconds(merge_start, SteadyClock::now()));
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_cycle_ = cycle;
            }

            const double duration_ms = cycle.stages.cycle_total_seconds * 1000.0;
            last_merge_duration_ms_ = duration_ms;
            max_merge_duration_ms_  = std::max(max_merge_duration_ms_, duration_ms);
            last_cycle_rejected_    = rejected_this_cycle;
            publishDiagnostics();
        }

        OutputPublishResult publishMap()
        {
            OutputPublishResult result;
            result.serialize_attempted = true;
            const auto serialize_start = SteadyClock::now();
            octomap_msgs::msg::Octomap message;
            try {
                if(test_serialization_failures_ > 0U) {
                    --test_serialization_failures_;
                    throw std::runtime_error("injected serialization failure");
                }
                if(!octomap_msgs::fullMapToMsg(merger_->tree(), message)) {
                    result.serialize_seconds = steadySeconds(
                            serialize_start, SteadyClock::now());
                    serialization_error_ = "failed to serialize merged OcTree";
                    RCLCPP_ERROR_THROTTLE(
                            get_logger(), *get_clock(), 5000, "%s",
                            serialization_error_.c_str());
                    return result;
                }
            } catch(const std::bad_alloc &) {
                result.serialize_seconds = steadySeconds(
                        serialize_start, SteadyClock::now());
                serialization_error_ = "merged OcTree serialization allocation failed";
                RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s",
                        serialization_error_.c_str());
                return result;
            } catch(const std::exception &) {
                result.serialize_seconds = steadySeconds(
                        serialize_start, SteadyClock::now());
                serialization_error_ = "merged OcTree serialization threw an exception";
                RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s",
                        serialization_error_.c_str());
                return result;
            } catch(...) {
                result.serialize_seconds = steadySeconds(
                        serialize_start, SteadyClock::now());
                serialization_error_ =
                        "merged OcTree serialization threw an unknown exception";
                RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s",
                        serialization_error_.c_str());
                return result;
            }
            result.serialize_succeeded = true;
            result.serialize_seconds = steadySeconds(serialize_start, SteadyClock::now());
            serialization_error_.clear();

            const auto prepare_start = SteadyClock::now();
            std::int64_t latest_stamp_ns = 0;
            std::uint64_t latest_stamp_epoch = 0;
            bool ambiguous_stamp_epoch = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for(const SourceTrack & source : sources_) {
                    if(source.last_accepted_stamp_ns > latest_stamp_ns) {
                        latest_stamp_ns = source.last_accepted_stamp_ns;
                        latest_stamp_epoch = source.last_accepted_ros_epoch;
                        ambiguous_stamp_epoch = false;
                    } else if(source.last_accepted_stamp_ns > 0
                              && source.last_accepted_stamp_ns == latest_stamp_ns
                              && source.last_accepted_ros_epoch != latest_stamp_epoch)
                    {
                        ambiguous_stamp_epoch = true;
                    }
                }
            }
            result.output_stamp_ns = latest_stamp_ns;
            result.output_stamp_epoch = ambiguous_stamp_epoch
                                                ? std::numeric_limits<std::uint64_t>::max()
                                                : latest_stamp_epoch;
            message.header.frame_id = map_frame_;
            message.header.stamp = rclcpp::Time(
                    latest_stamp_ns, get_clock()->get_clock_type());
            result.output_prepare_seconds = steadySeconds(
                    prepare_start, SteadyClock::now());

            result.publish_attempted = true;
            const auto publish_start = SteadyClock::now();
            try {
                map_publisher_->publish(message);
                result.publish_succeeded = true;
                result.output_publish_sequence = ++output_publish_sequence_;
            } catch(const std::exception & exception) {
                publish_error_ = std::string("global map publish failed: ") + exception.what();
                RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s", publish_error_.c_str());
            } catch(...) {
                publish_error_ = "global map publish failed with unknown exception";
                RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s", publish_error_.c_str());
            }
            result.publish_seconds = steadySeconds(publish_start, SteadyClock::now());
            if(result.publish_succeeded) {
                publish_error_.clear();
            }

            const std::int64_t publish_ros_ns = now_ros().nanoseconds();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                const RosClockSample publish_ros = observeRosClockLocked(publish_ros_ns);
                result.publish_header_age = mergeHeaderAge(
                        result.output_stamp_ns, publish_ros.now_ns,
                        result.output_stamp_epoch, publish_ros.epoch);
            }
            return result;
        }

        void publishDiagnostics()
        {
            const auto now = SteadyClock::now();
            std::size_t received_sources = 0U;
            std::size_t accepted_sources = 0U;
            std::size_t stale_sources    = 0U;
            std::size_t received_updates = 0U;
            std::size_t coalesced_updates = 0U;
            std::size_t accepted_updates = 0U;
            std::size_t unchanged_updates = 0U;
            std::size_t rejected_updates = 0U;
            std::size_t max_source_voxels = 0U;
            std::size_t max_message_bytes = 0U;
            std::vector<std::pair<std::string, std::size_t>> source_voxel_counts;
            source_voxel_counts.reserve(sources_.size());
            MergeCycleRecord last_cycle;
            std::vector<SourceCycleRecord> last_source_records;
            last_source_records.reserve(sources_.size());
            std::ostringstream missing_list;
            std::ostringstream stale_list;
            std::ostringstream rejected_list;

            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name        = "global_map_merger";
            status.hardware_id = "swarm";

            {
                std::lock_guard<std::mutex> lock(mutex_);
                for(const SourceTrack & source : sources_) {
                    received_updates += source.received_count;
                    coalesced_updates += source.coalesced_count;
                    accepted_updates += source.accepted_count;
                    unchanged_updates += source.unchanged_count;
                    rejected_updates += source.rejected_count;
                    max_source_voxels = std::max(max_source_voxels, source.voxel_count);
                    max_message_bytes = std::max(max_message_bytes, source.last_message_bytes);
                    source_voxel_counts.emplace_back(source.topic, source.voxel_count);
                    if(source.received_count > 0U) {
                        ++received_sources;
                    }
                    if(source.accepted_count == 0U) {
                        if(missing_list.tellp() > 0) {
                            missing_list << ',';
                        }
                        missing_list << source.topic;
                    } else {
                        ++accepted_sources;
                        const double age = std::chrono::duration<double>(
                                                   now - source.last_accepted_steady)
                                                   .count();
                        if(age > stale_timeout_s_) {
                            ++stale_sources;
                            if(stale_list.tellp() > 0) {
                                stale_list << ',';
                            }
                            stale_list << source.topic;
                        }
                    }
                    if(!source.last_rejection.empty()) {
                        if(rejected_list.tellp() > 0) {
                            rejected_list << ',';
                        }
                        rejected_list << source.topic << ':' << source.last_rejection;
                    }
                    if(source.last_record.valid) {
                        last_source_records.push_back(source.last_record);
                    }
                }
                last_cycle = last_cycle_;
            }

            const double source_utilization = static_cast<double>(max_source_voxels)
                                              / max_voxels_per_source_;
            const double global_utilization = static_cast<double>(merger_->knownCount())
                                              / max_global_voxels_;
            const double serialized_utilization = static_cast<double>(max_message_bytes)
                                                  / max_serialized_bytes_per_source_;
            const double max_utilization = std::max(
                    {source_utilization, global_utilization, serialized_utilization});

            if(!serialization_error_.empty()) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = serialization_error_;
            } else if(!publish_error_.empty()) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                status.message = publish_error_;
            } else if(accepted_sources < sources_.size()) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "waiting for valid source maps";
            } else if(stale_sources > 0U) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "one or more accepted source maps are stale";
            } else if(!rejected_list.str().empty() || last_cycle_rejected_ > 0U) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "one or more source updates were rejected";
            } else if(max_utilization >= 0.8) {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "map resource utilization is above 80 percent";
            } else {
                status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = "all source maps accepted";
            }

            status.values.push_back(numericValue("expected_sources", sources_.size()));
            status.values.push_back(numericValue("received_sources", received_sources));
            status.values.push_back(numericValue("accepted_sources", accepted_sources));
            status.values.push_back(numericValue("missing_sources", sources_.size() - accepted_sources));
            status.values.push_back(numericValue("stale_sources", stale_sources));
            status.values.push_back(value("missing_source_topics", missing_list.str()));
            status.values.push_back(value("stale_source_topics", stale_list.str()));
            status.values.push_back(value("last_rejections", rejected_list.str()));
            status.values.push_back(numericValue("received_updates", received_updates));
            status.values.push_back(numericValue("coalesced_updates", coalesced_updates));
            status.values.push_back(numericValue("accepted_updates", accepted_updates));
            status.values.push_back(numericValue("unchanged_updates", unchanged_updates));
            status.values.push_back(numericValue("rejected_updates", rejected_updates));
            status.values.push_back(numericValue("source_revision", merger_->sourceRevision()));
            status.values.push_back(numericValue("global_revision", merger_->globalRevision()));
            status.values.push_back(numericValue("known_voxels", merger_->knownCount()));
            status.values.push_back(numericValue("free_voxels", merger_->freeCount()));
            status.values.push_back(numericValue("occupied_voxels", merger_->occupiedCount()));
            status.values.push_back(numericValue(
                    "total_source_voxels", merger_->totalSourceVoxelCount()));
            for(const auto & [topic, voxel_count] : source_voxel_counts) {
                status.values.push_back(numericValue(
                        "source_voxels:" + topic, voxel_count));
            }
            status.values.push_back(numericValue("max_source_voxels", max_source_voxels));
            status.values.push_back(numericValue("max_serialized_bytes", max_message_bytes));
            status.values.push_back(numericValue(
                    "max_serialized_bytes_per_source", max_serialized_bytes_per_source_));
            status.values.push_back(numericValue(
                    "max_voxels_per_source", max_voxels_per_source_));
            status.values.push_back(numericValue("max_global_voxels", max_global_voxels_));
            status.values.push_back(numericValue(
                    "source_voxel_utilization", source_utilization));
            status.values.push_back(numericValue(
                    "global_voxel_utilization", global_utilization));
            status.values.push_back(numericValue(
                    "serialized_byte_utilization", serialized_utilization));
            status.values.push_back(numericValue(
                    "max_resource_utilization", max_utilization));
            status.values.push_back(numericValue(
                    "last_merge_duration_ms", last_merge_duration_ms_));
            status.values.push_back(numericValue(
                    "max_merge_duration_ms", max_merge_duration_ms_));

            status.values.push_back(numericValue(
                    "merge_cycle_valid", static_cast<int>(last_cycle.valid)));
            status.values.push_back(numericValue("merge_cycle_id", last_cycle.cycle_id));
            status.values.push_back(value("merge_cycle_outcome", last_cycle.outcome));
            status.values.push_back(value(
                    "merge_cycle_claim_source_order", last_cycle.claim_source_order));
            status.values.push_back(value(
                    "merge_cycle_admission_sequence_order",
                    last_cycle.admission_sequence_order));
            status.values.push_back(numericValue(
                    "merge_cycle_source_revision_before",
                    last_cycle.source_revision_before));
            status.values.push_back(numericValue(
                    "merge_cycle_source_revision_after",
                    last_cycle.source_revision_after));
            status.values.push_back(numericValue(
                    "merge_cycle_global_revision_before",
                    last_cycle.global_revision_before));
            status.values.push_back(numericValue(
                    "merge_cycle_global_revision_after",
                    last_cycle.global_revision_after));
            status.values.push_back(numericValue(
                    "merge_cycle_claimed_sources", last_cycle.claimed_sources));
            status.values.push_back(numericValue(
                    "merge_cycle_changed_sources", last_cycle.changed_sources));
            status.values.push_back(numericValue(
                    "merge_cycle_unchanged_sources", last_cycle.unchanged_sources));
            status.values.push_back(numericValue(
                    "merge_cycle_rejected_sources", last_cycle.rejected_sources));
            status.values.push_back(numericValue(
                    "merge_cycle_merge_applied",
                    static_cast<int>(last_cycle.merge_applied)));
            status.values.push_back(numericValue(
                    "merge_cycle_receive_to_claim_max_seconds",
                    last_cycle.receive_to_claim_max_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_claim_seconds", last_cycle.stages.claim_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_decode_seconds", last_cycle.stages.decode_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_decode_cleanup_seconds",
                    last_cycle.stages.decode_cleanup_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_normalize_seconds", last_cycle.stages.normalize_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_snapshot_compare_seconds",
                    last_cycle.stages.snapshot_compare_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_delta_preflight_seconds",
                    last_cycle.stages.delta_preflight_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_contribution_tree_apply_seconds",
                    last_cycle.stages.contribution_tree_apply_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_update_inner_occupancy_seconds",
                    last_cycle.stages.update_inner_occupancy_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_source_commit_seconds",
                    last_cycle.stages.source_commit_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_bookkeeping_seconds",
                    last_cycle.stages.bookkeeping_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_serialize_seconds", last_cycle.stages.serialize_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_output_prepare_seconds",
                    last_cycle.stages.output_prepare_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_publish_seconds", last_cycle.stages.publish_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_total_seconds", last_cycle.stages.cycle_total_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_accounting_remainder_seconds",
                    last_cycle.stages.accounting_remainder_seconds));
            status.values.push_back(numericValue(
                    "merge_cycle_serialize_attempted",
                    static_cast<int>(last_cycle.output.serialize_attempted)));
            status.values.push_back(numericValue(
                    "merge_cycle_serialize_succeeded",
                    static_cast<int>(last_cycle.output.serialize_succeeded)));
            status.values.push_back(numericValue(
                    "merge_cycle_publish_attempted",
                    static_cast<int>(last_cycle.output.publish_attempted)));
            status.values.push_back(numericValue(
                    "merge_cycle_publish_succeeded",
                    static_cast<int>(last_cycle.output.publish_succeeded)));
            status.values.push_back(numericValue(
                    "merge_cycle_output_publish_sequence",
                    last_cycle.output.output_publish_sequence));
            status.values.push_back(numericValue(
                    "merge_cycle_output_stamp_ns", last_cycle.output.output_stamp_ns));
            status.values.push_back(numericValue(
                    "merge_cycle_output_header_age_valid",
                    static_cast<int>(last_cycle.output.publish_header_age.valid)));
            status.values.push_back(numericValue(
                    "merge_cycle_output_header_age_seconds",
                    last_cycle.output.publish_header_age.seconds));

            for(const SourceCycleRecord & record : last_source_records) {
                const std::string prefix =
                        "last_source_record:" + std::to_string(record.source_index) + ":";
                status.values.push_back(value(prefix + "topic", record.topic));
                status.values.push_back(numericValue(prefix + "cycle_id", record.cycle_id));
                status.values.push_back(value(prefix + "outcome", record.outcome));
                status.values.push_back(value(prefix + "reason", record.reason));
                status.values.push_back(numericValue(
                        prefix + "admission_sequence", record.admission_sequence));
                status.values.push_back(numericValue(prefix + "stamp_ns", record.stamp_ns));
                status.values.push_back(numericValue(
                        prefix + "receive_ros_ns", record.receive_ros_ns));
                status.values.push_back(numericValue(
                        prefix + "claim_ros_ns", record.claim_ros_ns));
                status.values.push_back(numericValue(
                        prefix + "receive_ros_epoch", record.receive_ros_epoch));
                status.values.push_back(numericValue(
                        prefix + "claim_ros_epoch", record.claim_ros_epoch));
                status.values.push_back(numericValue(
                        prefix + "coalesced_before_claim", record.coalesced_before_claim));
                status.values.push_back(numericValue(
                        prefix + "source_revision_before", record.source_revision_before));
                status.values.push_back(numericValue(
                        prefix + "source_revision_after", record.source_revision_after));
                status.values.push_back(numericValue(
                        prefix + "global_revision_before", record.global_revision_before));
                status.values.push_back(numericValue(
                        prefix + "global_revision_after", record.global_revision_after));
                status.values.push_back(numericValue(
                        prefix + "added_keys", record.added_keys));
                status.values.push_back(numericValue(
                        prefix + "removed_keys", record.removed_keys));
                status.values.push_back(numericValue(
                        prefix + "flipped_keys", record.flipped_keys));
                status.values.push_back(numericValue(
                        prefix + "global_changed", static_cast<int>(record.global_changed)));
                status.values.push_back(numericValue(
                        prefix + "merge_applied", static_cast<int>(record.merge_applied)));
                status.values.push_back(numericValue(
                        prefix + "receive_to_claim_seconds",
                        record.receive_to_claim_seconds));
                status.values.push_back(numericValue(
                        prefix + "decode_seconds", record.decode_seconds));
                status.values.push_back(numericValue(
                        prefix + "decode_cleanup_seconds",
                        record.decode_cleanup_seconds));
                status.values.push_back(numericValue(
                        prefix + "normalize_seconds", record.timing.normalize_seconds));
                status.values.push_back(numericValue(
                        prefix + "snapshot_compare_seconds",
                        record.timing.snapshot_compare_seconds));
                status.values.push_back(numericValue(
                        prefix + "delta_preflight_seconds",
                        record.timing.delta_preflight_seconds));
                status.values.push_back(numericValue(
                        prefix + "contribution_tree_apply_seconds",
                        record.timing.contribution_tree_apply_seconds));
                status.values.push_back(numericValue(
                        prefix + "update_inner_occupancy_seconds",
                        record.timing.update_inner_occupancy_seconds));
                status.values.push_back(numericValue(
                        prefix + "source_commit_seconds",
                        record.timing.source_commit_seconds));
                status.values.push_back(numericValue(
                        prefix + "receive_header_age_valid",
                        static_cast<int>(record.receive_header_age.valid)));
                status.values.push_back(numericValue(
                        prefix + "receive_header_age_seconds",
                        record.receive_header_age.seconds));
                status.values.push_back(numericValue(
                        prefix + "claim_header_age_valid",
                        static_cast<int>(record.claim_header_age.valid)));
                status.values.push_back(numericValue(
                        prefix + "claim_header_age_seconds",
                        record.claim_header_age.seconds));
            }

            diagnostic_msgs::msg::DiagnosticArray array;
            array.header.stamp = now_ros();
            array.status.push_back(std::move(status));
            diagnostics_publisher_->publish(array);
        }

        rclcpp::Time now_ros() const
        {
            return get_clock()->now();
        }

        std::unique_ptr<OctoMapMerger> merger_;
        std::vector<std::string> source_topics_;
        std::vector<SourceTrack> sources_;
        std::mutex mutex_;

        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr map_publisher_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string map_frame_;
        std::string output_topic_;
        std::string diagnostics_topic_;
        double      resolution_ {0.1};
        double      merge_rate_hz_ {1.0};
        double      stale_timeout_s_ {5.0};
        std::size_t max_serialized_bytes_per_source_ {67'108'864U};
        std::size_t max_voxels_per_source_ {5'000'000U};
        std::size_t max_global_voxels_ {10'000'000U};
        std::size_t test_serialization_failures_ {0U};
        double      last_merge_duration_ms_ {0.0};
        double      max_merge_duration_ms_ {0.0};
        std::size_t last_cycle_rejected_ {0U};
        std::string serialization_error_;
        std::string publish_error_;
        std::uint64_t next_admission_sequence_ {1U};
        std::uint64_t merge_cycle_id_ {0U};
        std::uint64_t output_publish_sequence_ {0U};
        std::int64_t  last_ros_now_ns_ {};
        std::uint64_t ros_clock_epoch_ {};
        bool          ros_clock_initialized_ {false};
        MergeCycleRecord last_cycle_;
    };

}// namespace SwarmController

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmController::GlobalMapMergerNode>());
    } catch(const std::exception & exception) {
        RCLCPP_FATAL(rclcpp::get_logger("global_map_merger"), "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
