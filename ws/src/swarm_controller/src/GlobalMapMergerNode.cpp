#include "swarm_controller/OctoMapMerger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
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
        struct SourceTrack {
            std::string topic;
            rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subscription;
            octomap_msgs::msg::Octomap::SharedPtr pending;
            std::int64_t pending_stamp_ns {0};
            std::int64_t latest_enqueued_stamp_ns {0};
            std::int64_t last_accepted_stamp_ns {0};
            SteadyClock::time_point last_received_steady {};
            SteadyClock::time_point last_accepted_steady {};
            std::size_t received_count {0U};
            std::size_t coalesced_count {0U};
            std::size_t accepted_count {0U};
            std::size_t unchanged_count {0U};
            std::size_t rejected_count {0U};
            std::size_t last_message_bytes {0U};
            std::size_t voxel_count {0U};
            std::string last_rejection;
        };

        struct PendingMap {
            std::size_t index {0U};
            octomap_msgs::msg::Octomap::SharedPtr message;
            std::int64_t stamp_ns {0};
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

        void rejectEnvelope(
                SourceTrack & source, const std::string & reason, const bool throttle_log)
        {
            ++source.rejected_count;
            source.last_rejection = reason;
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
            const std::int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();

            std::lock_guard<std::mutex> lock(mutex_);
            SourceTrack & source = sources_.at(index);
            ++source.received_count;
            source.last_received_steady = now;

            if(message->header.frame_id != map_frame_) {
                rejectEnvelope(source, "frame_id does not match map_frame", true);
                return;
            }
            if(message->binary) {
                rejectEnvelope(source, "binary Octomap is not accepted; expected full map", true);
                return;
            }
            if(message->data.size() > max_serialized_bytes_per_source_) {
                rejectEnvelope(source, "serialized map exceeds byte limit", true);
                return;
            }
            if(stamp_ns <= 0) {
                rejectEnvelope(source, "observation stamp must be nonzero", true);
                return;
            }
            if(stamp_ns <= source.latest_enqueued_stamp_ns) {
                rejectEnvelope(source, "observation stamp is not strictly increasing", false);
                return;
            }

            if(source.pending) {
                ++source.coalesced_count;
            }
            source.pending          = message;
            source.pending_stamp_ns = stamp_ns;
            source.latest_enqueued_stamp_ns = stamp_ns;
        }

        void recordProcessedResult(
                const std::size_t index, const PendingMap & pending,
                const SourceUpdateResult & result, const SteadyClock::time_point now)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            SourceTrack & source = sources_.at(index);
            if(!result.accepted()) {
                ++source.rejected_count;
                source.last_rejection = result.reason;
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
            source.last_accepted_steady   = now;
            source.last_message_bytes     = pending.message->data.size();
            source.voxel_count            = merger_->sourceVoxelCount(source.topic);
            source.last_rejection.clear();
        }

        void onMergeTimer()
        {
            const auto merge_start = SteadyClock::now();
            std::vector<PendingMap> pending_maps;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                pending_maps.reserve(sources_.size());
                for(std::size_t i = 0U; i < sources_.size(); ++i) {
                    if(!sources_[i].pending) {
                        continue;
                    }
                    pending_maps.push_back(PendingMap {
                            i,
                            std::move(sources_[i].pending),
                            sources_[i].pending_stamp_ns,
                    });
                    sources_[i].pending_stamp_ns = 0;
                }
            }

            std::size_t accepted_this_cycle = 0U;
            std::size_t rejected_this_cycle = 0U;
            for(const PendingMap & pending : pending_maps) {
                const std::string source_id = sources_[pending.index].topic;
                SourceUpdateResult result;
                try {
                    std::unique_ptr<octomap::AbstractOcTree> abstract(
                            octomap_msgs::msgToMap(*pending.message));
                    octomap::OcTree * source_tree =
                            dynamic_cast<octomap::OcTree *>(abstract.get());
                    if(source_tree == nullptr) {
                        result.status = SourceUpdateStatus::Invalid;
                        result.source_revision = merger_->sourceRevision();
                        result.global_revision = merger_->globalRevision();
                        result.reason = "message does not contain an OcTree";
                    } else {
                        result = merger_->updateSource(source_id, *source_tree);
                    }
                } catch(const std::exception & exception) {
                    result.status = SourceUpdateStatus::Invalid;
                    result.source_revision = merger_->sourceRevision();
                    result.global_revision = merger_->globalRevision();
                    result.reason = std::string("Octomap deserialization failed: ")
                                    + exception.what();
                }

                recordProcessedResult(
                        pending.index, pending, result, SteadyClock::now());
                if(result.accepted()) {
                    ++accepted_this_cycle;
                } else {
                    ++rejected_this_cycle;
                }
            }

            if(accepted_this_cycle > 0U) {
                publishMap();
            }

            const double duration_ms = std::chrono::duration<double, std::milli>(
                                               SteadyClock::now() - merge_start)
                                               .count();
            last_merge_duration_ms_ = duration_ms;
            max_merge_duration_ms_  = std::max(max_merge_duration_ms_, duration_ms);
            last_cycle_rejected_    = rejected_this_cycle;
            publishDiagnostics();
        }

        void publishMap()
        {
            octomap_msgs::msg::Octomap message;
            if(!octomap_msgs::fullMapToMsg(merger_->tree(), message)) {
                serialization_error_ = "failed to serialize merged OcTree";
                RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 5000, "%s",
                        serialization_error_.c_str());
                return;
            }
            serialization_error_.clear();

            std::int64_t latest_stamp_ns = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                for(const SourceTrack & source : sources_) {
                    latest_stamp_ns = std::max(
                            latest_stamp_ns, source.last_accepted_stamp_ns);
                }
            }
            message.header.frame_id = map_frame_;
            message.header.stamp = rclcpp::Time(
                    latest_stamp_ns, get_clock()->get_clock_type());
            map_publisher_->publish(message);
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
                }
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
        double      last_merge_duration_ms_ {0.0};
        double      max_merge_duration_ms_ {0.0};
        std::size_t last_cycle_rejected_ {0U};
        std::string serialization_error_;
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
