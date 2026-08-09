#include "perception_map_update/CanonicalCodec.hpp"
#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/OctoMapViewAdapter.hpp"
#include "perception_map_update/ResyncStateMachine.hpp"
#include "perception_map_update/ros/MapUpdateConversions.hpp"
#include "perception_map_update/ros/MapUpdateParameters.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "perception_interfaces/msg/local_map_state.hpp"
#include "perception_interfaces/msg/map_update.hpp"
#include "perception_interfaces/srv/request_map_resync.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>

namespace {

    Perception::SessionID generate_session_id()
    {
        const auto boot = std::chrono::steady_clock::now().time_since_epoch();
        std::random_device random;
        return {
                static_cast<std::uint64_t>(
                        std::chrono::duration_cast<std::chrono::nanoseconds>(boot).count()),
                random()};
    }

    std::int64_t monotonic_now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                       std::chrono::steady_clock::now().time_since_epoch())
                .count();
    }

    std::int64_t positive_period_ns(double seconds, const char * name)
    {
        if(!std::isfinite(seconds) || seconds <= 0.0
           || seconds
                      > static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9) {
            throw std::invalid_argument(std::string(name) + " must be finite and positive");
        }
        return static_cast<std::int64_t>(seconds * 1e9);
    }

    const char * receiver_state_name(PerceptionMapUpdate::ReceiverState state) noexcept
    {
        switch(state) {
            case PerceptionMapUpdate::ReceiverState::Empty:
                return "empty";
            case PerceptionMapUpdate::ReceiverState::Ready:
                return "ready";
            case PerceptionMapUpdate::ReceiverState::ResyncRequired:
                return "resync_required";
            case PerceptionMapUpdate::ReceiverState::Removed:
                return "removed";
        }
        return "unknown";
    }

}// namespace

class PerceptionMapUpdateReceiverNode final : public rclcpp::Node
{
public:
    PerceptionMapUpdateReceiverNode()
        : Node("perception_map_update_receiver_node")
        , requester_session_(generate_session_id())
    {
        update_topic_ = declare_parameter<std::string>(
                "map_update_topic", "local_map/updates");
        state_topic_ = declare_parameter<std::string>(
                "local_map_state_topic", "local_map/state");
        resync_service_name_ = declare_parameter<std::string>(
                "map_resync_service", "local_map/request_resync");
        octomap_topic_ = declare_parameter<std::string>(
                "reconstructed_octomap_topic", "map_update_receiver/octomap");
        expected_vehicle_id_ = declare_parameter<std::string>("expected_vehicle_id", "");
        requester_id_ = declare_parameter<std::string>(
                "requester_id", "map-update-reference-receiver");
        const auto retry_period_ns = positive_period_ns(
                declare_parameter<double>("resync_retry_period_s", 0.5),
                "resync_retry_period_s");
        limits_ = PerceptionMapUpdate::Ros::declare_map_update_limits(*this);
        if(!expected_vehicle_id_.empty()
           && !PerceptionMapUpdate::CanonicalCodec::validate_string(
                      expected_vehicle_id_,
                      limits_.max_identity_string_bytes,
                      "expected_vehicle_id",
                      false)) {
            throw std::invalid_argument("expected_vehicle_id is invalid");
        }
        if(!PerceptionMapUpdate::CanonicalCodec::validate_string(
                   requester_id_,
                   limits_.max_identity_string_bytes,
                   "requester_id",
                   false)) {
            throw std::invalid_argument("requester_id is invalid");
        }
        applier_ = std::make_unique<PerceptionMapUpdate::MapUpdateApplier>(limits_);

        update_subscription_ =
                create_subscription<perception_interfaces::msg::MapUpdate>(
                        update_topic_,
                        rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliable()
                                .durability_volatile(),
                        [this](const perception_interfaces::msg::MapUpdate::SharedPtr message) {
                            on_update(*message);
                        });
        state_subscription_ =
                create_subscription<perception_interfaces::msg::LocalMapState>(
                        state_topic_,
                        rclcpp::QoS(rclcpp::KeepLast(1))
                                .reliable()
                                .durability_volatile(),
                        [this](
                                const perception_interfaces::msg::LocalMapState::SharedPtr message) {
                            on_source_state(*message);
                        });
        octomap_publisher_ = create_publisher<octomap_msgs::msg::Octomap>(
                octomap_topic_, rclcpp::QoS(1).reliable().transient_local());
        diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                "/diagnostics", 10);
        resync_client_ = create_client<perception_interfaces::srv::RequestMapResync>(
                resync_service_name_);
        retry_period_ns_ = retry_period_ns;
        timer_ = create_wall_timer(
                std::chrono::nanoseconds(retry_period_ns_),
                [this]() {
                    maybe_request_resync();
                    publish_diagnostics();
                });
    }

private:
    struct Metrics {
        std::uint64_t accepted_keyframes = 0U;
        std::uint64_t accepted_deltas = 0U;
        std::uint64_t accepted_removes = 0U;
        std::uint64_t accepted_summaries = 0U;
        std::uint64_t duplicates = 0U;
        std::uint64_t stale = 0U;
        std::uint64_t gaps = 0U;
        std::uint64_t conflicts = 0U;
        std::uint64_t malformed = 0U;
        std::uint64_t admission_rejections = 0U;
        std::uint64_t resource_rejections = 0U;
        std::uint64_t resync_requests = 0U;
        std::uint64_t resync_responses = 0U;
        std::uint64_t octomap_failures = 0U;
        std::int64_t last_apply_duration_ns = 0;
        std::string last_diagnostic;
    };

    PerceptionMapUpdate::MapUpdateLimits limits_;
    std::unique_ptr<PerceptionMapUpdate::MapUpdateApplier> applier_;
    Perception::SessionID requester_session_;
    std::optional<PerceptionMapUpdate::SourceIdentity> admitted_source_;
    std::uint64_t latest_source_revision_ = 0U;
    std::uint64_t source_state_sequence_ = 0U;
    bool unsolicited_baseline_allowed_ = false;

    bool resync_active_ = false;
    bool resync_request_in_flight_ = false;
    std::uint64_t resync_generation_ = 0U;
    std::string client_request_id_;
    std::string expected_correlation_id_;
    PerceptionMapUpdate::ResyncReason resync_reason_ =
            PerceptionMapUpdate::ResyncReason::InitialBaseline;
    std::int64_t last_resync_request_ns_ = 0;
    std::int64_t retry_period_ns_ = 0;
    Metrics metrics_;

    std::string update_topic_;
    std::string state_topic_;
    std::string resync_service_name_;
    std::string octomap_topic_;
    std::string expected_vehicle_id_;
    std::string requester_id_;

    rclcpp::Subscription<perception_interfaces::msg::MapUpdate>::SharedPtr update_subscription_;
    rclcpp::Subscription<perception_interfaces::msg::LocalMapState>::SharedPtr state_subscription_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
    rclcpp::Client<perception_interfaces::srv::RequestMapResync>::SharedPtr resync_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    void on_source_state(const perception_interfaces::msg::LocalMapState & message)
    {
        if(message.vehicle_id.empty() || message.mapper_session_boot_time_ns == 0U
           || message.map_epoch == 0U
           || (!expected_vehicle_id_.empty()
               && message.vehicle_id != expected_vehicle_id_)) {
            ++metrics_.admission_rejections;
            metrics_.last_diagnostic = "rejected invalid or unexpected source metadata";
            return;
        }
        const PerceptionMapUpdate::SourceIdentity source {
                message.vehicle_id,
                {message.mapper_session_boot_time_ns, message.mapper_session_random_suffix},
                message.map_epoch};
        if(admitted_source_.has_value() && *admitted_source_ == source) {
            if(message.state_sequence <= source_state_sequence_
               || message.revision < latest_source_revision_) {
                return;
            }
        } else {
            if(!applier_->admit_source(source)) {
                ++metrics_.admission_rejections;
                metrics_.last_diagnostic = "source metadata belongs to a retired chain";
                return;
            }
            admitted_source_ = source;
            source_state_sequence_ = 0U;
            latest_source_revision_ = 0U;
            unsolicited_baseline_allowed_ = true;
            clear_resync();
        }
        source_state_sequence_ = message.state_sequence;
        latest_source_revision_ = message.revision;
        if(message.revision == 0U) {
            return;
        }
        if(applier_->state() == PerceptionMapUpdate::ReceiverState::Empty) {
            begin_resync(PerceptionMapUpdate::ResyncReason::InitialBaseline);
        } else if(applier_->state() == PerceptionMapUpdate::ReceiverState::ResyncRequired) {
            begin_resync(PerceptionMapUpdate::ResyncReason::EpochChange);
        }
    }

    void on_update(const perception_interfaces::msg::MapUpdate & message)
    {
        const auto decoded = PerceptionMapUpdate::Ros::decode_map_update(message, limits_);
        if(!decoded.success || !decoded.update.has_value()) {
            ++metrics_.malformed;
            metrics_.last_diagnostic = decoded.diagnostic;
            if(applier_->require_resync()) {
                begin_resync(PerceptionMapUpdate::ResyncReason::LocalStateInvalid);
            }
            return;
        }
        const auto & update = *decoded.update;
        const bool needs_baseline =
                applier_->state() == PerceptionMapUpdate::ReceiverState::Empty
                || applier_->state() == PerceptionMapUpdate::ReceiverState::ResyncRequired;
        if(needs_baseline && update.kind == PerceptionMapUpdate::UpdateKind::Keyframe) {
            if(!expected_correlation_id_.empty()
               && update.correlation_id != expected_correlation_id_) {
                ++metrics_.admission_rejections;
                metrics_.last_diagnostic = "keyframe correlation does not match resync generation";
                return;
            }
            if(expected_correlation_id_.empty() && !unsolicited_baseline_allowed_) {
                ++metrics_.admission_rejections;
                metrics_.last_diagnostic = "uncorrelated keyframe cannot recover current gap";
                return;
            }
        }

        const auto apply_start = std::chrono::steady_clock::now();
        const auto result = applier_->apply(update);
        metrics_.last_apply_duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                  std::chrono::steady_clock::now() - apply_start)
                                                  .count();
        switch(result.status) {
            case PerceptionMapUpdate::ApplyUpdateStatus::AppliedKeyframe:
                ++metrics_.accepted_keyframes;
                unsolicited_baseline_allowed_ = false;
                clear_resync();
                publish_octomap_view();
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::AppliedDelta:
                ++metrics_.accepted_deltas;
                publish_octomap_view();
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::AppliedRemove:
                ++metrics_.accepted_removes;
                publish_octomap_view();
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::AcceptedSummary:
                ++metrics_.accepted_summaries;
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::IgnoredDuplicate:
                ++metrics_.duplicates;
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::RejectedStale:
                ++metrics_.stale;
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::RejectedGap:
                ++metrics_.gaps;
                begin_resync(PerceptionMapUpdate::ResyncReason::Gap);
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::RejectedConflict:
                ++metrics_.conflicts;
                begin_resync(PerceptionMapUpdate::ResyncReason::HashConflict);
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::RejectedAdmission:
                ++metrics_.admission_rejections;
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::RejectedResourceLimit:
                ++metrics_.resource_rejections;
                begin_resync(PerceptionMapUpdate::ResyncReason::LocalStateInvalid);
                break;
            case PerceptionMapUpdate::ApplyUpdateStatus::RejectedInvalid:
                ++metrics_.malformed;
                begin_resync(PerceptionMapUpdate::ResyncReason::LocalStateInvalid);
                break;
        }
        if(!result.diagnostic.empty()) {
            metrics_.last_diagnostic = result.diagnostic;
        } else if(result.status <= PerceptionMapUpdate::ApplyUpdateStatus::IgnoredDuplicate) {
            metrics_.last_diagnostic.clear();
        }
    }

    void begin_resync(PerceptionMapUpdate::ResyncReason reason)
    {
        if(!admitted_source_.has_value() || latest_source_revision_ == 0U) {
            return;
        }
        if(resync_active_) {
            return;
        }
        resync_active_ = true;
        resync_reason_ = reason;
        expected_correlation_id_.clear();
        ++resync_generation_;
        client_request_id_ = "receiver-" + std::to_string(requester_session_.boot_time_ns)
                             + '-' + std::to_string(resync_generation_);
        last_resync_request_ns_ = 0;
    }

    void clear_resync()
    {
        resync_active_ = false;
        resync_request_in_flight_ = false;
        client_request_id_.clear();
        expected_correlation_id_.clear();
        last_resync_request_ns_ = 0;
    }

    void maybe_request_resync()
    {
        if(!resync_active_ || resync_request_in_flight_ || !admitted_source_.has_value()) {
            return;
        }
        const auto now = monotonic_now_ns();
        if(last_resync_request_ns_ != 0
           && now - last_resync_request_ns_ < retry_period_ns_) {
            return;
        }
        if(!resync_client_->service_is_ready()) {
            metrics_.last_diagnostic = "resync service is not ready";
            return;
        }
        if(!PerceptionMapUpdate::CanonicalCodec::validate_string(
                   client_request_id_,
                   limits_.max_correlation_id_bytes,
                   "client_request_id",
                   false)) {
            ++metrics_.resource_rejections;
            metrics_.last_diagnostic = "resync client request id exceeds configured limit";
            return;
        }

        using Service = perception_interfaces::srv::RequestMapResync;
        auto request = std::make_shared<Service::Request>();
        request->requester_id = requester_id_;
        request->requester_session_boot_time_ns = requester_session_.boot_time_ns;
        request->requester_session_random_suffix = requester_session_.random_suffix;
        request->client_request_id = client_request_id_;
        const bool retained_current_source = applier_->reconstructed_map().has_value()
                                             && applier_->reconstructed_map()->source
                                                        == *admitted_source_;
        const bool bootstrap = resync_reason_
                                       == PerceptionMapUpdate::ResyncReason::InitialBaseline
                               && !retained_current_source;
        request->bootstrap_latest = bootstrap;
        if(!bootstrap) {
            request->expected_vehicle_id = admitted_source_->vehicle_id;
            request->expected_mapper_session_boot_time_ns =
                    admitted_source_->mapper_session.boot_time_ns;
            request->expected_mapper_session_random_suffix =
                    admitted_source_->mapper_session.random_suffix;
            request->expected_map_epoch = admitted_source_->map_epoch;
        }
        if(retained_current_source) {
            request->receiver_revision = applier_->reconstructed_map()->revision;
            std::copy(
                    applier_->reconstructed_map()->content_hash.begin(),
                    applier_->reconstructed_map()->content_hash.end(),
                    request->receiver_content_hash.begin());
        }
        request->reason = static_cast<std::uint8_t>(resync_reason_);
        const auto generation = resync_generation_;
        resync_request_in_flight_ = true;
        last_resync_request_ns_ = now;
        ++metrics_.resync_requests;
        resync_client_->async_send_request(
                request,
                [this, generation](rclcpp::Client<Service>::SharedFuture future) {
                    if(generation != resync_generation_ || !resync_active_) {
                        return;
                    }
                    resync_request_in_flight_ = false;
                    try {
                        const auto response = future.get();
                        ++metrics_.resync_responses;
                        if(!response->accepted) {
                            metrics_.last_diagnostic = response->diagnostic;
                            return;
                        }
                        const PerceptionMapUpdate::SourceIdentity response_source {
                                response->current_vehicle_id,
                                {response->current_mapper_session_boot_time_ns,
                                 response->current_mapper_session_random_suffix},
                                response->current_map_epoch};
                        const auto correlation =
                                PerceptionMapUpdate::CanonicalCodec::validate_string(
                                        response->correlation_id,
                                        limits_.max_correlation_id_bytes,
                                        "correlation_id",
                                        false);
                        if(!admitted_source_.has_value()
                           || response_source != *admitted_source_ || !correlation) {
                            metrics_.last_diagnostic =
                                    "resync response source or correlation is invalid";
                            return;
                        }
                        expected_correlation_id_ = response->correlation_id;
                        metrics_.last_diagnostic.clear();
                    }
                    catch(const std::exception & error) {
                        metrics_.last_diagnostic = error.what();
                    }
                });
    }

    void publish_octomap_view()
    {
        if(!applier_->reconstructed_map().has_value()) {
            return;
        }
        octomap_msgs::msg::Octomap message;
        std::string diagnostic;
        if(!PerceptionMapUpdate::OctoMapViewAdapter::materialize(
                   *applier_->reconstructed_map(), message, diagnostic, limits_)) {
            ++metrics_.octomap_failures;
            metrics_.last_diagnostic = std::move(diagnostic);
            return;
        }
        message.header.stamp = get_clock()->now();
        message.header.frame_id = applier_->reconstructed_map()->geometry.frame_id;
        octomap_publisher_->publish(message);
    }

    void publish_diagnostics()
    {
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = get_clock()->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = get_fully_qualified_name() + std::string(": map_update_receiver");
        status.hardware_id = "reference-receiver";
        status.level = metrics_.last_diagnostic.empty()
                               ? diagnostic_msgs::msg::DiagnosticStatus::OK
                               : diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = metrics_.last_diagnostic.empty()
                                 ? receiver_state_name(applier_->state())
                                 : metrics_.last_diagnostic;
        const auto append = [&](const std::string & key, const auto & value) {
            diagnostic_msgs::msg::KeyValue field;
            field.key = key;
            field.value = std::to_string(value);
            status.values.push_back(std::move(field));
        };
        append("receiver_state", static_cast<std::uint8_t>(applier_->state()));
        append("revision",
               applier_->reconstructed_map().has_value()
                       ? applier_->reconstructed_map()->revision
                       : 0U);
        append("map_epoch",
               applier_->reconstructed_map().has_value()
                       ? applier_->reconstructed_map()->source.map_epoch
                       : 0U);
        append("cells",
               applier_->reconstructed_map().has_value()
                       ? applier_->reconstructed_map()->cells.size()
                       : 0U);
        append("accepted_keyframes", metrics_.accepted_keyframes);
        append("accepted_deltas", metrics_.accepted_deltas);
        append("duplicates", metrics_.duplicates);
        append("stale", metrics_.stale);
        append("gaps", metrics_.gaps);
        append("conflicts", metrics_.conflicts);
        append("malformed", metrics_.malformed);
        append("admission_rejections", metrics_.admission_rejections);
        append("resource_rejections", metrics_.resource_rejections);
        append("apply_duration_ns", metrics_.last_apply_duration_ns);
        append("resync_generation", resync_generation_);
        append("resync_requests", metrics_.resync_requests);
        append("resync_responses", metrics_.resync_responses);
        array.status.push_back(std::move(status));
        diagnostics_publisher_->publish(array);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<PerceptionMapUpdateReceiverNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("perception_map_update_receiver_node"),
                "%s",
                error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
