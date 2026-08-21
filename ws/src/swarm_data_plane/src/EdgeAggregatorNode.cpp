#include "swarm_data_plane/EdgeAggregator.hpp"
#include "swarm_data_plane/RoutedResync.hpp"
#include "swarm_data_plane/RuntimeSnapshotCache.hpp"
#include "swarm_data_plane/ros/AggregateConversions.hpp"
#include "swarm_data_plane/ros/RoleConversions.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"
#include "swarm_data_plane/ros/RoutedResyncConversions.hpp"
#include "swarm_data_plane/ros/TopologyConversions.hpp"

#include "swarm_data_interfaces/msg/capability_evidence.hpp"
#include "swarm_data_interfaces/msg/link_diagnostic.hpp"
#include "swarm_data_interfaces/msg/role_snapshot.hpp"
#include "swarm_data_interfaces/msg/role_transition_descriptor.hpp"
#include "swarm_data_interfaces/msg/aggregate_map_update.hpp"
#include "swarm_data_interfaces/msg/routed_map_update.hpp"
#include "swarm_data_interfaces/msg/topology_snapshot.hpp"
#include "swarm_data_interfaces/srv/request_routed_map_resync.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane {

    namespace {

        std::uint64_t steady_now_ns()
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
        }

        VehicleIdentity load_identity(rclcpp::Node & node)
        {
            const auto fleet_id = node.declare_parameter<std::string>(
                    "fleet_id", "fleet-a");
            const auto vehicle_id = node.declare_parameter<std::string>(
                    "vehicle_id", "edge-aggregator");
            const auto boot = node.declare_parameter<std::int64_t>(
                    "session_boot_time_ns", 300);
            const auto suffix = node.declare_parameter<std::int64_t>(
                    "session_random_suffix", 1);
            if(fleet_id.empty() || vehicle_id.empty() || boot <= 0 || suffix <= 0
               || static_cast<std::uint64_t>(suffix)
                          > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument("EdgeAggregator identity parameters are invalid");
            }
            return {fleet_id, vehicle_id,
                    {static_cast<std::uint64_t>(boot),
                     static_cast<std::uint32_t>(suffix)}};
        }

        std::uint64_t positive_u64(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(name, default_value);
            if(value <= 0) {
                throw std::invalid_argument(name + " must be positive");
            }
            return static_cast<std::uint64_t>(value);
        }

        std::uint32_t positive_u32(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(name, default_value);
            if(value <= 0
               || static_cast<std::uint64_t>(value)
                          > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument(name + " is outside uint32 range");
            }
            return static_cast<std::uint32_t>(value);
        }

    }// namespace

    class EdgeAggregatorNode final : public rclcpp::Node
    {
        using ResyncService = swarm_data_interfaces::srv::RequestRoutedMapResync;

        struct ContributorResyncState {
            rclcpp::Client<ResyncService>::SharedPtr client;
            bool in_flight = false;
            std::uint64_t next_request_id = 1U;
            std::optional<swarm_data_interfaces::msg::RoutedMapUpdate> pending_keyframe;
        };

    public:
        EdgeAggregatorNode()
            : Node("edge_aggregator"),
              identity_(load_identity(*this)),
              snapshots_(identity_)
        {
            const auto input_topics = declare_parameter<std::vector<std::string>>(
                    "input_topics", {"/c5d/edge/input_0", "/c5d/edge/input_1"});
            if(input_topics.empty()) {
                throw std::invalid_argument("input_topics must not be empty");
            }
            const auto resync_services = declare_parameter<std::vector<std::string>>(
                    "resync_services",
                    {"/c5d/source_0/resync", "/c5d/source_1/resync"});
            if(resync_services.size() != input_topics.size()
               || std::any_of(
                       resync_services.begin(), resync_services.end(),
                       [](const std::string & value) { return value.empty(); })) {
                throw std::invalid_argument(
                        "resync_services must contain one non-empty service per input topic");
            }
            const auto output_topic = declare_parameter<std::string>(
                    "output_topic", "/c5d/aggregate/output");
            const auto diagnostic_topic = declare_parameter<std::string>(
                    "diagnostic_topic", "/swarm/runtime/aggregate_diagnostics");
            const auto evidence_topic = declare_parameter<std::string>(
                    "evidence_topic", "/swarm/runtime/evidence");
            const auto topology_topic = declare_parameter<std::string>(
                    "topology_topic", "/swarm/runtime/topology");
            const auto role_topic = declare_parameter<std::string>(
                    "role_topic", "/swarm/runtime/roles");
            const auto transition_topic = declare_parameter<std::string>(
                    "transition_topic", "/swarm/runtime/transition");
            const auto runtime_gate_required = declare_parameter<bool>(
                    "runtime_gate_required", true);

            EdgeAggregatorConfig config;
            config.aggregate_source = {
                    identity_.vehicle_id,
                    {positive_u64(*this, "aggregate_mapper_session_boot_time_ns", 301),
                     positive_u32(*this, "aggregate_mapper_session_random_suffix", 1)},
                    positive_u64(*this, "aggregate_map_epoch", 1)};
            config.aggregate_producer = {
                    identity_.vehicle_id,
                    {positive_u64(*this, "aggregate_producer_session_boot_time_ns", 302),
                     positive_u32(*this, "aggregate_producer_session_random_suffix", 1)}};
            config.route_epoch = positive_u64(*this, "route_epoch", 1);
            const auto ttl_hops = positive_u32(*this, "ttl_hops", 8);
            if(ttl_hops > std::numeric_limits<std::uint16_t>::max()) {
                throw std::invalid_argument("ttl_hops is outside uint16 range");
            }
            config.ttl_hops = static_cast<std::uint16_t>(ttl_hops);
            config.validity_budget_ns = positive_u64(
                    *this, "validity_budget_ns", 5'000'000'000LL);
            config.origin_clock_domain = declare_parameter<std::string>(
                    "origin_clock_domain", "steady");
            config.origin_clock_session = {
                    positive_u64(*this, "origin_clock_session_boot_time_ns", 303),
                    positive_u32(*this, "origin_clock_session_random_suffix", 1)};
            config.sensor_id = declare_parameter<std::string>(
                    "aggregate_sensor_id", "edge-aggregator");
            config.sensor_session = {
                    positive_u64(*this, "aggregate_sensor_session_boot_time_ns", 304),
                    positive_u32(*this, "aggregate_sensor_session_random_suffix", 1)};
            aggregator_ = std::make_unique<EdgeAggregator>(std::move(config));
            runtime_gate_required_ = runtime_gate_required;

            aggregate_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::AggregateMapUpdate>(
                    output_topic, rclcpp::QoS(8).reliable());
            diagnostic_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    diagnostic_topic, rclcpp::QoS(32).reliable());
            evidence_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::CapabilityEvidence>(
                    evidence_topic, rclcpp::QoS(32).reliable());

            const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
            topology_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::TopologySnapshot>(
                    topology_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::TopologySnapshot::ConstSharedPtr message) {
                        apply_topology(*message);
                    });
            role_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoleSnapshot>(
                    role_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::RoleSnapshot::ConstSharedPtr message) {
                        apply_role(*message);
                    });
            transition_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoleTransitionDescriptor>(
                    transition_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::RoleTransitionDescriptor::ConstSharedPtr message) {
                        apply_transition(*message);
                    });

            resync_states_.reserve(resync_services.size());
            for(const auto & service : resync_services) {
                ContributorResyncState state;
                state.client = create_client<ResyncService>(service);
                resync_states_.push_back(std::move(state));
            }
            input_subscriptions_.reserve(input_topics.size());
            for(std::size_t index = 0U; index < input_topics.size(); ++index) {
                input_subscriptions_.push_back(
                        create_subscription<swarm_data_interfaces::msg::RoutedMapUpdate>(
                                input_topics[index], rclcpp::QoS(16).reliable(),
                                [this, index](const swarm_data_interfaces::msg::RoutedMapUpdate::ConstSharedPtr message) {
                                    on_input(index, *message);
                                }));
            }
            evidence_timer_ = create_wall_timer(
                    std::chrono::seconds(1), [this]() { publish_evidence(); });
        }

    private:
        void apply_topology(
                const swarm_data_interfaces::msg::TopologySnapshot & message)
        {
            auto decoded = Ros::decode_topology_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                RCLCPP_WARN(get_logger(), "rejected topology snapshot: %s",
                            decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            snapshots_.apply_topology(std::move(*decoded.snapshot));
        }

        void apply_role(const swarm_data_interfaces::msg::RoleSnapshot & message)
        {
            auto decoded = Ros::decode_role_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                RCLCPP_WARN(get_logger(), "rejected role snapshot: %s",
                            decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            snapshots_.apply_role(std::move(*decoded.snapshot));
        }

        void apply_transition(
                const swarm_data_interfaces::msg::RoleTransitionDescriptor & message)
        {
            auto decoded = Ros::decode_role_transition(message);
            if(!decoded.success || !decoded.transition.has_value()) {
                RCLCPP_WARN(get_logger(), "rejected role transition: %s",
                            decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            snapshots_.apply_transition(std::move(*decoded.transition));
        }

        void publish_evidence()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(evidence_revision_ == std::numeric_limits<std::uint64_t>::max()) {
                return;
            }
            CapabilityEvidence evidence;
            evidence.identity = identity_;
            evidence.evidence_revision = evidence_revision_++;
            evidence.effective_capabilities = {CapabilityKind::MapAggregation};
            evidence.vehicle_health = VehicleHealth::Healthy;
            evidence.resource_health = ResourceHealth::Healthy;
            evidence.service_health = {
                    {ServiceKind::Aggregation, ResourceHealth::Healthy}};
            snapshots_.apply_evidence(evidence);
            swarm_data_interfaces::msg::CapabilityEvidence message;
            std::string diagnostic;
            if(Ros::encode_capability_evidence(evidence, message, diagnostic)) {
                evidence_publisher_->publish(message);
            }
        }

        void on_input(
                std::size_t input_index,
                const swarm_data_interfaces::msg::RoutedMapUpdate & message)
        {
            const auto receive_ns = steady_now_ns();
            auto decoded = Ros::decode_routed_map_update(message);
            if(!decoded.success || !decoded.message.has_value()) {
                publish_diagnostic(message.message_id, false, decoded.diagnostic);
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if(runtime_gate_required_) {
                const auto admission = snapshots_.service_admission(ServiceKind::Aggregation);
                if(!admission) {
                    publish_diagnostic(message.message_id, false,
                                       "AggregationService admission denied");
                    return;
                }
            }
            const auto result = aggregator_->receive(*decoded.message, receive_ns);
            if(!result || !result.update.has_value()) {
                if(result.status == EdgeAggregatorStatus::RejectedResync) {
                    if(decoded.message->update->kind
                               == PerceptionMapUpdate::UpdateKind::Keyframe
                       && !message.correlation_id.empty()) {
                        resync_states_[input_index].pending_keyframe = message;
                    }
                    request_resync(input_index, *decoded.message);
                }
                publish_diagnostic(
                        message.message_id,
                        false,
                        result.diagnostic,
                        result.status == EdgeAggregatorStatus::RejectedResync
                                ? "aggregation_resync"
                                : "aggregation");
                return;
            }
            swarm_data_interfaces::msg::AggregateMapUpdate output;
            std::string diagnostic;
            if(!Ros::encode_aggregate_map_update(*result.update, output, diagnostic)) {
                publish_diagnostic(message.message_id, false, diagnostic);
                return;
            }
            aggregate_publisher_->publish(output);
            publish_diagnostic(output.aggregate_update.message_id, true, {});
        }

        void request_resync(
                std::size_t input_index,
                const RoutedMapUpdate & rejected)
        {
            auto & state = resync_states_.at(input_index);
            if(state.in_flight || !state.client->service_is_ready()
               || state.next_request_id == std::numeric_limits<std::uint64_t>::max()) {
                return;
            }

            RoutedResyncIntent intent;
            intent.target_producer = rejected.producer;
            intent.route_epoch = rejected.route.route_epoch;
            intent.request.requester = {identity_.vehicle_id, identity_.session};
            intent.request.client_request_id = identity_.vehicle_id + "-"
                                               + std::to_string(input_index) + "-"
                                               + std::to_string(state.next_request_id++);
            intent.request.expected_source = rejected.update->source;
            intent.request.reason = PerceptionMapUpdate::ResyncReason::EpochChange;
            const auto contributors = aggregator_->contributors();
            const auto contributor = std::find_if(
                    contributors.begin(), contributors.end(),
                    [&](const EdgeContributorSnapshot & value) {
                        return value.source == rejected.update->source;
                    });
            if(contributor != contributors.end()) {
                intent.request.receiver_revision = contributor->revision;
                intent.request.receiver_content_identity = contributor->content_identity;
            }

            auto request = std::make_shared<ResyncService::Request>();
            std::string diagnostic;
            if(!Ros::encode_resync_intent(intent, request->intent, diagnostic)) {
                RCLCPP_ERROR(
                        get_logger(), "cannot encode contributor resync intent: %s",
                        diagnostic.c_str());
                return;
            }
            state.in_flight = true;
            const auto expected_producer = rejected.producer;
            const auto expected_source = rejected.update->source;
            state.client->async_send_request(
                    request,
                    [this, input_index, expected_producer, expected_source](
                            rclcpp::Client<ResyncService>::SharedFuture future) {
                        std::optional<swarm_data_interfaces::msg::RoutedMapUpdate> pending;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            auto & callback_state = resync_states_.at(input_index);
                            callback_state.in_flight = false;
                            const auto decoded = Ros::decode_resync_ack(future.get()->ack);
                            if(!decoded.success || !decoded.ack.has_value()
                               || !decoded.ack->accepted
                               || decoded.ack->target_producer != expected_producer
                               || decoded.ack->current_source != expected_source) {
                                const auto reason = decoded.ack.has_value()
                                        ? decoded.ack->diagnostic
                                        : decoded.diagnostic;
                                RCLCPP_WARN(
                                        get_logger(), "contributor resync was rejected: %s",
                                        reason.c_str());
                                return;
                            }
                            if(!aggregator_->expect_resync(
                                       expected_source, decoded.ack->correlation_id)) {
                                RCLCPP_WARN(
                                        get_logger(),
                                        "contributor resync correlation was not expected");
                                return;
                            }
                            pending = std::move(callback_state.pending_keyframe);
                            callback_state.pending_keyframe.reset();
                        }
                        if(pending.has_value()) {
                            on_input(input_index, *pending);
                        }
                    });
        }

        void publish_diagnostic(
                const std::string & message_id,
                bool delivered,
                const std::string & diagnostic,
                const std::string & fault_reason = "aggregation")
        {
            swarm_data_interfaces::msg::LinkDiagnostic output;
            output.protocol_version = kProtocolVersion;
            output.message_id = message_id;
            output.endpoint_id = identity_.vehicle_id;
            output.endpoint_session_boot_time_ns = identity_.session.boot_time_ns;
            output.endpoint_session_random_suffix = identity_.session.random_suffix;
            output.event = delivered
                                   ? swarm_data_interfaces::msg::LinkDiagnostic::EVENT_DELIVERED
                                   : swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED;
            output.local_receive_monotonic_ns = steady_now_ns();
            output.local_send_monotonic_ns = delivered ? steady_now_ns() : 0U;
            output.fault_reason = delivered ? "" : fault_reason;
            output.diagnostic = diagnostic;
            diagnostic_publisher_->publish(output);
        }

        VehicleIdentity identity_;
        RuntimeSnapshotCache snapshots_;
        std::unique_ptr<EdgeAggregator> aggregator_;
        bool runtime_gate_required_ = true;
        std::uint64_t evidence_revision_ = 1U;
        std::mutex mutex_;
        std::vector<ContributorResyncState> resync_states_;
        std::vector<rclcpp::Subscription<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr>
                input_subscriptions_;
        rclcpp::Subscription<swarm_data_interfaces::msg::TopologySnapshot>::SharedPtr
                topology_subscription_;
        rclcpp::Subscription<swarm_data_interfaces::msg::RoleSnapshot>::SharedPtr
                role_subscription_;
        rclcpp::Subscription<swarm_data_interfaces::msg::RoleTransitionDescriptor>::SharedPtr
                transition_subscription_;
        rclcpp::Publisher<swarm_data_interfaces::msg::AggregateMapUpdate>::SharedPtr
                aggregate_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                diagnostic_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::CapabilityEvidence>::SharedPtr
                evidence_publisher_;
        rclcpp::TimerBase::SharedPtr evidence_timer_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::EdgeAggregatorNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("edge_aggregator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
