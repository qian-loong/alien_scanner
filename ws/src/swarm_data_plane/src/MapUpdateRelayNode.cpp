#include "swarm_data_plane/PureRelay.hpp"
#include "swarm_data_plane/ros/RoleConversions.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"
#include "swarm_data_plane/ros/TopologyConversions.hpp"

#include "swarm_data_interfaces/msg/capability_evidence.hpp"
#include "swarm_data_interfaces/msg/link_diagnostic.hpp"
#include "swarm_data_interfaces/msg/role_snapshot.hpp"
#include "swarm_data_interfaces/msg/role_transition_descriptor.hpp"
#include "swarm_data_interfaces/msg/routed_map_update.hpp"
#include "swarm_data_interfaces/msg/topology_snapshot.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
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

        std::string bounded(std::string value, std::size_t limit)
        {
            if(value.size() > limit) {
                value.resize(limit);
            }
            return value;
        }

        VehicleIdentity load_identity(rclcpp::Node & node)
        {
            const auto fleet_id = node.declare_parameter<std::string>(
                    "fleet_id", "fleet-a");
            const auto vehicle_id = node.declare_parameter<std::string>(
                    "vehicle_id", "relay-0");
            const auto boot_time_ns = node.declare_parameter<std::int64_t>(
                    "session_boot_time_ns", 200);
            const auto random_suffix = node.declare_parameter<std::int64_t>(
                    "session_random_suffix", 1);
            if(fleet_id.empty() || vehicle_id.empty() || boot_time_ns <= 0
               || random_suffix <= 0
               || static_cast<std::uint64_t>(random_suffix)
                          > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument("Relay identity parameters are invalid");
            }
            return {
                    fleet_id,
                    vehicle_id,
                    {static_cast<std::uint64_t>(boot_time_ns),
                     static_cast<std::uint32_t>(random_suffix)}};
        }

        CapabilityEvidence healthy_relay_evidence(
                const VehicleIdentity & identity,
                std::uint64_t revision)
        {
            CapabilityEvidence evidence;
            evidence.identity = identity;
            evidence.evidence_revision = revision;
            evidence.effective_capabilities = {
                    CapabilityKind::RelayForwarding};
            evidence.vehicle_health = VehicleHealth::Healthy;
            evidence.resource_health = ResourceHealth::Healthy;
            evidence.service_health = {
                    {ServiceKind::Relay, ResourceHealth::Healthy}};
            return evidence;
        }

    }// namespace

    class MapUpdateRelayNode final : public rclcpp::Node
    {
    public:
        MapUpdateRelayNode()
            : Node("map_update_relay")
            , identity_(load_identity(*this))
            , snapshots_(identity_)
        {
            const auto route_ids = declare_parameter<std::vector<std::string>>(
                    "route_ids", {"route-map-0", "route-map-1"});
            const auto input_topics = declare_parameter<std::vector<std::string>>(
                    "input_topics",
                    {"route_map_0/input", "route_map_1/input"});
            const auto output_topics = declare_parameter<std::vector<std::string>>(
                    "output_topics",
                    {"route_map_0/output", "route_map_1/output"});
            if(route_ids.empty() || route_ids.size() != input_topics.size()
               || route_ids.size() != output_topics.size()) {
                throw std::invalid_argument(
                        "route_ids, input_topics, and output_topics must be non-empty and aligned");
            }
            const auto heartbeat_rate_hz = declare_parameter<double>(
                    "heartbeat_rate_hz", 5.0);
            if(heartbeat_rate_hz <= 0.0) {
                throw std::invalid_argument("heartbeat_rate_hz must be positive");
            }
            const auto topology_topic = declare_parameter<std::string>(
                    "topology_topic", "/swarm/runtime/topology");
            const auto role_topic = declare_parameter<std::string>(
                    "role_topic", "/swarm/runtime/roles");
            const auto transition_topic = declare_parameter<std::string>(
                    "transition_topic", "/swarm/runtime/transition");
            const auto evidence_topic = declare_parameter<std::string>(
                    "evidence_topic", "/swarm/runtime/evidence");
            const auto diagnostic_topic = declare_parameter<std::string>(
                    "diagnostic_topic", "/swarm/runtime/relay_diagnostics");
            const auto fault_service = declare_parameter<std::string>(
                    "fault_service", "set_faulted");
            const auto terminal_output = declare_parameter<bool>(
                    "terminal_output", true);

            const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
            topology_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::TopologySnapshot>(
                    topology_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::TopologySnapshot::
                                   ConstSharedPtr message) {
                        apply_topology(*message);
                    });
            role_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoleSnapshot>(
                    role_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::RoleSnapshot::
                                   ConstSharedPtr message) {
                        apply_role(*message);
                    });
            transition_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoleTransitionDescriptor>(
                    transition_topic, state_qos,
                    [this](
                            const swarm_data_interfaces::msg::
                                    RoleTransitionDescriptor::ConstSharedPtr message) {
                        apply_transition(*message);
                    });
            evidence_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::CapabilityEvidence>(
                    evidence_topic,
                    rclcpp::QoS(32).reliable().durability_volatile());
            diagnostic_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    diagnostic_topic,
                    rclcpp::QoS(32).reliable().durability_volatile());

            endpoints_.reserve(route_ids.size());
            for(std::size_t index = 0U; index < route_ids.size(); ++index) {
                RouteEndpoint endpoint;
                endpoint.route_id = route_ids[index];
                endpoint.relay = std::make_unique<PureRelay>(
                        identity_, route_ids[index], 0U, terminal_output);
                endpoint.publisher = create_publisher<
                        swarm_data_interfaces::msg::RoutedMapUpdate>(
                        output_topics[index], rclcpp::QoS(8).reliable());
                endpoints_.push_back(std::move(endpoint));
            }
            for(std::size_t index = 0U; index < endpoints_.size(); ++index) {
                endpoints_[index].subscription = create_subscription<
                        swarm_data_interfaces::msg::RoutedMapUpdate>(
                        input_topics[index], rclcpp::QoS(8).reliable(),
                        [this, index](
                                const swarm_data_interfaces::msg::RoutedMapUpdate::
                                        ConstSharedPtr message) {
                            on_map_update(index, *message);
                        });
            }

            fault_service_ = create_service<std_srvs::srv::SetBool>(
                    fault_service,
                    [this](
                            const std_srvs::srv::SetBool::Request::SharedPtr request,
                            std_srvs::srv::SetBool::Response::SharedPtr response) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        const bool changed = faulted_ != request->data;
                        faulted_ = request->data;
                        if(changed) {
                            const auto event = faulted_
                                    ? swarm_data_interfaces::msg::LinkDiagnostic::
                                              EVENT_LINK_DOWN
                                    : swarm_data_interfaces::msg::LinkDiagnostic::
                                              EVENT_LINK_UP;
                            const auto now_ns = steady_now_ns();
                            for(std::size_t index = 0U;
                                index < endpoints_.size(); ++index) {
                                publish_diagnostic(
                                        index, {}, 0U, 0U, event,
                                        "fault_injection",
                                        faulted_
                                                ? "Relay fault injection is active"
                                                : "Relay fault injection was cleared",
                                        now_ns, now_ns);
                            }
                        }
                        response->success = true;
                        response->message = faulted_
                                ? "Relay forwarding and heartbeat paused"
                                : "Relay fault injection cleared";
                    });
            const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / heartbeat_rate_hz));
            heartbeat_timer_ = create_wall_timer(
                    period, [this]() { publish_heartbeat(); });
        }

    private:
        struct RouteEndpoint {
            std::string route_id;
            std::unique_ptr<PureRelay> relay;
            rclcpp::Publisher<
                    swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr publisher;
            rclcpp::Subscription<
                    swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr subscription;
        };

        void apply_topology(
                const swarm_data_interfaces::msg::TopologySnapshot & message)
        {
            auto decoded = Ros::decode_topology_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                RCLCPP_WARN(
                        get_logger(), "rejected topology snapshot: %s",
                        decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = snapshots_.apply_topology(
                    std::move(*decoded.snapshot));
            if(!result) {
                RCLCPP_WARN(
                        get_logger(), "runtime topology cache rejected: %s",
                        result.diagnostic.c_str());
            }
        }

        void apply_role(const swarm_data_interfaces::msg::RoleSnapshot & message)
        {
            auto decoded = Ros::decode_role_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                RCLCPP_WARN(
                        get_logger(), "rejected role snapshot: %s",
                        decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = snapshots_.apply_role(std::move(*decoded.snapshot));
            if(!result) {
                RCLCPP_WARN(
                        get_logger(), "runtime role cache rejected: %s",
                        result.diagnostic.c_str());
            }
        }

        void apply_transition(
                const swarm_data_interfaces::msg::RoleTransitionDescriptor & message)
        {
            auto decoded = Ros::decode_role_transition(message);
            if(!decoded.success || !decoded.transition.has_value()) {
                RCLCPP_WARN(
                        get_logger(), "rejected role transition: %s",
                        decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = snapshots_.apply_transition(
                    std::move(*decoded.transition));
            if(!result) {
                RCLCPP_WARN(
                        get_logger(), "runtime transition cache rejected: %s",
                        result.diagnostic.c_str());
            }
        }

        void publish_heartbeat()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(faulted_ || evidence_revision_ == std::numeric_limits<std::uint64_t>::max()) {
                return;
            }
            auto evidence = healthy_relay_evidence(identity_, evidence_revision_++);
            const auto cached = snapshots_.apply_evidence(evidence);
            if(!cached) {
                RCLCPP_WARN(
                        get_logger(), "local evidence cache rejected: %s",
                        cached.diagnostic.c_str());
                return;
            }
            swarm_data_interfaces::msg::CapabilityEvidence message;
            std::string diagnostic;
            if(!Ros::encode_capability_evidence(evidence, message, diagnostic)) {
                RCLCPP_ERROR(
                        get_logger(), "failed to encode Relay evidence: %s",
                        diagnostic.c_str());
                return;
            }
            evidence_publisher_->publish(message);
        }

        void on_map_update(
                std::size_t endpoint_index,
                const swarm_data_interfaces::msg::RoutedMapUpdate & message)
        {
            const auto receive_ns = steady_now_ns();
            auto decoded = Ros::decode_routed_map_update(message);
            if(!decoded.success || !decoded.message.has_value()) {
                publish_diagnostic(
                        endpoint_index, message.message_id,
                        message.route_epoch, message.hop_count,
                        swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED,
                        "decode", decoded.diagnostic, receive_ns, 0U);
                return;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            if(faulted_) {
                publish_diagnostic(
                        endpoint_index, message.message_id,
                        message.route_epoch, message.hop_count,
                        swarm_data_interfaces::msg::LinkDiagnostic::EVENT_LINK_DOWN,
                        "fault_injection", "Relay fault injection is active",
                        receive_ns, 0U);
                return;
            }
            const auto forwarding_ns = steady_now_ns() - receive_ns;
            auto result = endpoints_[endpoint_index].relay->forward(
                    snapshots_, *decoded.message, forwarding_ns);
            if(!result || !result.message.has_value()) {
                const auto event = result.status == PureRelayStatus::RejectedExpired
                        ? swarm_data_interfaces::msg::LinkDiagnostic::EVENT_EXPIRED
                        : swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED;
                publish_diagnostic(
                        endpoint_index, message.message_id,
                        message.route_epoch, message.hop_count,
                        event, "relay_policy", result.diagnostic,
                        receive_ns, 0U);
                return;
            }

            swarm_data_interfaces::msg::RoutedMapUpdate output;
            std::string diagnostic;
            if(!Ros::encode_routed_map_update(
                       *result.message, output, diagnostic)) {
                publish_diagnostic(
                        endpoint_index, message.message_id,
                        message.route_epoch, message.hop_count,
                        swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED,
                        "encode", diagnostic, receive_ns, 0U);
                return;
            }
            output.map_update.header.stamp = message.map_update.header.stamp;
            endpoints_[endpoint_index].publisher->publish(output);
            publish_diagnostic(
                    endpoint_index, output.message_id,
                    output.route_epoch, output.hop_count,
                    swarm_data_interfaces::msg::LinkDiagnostic::EVENT_DELIVERED,
                    {}, {}, receive_ns, steady_now_ns());
        }

        void publish_diagnostic(
                std::size_t endpoint_index,
                std::string message_id,
                std::uint64_t route_epoch,
                std::uint16_t hop_count,
                std::uint8_t event,
                std::string fault_reason,
                std::string diagnostic,
                std::uint64_t receive_ns,
                std::uint64_t send_ns)
        {
            swarm_data_interfaces::msg::LinkDiagnostic output;
            output.protocol_version = kProtocolVersion;
            output.message_id = bounded(std::move(message_id), 128U);
            output.endpoint_id = bounded(
                    identity_.vehicle_id + ":" + endpoints_[endpoint_index].route_id,
                    128U);
            output.endpoint_session_boot_time_ns = identity_.session.boot_time_ns;
            output.endpoint_session_random_suffix = identity_.session.random_suffix;
            output.event = event;
            output.route_epoch = route_epoch;
            output.hop_count = hop_count;
            output.local_receive_monotonic_ns = receive_ns;
            output.local_send_monotonic_ns = send_ns;
            output.fault_reason = bounded(std::move(fault_reason), 64U);
            output.diagnostic = bounded(std::move(diagnostic), 256U);
            diagnostic_publisher_->publish(output);
        }

        VehicleIdentity identity_;
        RuntimeSnapshotCache snapshots_;
        std::mutex mutex_;
        bool faulted_ = false;
        std::uint64_t evidence_revision_ = 2U;
        std::vector<RouteEndpoint> endpoints_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::TopologySnapshot>::SharedPtr
                topology_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::RoleSnapshot>::SharedPtr
                role_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::RoleTransitionDescriptor>::SharedPtr
                transition_subscription_;
        rclcpp::Publisher<
                swarm_data_interfaces::msg::CapabilityEvidence>::SharedPtr
                evidence_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                diagnostic_publisher_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr fault_service_;
        rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::MapUpdateRelayNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("map_update_relay"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
