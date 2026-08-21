#include "perception_map_update/ros/MapUpdateConversions.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"
#include "swarm_data_plane/ros/TopologyConversions.hpp"

#include "swarm_data_interfaces/msg/link_diagnostic.hpp"
#include "swarm_data_interfaces/msg/routed_map_update.hpp"
#include "swarm_data_interfaces/msg/topology_snapshot.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

namespace SwarmDataPlane {

    namespace {

        std::uint64_t steady_now_ns()
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
        }

        std::uint64_t positive_u64_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(
                    name, default_value);
            if(value <= 0) {
                throw std::invalid_argument(name + " must be positive");
            }
            return static_cast<std::uint64_t>(value);
        }

        std::uint32_t positive_u32_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(
                    name, default_value);
            if(value <= 0
               || static_cast<std::uint64_t>(value)
                          > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument(name + " is outside positive uint32 range");
            }
            return static_cast<std::uint32_t>(value);
        }

        std::string message_id_for(
                const ProducerIdentity & producer,
                std::uint64_t sequence)
        {
            std::ostringstream output;
            output << "m-" << std::hex << std::setfill('0') << std::setw(16)
                   << producer.session.boot_time_ns << '-' << std::setw(8)
                   << producer.session.random_suffix << '-' << std::setw(16)
                   << sequence;
            return output.str();
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

        std::string bounded(std::string value, std::size_t limit)
        {
            if(value.size() > limit) {
                value.resize(limit);
            }
            return value;
        }

    }// namespace

    class TopologyMapRouteSourceNode final : public rclcpp::Node
    {
    public:
        TopologyMapRouteSourceNode() : Node("topology_map_route_source")
        {
            input_topic_ = declare_parameter<std::string>(
                    "input_topic", "map_updates");
            route_id_ = declare_parameter<std::string>(
                    "route_id", "route-map-0");
            topology_topic_ = declare_parameter<std::string>(
                    "topology_topic", "/swarm/runtime/topology");
            diagnostic_topic_ = declare_parameter<std::string>(
                    "diagnostic_topic", "/swarm/runtime/source_diagnostics");
            active_relay_vehicle_id_ = declare_parameter<std::string>(
                    "active_relay_vehicle_id", "relay-0");
            standby_relay_vehicle_id_ = declare_parameter<std::string>(
                    "standby_relay_vehicle_id", "relay-1");
            const auto active_output_topic = declare_parameter<std::string>(
                    "active_output_topic", "relay_0_input");
            const auto standby_output_topic = declare_parameter<std::string>(
                    "standby_output_topic", "relay_1_input");

            source_identity_.fleet_id = declare_parameter<std::string>(
                    "fleet_id", "fleet-a");
            source_identity_.vehicle_id = declare_parameter<std::string>(
                    "vehicle_id", "explorer-0");
            source_identity_.session.boot_time_ns = positive_u64_parameter(
                    *this, "vehicle_session_boot_time_ns", 100);
            source_identity_.session.random_suffix = positive_u32_parameter(
                    *this, "vehicle_session_random_suffix", 1);
            producer_.producer_id = declare_parameter<std::string>(
                    "producer_id", "mapper-endpoint-0");
            producer_.session.boot_time_ns = positive_u64_parameter(
                    *this, "producer_session_boot_time_ns", 1000);
            producer_.session.random_suffix = positive_u32_parameter(
                    *this, "producer_session_random_suffix", 1);
            origin_clock_.domain = declare_parameter<std::string>(
                    "origin_clock_domain", "steady");
            origin_clock_.session.boot_time_ns = positive_u64_parameter(
                    *this, "origin_clock_session_boot_time_ns", 2000);
            origin_clock_.session.random_suffix = positive_u32_parameter(
                    *this, "origin_clock_session_random_suffix", 1);
            const auto depth = positive_u64_parameter(*this, "qos_depth", 4);
            if(route_id_.empty() || source_identity_.fleet_id.empty()
               || source_identity_.vehicle_id.empty()
               || active_relay_vehicle_id_.empty()
               || standby_relay_vehicle_id_.empty()
               || active_relay_vehicle_id_ == standby_relay_vehicle_id_) {
                throw std::invalid_argument("topology route source parameters are invalid");
            }

            const auto map_qos = Ros::map_update_qos(
                    static_cast<std::size_t>(depth));
            active_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::RoutedMapUpdate>(
                    active_output_topic, map_qos);
            standby_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::RoutedMapUpdate>(
                    standby_output_topic, map_qos);
            diagnostic_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    diagnostic_topic_,
                    rclcpp::QoS(32).reliable().durability_volatile());
            topology_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::TopologySnapshot>(
                    topology_topic_, rclcpp::QoS(1).reliable().transient_local(),
                    [this](const swarm_data_interfaces::msg::TopologySnapshot::
                                   ConstSharedPtr message) {
                        on_topology(*message);
                    });
            input_subscription_ = create_subscription<
                    perception_interfaces::msg::MapUpdate>(
                    input_topic_, map_qos,
                    [this](const perception_interfaces::msg::MapUpdate::
                                   ConstSharedPtr message) {
                        on_map_update(*message);
                    });
        }

    private:
        void on_topology(
                const swarm_data_interfaces::msg::TopologySnapshot & message)
        {
            auto decoded = Ros::decode_topology_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                RCLCPP_WARN(
                        get_logger(), "rejected topology snapshot: %s",
                        decoded.diagnostic.c_str());
                return;
            }
            const auto found = std::find_if(
                    decoded.snapshot->routes.begin(),
                    decoded.snapshot->routes.end(),
                    [&](const RouteDescriptor & route) {
                        return route.route_id == route_id_;
                    });
            if(found == decoded.snapshot->routes.end()
               || found->graph != LogicalGraphKind::Map
               || found->source != source_identity_
               || found->hops.empty()) {
                RCLCPP_WARN(
                        get_logger(),
                        "topology snapshot does not contain the configured source route");
                return;
            }
            const auto first_link = std::find_if(
                    decoded.snapshot->links.begin(),
                    decoded.snapshot->links.end(),
                    [&](const LinkDescriptor & link) {
                        return link.link_id == found->hops.front().link_id
                               && link.link_epoch == found->hops.front().link_epoch;
                    });
            if(first_link == decoded.snapshot->links.end()
               || (first_link->target.vehicle_id != active_relay_vehicle_id_
                   && first_link->target.vehicle_id != standby_relay_vehicle_id_)) {
                RCLCPP_WARN(
                        get_logger(),
                        "source route first hop is not bound to an active/standby Relay");
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if(route_.has_value()
               && found->topology_epoch < route_->topology_epoch) {
                return;
            }
            route_ = *found;
            first_hop_target_vehicle_id_ = first_link->target.vehicle_id;
        }

        void on_map_update(const perception_interfaces::msg::MapUpdate & message)
        {
            auto decoded = PerceptionMapUpdate::Ros::decode_map_update(message);
            if(!decoded.success || !decoded.update.has_value()) {
                publish_rejection({}, 0U, decoded.diagnostic);
                return;
            }

            RouteDescriptor route;
            std::string first_hop_target;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!route_.has_value()) {
                    publish_rejection({}, 0U, "committed route is unavailable");
                    return;
                }
                route = *route_;
                first_hop_target = first_hop_target_vehicle_id_;
            }
            if(next_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
                publish_rejection({}, route.route_epoch, "producer sequence is exhausted");
                return;
            }

            const auto sequence = next_sequence_++;
            RoutedMapUpdate routed;
            routed.message_id = message_id_for(producer_, sequence);
            routed.producer = producer_;
            routed.sequence = sequence;
            routed.correlation_id = decoded.update->correlation_id;
            routed.priority = priority_for(decoded.update->kind);
            routed.origin = origin_clock_;
            routed.origin.time_ns = steady_now_ns();
            routed.validity_budget_ns = route.validity_budget_ns;
            routed.route = {route.route_epoch, 0U, route.ttl_hops};
            routed.payload_bytes = decoded.update->canonical_payload_bytes;
            routed.payload_hash = decoded.update->update_hash;
            routed.update = std::make_shared<const PerceptionMapUpdate::MapUpdate>(
                    std::move(*decoded.update));

            swarm_data_interfaces::msg::RoutedMapUpdate output;
            std::string diagnostic;
            if(!Ros::encode_routed_map_update(routed, output, diagnostic)) {
                publish_rejection(
                        routed.message_id, route.route_epoch, diagnostic);
                return;
            }
            output.map_update.header.stamp = message.header.stamp;
            if(first_hop_target == active_relay_vehicle_id_) {
                active_publisher_->publish(output);
            } else if(first_hop_target == standby_relay_vehicle_id_) {
                standby_publisher_->publish(output);
            } else {
                publish_rejection(
                        routed.message_id,
                        route.route_epoch,
                        "committed route first hop is not bound to a configured Relay");
            }
        }

        void publish_rejection(
                std::string message_id,
                std::uint64_t route_epoch,
                std::string diagnostic)
        {
            swarm_data_interfaces::msg::LinkDiagnostic output;
            output.protocol_version = kProtocolVersion;
            output.message_id = bounded(std::move(message_id), 128U);
            output.endpoint_id = bounded(producer_.producer_id, 128U);
            output.endpoint_session_boot_time_ns = producer_.session.boot_time_ns;
            output.endpoint_session_random_suffix = producer_.session.random_suffix;
            output.event = swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED;
            output.route_epoch = route_epoch;
            output.local_receive_monotonic_ns = steady_now_ns();
            output.fault_reason = "route_source";
            output.diagnostic = bounded(std::move(diagnostic), 256U);
            diagnostic_publisher_->publish(output);
        }

        std::string input_topic_;
        std::string route_id_;
        std::string topology_topic_;
        std::string diagnostic_topic_;
        std::string active_relay_vehicle_id_;
        std::string standby_relay_vehicle_id_;
        VehicleIdentity source_identity_;
        ProducerIdentity producer_;
        OriginClock origin_clock_;
        std::mutex mutex_;
        std::optional<RouteDescriptor> route_;
        std::string first_hop_target_vehicle_id_;
        std::uint64_t next_sequence_ = 1U;
        rclcpp::Publisher<
                swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                active_publisher_;
        rclcpp::Publisher<
                swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                standby_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                diagnostic_publisher_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::TopologySnapshot>::SharedPtr
                topology_subscription_;
        rclcpp::Subscription<perception_interfaces::msg::MapUpdate>::SharedPtr
                input_subscription_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(
                std::make_shared<SwarmDataPlane::TopologyMapRouteSourceNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("topology_map_route_source"),
                "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
