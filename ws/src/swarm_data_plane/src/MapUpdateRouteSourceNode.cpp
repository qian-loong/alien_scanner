#include "perception_map_update/ros/MapUpdateConversions.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"
#include "swarm_data_interfaces/msg/link_diagnostic.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace SwarmDataPlane {

    namespace {

        std::uint64_t positive_u64_parameter(
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

        std::uint32_t u32_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(name, default_value);
            if(value < 0
               || static_cast<std::uint64_t>(value)
                          > std::numeric_limits<std::uint32_t>::max()) {
                throw std::invalid_argument(name + " is outside uint32 range");
            }
            return static_cast<std::uint32_t>(value);
        }

        std::uint16_t positive_u16_parameter(
                rclcpp::Node & node,
                const std::string & name,
                std::int64_t default_value)
        {
            const auto value = node.declare_parameter<std::int64_t>(name, default_value);
            if(value <= 0
               || static_cast<std::uint64_t>(value)
                          > std::numeric_limits<std::uint16_t>::max()) {
                throw std::invalid_argument(name + " is outside positive uint16 range");
            }
            return static_cast<std::uint16_t>(value);
        }

        std::string message_id_for(
                const ProducerIdentity & producer,
                std::uint64_t sequence)
        {
            std::ostringstream output;
            output << "m-" << std::hex << std::setfill('0') << std::setw(16)
                   << producer.session.boot_time_ns << '-' << std::setw(8)
                   << producer.session.random_suffix << '-' << std::setw(16) << sequence;
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

        std::uint64_t steady_now_ns()
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
        }

        std::string bounded_diagnostic(std::string diagnostic)
        {
            constexpr std::size_t kLimit = 256U;
            if(diagnostic.size() > kLimit) {
                diagnostic.resize(kLimit);
            }
            return diagnostic;
        }

    }// namespace

    class MapUpdateRouteSourceNode final : public rclcpp::Node
    {
    public:
        MapUpdateRouteSourceNode() : Node("map_update_route_source")
        {
            input_topic_ = declare_parameter<std::string>(
                    "input_topic", "map_updates");
            output_topic_ = declare_parameter<std::string>(
                    "output_topic", "routed_map_updates");
            diagnostic_topic_ = declare_parameter<std::string>(
                    "diagnostic_topic", "data_plane_diagnostics");
            producer_.producer_id = declare_parameter<std::string>(
                    "producer_id", "mapper_endpoint");
            producer_.session.boot_time_ns = positive_u64_parameter(
                    *this, "producer_session_boot_time_ns", 1);
            producer_.session.random_suffix = u32_parameter(
                    *this, "producer_session_random_suffix", 1);
            origin_clock_.domain = declare_parameter<std::string>(
                    "origin_clock_domain", "steady");
            origin_clock_.session.boot_time_ns = positive_u64_parameter(
                    *this, "origin_clock_session_boot_time_ns", 1);
            origin_clock_.session.random_suffix = u32_parameter(
                    *this, "origin_clock_session_random_suffix", 1);
            route_epoch_ = positive_u64_parameter(*this, "route_epoch", 1);
            ttl_hops_ = positive_u16_parameter(*this, "ttl_hops", 8);
            validity_budget_ns_ = positive_u64_parameter(
                    *this, "validity_budget_ns", 5'000'000'000LL);
            const auto depth = positive_u64_parameter(*this, "qos_depth", 4);

            routed_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::RoutedMapUpdate>(
                    output_topic_, Ros::map_update_qos(static_cast<std::size_t>(depth)));
            diagnostic_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    diagnostic_topic_, rclcpp::QoS(16).reliable().durability_volatile());
            input_subscription_ = create_subscription<perception_interfaces::msg::MapUpdate>(
                    input_topic_,
                    Ros::map_update_qos(static_cast<std::size_t>(depth)),
                    [this](perception_interfaces::msg::MapUpdate::ConstSharedPtr message) {
                        on_map_update(*message);
                    });
        }

    private:
        void publish_rejection(std::string message_id, std::string diagnostic)
        {
            swarm_data_interfaces::msg::LinkDiagnostic event;
            event.protocol_version = kProtocolVersion;
            event.message_id = std::move(message_id);
            event.endpoint_id = producer_.producer_id;
            event.endpoint_session_boot_time_ns = producer_.session.boot_time_ns;
            event.endpoint_session_random_suffix = producer_.session.random_suffix;
            event.event = swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED;
            event.route_epoch = route_epoch_;
            event.local_receive_monotonic_ns = steady_now_ns();
            event.fault_reason = "conversion";
            event.diagnostic = bounded_diagnostic(std::move(diagnostic));
            diagnostic_publisher_->publish(event);
        }

        void on_map_update(const perception_interfaces::msg::MapUpdate & message)
        {
            if(next_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
                publish_rejection({}, "producer sequence is exhausted");
                return;
            }
            auto decoded = PerceptionMapUpdate::Ros::decode_map_update(message);
            if(!decoded.success || !decoded.update.has_value()) {
                publish_rejection({}, decoded.diagnostic);
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
            routed.validity_budget_ns = validity_budget_ns_;
            routed.route = {route_epoch_, 0U, ttl_hops_};
            routed.payload_bytes = decoded.update->canonical_payload_bytes;
            routed.payload_hash = decoded.update->update_hash;
            routed.update = std::make_shared<const PerceptionMapUpdate::MapUpdate>(
                    std::move(*decoded.update));

            swarm_data_interfaces::msg::RoutedMapUpdate output;
            std::string diagnostic;
            if(!Ros::encode_routed_map_update(routed, output, diagnostic)) {
                publish_rejection(routed.message_id, diagnostic);
                return;
            }
            output.map_update.header.stamp = message.header.stamp;
            routed_publisher_->publish(output);
        }

        std::string input_topic_;
        std::string output_topic_;
        std::string diagnostic_topic_;
        ProducerIdentity producer_;
        OriginClock origin_clock_;
        std::uint64_t route_epoch_ = 1U;
        std::uint16_t ttl_hops_ = 8U;
        std::uint64_t validity_budget_ns_ = 5'000'000'000U;
        std::uint64_t next_sequence_ = 1U;
        rclcpp::Publisher<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                routed_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                diagnostic_publisher_;
        rclcpp::Subscription<perception_interfaces::msg::MapUpdate>::SharedPtr
                input_subscription_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::MapUpdateRouteSourceNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("map_update_route_source"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
