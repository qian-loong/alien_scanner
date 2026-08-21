#include "swarm_data_plane/AggregateContract.hpp"
#include "swarm_data_plane/ros/AggregateConversions.hpp"

#include "swarm_data_interfaces/msg/aggregate_map_update.hpp"
#include "swarm_data_interfaces/msg/link_diagnostic.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace SwarmDataPlane {

    namespace {

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

    class AggregateReceiverNode final : public rclcpp::Node
    {
    public:
        AggregateReceiverNode() : Node("aggregate_receiver")
        {
            const auto input_topic = declare_parameter<std::string>(
                    "input_topic", "/c5d/aggregate/output");
            const auto accepted_topic = declare_parameter<std::string>(
                    "accepted_topic", "/c5d/aggregate/accepted");
            const auto diagnostic_topic = declare_parameter<std::string>(
                    "diagnostic_topic", "/swarm/runtime/aggregate_receiver_diagnostics");
            const auto aggregate_id = declare_parameter<std::string>(
                    "aggregate_id", "edge-aggregator");
            const ProducerIdentity producer {
                    aggregate_id,
                    {positive_u64(*this, "aggregate_producer_session_boot_time_ns", 302),
                     positive_u32(*this, "aggregate_producer_session_random_suffix", 1)}};
            const PerceptionMapUpdate::SourceIdentity source {
                    aggregate_id,
                    {positive_u64(*this, "aggregate_mapper_session_boot_time_ns", 301),
                     positive_u32(*this, "aggregate_mapper_session_random_suffix", 1)},
                    positive_u64(*this, "aggregate_map_epoch", 1)};
            if(!ingress_.admit_producer(producer) || !ingress_.admit_source(source)) {
                throw std::invalid_argument("aggregate receiver identity admission failed");
            }
            accepted_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::AggregateMapUpdate>(
                    accepted_topic, rclcpp::QoS(8).reliable());
            diagnostic_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    diagnostic_topic, rclcpp::QoS(32).reliable());
            subscription_ = create_subscription<
                    swarm_data_interfaces::msg::AggregateMapUpdate>(
                    input_topic, rclcpp::QoS(8).reliable(),
                    [this](const swarm_data_interfaces::msg::AggregateMapUpdate::ConstSharedPtr message) {
                        on_message(*message);
                    });
        }

    private:
        void on_message(
                const swarm_data_interfaces::msg::AggregateMapUpdate & message)
        {
            auto decoded = Ros::decode_aggregate_map_update(message);
            if(!decoded.success || !decoded.update.has_value()) {
                publish_diagnostic(message.aggregate_update.message_id, false, decoded.diagnostic);
                return;
            }
            const auto result = ingress_.receive(
                    *decoded.update,
                    static_cast<std::uint64_t>(get_clock()->now().nanoseconds()));
            if(!result.manifest_changed
               && result.map_result.status != IngressStatus::IgnoredDuplicate) {
                publish_diagnostic(
                        message.aggregate_update.message_id,
                        false,
                        result.map_result.diagnostic);
                return;
            }
            accepted_publisher_->publish(message);
            publish_diagnostic(message.aggregate_update.message_id, true, {});
        }

        void publish_diagnostic(
                const std::string & message_id,
                bool accepted,
                const std::string & diagnostic)
        {
            swarm_data_interfaces::msg::LinkDiagnostic output;
            output.protocol_version = kProtocolVersion;
            output.message_id = message_id;
            output.endpoint_id = "central-aggregate-receiver";
            output.event = accepted
                                   ? swarm_data_interfaces::msg::LinkDiagnostic::EVENT_DELIVERED
                                   : swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED;
            output.fault_reason = accepted ? "" : "aggregate_ingress";
            output.diagnostic = diagnostic;
            diagnostic_publisher_->publish(output);
        }

        AggregateIngress ingress_;
        rclcpp::Subscription<swarm_data_interfaces::msg::AggregateMapUpdate>::SharedPtr
                subscription_;
        rclcpp::Publisher<swarm_data_interfaces::msg::AggregateMapUpdate>::SharedPtr
                accepted_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                diagnostic_publisher_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::AggregateReceiverNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("aggregate_receiver"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
