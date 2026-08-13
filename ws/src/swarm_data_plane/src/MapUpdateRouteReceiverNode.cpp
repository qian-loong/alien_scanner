#include "swarm_data_plane/MapUpdateIngress.hpp"
#include "swarm_data_plane/RoutedResync.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"
#include "swarm_data_plane/ros/RoutedResyncConversions.hpp"
#include "swarm_data_interfaces/msg/delivery_ack.hpp"
#include "swarm_data_interfaces/srv/request_routed_map_resync.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

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

        bool is_delivered(IngressStatus status)
        {
            return status == IngressStatus::AppliedKeyframe
                   || status == IngressStatus::AppliedDelta
                   || status == IngressStatus::AppliedRemove
                   || status == IngressStatus::AcceptedSummary;
        }

        PerceptionMapUpdate::ResyncReason resync_reason(IngressStatus status)
        {
            switch(status) {
                case IngressStatus::RejectedConflict:
                    return PerceptionMapUpdate::ResyncReason::HashConflict;
                case IngressStatus::RejectedRoute:
                    return PerceptionMapUpdate::ResyncReason::EpochChange;
                case IngressStatus::RejectedInvalid:
                case IngressStatus::RejectedResourceLimit:
                    return PerceptionMapUpdate::ResyncReason::LocalStateInvalid;
                default:
                    return PerceptionMapUpdate::ResyncReason::Gap;
            }
        }

    }// namespace

    class MapUpdateRouteReceiverNode final : public rclcpp::Node
    {
    public:
        MapUpdateRouteReceiverNode() : Node("map_update_route_receiver")
        {
            input_topic_ = declare_parameter<std::string>(
                    "input_topic", "routed_map_updates");
            accepted_topic_ = declare_parameter<std::string>(
                    "accepted_topic", "accepted_map_updates");
            ack_topic_ = declare_parameter<std::string>(
                    "ack_topic", "delivery_acks");
            resync_service_ = declare_parameter<std::string>(
                    "resync_service", "request_routed_map_resync");
            requester_id_ = declare_parameter<std::string>(
                    "requester_id", "map_receiver");
            requester_session_.boot_time_ns = positive_u64_parameter(
                    *this, "requester_session_boot_time_ns", 1);
            requester_session_.random_suffix = u32_parameter(
                    *this, "requester_session_random_suffix", 1);

            expected_producer_.producer_id = declare_parameter<std::string>(
                    "expected_producer_id", "mapper_endpoint");
            expected_producer_.session.boot_time_ns = positive_u64_parameter(
                    *this, "expected_producer_session_boot_time_ns", 1);
            expected_producer_.session.random_suffix = u32_parameter(
                    *this, "expected_producer_session_random_suffix", 1);
            expected_source_.vehicle_id = declare_parameter<std::string>(
                    "expected_vehicle_id", "drone_0");
            expected_source_.mapper_session.boot_time_ns = positive_u64_parameter(
                    *this, "expected_mapper_session_boot_time_ns", 1);
            expected_source_.mapper_session.random_suffix = u32_parameter(
                    *this, "expected_mapper_session_random_suffix", 1);
            expected_source_.map_epoch = positive_u64_parameter(
                    *this, "expected_map_epoch", 1);
            const auto depth = positive_u64_parameter(*this, "qos_depth", 4);

            if(!ingress_.admit_producer(expected_producer_)
               || !ingress_.admit_source(expected_source_)) {
                throw std::invalid_argument("expected producer or source admission is invalid");
            }

            accepted_publisher_ = create_publisher<perception_interfaces::msg::MapUpdate>(
                    accepted_topic_, Ros::map_update_qos(static_cast<std::size_t>(depth)));
            ack_publisher_ = create_publisher<swarm_data_interfaces::msg::DeliveryAck>(
                    ack_topic_, rclcpp::QoS(16).reliable().durability_volatile());
            resync_client_ = create_client<
                    swarm_data_interfaces::srv::RequestRoutedMapResync>(resync_service_);
            input_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoutedMapUpdate>(
                    input_topic_,
                    Ros::map_update_qos(static_cast<std::size_t>(depth)),
                    [this](
                            swarm_data_interfaces::msg::RoutedMapUpdate::ConstSharedPtr message) {
                        process_message(*message);
                    });
        }

    private:
        void publish_ack(
                const swarm_data_interfaces::msg::RoutedMapUpdate & message,
                const IngressResult & result)
        {
            swarm_data_interfaces::msg::DeliveryAck ack;
            ack.protocol_version = kProtocolVersion;
            ack.message_id = message.message_id;
            ack.receiver_id = requester_id_;
            ack.receiver_session_boot_time_ns = requester_session_.boot_time_ns;
            ack.receiver_session_random_suffix = requester_session_.random_suffix;
            if(is_delivered(result.status)) {
                ack.status = swarm_data_interfaces::msg::DeliveryAck::STATUS_DELIVERED;
            } else if(result.status == IngressStatus::IgnoredDuplicate) {
                ack.status = swarm_data_interfaces::msg::DeliveryAck::STATUS_DUPLICATE;
            } else if(result.status == IngressStatus::RejectedExpired) {
                ack.status = swarm_data_interfaces::msg::DeliveryAck::STATUS_EXPIRED;
            } else {
                ack.status = swarm_data_interfaces::msg::DeliveryAck::STATUS_REJECTED;
            }
            ack.resync_required = result.resync_required;
            ack.correlation_id = message.correlation_id;
            ack.source_vehicle_id = message.map_update.vehicle_id;
            ack.source_mapper_session_boot_time_ns =
                    message.map_update.mapper_session_boot_time_ns;
            ack.source_mapper_session_random_suffix =
                    message.map_update.mapper_session_random_suffix;
            ack.source_map_epoch = message.map_update.map_epoch;
            ack.source_revision = message.map_update.new_revision;
            ack.receive_monotonic_ns = steady_now_ns();
            ack.diagnostic = bounded_diagnostic(result.diagnostic);
            ack_publisher_->publish(ack);
        }

        void process_message(const swarm_data_interfaces::msg::RoutedMapUpdate & message)
        {
            auto decoded = Ros::decode_routed_map_update(message);
            if(!decoded.success || !decoded.message.has_value()) {
                IngressResult rejected;
                rejected.status = IngressStatus::RejectedInvalid;
                rejected.resync_required = true;
                rejected.diagnostic = decoded.diagnostic;
                publish_ack(message, rejected);
                request_resync(rejected.status, message.route_epoch);
                return;
            }

            const auto update_kind = decoded.message->update->kind;
            auto result = ingress_.receive(*decoded.message, steady_now_ns());
            publish_ack(message, result);
            if(is_delivered(result.status)) {
                accepted_publisher_->publish(message.map_update);
                return;
            }
            if(result.resync_required
               && update_kind == PerceptionMapUpdate::UpdateKind::Keyframe
               && !message.correlation_id.empty()) {
                pending_keyframe_ = message;
            }
            if(result.resync_required) {
                request_resync(result.status, message.route_epoch);
            }
        }

        void request_resync(IngressStatus cause, std::uint64_t route_epoch)
        {
            if(resync_in_flight_ || !resync_client_->service_is_ready()) {
                return;
            }
            RoutedResyncIntent intent;
            intent.target_producer = expected_producer_;
            intent.route_epoch = route_epoch;
            intent.request.requester = {requester_id_, requester_session_};
            intent.request.client_request_id =
                    requester_id_ + "-" + std::to_string(next_request_id_++);
            intent.request.expected_source = expected_source_;
            intent.request.reason = resync_reason(cause);
            if(ingress_.map_applier().reconstructed_map().has_value()) {
                intent.request.receiver_revision =
                        ingress_.map_applier().reconstructed_map()->revision;
                intent.request.receiver_content_hash =
                        ingress_.map_applier().reconstructed_map()->content_hash;
            }

            auto request = std::make_shared<
                    swarm_data_interfaces::srv::RequestRoutedMapResync::Request>();
            std::string diagnostic;
            if(!Ros::encode_resync_intent(intent, request->intent, diagnostic)) {
                RCLCPP_ERROR(get_logger(), "cannot encode resync intent: %s", diagnostic.c_str());
                return;
            }
            resync_in_flight_ = true;
            resync_client_->async_send_request(
                    request,
                    [this](rclcpp::Client<
                                   swarm_data_interfaces::srv::RequestRoutedMapResync>::
                                   SharedFuture future) {
                        resync_in_flight_ = false;
                        const auto decoded = Ros::decode_resync_ack(future.get()->ack);
                        if(!decoded.success || !decoded.ack.has_value()
                           || !decoded.ack->accepted) {
                            RCLCPP_WARN(
                                    get_logger(),
                                    "routed resync was rejected: %s",
                                    decoded.diagnostic.c_str());
                            return;
                        }
                        if(!ingress_.expect_resync(decoded.ack->correlation_id)) {
                            RCLCPP_WARN(
                                    get_logger(),
                                    "resync correlation was not expected by the ingress");
                            return;
                        }
                        if(pending_keyframe_.has_value()) {
                            auto pending = std::move(*pending_keyframe_);
                            pending_keyframe_.reset();
                            process_message(pending);
                        }
                    });
        }

        std::string input_topic_;
        std::string accepted_topic_;
        std::string ack_topic_;
        std::string resync_service_;
        std::string requester_id_;
        Perception::SessionID requester_session_ {0U, 0U};
        ProducerIdentity expected_producer_;
        PerceptionMapUpdate::SourceIdentity expected_source_;
        MapUpdateIngress ingress_;
        bool resync_in_flight_ = false;
        std::uint64_t next_request_id_ = 1U;
        std::optional<swarm_data_interfaces::msg::RoutedMapUpdate> pending_keyframe_;
        rclcpp::Publisher<perception_interfaces::msg::MapUpdate>::SharedPtr
                accepted_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::DeliveryAck>::SharedPtr ack_publisher_;
        rclcpp::Client<swarm_data_interfaces::srv::RequestRoutedMapResync>::SharedPtr
                resync_client_;
        rclcpp::Subscription<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                input_subscription_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::MapUpdateRouteReceiverNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(rclcpp::get_logger("map_update_route_receiver"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
