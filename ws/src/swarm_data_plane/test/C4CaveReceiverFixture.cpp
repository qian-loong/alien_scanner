#include "LogicalLinkAdapter.hpp"

#include "perception_map_update/MapUpdateApplier.hpp"
#include "perception_map_update/OctoMapViewAdapter.hpp"
#include "perception_map_update/ros/MapUpdateConversions.hpp"
#include "swarm_data_plane/MapUpdateIngress.hpp"
#include "swarm_data_plane/ros/QosProfiles.hpp"
#include "swarm_data_plane/ros/RoutedMapConversions.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "perception_interfaces/msg/map_update.hpp"
#include "perception_interfaces/srv/request_map_resync.hpp"
#include "swarm_data_interfaces/msg/delivery_ack.hpp"
#include "swarm_data_interfaces/msg/routed_map_update.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        using ResyncService = perception_interfaces::srv::RequestMapResync;

        std::uint64_t steady_now_ns()
        {
            return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
        }

        std::chrono::nanoseconds seconds_to_duration(double seconds, const char * name)
        {
            if(!std::isfinite(seconds) || seconds < 0.0) {
                throw std::invalid_argument(std::string(name) + " must be finite and non-negative");
            }
            const auto nanoseconds = seconds * 1.0e9;
            if(nanoseconds > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
                throw std::invalid_argument(std::string(name) + " is too large");
            }
            return std::chrono::nanoseconds(static_cast<std::int64_t>(nanoseconds));
        }

        bool is_applied(IngressStatus status) noexcept
        {
            return status == IngressStatus::AppliedKeyframe
                   || status == IngressStatus::AppliedDelta
                   || status == IngressStatus::AppliedRemove;
        }

        std::string ingress_status_name(IngressStatus status)
        {
            switch(status) {
                case IngressStatus::AppliedKeyframe:
                    return "APPLIED_KEYFRAME";
                case IngressStatus::AppliedDelta:
                    return "APPLIED_DELTA";
                case IngressStatus::AppliedRemove:
                    return "APPLIED_REMOVE";
                case IngressStatus::AcceptedSummary:
                    return "ACCEPTED_SUMMARY";
                case IngressStatus::IgnoredDuplicate:
                    return "IGNORED_DUPLICATE";
                case IngressStatus::RejectedAdmission:
                    return "REJECTED_ADMISSION";
                case IngressStatus::RejectedStale:
                    return "REJECTED_STALE";
                case IngressStatus::RejectedGap:
                    return "REJECTED_GAP";
                case IngressStatus::RejectedConflict:
                    return "REJECTED_CONFLICT";
                case IngressStatus::RejectedInvalid:
                    return "REJECTED_INVALID";
                case IngressStatus::RejectedExpired:
                    return "REJECTED_EXPIRED";
                case IngressStatus::RejectedRoute:
                    return "REJECTED_ROUTE";
                case IngressStatus::RejectedResourceLimit:
                    return "REJECTED_RESOURCE_LIMIT";
            }
            return "UNKNOWN";
        }

        geometry_msgs::msg::Point voxel_center(
                const PerceptionMapUpdate::ReconstructedMap & map,
                const PerceptionMapUpdate::CanonicalCell & cell)
        {
            geometry_msgs::msg::Point point;
            point.x = map.geometry.lattice_origin.x
                      + (static_cast<double>(cell.index.x) + 0.5)
                                * map.geometry.resolution_m;
            point.y = map.geometry.lattice_origin.y
                      + (static_cast<double>(cell.index.y) + 0.5)
                                * map.geometry.resolution_m;
            point.z = map.geometry.lattice_origin.z
                      + (static_cast<double>(cell.index.z) + 0.5)
                                * map.geometry.resolution_m;
            return point;
        }

    }// namespace

    enum class RecoveryPhase : std::uint8_t
    {
        Healthy,
        DeltaDropped,
        GapHold,
        RequestingResync,
        AwaitingKeyframe,
        Recovered,
        Failed
    };

    struct SourceChannel {
        SourceChannel(
                std::string label_value,
                std::string expected_vehicle_value,
                bool inject_drop_value)
                : label(std::move(label_value)),
                  expected_vehicle(std::move(expected_vehicle_value)),
                  inject_drop(inject_drop_value),
                  link(LogicalLinkConfig {})
        {
        }

        std::string label;
        std::string expected_vehicle;
        bool inject_drop = false;
        bool dropped_first_delta = false;
        bool gap_seen = false;
        bool request_in_flight = false;
        bool recovered = false;
        bool producer_admitted = false;
        bool source_admitted = false;
        std::uint64_t dropped_sequence = 0U;
        std::uint64_t dropped_revision = 0U;
        std::uint64_t gap_sequence = 0U;
        std::uint64_t accepted_revision = 0U;
        std::uint64_t oracle_revision = 0U;
        std::uint64_t recovery_revision = 0U;
        std::uint64_t resync_due_ns = 0U;
        std::uint64_t resync_generation = 0U;
        std::string expected_correlation;
        std::string last_event = "waiting for first keyframe";
        RecoveryPhase phase = RecoveryPhase::Healthy;
        std::optional<ProducerIdentity> producer;
        std::optional<PerceptionMapUpdate::SourceIdentity> source;
        PerceptionMapUpdate::MapUpdateApplier source_oracle;
        MapUpdateIngress ingress;
        LogicalLinkAdapter link;
        std::vector<RoutedMapUpdate> pending_recovery_messages;
        geometry_msgs::msg::Point position;
        bool has_position = false;
        rclcpp::Subscription<swarm_data_interfaces::msg::RoutedMapUpdate>::SharedPtr
                routed_subscription;
        rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr accepted_octomap_publisher;
        rclcpp::Publisher<perception_interfaces::msg::MapUpdate>::SharedPtr
                accepted_update_publisher;
        rclcpp::Client<ResyncService>::SharedPtr resync_client;
    };

    class C4CaveReceiverFixture final : public rclcpp::Node
    {
    public:
        C4CaveReceiverFixture()
                : Node("c4_cave_receiver_fixture"),
                  a1_("A1", declare_parameter<std::string>("a1.vehicle_id", "c4-a1"), false),
                  a2_("A2", declare_parameter<std::string>("a2.vehicle_id", "c4-a2"), true)
        {
            frame_id_ = declare_parameter<std::string>("frame_id", "map");
            if(frame_id_.empty()) {
                throw std::invalid_argument("frame_id must not be empty");
            }
            const auto hold = seconds_to_duration(
                    declare_parameter<double>("resync_hold_s", 4.0), "resync_hold_s");
            resync_hold_ns_ = static_cast<std::uint64_t>(hold.count());
            const auto publish_period = seconds_to_duration(
                    1.0 / declare_positive_rate("publish_rate_hz", 5.0),
                    "publish_rate_hz period");

            ack_publisher_ = create_publisher<swarm_data_interfaces::msg::DeliveryAck>(
                    declare_parameter<std::string>(
                            "delivery_ack_topic", "/c4/cave/b/delivery_ack"),
                    rclcpp::QoS(64).reliable().durability_volatile());
            marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    declare_parameter<std::string>(
                            "status_marker_topic", "/c4/cave/b/communication_status"),
                    rclcpp::QoS(1).reliable().transient_local());
            difference_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    declare_parameter<std::string>(
                            "difference_marker_topic", "/c4/cave/b/map_difference_oracle"),
                    rclcpp::QoS(1).reliable().transient_local());

            configure_channel(
                    a1_,
                    declare_parameter<std::string>(
                            "a1.routed_topic", "/c4/cave/a1/routed_map_updates"),
                    declare_parameter<std::string>(
                            "a1.accepted_octomap_topic", "/c4/cave/b/a1/accepted_octomap"),
                    declare_parameter<std::string>(
                            "a1.accepted_update_topic", "/c4/cave/b/a1/accepted_updates"),
                    declare_parameter<std::string>(
                            "a1.resync_service", "/c4/cave/a1/local_map/request_resync"));
            configure_channel(
                    a2_,
                    declare_parameter<std::string>(
                            "a2.routed_topic", "/c4/cave/a2/routed_map_updates"),
                    declare_parameter<std::string>(
                            "a2.accepted_octomap_topic", "/c4/cave/b/a2/accepted_octomap"),
                    declare_parameter<std::string>(
                            "a2.accepted_update_topic", "/c4/cave/b/a2/accepted_updates"),
                    declare_parameter<std::string>(
                            "a2.resync_service", "/c4/cave/a2/local_map/request_resync"));

            configure_odometry(
                    a1_,
                    declare_parameter<std::string>("a1.odom_topic", "/c4/cave/a1/odom"));
            configure_odometry(
                    a2_,
                    declare_parameter<std::string>("a2.odom_topic", "/c4/cave/a2/odom"));
            b_odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
                    declare_parameter<std::string>("b.odom_topic", "/c4/cave/b/odom"),
                    rclcpp::QoS(10).best_effort().durability_volatile(),
                    [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                        b_position_ = message->pose.pose.position;
                        has_b_position_ = true;
                    });

            requester_session_ = {steady_now_ns(), 0xC4U};
            timer_ = create_wall_timer(publish_period, [this]() { on_timer(); });
        }

    private:
        double declare_positive_rate(const std::string & name, double default_value)
        {
            const auto value = declare_parameter<double>(name, default_value);
            if(!std::isfinite(value) || value <= 0.0) {
                throw std::invalid_argument(name + " must be finite and positive");
            }
            return value;
        }

        void configure_channel(
                SourceChannel & channel,
                const std::string & routed_topic,
                const std::string & accepted_octomap_topic,
                const std::string & accepted_update_topic,
                const std::string & resync_service)
        {
            channel.accepted_octomap_publisher = create_publisher<octomap_msgs::msg::Octomap>(
                    accepted_octomap_topic, rclcpp::QoS(1).reliable().transient_local());
            channel.accepted_update_publisher =
                    create_publisher<perception_interfaces::msg::MapUpdate>(
                            accepted_update_topic,
                            Ros::map_update_qos(8U));
            channel.resync_client = create_client<ResyncService>(resync_service);
            channel.routed_subscription = create_subscription<
                    swarm_data_interfaces::msg::RoutedMapUpdate>(
                    routed_topic,
                    Ros::map_update_qos(8U),
                    [this, &channel](
                            swarm_data_interfaces::msg::RoutedMapUpdate::ConstSharedPtr message) {
                        on_routed(channel, *message);
                    });
        }

        void configure_odometry(SourceChannel & channel, const std::string & topic)
        {
            odom_subscriptions_.push_back(create_subscription<nav_msgs::msg::Odometry>(
                    topic,
                    rclcpp::QoS(10).best_effort().durability_volatile(),
                    [&channel](nav_msgs::msg::Odometry::ConstSharedPtr message) {
                        channel.position = message->pose.pose.position;
                        channel.has_position = true;
                    }));
        }

        bool admit_if_needed(SourceChannel & channel, const RoutedMapUpdate & routed)
        {
            if(routed.update->source.vehicle_id != channel.expected_vehicle) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = "rejected unexpected vehicle "
                                     + routed.update->source.vehicle_id;
                return false;
            }
            if(!channel.producer_admitted) {
                if(!channel.ingress.admit_producer(routed.producer)) {
                    channel.phase = RecoveryPhase::Failed;
                    channel.last_event = "failed to admit routed producer";
                    return false;
                }
                channel.producer = routed.producer;
                channel.producer_admitted = true;
            }
            if(!channel.source_admitted) {
                if(!channel.ingress.admit_source(routed.update->source)
                   || !channel.source_oracle.admit_source(routed.update->source)) {
                    channel.phase = RecoveryPhase::Failed;
                    channel.last_event = "failed to admit map source";
                    return false;
                }
                channel.source = routed.update->source;
                channel.source_admitted = true;
            }
            return channel.producer == routed.producer
                   && channel.source == routed.update->source;
        }

        void on_routed(
                SourceChannel & channel,
                const swarm_data_interfaces::msg::RoutedMapUpdate & message)
        {
            auto decoded = Ros::decode_routed_map_update(message);
            if(!decoded.success || !decoded.message.has_value()) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = "decode rejected: " + decoded.diagnostic;
                return;
            }
            auto routed = std::move(*decoded.message);
            if(!admit_if_needed(channel, routed)) {
                publish_ack(
                        channel,
                        routed,
                        swarm_data_interfaces::msg::DeliveryAck::STATUS_REJECTED,
                        false,
                        channel.last_event);
                return;
            }

            const auto & oracle_map = channel.source_oracle.reconstructed_map();
            const bool equivalent_resync_keyframe =
                    routed.update->kind == PerceptionMapUpdate::UpdateKind::Keyframe
                    && !routed.correlation_id.empty() && oracle_map.has_value()
                    && oracle_map->source == routed.update->source
                    && oracle_map->revision == routed.update->new_revision
                    && oracle_map->content_identity
                               == PerceptionMapUpdate::VersionedContentDigest {
                                       routed.update->content_identity,
                                       routed.update->content_hash};
            const auto oracle_result = equivalent_resync_keyframe
                                               ? PerceptionMapUpdate::ApplyUpdateResult {
                                                         PerceptionMapUpdate::
                                                                 ApplyUpdateStatus::
                                                                         IgnoredDuplicate,
                                                         false,
                                                         {}}
                                               : channel.source_oracle.apply(*routed.update);
            if(channel.source_oracle.reconstructed_map().has_value()) {
                channel.oracle_revision =
                        channel.source_oracle.reconstructed_map()->revision;
            }
            if(oracle_result.status == PerceptionMapUpdate::ApplyUpdateStatus::RejectedInvalid
               || oracle_result.status
                          == PerceptionMapUpdate::ApplyUpdateStatus::RejectedConflict) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = "source oracle rejected update";
            }

            const auto now_ns = steady_now_ns();
            if(channel.inject_drop && !channel.dropped_first_delta
               && routed.update->kind == PerceptionMapUpdate::UpdateKind::Delta) {
                channel.dropped_first_delta = true;
                channel.dropped_sequence = routed.sequence;
                channel.dropped_revision = routed.update->new_revision;
                channel.phase = RecoveryPhase::DeltaDropped;
                channel.last_event = "test link dropped delta sequence "
                                     + std::to_string(routed.sequence);
                channel.link.set_partitioned(true, now_ns);
                channel.link.submit(routed, now_ns);
                channel.link.set_partitioned(false, now_ns);
                publish_ack(
                        channel,
                        routed,
                        swarm_data_interfaces::msg::DeliveryAck::STATUS_REJECTED,
                        false,
                        "test-only deterministic delta drop");
                return;
            }

            const auto submitted = channel.link.submit(routed, now_ns);
            if(!submitted.accepted) {
                publish_ack(
                        channel,
                        routed,
                        swarm_data_interfaces::msg::DeliveryAck::STATUS_REJECTED,
                        channel.ingress.map_applier().state()
                                == PerceptionMapUpdate::ReceiverState::ResyncRequired,
                        "logical link rejected message");
                return;
            }
            deliver_ready(channel, now_ns);
        }

        void deliver_ready(SourceChannel & channel, std::uint64_t now_ns)
        {
            for(auto & routed : channel.link.poll(now_ns)) {
                if(channel.request_in_flight
                   && routed.update->kind == PerceptionMapUpdate::UpdateKind::Keyframe
                   && !routed.correlation_id.empty()
                   && channel.expected_correlation.empty()) {
                    if(channel.pending_recovery_messages.size() < 64U) {
                        channel.pending_recovery_messages.push_back(std::move(routed));
                    }
                    continue;
                }
                if(!channel.pending_recovery_messages.empty()
                   && channel.expected_correlation.empty()) {
                    if(channel.pending_recovery_messages.size() < 64U) {
                        channel.pending_recovery_messages.push_back(std::move(routed));
                    }
                    continue;
                }
                apply_delivered(channel, routed, now_ns);
            }
        }

        void apply_delivered(
                SourceChannel & channel,
                const RoutedMapUpdate & routed,
                std::uint64_t now_ns)
        {
            const auto result = channel.ingress.receive(routed, now_ns);
            if(channel.ingress.map_applier().reconstructed_map().has_value()) {
                channel.accepted_revision =
                        channel.ingress.map_applier().reconstructed_map()->revision;
            }

            std::uint8_t ack_status = swarm_data_interfaces::msg::DeliveryAck::STATUS_REJECTED;
            if(is_applied(result.status)
               || result.status == IngressStatus::AcceptedSummary) {
                ack_status = swarm_data_interfaces::msg::DeliveryAck::STATUS_DELIVERED;
            } else if(result.status == IngressStatus::IgnoredDuplicate) {
                ack_status = swarm_data_interfaces::msg::DeliveryAck::STATUS_DUPLICATE;
            } else if(result.status == IngressStatus::RejectedExpired) {
                ack_status = swarm_data_interfaces::msg::DeliveryAck::STATUS_EXPIRED;
            }
            publish_ack(
                    channel,
                    routed,
                    ack_status,
                    result.resync_required,
                    ingress_status_name(result.status)
                            + (result.diagnostic.empty() ? "" : ": " + result.diagnostic));

            if(result.status == IngressStatus::RejectedGap && channel.inject_drop) {
                if(!channel.gap_seen) {
                    channel.gap_seen = true;
                    channel.gap_sequence = routed.sequence;
                    channel.resync_due_ns = now_ns + resync_hold_ns_;
                }
                if(!channel.request_in_flight && channel.expected_correlation.empty()) {
                    channel.phase = RecoveryPhase::GapHold;
                    channel.last_event = "gap: expected sequence "
                                         + std::to_string(channel.dropped_sequence)
                                         + ", received " + std::to_string(routed.sequence);
                }
            } else if(result.status == IngressStatus::AppliedKeyframe
                      && channel.inject_drop && !routed.correlation_id.empty()) {
                channel.recovered = true;
                channel.recovery_revision = routed.update->new_revision;
                channel.phase = RecoveryPhase::Recovered;
                channel.last_event = "correlated keyframe recovered revision "
                                     + std::to_string(channel.recovery_revision);
            } else if(is_applied(result.status) && !channel.inject_drop) {
                channel.phase = RecoveryPhase::Healthy;
                channel.last_event = ingress_status_name(result.status) + " revision "
                                     + std::to_string(routed.update->new_revision);
            }

            if(is_applied(result.status)) {
                publish_accepted_update(channel, routed);
                publish_accepted_octomap(channel);
            }
        }

        void publish_accepted_update(
                SourceChannel & channel,
                const RoutedMapUpdate & routed)
        {
            perception_interfaces::msg::MapUpdate message;
            std::string diagnostic;
            if(!PerceptionMapUpdate::Ros::encode_map_update(
                       *routed.update, message, diagnostic)) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = "accepted update encode failed: " + diagnostic;
                return;
            }
            message.header.stamp = get_clock()->now();
            channel.accepted_update_publisher->publish(message);
        }

        void publish_accepted_octomap(SourceChannel & channel)
        {
            const auto & reconstructed =
                    channel.ingress.map_applier().reconstructed_map();
            if(!reconstructed.has_value()) {
                return;
            }
            octomap_msgs::msg::Octomap message;
            std::string diagnostic;
            if(!PerceptionMapUpdate::OctoMapViewAdapter::materialize(
                       *reconstructed, message, diagnostic)) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = "OctoMap materialization failed: " + diagnostic;
                return;
            }
            message.header.stamp = get_clock()->now();
            message.header.frame_id = reconstructed->geometry.frame_id;
            channel.accepted_octomap_publisher->publish(message);
        }

        void publish_ack(
                const SourceChannel & channel,
                const RoutedMapUpdate & routed,
                std::uint8_t status,
                bool resync_required,
                std::string diagnostic)
        {
            swarm_data_interfaces::msg::DeliveryAck ack;
            ack.protocol_version = kProtocolVersion;
            ack.message_id = routed.message_id;
            ack.receiver_id = "c4-cave-b";
            ack.receiver_session_boot_time_ns = requester_session_.boot_time_ns;
            ack.receiver_session_random_suffix = requester_session_.random_suffix;
            ack.status = status;
            ack.resync_required = resync_required;
            ack.correlation_id = routed.correlation_id;
            ack.source_vehicle_id = routed.update->source.vehicle_id;
            ack.source_mapper_session_boot_time_ns =
                    routed.update->source.mapper_session.boot_time_ns;
            ack.source_mapper_session_random_suffix =
                    routed.update->source.mapper_session.random_suffix;
            ack.source_map_epoch = routed.update->source.map_epoch;
            ack.source_revision = routed.update->new_revision;
            ack.receive_monotonic_ns = steady_now_ns();
            if(diagnostic.size() > 256U) {
                diagnostic.resize(256U);
            }
            ack.diagnostic = channel.label + ": " + std::move(diagnostic);
            if(ack.diagnostic.size() > 256U) {
                ack.diagnostic.resize(256U);
            }
            ack_publisher_->publish(ack);
        }

        void request_resync(SourceChannel & channel)
        {
            if(channel.request_in_flight || !channel.source.has_value()
               || !channel.resync_client->service_is_ready()) {
                return;
            }
            const auto & reconstructed =
                    channel.ingress.map_applier().reconstructed_map();
            if(!reconstructed.has_value()) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = "cannot resync without retained baseline";
                return;
            }

            auto request = std::make_shared<ResyncService::Request>();
            ++channel.resync_generation;
            request->requester_id = "c4-cave-b-" + channel.label;
            request->requester_session_boot_time_ns = requester_session_.boot_time_ns;
            request->requester_session_random_suffix = requester_session_.random_suffix;
            request->client_request_id = "c4-cave-gap-" + channel.label + '-'
                                         + std::to_string(channel.resync_generation);
            request->bootstrap_latest = false;
            request->expected_vehicle_id = channel.source->vehicle_id;
            request->expected_mapper_session_boot_time_ns =
                    channel.source->mapper_session.boot_time_ns;
            request->expected_mapper_session_random_suffix =
                    channel.source->mapper_session.random_suffix;
            request->expected_map_epoch = channel.source->map_epoch;
            request->receiver_revision = reconstructed->revision;
            request->receiver_content_identity.scheme = static_cast<std::uint16_t>(
                    reconstructed->content_identity.descriptor.scheme);
            request->receiver_content_identity.chunk_edge =
                    reconstructed->content_identity.descriptor.chunk_edge;
            request->receiver_content_identity.coordinate_key_version =
                    reconstructed->content_identity.descriptor.coordinate_key_version;
            request->receiver_content_identity.node_encoding_version =
                    reconstructed->content_identity.descriptor.node_encoding_version;
            std::copy(
                    reconstructed->content_identity.digest.begin(),
                    reconstructed->content_identity.digest.end(),
                    request->receiver_content_hash.begin());
            request->reason = ResyncService::Request::REASON_GAP;

            channel.request_in_flight = true;
            channel.phase = RecoveryPhase::RequestingResync;
            channel.last_event = "requesting correlated keyframe";
            auto * channel_ptr = &channel;
            channel.resync_client->async_send_request(
                    request,
                    [this, channel_ptr](rclcpp::Client<ResyncService>::SharedFuture future) {
                        on_resync_response(*channel_ptr, future);
                    });
        }

        void on_resync_response(
                SourceChannel & channel,
                rclcpp::Client<ResyncService>::SharedFuture future)
        {
            channel.request_in_flight = false;
            try {
                const auto response = future.get();
                if(!response->accepted || !channel.source.has_value()
                   || response->current_vehicle_id != channel.source->vehicle_id
                   || response->current_mapper_session_boot_time_ns
                              != channel.source->mapper_session.boot_time_ns
                   || response->current_mapper_session_random_suffix
                              != channel.source->mapper_session.random_suffix
                   || response->current_map_epoch != channel.source->map_epoch
                   || !channel.ingress.expect_resync(response->correlation_id)) {
                    channel.phase = RecoveryPhase::Failed;
                    channel.last_event = response->diagnostic.empty()
                                                 ? "resync response rejected"
                                                 : response->diagnostic;
                    return;
                }
                channel.expected_correlation = response->correlation_id;
                channel.phase = RecoveryPhase::AwaitingKeyframe;
                channel.last_event = "awaiting correlation " + response->correlation_id;

                auto pending = std::move(channel.pending_recovery_messages);
                channel.pending_recovery_messages.clear();
                const auto now_ns = steady_now_ns();
                for(const auto & routed : pending) {
                    apply_delivered(channel, routed, now_ns);
                }
            }
            catch(const std::exception & error) {
                channel.phase = RecoveryPhase::Failed;
                channel.last_event = error.what();
            }
        }

        void on_timer()
        {
            const auto now_ns = steady_now_ns();
            deliver_ready(a1_, now_ns);
            deliver_ready(a2_, now_ns);
            if(a2_.gap_seen && !a2_.recovered && !a2_.request_in_flight
               && a2_.expected_correlation.empty() && now_ns >= a2_.resync_due_ns
               && a2_.phase != RecoveryPhase::Failed) {
                request_resync(a2_);
            }
            publish_status_markers();
            publish_difference_markers();
        }

        std_msgs::msg::ColorRGBA phase_color(RecoveryPhase phase) const
        {
            std_msgs::msg::ColorRGBA color;
            color.a = 0.95F;
            switch(phase) {
                case RecoveryPhase::Healthy:
                case RecoveryPhase::Recovered:
                    color.r = 0.15F;
                    color.g = 0.85F;
                    color.b = 0.35F;
                    break;
                case RecoveryPhase::RequestingResync:
                case RecoveryPhase::AwaitingKeyframe:
                    color.r = 1.0F;
                    color.g = 0.75F;
                    color.b = 0.10F;
                    break;
                case RecoveryPhase::DeltaDropped:
                case RecoveryPhase::GapHold:
                case RecoveryPhase::Failed:
                    color.r = 0.95F;
                    color.g = 0.15F;
                    color.b = 0.15F;
                    break;
            }
            return color;
        }

        std::string phase_name(RecoveryPhase phase) const
        {
            switch(phase) {
                case RecoveryPhase::Healthy:
                    return "HEALTHY";
                case RecoveryPhase::DeltaDropped:
                    return "DELTA DROPPED";
                case RecoveryPhase::GapHold:
                    return "GAP / MAP FROZEN";
                case RecoveryPhase::RequestingResync:
                    return "RESYNC REQUEST";
                case RecoveryPhase::AwaitingKeyframe:
                    return "WAITING KEYFRAME";
                case RecoveryPhase::Recovered:
                    return "RECOVERED";
                case RecoveryPhase::Failed:
                    return "FAILED";
            }
            return "UNKNOWN";
        }

        visualization_msgs::msg::Marker source_status_marker(
                const SourceChannel & channel,
                int id) const
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = get_clock()->now();
            marker.ns = "c4_cave/status";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = channel.has_position
                                           ? channel.position
                                           : geometry_msgs::msg::Point {};
            marker.pose.position.z += 0.45;
            marker.pose.orientation.w = 1.0;
            marker.scale.z = 0.07;
            marker.color = phase_color(channel.phase);
            marker.text = channel.label + " -> B  " + phase_name(channel.phase)
                          + "\nB r" + std::to_string(channel.accepted_revision)
                          + " / source r" + std::to_string(channel.oracle_revision);
            return marker;
        }

        visualization_msgs::msg::Marker link_marker(
                const SourceChannel & channel,
                int id) const
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = get_clock()->now();
            marker.ns = "c4_cave/links";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.025;
            marker.color = phase_color(channel.phase);
            marker.color.a = 0.65F;
            if(channel.has_position && has_b_position_) {
                constexpr int kDashCount = 16;
                const auto interpolate = [&](double ratio) {
                    geometry_msgs::msg::Point point;
                    point.x = channel.position.x
                              + (b_position_.x - channel.position.x) * ratio;
                    point.y = channel.position.y
                              + (b_position_.y - channel.position.y) * ratio;
                    point.z = channel.position.z
                              + (b_position_.z - channel.position.z) * ratio;
                    return point;
                };
                for(int dash = 0; dash < kDashCount; dash += 2) {
                    marker.points.push_back(interpolate(
                            static_cast<double>(dash) / kDashCount));
                    marker.points.push_back(interpolate(
                            static_cast<double>(dash + 1) / kDashCount));
                }
            }
            return marker;
        }

        visualization_msgs::msg::Marker source_vehicle_marker(
                const SourceChannel & channel,
                int id,
                float red,
                float green,
                float blue) const
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = get_clock()->now();
            marker.ns = "c4_cave/vehicles";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = channel.has_position
                                           ? channel.position
                                           : geometry_msgs::msg::Point {};
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.34;
            marker.scale.y = 0.24;
            marker.scale.z = 0.14;
            marker.color.r = red;
            marker.color.g = green;
            marker.color.b = blue;
            marker.color.a = 1.0F;
            return marker;
        }

        visualization_msgs::msg::Marker b_vehicle_marker() const
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = get_clock()->now();
            marker.ns = "c4_cave/vehicles";
            marker.id = 103;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = has_b_position_
                                           ? b_position_
                                           : geometry_msgs::msg::Point {};
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.38;
            marker.scale.y = 0.28;
            marker.scale.z = 0.16;
            marker.color.r = 0.15F;
            marker.color.g = 0.55F;
            marker.color.b = 1.0F;
            marker.color.a = 1.0F;
            return marker;
        }

        void publish_status_markers()
        {
            visualization_msgs::msg::MarkerArray markers;
            markers.markers.push_back(
                    source_vehicle_marker(a1_, 101, 0.10F, 0.95F, 0.45F));
            markers.markers.push_back(
                    source_vehicle_marker(a2_, 102, 1.0F, 0.25F, 0.35F));
            markers.markers.push_back(b_vehicle_marker());
            markers.markers.push_back(link_marker(a1_, 1));
            markers.markers.push_back(link_marker(a2_, 2));
            markers.markers.push_back(source_status_marker(a1_, 11));
            markers.markers.push_back(source_status_marker(a2_, 12));

            visualization_msgs::msg::Marker b_status;
            b_status.header.frame_id = frame_id_;
            b_status.header.stamp = get_clock()->now();
            b_status.ns = "c4_cave/status";
            b_status.id = 20;
            b_status.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            b_status.action = visualization_msgs::msg::Marker::ADD;
            b_status.pose.position = has_b_position_
                                             ? b_position_
                                             : geometry_msgs::msg::Point {};
            b_status.pose.position.z += 0.35;
            b_status.pose.orientation.w = 1.0;
            b_status.scale.z = 0.08;
            b_status.color.r = 0.35F;
            b_status.color.g = 0.75F;
            b_status.color.b = 1.0F;
            b_status.color.a = 0.95F;
            b_status.text = "B  RECEIVER / RELAY CAPABILITY  (NO MAP MERGE)";
            markers.markers.push_back(std::move(b_status));
            marker_publisher_->publish(markers);
        }

        void append_difference_markers(
                const SourceChannel & channel,
                int source_id,
                visualization_msgs::msg::MarkerArray & output) const
        {
            visualization_msgs::msg::Marker missing;
            visualization_msgs::msg::Marker receiver_only;
            for(auto * marker : {&missing, &receiver_only}) {
                marker->header.frame_id = frame_id_;
                marker->header.stamp = get_clock()->now();
                marker->type = visualization_msgs::msg::Marker::POINTS;
                marker->action = visualization_msgs::msg::Marker::ADD;
                marker->pose.orientation.w = 1.0;
                marker->scale.x = 0.12;
                marker->scale.y = 0.12;
            }
            missing.ns = "c4_cave/oracle_missing_at_b/" + channel.label;
            missing.id = source_id;
            missing.color.r = 1.0F;
            missing.color.g = 0.08F;
            missing.color.b = 0.08F;
            missing.color.a = 0.85F;
            receiver_only.ns = "c4_cave/oracle_receiver_only/" + channel.label;
            receiver_only.id = source_id + 100;
            receiver_only.color.r = 1.0F;
            receiver_only.color.g = 0.72F;
            receiver_only.color.b = 0.05F;
            receiver_only.color.a = 0.85F;

            const auto & source_map = channel.source_oracle.reconstructed_map();
            const auto & receiver_map =
                    channel.ingress.map_applier().reconstructed_map();
            if(source_map.has_value() && receiver_map.has_value()
               && source_map->geometry == receiver_map->geometry) {
                constexpr std::size_t kMaxDifferencePoints = 50'000U;
                auto source_cursor = source_map->cells.cursor();
                auto receiver_cursor = receiver_map->cells.cursor();
                while(!source_cursor.done() || !receiver_cursor.done()) {
                    if(missing.points.size() + receiver_only.points.size()
                       >= kMaxDifferencePoints) {
                        break;
                    }
                    if(receiver_cursor.done()
                       || (!source_cursor.done()
                           && source_cursor.value().index
                                      < receiver_cursor.value().index)) {
                        missing.points.push_back(
                                voxel_center(*source_map, source_cursor.value()));
                        source_cursor.advance();
                        continue;
                    }
                    if(source_cursor.done()
                       || receiver_cursor.value().index
                                  < source_cursor.value().index) {
                        receiver_only.points.push_back(voxel_center(
                                *receiver_map, receiver_cursor.value()));
                        receiver_cursor.advance();
                        continue;
                    }
                    if(source_cursor.value().state
                       != receiver_cursor.value().state) {
                        missing.points.push_back(
                                voxel_center(*source_map, source_cursor.value()));
                        receiver_only.points.push_back(voxel_center(
                                *receiver_map, receiver_cursor.value()));
                    }
                    source_cursor.advance();
                    receiver_cursor.advance();
                }
            }
            output.markers.push_back(std::move(missing));
            output.markers.push_back(std::move(receiver_only));
        }

        void publish_difference_markers()
        {
            visualization_msgs::msg::MarkerArray markers;
            append_difference_markers(a1_, 1, markers);
            append_difference_markers(a2_, 2, markers);
            difference_publisher_->publish(markers);
        }

        SourceChannel a1_;
        SourceChannel a2_;
        std::string frame_id_;
        std::uint64_t resync_hold_ns_ = 4'000'000'000U;
        Perception::SessionID requester_session_ {0U, 0U};
        geometry_msgs::msg::Point b_position_;
        bool has_b_position_ = false;
        rclcpp::Publisher<swarm_data_interfaces::msg::DeliveryAck>::SharedPtr ack_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                difference_publisher_;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
                odom_subscriptions_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr b_odom_subscription_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}// namespace SwarmDataPlane::Test

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        rclcpp::spin(std::make_shared<SwarmDataPlane::Test::C4CaveReceiverFixture>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("c4_cave_receiver_fixture"), "%s", error.what());
        result = 1;
    }
    rclcpp::shutdown();
    return result;
}
