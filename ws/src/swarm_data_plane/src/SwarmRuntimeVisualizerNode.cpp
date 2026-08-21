#include "swarm_data_plane/RoleTypes.hpp"
#include "swarm_data_plane/TopologyTypes.hpp"
#include "swarm_data_plane/ros/RoleConversions.hpp"
#include "swarm_data_plane/ros/TopologyConversions.hpp"

#include "swarm_data_interfaces/msg/link_diagnostic.hpp"
#include "swarm_data_interfaces/msg/aggregate_map_update.hpp"
#include "swarm_data_interfaces/msg/role_snapshot.hpp"
#include "swarm_data_interfaces/msg/role_transition_descriptor.hpp"
#include "swarm_data_interfaces/msg/topology_snapshot.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace SwarmDataPlane {

    namespace {

        geometry_msgs::msg::Point point(double x, double y, double z)
        {
            geometry_msgs::msg::Point value;
            value.x = x;
            value.y = y;
            value.z = z;
            return value;
        }

        const char * membership_name(MembershipState state) noexcept
        {
            switch(state) {
                case MembershipState::Absent:
                    return "Absent";
                case MembershipState::Joining:
                    return "Joining";
                case MembershipState::Resyncing:
                    return "Resyncing";
                case MembershipState::Ready:
                    return "Ready";
                case MembershipState::Draining:
                    return "Draining";
                case MembershipState::Lost:
                    return "Lost";
                case MembershipState::Quarantined:
                    return "Quarantined";
            }
            return "Unknown";
        }

        const char * role_name(PrimaryRole role) noexcept
        {
            switch(role) {
                case PrimaryRole::Explorer:
                    return "Explorer";
                case PrimaryRole::Relay:
                    return "Relay";
                case PrimaryRole::EdgeAggregator:
                    return "EdgeAggregator";
                case PrimaryRole::Reserve:
                    return "Reserve";
            }
            return "Unknown";
        }

        const char * role_lifecycle_name(RoleLifecycle lifecycle) noexcept
        {
            switch(lifecycle) {
                case RoleLifecycle::Active:
                    return "Active";
                case RoleLifecycle::Draining:
                    return "Draining";
            }
            return "Unknown";
        }

        const char * transition_name(RoleTransitionState state) noexcept
        {
            switch(state) {
                case RoleTransitionState::Prepared:
                    return "Prepared";
                case RoleTransitionState::Quiescing:
                    return "Quiescing";
                case RoleTransitionState::HandoffReady:
                    return "HandoffReady";
                case RoleTransitionState::Committed:
                    return "Committed";
                case RoleTransitionState::RolledBack:
                    return "RolledBack";
            }
            return "Unknown";
        }

        const RoleAssignment * find_assignment(
                const RoleSnapshot & roles,
                const VehicleIdentity & identity) noexcept
        {
            const auto found = std::find_if(
                    roles.assignments.begin(), roles.assignments.end(),
                    [&](const RoleAssignment & assignment) {
                        return assignment.identity == identity;
                    });
            return found == roles.assignments.end() ? nullptr : &*found;
        }

    }// namespace

    class SwarmRuntimeVisualizerNode final : public rclcpp::Node
    {
    public:
        SwarmRuntimeVisualizerNode() : Node("swarm_runtime_visualizer")
        {
            frame_id_ = declare_parameter<std::string>("frame_id", "map");
            const auto topology_topic = declare_parameter<std::string>(
                    "topology_topic", "/swarm/runtime/topology");
            const auto role_topic = declare_parameter<std::string>(
                    "role_topic", "/swarm/runtime/roles");
            const auto transition_topic = declare_parameter<std::string>(
                    "transition_topic", "/swarm/runtime/transition");
            const auto diagnostic_topic = declare_parameter<std::string>(
                    "relay_diagnostic_topic",
                    "/swarm/runtime/relay_diagnostics");
            const auto aggregate_topic = declare_parameter<std::string>(
                    "aggregate_topic", "/c5d/aggregate/output");
            const auto aggregate_diagnostic_topic = declare_parameter<std::string>(
                    "aggregate_diagnostic_topic",
                    "/swarm/runtime/aggregate_diagnostics");
            const auto marker_topic = declare_parameter<std::string>(
                    "marker_topic", "/swarm/runtime/markers");
            const auto explorer_ids = declare_parameter<std::vector<std::string>>(
                    "explorer_vehicle_ids", {"explorer-0", "explorer-1"});
            const auto odom_topics = declare_parameter<std::vector<std::string>>(
                    "explorer_odom_topics",
                    {"/explorer_0/odom", "/explorer_1/odom"});
            if(explorer_ids.size() != odom_topics.size()) {
                throw std::invalid_argument(
                        "explorer_vehicle_ids and explorer_odom_topics must align");
            }

            publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
                    marker_topic, rclcpp::QoS(1).reliable().transient_local());
            const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
            topology_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::TopologySnapshot>(
                    topology_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::TopologySnapshot::
                                   ConstSharedPtr message) {
                        on_topology(*message);
                    });
            role_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoleSnapshot>(
                    role_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::RoleSnapshot::
                                   ConstSharedPtr message) {
                        on_roles(*message);
                    });
            transition_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::RoleTransitionDescriptor>(
                    transition_topic, state_qos,
                    [this](const swarm_data_interfaces::msg::
                                   RoleTransitionDescriptor::ConstSharedPtr message) {
                        on_transition(*message);
                    });
            relay_diagnostic_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    diagnostic_topic,
                    rclcpp::QoS(32).reliable().durability_volatile(),
                    [this](const swarm_data_interfaces::msg::LinkDiagnostic::
                                   ConstSharedPtr message) {
                        on_relay_diagnostic(*message);
                    });
            aggregate_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::AggregateMapUpdate>(
                    aggregate_topic,
                    rclcpp::QoS(8).reliable(),
                    [this](const swarm_data_interfaces::msg::AggregateMapUpdate::ConstSharedPtr message) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        aggregate_revision_ = message->manifest.aggregate_revision;
                        aggregate_contributors_.clear();
                        for(const auto & contributor : message->manifest.contributors) {
                            aggregate_contributors_[contributor.vehicle_id] = contributor.active;
                        }
                        aggregate_available_ = true;
                    });
            aggregate_diagnostic_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::LinkDiagnostic>(
                    aggregate_diagnostic_topic,
                    rclcpp::QoS(32).reliable().durability_volatile(),
                    [this](const swarm_data_interfaces::msg::LinkDiagnostic::ConstSharedPtr message) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        if(message->event == swarm_data_interfaces::msg::LinkDiagnostic::EVENT_REJECTED) {
                            aggregate_degraded_ = true;
                            if(message->fault_reason == "aggregation_resync") {
                                aggregate_resync_required_ = true;
                                aggregate_resync_visible_until_ =
                                        std::chrono::steady_clock::now()
                                        + std::chrono::seconds(1);
                            }
                        } else if(message->event == swarm_data_interfaces::msg::LinkDiagnostic::EVENT_DELIVERED) {
                            aggregate_degraded_ = false;
                            aggregate_resync_required_ = false;
                        }
                    });
            odom_subscriptions_.reserve(odom_topics.size());
            for(std::size_t index = 0U; index < odom_topics.size(); ++index) {
                odom_subscriptions_.push_back(
                        create_subscription<nav_msgs::msg::Odometry>(
                                odom_topics[index], rclcpp::QoS(10),
                                [this, vehicle_id = explorer_ids[index]](
                                        const nav_msgs::msg::Odometry::
                                                ConstSharedPtr message) {
                                    std::lock_guard<std::mutex> lock(mutex_);
                                    positions_[vehicle_id] = message->pose.pose.position;
                                }));
            }
            timer_ = create_wall_timer(
                    std::chrono::milliseconds(200), [this]() { publish(); });
        }

    private:
        void on_topology(
                const swarm_data_interfaces::msg::TopologySnapshot & message)
        {
            auto decoded = Ros::decode_topology_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            std::uint64_t max_map_route_epoch = 0U;
            for(const auto & route : decoded.snapshot->routes) {
                if(route.graph == LogicalGraphKind::Map) {
                    max_map_route_epoch = std::max(
                            max_map_route_epoch, route.route_epoch);
                }
            }
            if(max_map_route_epoch_.has_value()
               && max_map_route_epoch > *max_map_route_epoch_) {
                aggregate_resync_required_ = true;
                aggregate_resync_visible_until_ = std::chrono::steady_clock::now()
                                                  + std::chrono::seconds(1);
            }
            if(max_map_route_epoch != 0U) {
                max_map_route_epoch_ = max_map_route_epoch;
            }
            topology_ = std::move(*decoded.snapshot);
        }

        void on_roles(const swarm_data_interfaces::msg::RoleSnapshot & message)
        {
            auto decoded = Ros::decode_role_snapshot(message);
            if(!decoded.success || !decoded.snapshot.has_value()) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            roles_ = std::move(*decoded.snapshot);
            if(transition_.has_value()
               && transition_->state != RoleTransitionState::Committed
               && transition_->state != RoleTransitionState::RolledBack
               && roles_->role_epoch > transition_->base_role_epoch) {
                transition_.reset();
            }
        }

        void on_transition(
                const swarm_data_interfaces::msg::RoleTransitionDescriptor & message)
        {
            auto decoded = Ros::decode_role_transition(message);
            if(!decoded.success || !decoded.transition.has_value()) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            transition_ = std::move(*decoded.transition);
        }

        void on_relay_diagnostic(
                const swarm_data_interfaces::msg::LinkDiagnostic & message)
        {
            const auto separator = message.endpoint_id.find(':');
            const auto vehicle_id = message.endpoint_id.substr(0U, separator);
            if(vehicle_id.empty()) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if(message.event
               == swarm_data_interfaces::msg::LinkDiagnostic::EVENT_LINK_DOWN) {
                relay_faulted_[vehicle_id] = true;
            } else if(message.event
                      == swarm_data_interfaces::msg::LinkDiagnostic::EVENT_LINK_UP) {
                relay_faulted_[vehicle_id] = false;
            }
        }

        geometry_msgs::msg::Point position_for(
                const VehicleIdentity & identity,
                std::size_t fallback_index) const
        {
            const auto found = positions_.find(identity.vehicle_id);
            if(found != positions_.end()) {
                return found->second;
            }
            if(identity.vehicle_id == "relay-0") {
                return point(3.0, 1.8, 1.0);
            }
            if(identity.vehicle_id == "relay-1") {
                return point(3.0, -1.8, 1.0);
            }
            return point(0.0, 2.0 - 4.0 * static_cast<double>(fallback_index), 1.0);
        }

        void publish()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(!topology_.has_value() || !roles_.has_value()) {
                return;
            }
            visualization_msgs::msg::MarkerArray output;
            visualization_msgs::msg::Marker clear;
            clear.action = visualization_msgs::msg::Marker::DELETEALL;
            output.markers.push_back(clear);
            const auto stamp = now();

            std::map<std::string, geometry_msgs::msg::Point> member_positions;
            std::size_t fallback_index = 0U;
            for(const auto & member : topology_->members) {
                const auto position = position_for(
                        member.registration.identity, fallback_index++);
                member_positions[member.registration.identity.vehicle_id] = position;

                visualization_msgs::msg::Marker body;
                body.header.frame_id = frame_id_;
                body.header.stamp = stamp;
                body.ns = "runtime_members";
                body.id = static_cast<int>(output.markers.size());
                body.type = visualization_msgs::msg::Marker::SPHERE;
                body.action = visualization_msgs::msg::Marker::ADD;
                body.pose.position = position;
                body.pose.orientation.w = 1.0;
                body.scale.x = body.scale.y = body.scale.z = 0.45;
                body.color.a = 1.0F;
                const bool relay_fault = relay_faulted_[
                        member.registration.identity.vehicle_id];
                if(member.state == MembershipState::Lost || relay_fault) {
                    body.color.r = 1.0F;
                    body.color.g = 0.15F;
                } else if(member.state == MembershipState::Draining) {
                    body.color.r = 1.0F;
                    body.color.g = 0.65F;
                } else {
                    body.color.g = 0.75F;
                    body.color.b = 1.0F;
                }
                output.markers.push_back(body);

                visualization_msgs::msg::Marker label;
                label.header = body.header;
                label.ns = "runtime_labels";
                label.id = static_cast<int>(output.markers.size());
                label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                label.action = visualization_msgs::msg::Marker::ADD;
                label.pose.position = position;
                label.pose.position.z += 0.55;
                label.pose.orientation.w = 1.0;
                label.scale.z = 0.22;
                label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
                std::ostringstream text;
                text << member.registration.identity.vehicle_id << '\n'
                     << membership_name(member.state);
                if(const auto * assignment = find_assignment(
                           *roles_, member.registration.identity)) {
                    text << " | " << role_name(assignment->primary_role)
                         << ' ' << role_lifecycle_name(assignment->lifecycle)
                         << " | role e" << roles_->role_epoch;
                    if(!assignment->services.empty()) {
                        text << " + ";
                        for(std::size_t index = 0U; index < assignment->services.size(); ++index) {
                            if(index != 0U) {
                                text << ',';
                            }
                            text << (assignment->services[index].service == ServiceKind::Aggregation
                                             ? "AggregationService"
                                             : "RelayService");
                        }
                    }
                } else {
                    text << " | no active role";
                }
                if(transition_.has_value()
                   && std::find(
                              transition_->changed_members.begin(),
                              transition_->changed_members.end(),
                              member.registration.identity)
                              != transition_->changed_members.end()) {
                    text << '\n' << transition_name(transition_->state);
                    const auto handoff_ready = std::find_if(
                            transition_->acknowledgements.begin(),
                            transition_->acknowledgements.end(),
                            [&](const RoleTransitionAck & ack) {
                                return ack.identity == member.registration.identity
                                       && ack.kind
                                                  == RoleTransitionAckKind::
                                                          HandoffReady;
                            });
                    const auto quiesced = std::find_if(
                            transition_->acknowledgements.begin(),
                            transition_->acknowledgements.end(),
                            [&](const RoleTransitionAck & ack) {
                                return ack.identity == member.registration.identity
                                       && ack.kind
                                                  == RoleTransitionAckKind::Quiesced;
                            });
                    if(handoff_ready != transition_->acknowledgements.end()) {
                        text << " (ack HandoffReady)";
                    } else if(quiesced != transition_->acknowledgements.end()) {
                        text << " (ack Quiesced)";
                    }
                }
                label.text = text.str();
                output.markers.push_back(label);
            }

            std::vector<std::string> active_link_ids;
            for(const auto & route : topology_->routes) {
                if(route.graph != LogicalGraphKind::Map) {
                    continue;
                }
                for(const auto & hop : route.hops) {
                    active_link_ids.push_back(hop.link_id);
                }
            }
            visualization_msgs::msg::Marker active_routes;
            active_routes.header.frame_id = frame_id_;
            active_routes.header.stamp = stamp;
            active_routes.ns = "active_map_routes";
            active_routes.id = 0;
            active_routes.type = visualization_msgs::msg::Marker::LINE_LIST;
            active_routes.action = visualization_msgs::msg::Marker::ADD;
            active_routes.scale.x = 0.10;
            active_routes.color.g = 1.0F;
            active_routes.color.a = 1.0F;
            visualization_msgs::msg::Marker standby_routes = active_routes;
            standby_routes.ns = "standby_map_edges";
            standby_routes.color.r = 0.55F;
            standby_routes.color.g = 0.55F;
            standby_routes.color.b = 0.55F;
            standby_routes.scale.x = 0.04;
            for(const auto & edge : topology_->edges) {
                if(edge.graph != LogicalGraphKind::Map) {
                    continue;
                }
                const auto source = member_positions.find(edge.source.vehicle_id);
                const auto target = member_positions.find(edge.target.vehicle_id);
                if(source == member_positions.end() || target == member_positions.end()) {
                    continue;
                }
                auto & marker = std::find(
                                       active_link_ids.begin(), active_link_ids.end(),
                                       edge.link_id)
                                               != active_link_ids.end()
                        ? active_routes
                        : standby_routes;
                marker.points.push_back(source->second);
                marker.points.push_back(target->second);
            }
            output.markers.push_back(active_routes);
            output.markers.push_back(standby_routes);

            visualization_msgs::msg::Marker receiver;
            receiver.header.frame_id = frame_id_;
            receiver.header.stamp = stamp;
            receiver.ns = "central_receiver";
            receiver.id = 0;
            receiver.type = visualization_msgs::msg::Marker::CUBE;
            receiver.action = visualization_msgs::msg::Marker::ADD;
            receiver.pose.position = point(6.0, 0.0, 1.0);
            receiver.pose.orientation.w = 1.0;
            receiver.scale.x = 0.7;
            receiver.scale.y = 1.0;
            receiver.scale.z = 0.5;
            receiver.color.r = 0.85F;
            receiver.color.g = 0.35F;
            receiver.color.b = 0.95F;
            receiver.color.a = 1.0F;
            output.markers.push_back(receiver);

            visualization_msgs::msg::Marker aggregate_label;
            aggregate_label.header = receiver.header;
            aggregate_label.ns = "aggregate_status";
            aggregate_label.id = 0;
            aggregate_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            aggregate_label.action = visualization_msgs::msg::Marker::ADD;
            aggregate_label.pose.position = point(6.0, 0.0, 1.8);
            aggregate_label.pose.orientation.w = 1.0;
            aggregate_label.scale.z = 0.24;
            aggregate_label.color.r = aggregate_degraded_ ? 1.0F : 0.8F;
            aggregate_label.color.g = aggregate_degraded_ ? 0.2F : 1.0F;
            aggregate_label.color.b = 0.2F;
            aggregate_label.color.a = 1.0F;
            const bool aggregate_resync_visible = aggregate_resync_required_
                    || std::chrono::steady_clock::now()
                               < aggregate_resync_visible_until_;
            if(aggregate_available_) {
                const auto active_count = static_cast<std::size_t>(std::count_if(
                        aggregate_contributors_.begin(), aggregate_contributors_.end(),
                        [](const auto & contributor) { return contributor.second; }));
                aggregate_label.text = "aggregate r" + std::to_string(aggregate_revision_)
                        + " | active " + std::to_string(active_count) + "/"
                        + std::to_string(aggregate_contributors_.size())
                        + " | degraded " + (aggregate_degraded_ ? "yes" : "no")
                        + " | resync " + (aggregate_resync_visible ? "yes" : "no");
            }
            else {
                aggregate_label.text = "aggregate unavailable";
            }
            output.markers.push_back(aggregate_label);

            visualization_msgs::msg::Marker contributor_label = aggregate_label;
            contributor_label.ns = "aggregate_contributors";
            contributor_label.pose.position.z = 1.45;
            contributor_label.scale.z = 0.18;
            contributor_label.color.r = 1.0F;
            contributor_label.color.g = 1.0F;
            contributor_label.color.b = 1.0F;
            std::ostringstream contributor_text;
            for(const auto & [vehicle_id, active] : aggregate_contributors_) {
                if(contributor_text.tellp() > 0) {
                    contributor_text << '\n';
                }
                contributor_text << vehicle_id << ' ' << (active ? "active" : "inactive");
            }
            contributor_label.text = contributor_text.str();
            output.markers.push_back(contributor_label);

            visualization_msgs::msg::Marker epoch;
            epoch.header = receiver.header;
            epoch.ns = "runtime_epoch";
            epoch.id = 0;
            epoch.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            epoch.action = visualization_msgs::msg::Marker::ADD;
            epoch.pose.position = point(3.0, 0.0, 3.0);
            epoch.pose.orientation.w = 1.0;
            epoch.scale.z = 0.28;
            epoch.color.r = epoch.color.g = epoch.color.b = epoch.color.a = 1.0F;
            epoch.text = "topology e" + std::to_string(topology_->topology_epoch)
                         + " | role e" + std::to_string(roles_->role_epoch);
            output.markers.push_back(epoch);
            publisher_->publish(output);
        }

        std::string frame_id_ {"map"};
        std::mutex mutex_;
        std::optional<TopologySnapshot> topology_;
        std::optional<RoleSnapshot> roles_;
        std::optional<RoleTransition> transition_;
        std::optional<std::uint64_t> max_map_route_epoch_;
        std::map<std::string, geometry_msgs::msg::Point> positions_;
        std::map<std::string, bool> relay_faulted_;
        std::map<std::string, bool> aggregate_contributors_;
        std::uint64_t aggregate_revision_ = 0U;
        bool aggregate_available_ = false;
        bool aggregate_degraded_ = false;
        bool aggregate_resync_required_ = false;
        std::chrono::steady_clock::time_point aggregate_resync_visible_until_ {};
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
                publisher_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::TopologySnapshot>::SharedPtr
                topology_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::RoleSnapshot>::SharedPtr
                role_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::RoleTransitionDescriptor>::SharedPtr
                transition_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                relay_diagnostic_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::AggregateMapUpdate>::SharedPtr
                aggregate_subscription_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::LinkDiagnostic>::SharedPtr
                aggregate_diagnostic_subscription_;
        std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
                odom_subscriptions_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(
                std::make_shared<SwarmDataPlane::SwarmRuntimeVisualizerNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("swarm_runtime_visualizer"),
                "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
