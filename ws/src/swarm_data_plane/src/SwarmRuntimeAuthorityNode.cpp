#include "swarm_data_plane/RuntimeAuthority.hpp"
#include "swarm_data_plane/ros/RoleConversions.hpp"
#include "swarm_data_plane/ros/TopologyConversions.hpp"
#include "swarm_data_plane/ros/VehicleIdentityConversions.hpp"

#include "swarm_data_interfaces/action/apply_role_transition.hpp"
#include "swarm_data_interfaces/msg/capability_evidence.hpp"
#include "swarm_data_interfaces/msg/role_snapshot.hpp"
#include "swarm_data_interfaces/msg/role_transition_ack.hpp"
#include "swarm_data_interfaces/msg/role_transition_descriptor.hpp"
#include "swarm_data_interfaces/msg/topology_snapshot.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
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

        void require(const TopologyResult & result, const std::string & operation)
        {
            if(!result) {
                throw std::runtime_error(operation + ": " + result.diagnostic);
            }
        }

        void require(const RoleResult & result, const std::string & operation)
        {
            if(!result) {
                throw std::runtime_error(operation + ": " + result.diagnostic);
            }
        }

        VehicleRegistration registration(const VehicleIdentity & identity)
        {
            SensorDescriptorIdentity sensor;
            sensor.sensor_id = "lidar";
            sensor.descriptor_hash[0] = 0xA5U;
            return {
                    identity,
                    1U,
                    {{"runtime", {identity.session.boot_time_ns + 1U, 2U}}},
                    {sensor}};
        }

        void make_ready(TopologyState & state, const VehicleIdentity & identity)
        {
            require(state.register_vehicle(registration(identity)), "register member");
            require(
                    state.transition_member(identity, MembershipState::Resyncing),
                    "begin member resync");
            require(
                    state.set_prerequisites(identity, {true, true, true, true}),
                    "set member prerequisites");
            require(
                    state.transition_member(identity, MembershipState::Ready),
                    "mark member ready");
        }

        LinkDescriptor link(
                std::string id,
                const VehicleIdentity & source,
                const VehicleIdentity & target)
        {
            return {std::move(id), 1U, source, target, LinkHealth::Up,
                    1'000U, 1'000'000U, 100U};
        }

        CapabilityRegistration capability_registration(
                const VehicleIdentity & identity,
                CapabilityKind capability)
        {
            return {identity, 1U, {capability}};
        }

        CapabilityEvidence evidence(
                const VehicleIdentity & identity,
                CapabilityKind capability)
        {
            CapabilityEvidence value;
            value.identity = identity;
            value.evidence_revision = 1U;
            value.effective_capabilities = {capability};
            value.vehicle_health = VehicleHealth::Healthy;
            value.resource_health = ResourceHealth::Healthy;
            if(capability == CapabilityKind::RelayForwarding) {
                value.service_health = {
                        {ServiceKind::Relay, ResourceHealth::Healthy}};
            } else if(capability == CapabilityKind::MapAggregation) {
                value.service_health = {
                        {ServiceKind::Aggregation, ResourceHealth::Healthy}};
            }
            return value;
        }

        ServiceBudget relay_budget()
        {
            return {4'096U, 8'192U, 1'000'000U, 0U, 2U};
        }

        ServiceBudget aggregation_budget()
        {
            return {8'192U, 64U * 1024U * 1024U, 10'000'000U, 8U, 1U};
        }

        RoleAssignment explorer_assignment(const VehicleIdentity & identity)
        {
            return {identity, 1U, PrimaryRole::Explorer,
                    RoleLifecycle::Active, {}};
        }

        RoleAssignment relay_assignment(const VehicleIdentity & identity)
        {
            return {
                    identity,
                    1U,
                    PrimaryRole::Relay,
                    RoleLifecycle::Active,
                    {{ServiceKind::Relay, ServiceLifecycle::Active,
                      relay_budget()}}};
        }

        RoleAssignment aggregator_assignment(const VehicleIdentity & identity)
        {
            return {
                    identity,
                    1U,
                    PrimaryRole::EdgeAggregator,
                    RoleLifecycle::Active,
                    {{ServiceKind::Aggregation, ServiceLifecycle::Active,
                      aggregation_budget()}}};
        }

        struct RuntimeIdentities {
            VehicleIdentity explorer_0;
            VehicleIdentity explorer_1;
            VehicleIdentity relay_0;
            VehicleIdentity relay_1;
            VehicleIdentity edge_aggregator;
        };

        RuntimeIdentities load_identities(rclcpp::Node & node)
        {
            const auto fleet_id = node.declare_parameter<std::string>(
                    "fleet_id", "fleet-a");
            const auto identity = [&](
                                          const std::string & prefix,
                                          const std::string & vehicle_id,
                                          std::int64_t boot_time_ns) {
                const auto configured_vehicle = node.declare_parameter<std::string>(
                        prefix + ".vehicle_id", vehicle_id);
                const auto configured_boot = node.declare_parameter<std::int64_t>(
                        prefix + ".session_boot_time_ns", boot_time_ns);
                const auto configured_suffix = node.declare_parameter<std::int64_t>(
                        prefix + ".session_random_suffix", 1);
                if(fleet_id.empty() || configured_vehicle.empty()
                   || configured_boot <= 0 || configured_suffix <= 0) {
                    throw std::invalid_argument(
                            prefix + " identity parameters must be positive and non-empty");
                }
                return VehicleIdentity {
                        fleet_id,
                        configured_vehicle,
                        {static_cast<std::uint64_t>(configured_boot),
                         static_cast<std::uint32_t>(configured_suffix)}};
            };
            return {
                    identity("explorer_0", "explorer-0", 100),
                    identity("explorer_1", "explorer-1", 110),
                    identity("relay_0", "relay-0", 200),
                    identity("relay_1", "relay-1", 210),
                    identity("edge_aggregator", "edge-aggregator", 300)};
        }

        RuntimeAuthority make_runtime_authority(
                const RuntimeIdentities & ids,
                std::uint64_t heartbeat_timeout_ns,
                std::uint64_t initial_time_ns,
                bool enable_edge_aggregator)
        {
            std::vector<WhitelistEntry> whitelist {
                    {ids.explorer_0.vehicle_id, 8U, 8U},
                    {ids.explorer_1.vehicle_id, 8U, 8U},
                    {ids.relay_0.vehicle_id, 8U, 8U},
                    {ids.relay_1.vehicle_id, 8U, 8U}};
            if(enable_edge_aggregator) {
                whitelist.push_back({ids.edge_aggregator.vehicle_id, 8U, 8U});
            }
            TopologyState topology(ids.explorer_0.fleet_id, std::move(whitelist));
            make_ready(topology, ids.explorer_0);
            make_ready(topology, ids.explorer_1);
            make_ready(topology, ids.relay_0);
            make_ready(topology, ids.relay_1);
            if(enable_edge_aggregator) {
                make_ready(topology, ids.edge_aggregator);
            }

            const auto map_e0_r0 = link("map-e0-r0", ids.explorer_0, ids.relay_0);
            const auto map_e0_r1 = link("map-e0-r1", ids.explorer_0, ids.relay_1);
            const auto map_e1_r0 = link("map-e1-r0", ids.explorer_1, ids.relay_0);
            const auto map_e1_r1 = link("map-e1-r1", ids.explorer_1, ids.relay_1);
            const auto map_r0_edge = link(
                    "map-r0-edge", ids.relay_0, ids.edge_aggregator);
            const auto map_r1_edge = link(
                    "map-r1-edge", ids.relay_1, ids.edge_aggregator);
            const auto control_e0_r0 = link(
                    "control-e0-r0", ids.explorer_0, ids.relay_0);
            const auto control_r0_e1 = link(
                    "control-r0-e1", ids.relay_0, ids.explorer_1);
            const auto control_e1_r1 = link(
                    "control-e1-r1", ids.explorer_1, ids.relay_1);
            const auto control_r1_e0 = link(
                    "control-r1-e0", ids.relay_1, ids.explorer_0);

            TopologyCandidate topology_candidate;
            topology_candidate.base_topology_epoch = topology.snapshot().topology_epoch;
            topology_candidate.links = {
                    map_e0_r0, map_e0_r1, map_e1_r0, map_e1_r1,
                    control_e0_r0, control_r0_e1, control_e1_r1,
                    control_r1_e0};
            if(enable_edge_aggregator) {
                topology_candidate.links.push_back(map_r0_edge);
                topology_candidate.links.push_back(map_r1_edge);
            }
            for(const auto & value : topology_candidate.links) {
                topology_candidate.edges.push_back(
                        {LogicalGraphKind::Communication,
                         value.link_id, value.link_epoch,
                         value.source, value.target});
            }
            for(const auto & value : {
                        map_e0_r0, map_e0_r1, map_e1_r0, map_e1_r1}) {
                topology_candidate.edges.push_back(
                        {LogicalGraphKind::Map,
                         value.link_id, value.link_epoch,
                         value.source, value.target});
            }
            if(enable_edge_aggregator) {
                for(const auto & value : {map_r0_edge, map_r1_edge}) {
                    topology_candidate.edges.push_back(
                            {LogicalGraphKind::Map,
                             value.link_id, value.link_epoch,
                             value.source, value.target});
                }
            }
            for(const auto & value : {
                        control_e0_r0, control_r0_e1,
                        control_e1_r1, control_r1_e0}) {
                topology_candidate.edges.push_back(
                        {LogicalGraphKind::Control,
                         value.link_id, value.link_epoch,
                         value.source, value.target});
            }
            topology_candidate.routes.push_back(
                    {"route-map-0", LogicalGraphKind::Map,
                     ids.explorer_0,
                     enable_edge_aggregator ? ids.edge_aggregator : ids.relay_0,
                     topology.snapshot().topology_epoch, 1U,
                     static_cast<std::uint16_t>(enable_edge_aggregator ? 3U : 2U),
                     1'000'000'000U,
                     enable_edge_aggregator
                             ? std::vector<RouteHop> {
                                       {map_e0_r0.link_id, map_e0_r0.link_epoch},
                                       {map_r0_edge.link_id, map_r0_edge.link_epoch}}
                             : std::vector<RouteHop> {
                                       {map_e0_r0.link_id, map_e0_r0.link_epoch}}});
            topology_candidate.routes.push_back(
                    {"route-map-1", LogicalGraphKind::Map,
                     ids.explorer_1,
                     enable_edge_aggregator ? ids.edge_aggregator : ids.relay_0,
                     topology.snapshot().topology_epoch, 1U,
                     static_cast<std::uint16_t>(enable_edge_aggregator ? 3U : 2U),
                     1'000'000'000U,
                     enable_edge_aggregator
                             ? std::vector<RouteHop> {
                                       {map_e1_r0.link_id, map_e1_r0.link_epoch},
                                       {map_r0_edge.link_id, map_r0_edge.link_epoch}}
                             : std::vector<RouteHop> {
                                       {map_e1_r0.link_id, map_e1_r0.link_epoch}}});
            topology_candidate.routes.push_back(
                    {"route-control-0", LogicalGraphKind::Control,
                     ids.explorer_0, ids.explorer_1,
                     topology.snapshot().topology_epoch, 1U, 3U,
                     1'000'000'000U,
                     {{control_e0_r0.link_id, control_e0_r0.link_epoch},
                      {control_r0_e1.link_id, control_r0_e1.link_epoch}}});
            topology_candidate.routes.push_back(
                    {"route-control-1", LogicalGraphKind::Control,
                     ids.explorer_1, ids.explorer_0,
                     topology.snapshot().topology_epoch, 1U, 3U,
                     1'000'000'000U,
                     {{control_e1_r1.link_id, control_e1_r1.link_epoch},
                      {control_r1_e0.link_id, control_r1_e0.link_epoch}}});
            require(
                    topology.replace_topology(std::move(topology_candidate)),
                    "commit initial topology");

            RoleState roles(topology.snapshot());
            for(const auto & value : {
                        std::pair {ids.explorer_0, CapabilityKind::Exploration},
                        std::pair {ids.explorer_1, CapabilityKind::Exploration},
                        std::pair {ids.relay_0, CapabilityKind::RelayForwarding},
                        std::pair {ids.relay_1, CapabilityKind::RelayForwarding}}) {
                require(
                        roles.register_capabilities(
                                capability_registration(value.first, value.second),
                                topology.snapshot()),
                        "register runtime capability");
                require(
                        roles.update_capability_evidence(
                                evidence(value.first, value.second),
                        topology.snapshot()),
                        "seed runtime capability evidence");
            }
            if(enable_edge_aggregator) {
                require(
                        roles.register_capabilities(
                                capability_registration(
                                        ids.edge_aggregator,
                                        CapabilityKind::MapAggregation),
                                topology.snapshot()),
                        "register EdgeAggregator capability");
                require(
                        roles.update_capability_evidence(
                                evidence(
                                        ids.edge_aggregator,
                                        CapabilityKind::MapAggregation),
                                topology.snapshot()),
                        "seed EdgeAggregator capability evidence");
            }
            RoleCandidate role_candidate;
            role_candidate.base_role_epoch = roles.snapshot().role_epoch;
            role_candidate.topology_epoch = topology.snapshot().topology_epoch;
            role_candidate.assignments = {
                    explorer_assignment(ids.explorer_0),
                    explorer_assignment(ids.explorer_1),
                    relay_assignment(ids.relay_0)};
            if(enable_edge_aggregator) {
                role_candidate.assignments.push_back(
                        aggregator_assignment(ids.edge_aggregator));
            }
            require(
                    roles.begin_transition(
                            "initial", std::move(role_candidate), topology.snapshot()),
                    "prepare initial roles");
            require(
                    roles.commit_transition("initial", topology.snapshot()),
                    "commit initial roles");

            RuntimeAuthorityConfig config;
            config.active_relay = ids.relay_0;
            config.standby_relay = ids.relay_1;
            config.route_failovers.push_back(
                    {"route-map-0",
                     {"route-map-0", LogicalGraphKind::Map,
                      ids.explorer_0,
                      enable_edge_aggregator ? ids.edge_aggregator : ids.relay_1,
                      topology.snapshot().topology_epoch, 2U,
                      static_cast<std::uint16_t>(enable_edge_aggregator ? 3U : 2U),
                      1'000'000'000U,
                      enable_edge_aggregator
                              ? std::vector<RouteHop> {
                                        {map_e0_r1.link_id, map_e0_r1.link_epoch},
                                        {map_r1_edge.link_id, map_r1_edge.link_epoch}}
                              : std::vector<RouteHop> {
                                        {map_e0_r1.link_id, map_e0_r1.link_epoch}}}});
            config.route_failovers.push_back(
                    {"route-map-1",
                     {"route-map-1", LogicalGraphKind::Map,
                      ids.explorer_1,
                      enable_edge_aggregator ? ids.edge_aggregator : ids.relay_1,
                      topology.snapshot().topology_epoch, 2U,
                      static_cast<std::uint16_t>(enable_edge_aggregator ? 3U : 2U),
                      1'000'000'000U,
                      enable_edge_aggregator
                              ? std::vector<RouteHop> {
                                        {map_e1_r1.link_id, map_e1_r1.link_epoch},
                                        {map_r1_edge.link_id, map_r1_edge.link_epoch}}
                              : std::vector<RouteHop> {
                                        {map_e1_r1.link_id, map_e1_r1.link_epoch}}}});
            config.failover_assignment = relay_assignment(ids.relay_1);
            config.heartbeat_timeout_ns = heartbeat_timeout_ns;
            config.initial_time_ns = initial_time_ns;
            return RuntimeAuthority(
                    std::move(topology), std::move(roles), std::move(config));
        }

    }// namespace

    class SwarmRuntimeAuthorityNode final : public rclcpp::Node
    {
    public:
        using ApplyRoleTransition =
                swarm_data_interfaces::action::ApplyRoleTransition;
        using TransitionGoalHandle =
                rclcpp_action::ClientGoalHandle<ApplyRoleTransition>;

        SwarmRuntimeAuthorityNode() : Node("swarm_runtime_authority")
        {
            const auto identities = load_identities(*this);
            transition_target_ = identities.explorer_0;
            const auto heartbeat_timeout_ms = declare_parameter<std::int64_t>(
                    "heartbeat_timeout_ms", 1500);
            const auto tick_rate_hz = declare_parameter<double>(
                    "tick_rate_hz", 10.0);
            const auto enable_edge_aggregator = declare_parameter<bool>(
                    "enable_edge_aggregator", false);
            if(heartbeat_timeout_ms <= 0 || tick_rate_hz <= 0.0) {
                throw std::invalid_argument(
                        "heartbeat_timeout_ms and tick_rate_hz must be positive");
            }

            const auto topology_topic = declare_parameter<std::string>(
                    "topology_topic", "/swarm/runtime/topology");
            const auto role_topic = declare_parameter<std::string>(
                    "role_topic", "/swarm/runtime/roles");
            const auto transition_topic = declare_parameter<std::string>(
                    "transition_topic", "/swarm/runtime/transition");
            const auto evidence_topic = declare_parameter<std::string>(
                    "evidence_topic", "/swarm/runtime/evidence");
            const auto transition_action = declare_parameter<std::string>(
                    "explorer_transition_action",
                    "/drone_0/apply_role_transition");
            const auto transition_service = declare_parameter<std::string>(
                    "explorer_transition_service",
                    "/swarm/runtime/quiesce_explorer_0");

            const auto initial_time_ns = steady_now_ns();
            runtime_ = std::make_unique<RuntimeAuthority>(make_runtime_authority(
                    identities,
                    static_cast<std::uint64_t>(heartbeat_timeout_ms) * 1'000'000U,
                    initial_time_ns,
                    enable_edge_aggregator));

            const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
            topology_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::TopologySnapshot>(
                    topology_topic, state_qos);
            role_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::RoleSnapshot>(role_topic, state_qos);
            transition_publisher_ = create_publisher<
                    swarm_data_interfaces::msg::RoleTransitionDescriptor>(
                    transition_topic, state_qos);
            evidence_subscription_ = create_subscription<
                    swarm_data_interfaces::msg::CapabilityEvidence>(
                    evidence_topic,
                    rclcpp::QoS(32).reliable().durability_volatile(),
                    [this](
                            const swarm_data_interfaces::msg::CapabilityEvidence::
                                    ConstSharedPtr message) {
                        on_evidence(*message);
                    });
            transition_client_ = rclcpp_action::create_client<ApplyRoleTransition>(
                    this, transition_action);
            transition_service_ = create_service<std_srvs::srv::Trigger>(
                    transition_service,
                    [this](
                            const std_srvs::srv::Trigger::Request::SharedPtr,
                            std_srvs::srv::Trigger::Response::SharedPtr response) {
                        begin_explorer_transition(*response);
                    });
            const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / tick_rate_hz));
            timer_ = create_wall_timer(period, [this]() { on_tick(); });
            publish_snapshots();
        }

    private:
        void begin_explorer_transition(
                std_srvs::srv::Trigger::Response & response)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(transition_in_flight_) {
                response.success = false;
                response.message = "another Explorer transition is in flight";
                return;
            }
            if(!transition_client_->action_server_is_ready()) {
                response.success = false;
                response.message = "Explorer transition action is unavailable";
                return;
            }

            RoleCandidate candidate;
            candidate.base_role_epoch = runtime_->roles().role_epoch;
            candidate.topology_epoch = runtime_->topology().topology_epoch;
            candidate.assignments = runtime_->roles().assignments;
            const auto assignment = std::find_if(
                    candidate.assignments.begin(), candidate.assignments.end(),
                    [&](const RoleAssignment & value) {
                        return value.identity == transition_target_;
                    });
            if(assignment == candidate.assignments.end()
               || assignment->primary_role != PrimaryRole::Explorer
               || assignment->lifecycle != RoleLifecycle::Active) {
                response.success = false;
                response.message = "Explorer is not in an Active role";
                return;
            }
            assignment->lifecycle = RoleLifecycle::Draining;
            const auto transition_id =
                    "explorer-quiesce-"
                    + std::to_string(candidate.base_role_epoch + 1U);
            const auto prepared = runtime_->begin_role_transition(
                    transition_id, std::move(candidate));
            if(!prepared || runtime_->active_transition() == nullptr) {
                response.success = false;
                response.message = prepared.diagnostic.empty()
                        ? "Explorer role transition produced no active candidate"
                        : prepared.diagnostic;
                return;
            }

            ApplyRoleTransition::Goal goal;
            goal.protocol_version = ApplyRoleTransition::Goal::CURRENT_PROTOCOL_VERSION;
            std::string diagnostic;
            if(!Ros::encode_role_transition(
                       *runtime_->active_transition(), goal.transition, diagnostic)) {
                runtime_->rollback_role_transition(transition_id);
                response.success = false;
                response.message = diagnostic;
                return;
            }
            Ros::Detail::encode_vehicle_identity(transition_target_, goal.target);
            transition_in_flight_ = true;
            transition_in_flight_id_ = transition_id;
            publish_snapshots_locked();

            rclcpp_action::Client<ApplyRoleTransition>::SendGoalOptions options;
            options.goal_response_callback =
                    [this](const TransitionGoalHandle::SharedPtr & goal_handle) {
                        if(goal_handle) {
                            return;
                        }
                        std::lock_guard<std::mutex> lock(mutex_);
                        rollback_transition_locked(
                                "Explorer rejected the role transition goal");
                    };
            options.feedback_callback =
                    [this](
                            TransitionGoalHandle::SharedPtr,
                            const std::shared_ptr<const ApplyRoleTransition::Feedback>
                                    feedback) {
                        on_transition_feedback(*feedback);
                    };
            options.result_callback =
                    [this](const TransitionGoalHandle::WrappedResult & result) {
                        on_transition_result(result);
                    };
            transition_client_->async_send_goal(goal, options);
            response.success = true;
            response.message = "Explorer role transition started";
        }

        void on_transition_feedback(
                const ApplyRoleTransition::Feedback & feedback)
        {
            if(feedback.stage != ApplyRoleTransition::Feedback::STAGE_QUIESCED) {
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            if(!transition_in_flight_) {
                return;
            }
            const auto acknowledged = runtime_->acknowledge_role_transition(
                    transition_in_flight_id_, transition_target_,
                    RoleTransitionAckKind::Quiesced);
            if(!acknowledged) {
                rollback_transition_locked(
                        "authority rejected the Quiesced acknowledgement: "
                        + acknowledged.diagnostic);
                return;
            }
            publish_snapshots_locked();
        }

        void on_transition_result(
                const TransitionGoalHandle::WrappedResult & result)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if(!transition_in_flight_) {
                return;
            }
            if(result.code != rclcpp_action::ResultCode::SUCCEEDED
               || result.result == nullptr || !result.result->success) {
                rollback_transition_locked(
                        result.result == nullptr || result.result->diagnostic.empty()
                                ? "Explorer role transition action failed"
                                : result.result->diagnostic);
                return;
            }
            for(const auto & acknowledgement : result.result->acknowledgements) {
                const auto identity = Ros::Detail::decode_vehicle_identity(
                        acknowledgement.identity);
                RoleTransitionAckKind kind;
                if(acknowledgement.kind
                   == swarm_data_interfaces::msg::RoleTransitionAck::ACK_QUIESCED) {
                    kind = RoleTransitionAckKind::Quiesced;
                } else if(acknowledgement.kind
                          == swarm_data_interfaces::msg::RoleTransitionAck::
                                  ACK_HANDOFF_READY) {
                    kind = RoleTransitionAckKind::HandoffReady;
                } else {
                    rollback_transition_locked(
                            "Explorer returned an unknown transition ack kind");
                    return;
                }
                const auto accepted = runtime_->acknowledge_role_transition(
                        transition_in_flight_id_, identity, kind);
                if(!accepted) {
                    rollback_transition_locked(
                            "authority rejected an Explorer transition ack: "
                            + accepted.diagnostic);
                    return;
                }
            }
            publish_snapshots_locked();
            const auto committed = runtime_->commit_role_transition(
                    transition_in_flight_id_);
            if(!committed) {
                rollback_transition_locked(
                        "authority could not commit the Explorer role: "
                        + committed.diagnostic);
                return;
            }
            if(const auto * terminal = runtime_->last_transition()) {
                publish_transition_locked(*terminal);
            }
            transition_in_flight_ = false;
            transition_in_flight_id_.clear();
            publish_snapshots_locked();
            RCLCPP_INFO(
                    get_logger(),
                    "Explorer role transition committed after runtime acknowledgements");
        }

        void rollback_transition_locked(const std::string & reason)
        {
            if(!transition_in_flight_) {
                return;
            }
            const auto rolled_back = runtime_->rollback_role_transition(
                    transition_in_flight_id_);
            if(rolled_back) {
                if(const auto * terminal = runtime_->last_transition()) {
                    publish_transition_locked(*terminal);
                }
                publish_snapshots_locked();
            }
            transition_in_flight_ = false;
            transition_in_flight_id_.clear();
            RCLCPP_WARN(get_logger(), "Explorer role transition rolled back: %s",
                        reason.c_str());
        }

        void on_evidence(
                const swarm_data_interfaces::msg::CapabilityEvidence & message)
        {
            auto decoded = Ros::decode_capability_evidence(message);
            if(!decoded.success || !decoded.evidence.has_value()) {
                RCLCPP_WARN(
                        get_logger(), "rejected capability evidence: %s",
                        decoded.diagnostic.c_str());
                return;
            }
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = runtime_->observe_evidence(
                    std::move(*decoded.evidence), steady_now_ns());
            if(!result) {
                RCLCPP_WARN(
                        get_logger(), "runtime evidence rejected: %s",
                        result.diagnostic.c_str());
            }
        }

        void on_tick()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto result = runtime_->tick(steady_now_ns());
            if(result.status == RuntimeAuthorityStatus::FailoverCommitted) {
                publish_snapshots_locked();
                RCLCPP_WARN(
                        get_logger(),
                        "Relay heartbeat timeout committed topology/role failover");
            } else if(!result) {
                RCLCPP_ERROR(
                        get_logger(), "runtime failover rejected: %s",
                        result.diagnostic.c_str());
            }
        }

        void publish_snapshots()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            publish_snapshots_locked();
        }

        void publish_snapshots_locked()
        {
            swarm_data_interfaces::msg::TopologySnapshot topology_message;
            swarm_data_interfaces::msg::RoleSnapshot role_message;
            std::string diagnostic;
            if(!Ros::encode_topology_snapshot(
                       runtime_->topology(), topology_message, diagnostic)
               || !Ros::encode_role_snapshot(
                       runtime_->roles(), role_message, diagnostic)) {
                throw std::runtime_error(
                        "failed to encode runtime snapshot: " + diagnostic);
            }
            topology_publisher_->publish(topology_message);
            role_publisher_->publish(role_message);
            if(const auto * transition = runtime_->active_transition()) {
                publish_transition_locked(*transition);
            }
        }

        void publish_transition_locked(const RoleTransition & transition)
        {
            swarm_data_interfaces::msg::RoleTransitionDescriptor transition_message;
            std::string diagnostic;
            if(!Ros::encode_role_transition(
                       transition, transition_message, diagnostic)) {
                throw std::runtime_error(
                        "failed to encode runtime transition: " + diagnostic);
            }
            transition_publisher_->publish(transition_message);
        }

        std::mutex mutex_;
        std::unique_ptr<RuntimeAuthority> runtime_;
        VehicleIdentity transition_target_;
        bool transition_in_flight_ = false;
        std::string transition_in_flight_id_;
        rclcpp::Publisher<swarm_data_interfaces::msg::TopologySnapshot>::SharedPtr
                topology_publisher_;
        rclcpp::Publisher<swarm_data_interfaces::msg::RoleSnapshot>::SharedPtr
                role_publisher_;
        rclcpp::Publisher<
                swarm_data_interfaces::msg::RoleTransitionDescriptor>::SharedPtr
                transition_publisher_;
        rclcpp::Subscription<
                swarm_data_interfaces::msg::CapabilityEvidence>::SharedPtr
                evidence_subscription_;
        rclcpp_action::Client<ApplyRoleTransition>::SharedPtr transition_client_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr transition_service_;
        rclcpp::TimerBase::SharedPtr timer_;
    };

}// namespace SwarmDataPlane

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(
                std::make_shared<SwarmDataPlane::SwarmRuntimeAuthorityNode>());
    }
    catch(const std::exception & error) {
        RCLCPP_FATAL(
                rclcpp::get_logger("swarm_runtime_authority"),
                "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
