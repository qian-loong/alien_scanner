#include "swarm_data_plane/ros/TopologyConversions.hpp"

#include "swarm_data_plane/ros/VehicleIdentityConversions.hpp"

#include <algorithm>
#include <utility>

namespace SwarmDataPlane::Ros {

    namespace {

        constexpr std::size_t kRosMaxMembers = 64U;
        constexpr std::size_t kRosMaxLinks = 256U;
        constexpr std::size_t kRosMaxEdges = 512U;
        constexpr std::size_t kRosMaxRoutes = 256U;
        constexpr std::size_t kRosMaxRouteHops = 64U;
        constexpr std::size_t kRosMaxComponentsPerMember = 32U;
        constexpr std::size_t kRosMaxSensorsPerMember = 32U;
        constexpr std::size_t kRosMaxIdentityBytes = 128U;

        TopologyLimits bounded_ros_limits(const TopologyLimits & limits)
        {
            TopologyLimits bounded = limits;
            bounded.max_identity_bytes = std::min(
                    bounded.max_identity_bytes, kRosMaxIdentityBytes);
            bounded.max_members = std::min(bounded.max_members, kRosMaxMembers);
            bounded.max_links = std::min(bounded.max_links, kRosMaxLinks);
            bounded.max_edges = std::min(bounded.max_edges, kRosMaxEdges);
            bounded.max_routes = std::min(bounded.max_routes, kRosMaxRoutes);
            bounded.max_route_hops = std::min(bounded.max_route_hops, kRosMaxRouteHops);
            bounded.max_components_per_member = std::min(
                    bounded.max_components_per_member,
                    kRosMaxComponentsPerMember);
            bounded.max_sensors_per_member = std::min(
                    bounded.max_sensors_per_member,
                    kRosMaxSensorsPerMember);
            return bounded;
        }

        void encode_member(
                const MemberRecord & member,
                swarm_data_interfaces::msg::MemberRecord & message)
        {
            Detail::encode_vehicle_identity(
                    member.registration.identity, message.identity);
            message.registration_generation = member.registration.registration_generation;
            message.components.clear();
            message.components.reserve(member.registration.components.size());
            for(const auto & component : member.registration.components) {
                swarm_data_interfaces::msg::ComponentRegistration converted;
                converted.component_id = component.component_id;
                converted.session_boot_time_ns = component.session.boot_time_ns;
                converted.session_random_suffix = component.session.random_suffix;
                message.components.push_back(std::move(converted));
            }
            message.sensors.clear();
            message.sensors.reserve(member.registration.sensors.size());
            for(const auto & sensor : member.registration.sensors) {
                swarm_data_interfaces::msg::SensorDescriptorIdentity converted;
                converted.sensor_id = sensor.sensor_id;
                std::copy(
                        sensor.descriptor_hash.begin(), sensor.descriptor_hash.end(),
                        converted.descriptor_hash.begin());
                message.sensors.push_back(std::move(converted));
            }
            message.state = static_cast<std::uint8_t>(member.state);
            message.availability = static_cast<std::uint8_t>(member.availability);
            message.link_ready = member.prerequisites.link_ready;
            message.clock_ready = member.prerequisites.clock_ready;
            message.alignment_ready = member.prerequisites.alignment_ready;
            message.map_ready = member.prerequisites.map_ready;
        }

        MemberRecord decode_member(
                const swarm_data_interfaces::msg::MemberRecord & message)
        {
            MemberRecord member;
            member.registration.identity =
                    Detail::decode_vehicle_identity(message.identity);
            member.registration.registration_generation = message.registration_generation;
            member.registration.components.reserve(message.components.size());
            for(const auto & component : message.components) {
                member.registration.components.push_back({
                        component.component_id,
                        {component.session_boot_time_ns, component.session_random_suffix}});
            }
            member.registration.sensors.reserve(message.sensors.size());
            for(const auto & sensor : message.sensors) {
                SensorDescriptorIdentity converted;
                converted.sensor_id = sensor.sensor_id;
                std::copy(
                        sensor.descriptor_hash.begin(), sensor.descriptor_hash.end(),
                        converted.descriptor_hash.begin());
                member.registration.sensors.push_back(std::move(converted));
            }
            member.state = static_cast<MembershipState>(message.state);
            member.availability = static_cast<MemberAvailability>(message.availability);
            member.prerequisites = {
                    message.link_ready,
                    message.clock_ready,
                    message.alignment_ready,
                    message.map_ready};
            return member;
        }

        void encode_link(
                const LinkDescriptor & link,
                swarm_data_interfaces::msg::LinkDescriptor & message)
        {
            message.link_id = link.link_id;
            message.link_epoch = link.link_epoch;
            Detail::encode_vehicle_identity(link.source, message.source);
            Detail::encode_vehicle_identity(link.target, message.target);
            message.health = static_cast<std::uint8_t>(link.health);
            message.latency_budget_ns = link.latency_budget_ns;
            message.bandwidth_budget_bps = link.bandwidth_budget_bps;
            message.loss_budget_ppm = link.loss_budget_ppm;
        }

        LinkDescriptor decode_link(
                const swarm_data_interfaces::msg::LinkDescriptor & message)
        {
            return {
                    message.link_id,
                    message.link_epoch,
                    Detail::decode_vehicle_identity(message.source),
                    Detail::decode_vehicle_identity(message.target),
                    static_cast<LinkHealth>(message.health),
                    message.latency_budget_ns,
                    message.bandwidth_budget_bps,
                    message.loss_budget_ppm};
        }

        void encode_edge(
                const GraphEdge & edge,
                swarm_data_interfaces::msg::GraphEdge & message)
        {
            message.graph = static_cast<std::uint8_t>(edge.graph);
            message.link_id = edge.link_id;
            message.link_epoch = edge.link_epoch;
            Detail::encode_vehicle_identity(edge.source, message.source);
            Detail::encode_vehicle_identity(edge.target, message.target);
        }

        GraphEdge decode_edge(const swarm_data_interfaces::msg::GraphEdge & message)
        {
            return {
                    static_cast<LogicalGraphKind>(message.graph),
                    message.link_id,
                    message.link_epoch,
                    Detail::decode_vehicle_identity(message.source),
                    Detail::decode_vehicle_identity(message.target)};
        }

        void encode_route(
                const RouteDescriptor & route,
                swarm_data_interfaces::msg::RouteDescriptor & message)
        {
            message.route_id = route.route_id;
            message.graph = static_cast<std::uint8_t>(route.graph);
            Detail::encode_vehicle_identity(route.source, message.source);
            Detail::encode_vehicle_identity(route.target, message.target);
            message.topology_epoch = route.topology_epoch;
            message.route_epoch = route.route_epoch;
            message.ttl_hops = route.ttl_hops;
            message.validity_budget_ns = route.validity_budget_ns;
            message.hops.clear();
            message.hops.reserve(route.hops.size());
            for(const auto & hop : route.hops) {
                swarm_data_interfaces::msg::RouteHop converted;
                converted.link_id = hop.link_id;
                converted.link_epoch = hop.link_epoch;
                message.hops.push_back(std::move(converted));
            }
        }

        RouteDescriptor decode_route(
                const swarm_data_interfaces::msg::RouteDescriptor & message)
        {
            RouteDescriptor route;
            route.route_id = message.route_id;
            route.graph = static_cast<LogicalGraphKind>(message.graph);
            route.source = Detail::decode_vehicle_identity(message.source);
            route.target = Detail::decode_vehicle_identity(message.target);
            route.topology_epoch = message.topology_epoch;
            route.route_epoch = message.route_epoch;
            route.ttl_hops = message.ttl_hops;
            route.validity_budget_ns = message.validity_budget_ns;
            route.hops.reserve(message.hops.size());
            for(const auto & hop : message.hops) {
                route.hops.push_back({hop.link_id, hop.link_epoch});
            }
            return route;
        }

        bool collection_sizes_within_limits(
                const swarm_data_interfaces::msg::TopologySnapshot & message,
                const TopologyLimits & limits,
                std::string & diagnostic)
        {
            const auto bounded = bounded_ros_limits(limits);
            const auto valid_text_size = [&](const std::string & value) {
                return value.size() <= bounded.max_identity_bytes;
            };
            if(message.members.size() > bounded.max_members
               || message.links.size() > bounded.max_links
               || message.edges.size() > bounded.max_edges
               || message.routes.size() > bounded.max_routes) {
                diagnostic = "topology snapshot collection exceeds configured limits";
                return false;
            }
            if(!valid_text_size(message.fleet_id)) {
                diagnostic = "topology fleet identity exceeds configured limits";
                return false;
            }
            for(const auto & member : message.members) {
                if(member.components.size() > bounded.max_components_per_member
                   || member.sensors.size() > bounded.max_sensors_per_member) {
                    diagnostic = "member inventory exceeds configured limits";
                    return false;
                }
                if(!valid_text_size(member.identity.fleet_id)
                   || !valid_text_size(member.identity.vehicle_id)) {
                    diagnostic = "member identity exceeds configured limits";
                    return false;
                }
                for(const auto & component : member.components) {
                    if(!valid_text_size(component.component_id)) {
                        diagnostic = "component identity exceeds configured limits";
                        return false;
                    }
                }
                for(const auto & sensor : member.sensors) {
                    if(!valid_text_size(sensor.sensor_id)) {
                        diagnostic = "sensor identity exceeds configured limits";
                        return false;
                    }
                }
            }
            for(const auto & link : message.links) {
                if(!valid_text_size(link.link_id)
                   || !valid_text_size(link.source.fleet_id)
                   || !valid_text_size(link.source.vehicle_id)
                   || !valid_text_size(link.target.fleet_id)
                   || !valid_text_size(link.target.vehicle_id)) {
                    diagnostic = "link identity exceeds configured limits";
                    return false;
                }
            }
            for(const auto & edge : message.edges) {
                if(!valid_text_size(edge.link_id)
                   || !valid_text_size(edge.source.fleet_id)
                   || !valid_text_size(edge.source.vehicle_id)
                   || !valid_text_size(edge.target.fleet_id)
                   || !valid_text_size(edge.target.vehicle_id)) {
                    diagnostic = "graph edge identity exceeds configured limits";
                    return false;
                }
            }
            for(const auto & route : message.routes) {
                if(route.hops.size() > bounded.max_route_hops) {
                    diagnostic = "route hop collection exceeds configured limits";
                    return false;
                }
                if(!valid_text_size(route.route_id)
                   || !valid_text_size(route.source.fleet_id)
                   || !valid_text_size(route.source.vehicle_id)
                   || !valid_text_size(route.target.fleet_id)
                   || !valid_text_size(route.target.vehicle_id)) {
                    diagnostic = "route identity exceeds configured limits";
                    return false;
                }
                for(const auto & hop : route.hops) {
                    if(!valid_text_size(hop.link_id)) {
                        diagnostic = "route hop identity exceeds configured limits";
                        return false;
                    }
                }
            }
            return true;
        }

    }// namespace

    bool encode_topology_snapshot(
            const TopologySnapshot & snapshot,
            swarm_data_interfaces::msg::TopologySnapshot & message,
            std::string & diagnostic,
            const TopologyLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        const auto validation = validate_topology_snapshot(snapshot, bounded);
        if(!validation) {
            diagnostic = validation.diagnostic;
            return false;
        }

        message = swarm_data_interfaces::msg::TopologySnapshot{};
        message.protocol_version = snapshot.protocol_version;
        message.fleet_id = snapshot.fleet_id;
        message.topology_epoch = snapshot.topology_epoch;
        message.members.reserve(snapshot.members.size());
        for(const auto & member : snapshot.members) {
            swarm_data_interfaces::msg::MemberRecord converted;
            encode_member(member, converted);
            message.members.push_back(std::move(converted));
        }
        message.links.reserve(snapshot.links.size());
        for(const auto & link : snapshot.links) {
            swarm_data_interfaces::msg::LinkDescriptor converted;
            encode_link(link, converted);
            message.links.push_back(std::move(converted));
        }
        message.edges.reserve(snapshot.edges.size());
        for(const auto & edge : snapshot.edges) {
            swarm_data_interfaces::msg::GraphEdge converted;
            encode_edge(edge, converted);
            message.edges.push_back(std::move(converted));
        }
        message.routes.reserve(snapshot.routes.size());
        for(const auto & route : snapshot.routes) {
            swarm_data_interfaces::msg::RouteDescriptor converted;
            encode_route(route, converted);
            message.routes.push_back(std::move(converted));
        }
        diagnostic.clear();
        return true;
    }

    DecodeTopologySnapshotResult decode_topology_snapshot(
            const swarm_data_interfaces::msg::TopologySnapshot & message,
            const TopologyLimits & limits)
    {
        const auto bounded = bounded_ros_limits(limits);
        std::string diagnostic;
        if(!collection_sizes_within_limits(message, limits, diagnostic)) {
            return {false, std::nullopt, std::move(diagnostic)};
        }

        TopologySnapshot decoded;
        decoded.protocol_version = message.protocol_version;
        decoded.fleet_id = message.fleet_id;
        decoded.topology_epoch = message.topology_epoch;
        decoded.members.reserve(message.members.size());
        for(const auto & member : message.members) {
            decoded.members.push_back(decode_member(member));
        }
        decoded.links.reserve(message.links.size());
        for(const auto & link : message.links) {
            decoded.links.push_back(decode_link(link));
        }
        decoded.edges.reserve(message.edges.size());
        for(const auto & edge : message.edges) {
            decoded.edges.push_back(decode_edge(edge));
        }
        decoded.routes.reserve(message.routes.size());
        for(const auto & route : message.routes) {
            decoded.routes.push_back(decode_route(route));
        }

        const auto validation = validate_topology_snapshot(decoded, bounded);
        if(!validation) {
            return {false, std::nullopt, validation.diagnostic};
        }
        return {true, std::move(decoded), {}};
    }

}// namespace SwarmDataPlane::Ros
