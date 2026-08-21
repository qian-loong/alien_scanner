#include "swarm_data_plane/TopologyTypes.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <set>
#include <tuple>

namespace SwarmDataPlane {

    namespace {

        template<typename Container, typename Compare>
        bool strictly_sorted(const Container & values, Compare compare)
        {
            for(std::size_t index = 1U; index < values.size(); ++index) {
                if(!compare(values[index - 1U], values[index])) {
                    return false;
                }
            }
            return true;
        }

        bool valid_text(const std::string & value, const TopologyLimits & limits)
        {
            return static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                    value, limits.max_identity_bytes, "topology identity", false));
        }

        bool valid_session(const Perception::SessionID & session) noexcept
        {
            return session.boot_time_ns != 0U;
        }

        bool valid_membership_state(MembershipState state) noexcept
        {
            switch(state) {
                case MembershipState::Absent:
                case MembershipState::Joining:
                case MembershipState::Resyncing:
                case MembershipState::Ready:
                case MembershipState::Draining:
                case MembershipState::Lost:
                case MembershipState::Quarantined:
                    return true;
            }
            return false;
        }

        bool valid_availability(MemberAvailability availability) noexcept
        {
            switch(availability) {
                case MemberAvailability::Removed:
                case MemberAvailability::Live:
                case MemberAvailability::Frozen:
                    return true;
            }
            return false;
        }

        bool valid_graph(LogicalGraphKind graph) noexcept
        {
            switch(graph) {
                case LogicalGraphKind::Communication:
                case LogicalGraphKind::Control:
                case LogicalGraphKind::Map:
                    return true;
            }
            return false;
        }

        bool valid_link_health(LinkHealth health) noexcept
        {
            switch(health) {
                case LinkHealth::Unknown:
                case LinkHealth::Up:
                case LinkHealth::Degraded:
                case LinkHealth::Down:
                    return true;
            }
            return false;
        }

        bool nonzero_hash(const PerceptionMapUpdate::Hash256 & hash) noexcept
        {
            return std::any_of(hash.begin(), hash.end(), [](std::uint8_t byte) {
                return byte != 0U;
            });
        }

        const MemberRecord * find_ready_member(
                const TopologySnapshot & snapshot,
                const VehicleIdentity & identity) noexcept
        {
            const auto found = std::find_if(
                    snapshot.members.begin(), snapshot.members.end(),
                    [&](const MemberRecord & member) {
                        return member.registration.identity == identity
                               && member.state == MembershipState::Ready;
                    });
            return found == snapshot.members.end() ? nullptr : &*found;
        }

        const LinkDescriptor * find_link(
                const TopologySnapshot & snapshot,
                const RouteHop & hop) noexcept
        {
            const auto found = std::find_if(
                    snapshot.links.begin(), snapshot.links.end(),
                    [&](const LinkDescriptor & link) {
                        return link.link_id == hop.link_id
                               && link.link_epoch == hop.link_epoch;
                    });
            return found == snapshot.links.end() ? nullptr : &*found;
        }

        bool has_graph_edge(
                const TopologySnapshot & snapshot,
                LogicalGraphKind graph,
                const LinkDescriptor & link) noexcept
        {
            return std::any_of(
                    snapshot.edges.begin(), snapshot.edges.end(),
                    [&](const GraphEdge & edge) {
                        return edge.graph == graph && edge.link_id == link.link_id
                               && edge.link_epoch == link.link_epoch
                               && edge.source == link.source && edge.target == link.target;
                    });
        }

        TopologyResult validate_member(
                const MemberRecord & member,
                const std::string & fleet_id,
                const TopologyLimits & limits)
        {
            const auto & registration = member.registration;
            if(registration.identity.fleet_id != fleet_id
               || !valid_text(registration.identity.fleet_id, limits)
               || !valid_text(registration.identity.vehicle_id, limits)
               || !valid_session(registration.identity.session)
               || registration.registration_generation == 0U) {
                return {TopologyStatus::RejectedInvalid, false,
                        "member identity or registration generation is invalid"};
            }
            if(registration.components.size() > limits.max_components_per_member
               || registration.sensors.size() > limits.max_sensors_per_member) {
                return {TopologyStatus::RejectedResourceLimit, false,
                        "member inventory exceeds configured limits"};
            }
            if(!valid_membership_state(member.state)
               || !valid_availability(member.availability)
               || member.availability != availability_for(member.state)) {
                return {TopologyStatus::RejectedInvalid, false,
                        "member state and availability are inconsistent"};
            }
            if(!strictly_sorted(
                       registration.components,
                       [](const auto & lhs, const auto & rhs) { return lhs < rhs; })
               && registration.components.size() > 1U) {
                return {TopologyStatus::RejectedInvalid, false,
                        "component inventory is not strictly canonical"};
            }
            std::set<std::string> component_ids;
            for(const auto & component : registration.components) {
                if(!valid_text(component.component_id, limits)
                   || !valid_session(component.session)
                   || !component_ids.insert(component.component_id).second) {
                    return {TopologyStatus::RejectedInvalid, false,
                            "component inventory contains an invalid or duplicate identity"};
                }
            }
            if(!strictly_sorted(
                       registration.sensors,
                       [](const auto & lhs, const auto & rhs) { return lhs < rhs; })
               && registration.sensors.size() > 1U) {
                return {TopologyStatus::RejectedInvalid, false,
                        "sensor inventory is not strictly canonical"};
            }
            std::set<std::string> sensor_ids;
            for(const auto & sensor : registration.sensors) {
                if(!valid_text(sensor.sensor_id, limits)
                   || !nonzero_hash(sensor.descriptor_hash)
                   || !sensor_ids.insert(sensor.sensor_id).second) {
                    return {TopologyStatus::RejectedInvalid, false,
                            "sensor inventory contains an invalid or duplicate descriptor"};
                }
            }
            return {TopologyStatus::Applied, false, {}};
        }

    }// namespace

    bool VehicleIdentity::operator==(const VehicleIdentity & other) const noexcept
    {
        return fleet_id == other.fleet_id && vehicle_id == other.vehicle_id
               && session == other.session;
    }

    bool VehicleIdentity::operator!=(const VehicleIdentity & other) const noexcept
    {
        return !(*this == other);
    }

    bool VehicleIdentity::operator<(const VehicleIdentity & other) const noexcept
    {
        if(fleet_id != other.fleet_id) {
            return fleet_id < other.fleet_id;
        }
        if(vehicle_id != other.vehicle_id) {
            return vehicle_id < other.vehicle_id;
        }
        return session < other.session;
    }

    bool ComponentRegistration::operator==(
            const ComponentRegistration & other) const noexcept
    {
        return component_id == other.component_id && session == other.session;
    }

    bool ComponentRegistration::operator<(
            const ComponentRegistration & other) const noexcept
    {
        if(component_id != other.component_id) {
            return component_id < other.component_id;
        }
        return session < other.session;
    }

    bool SensorDescriptorIdentity::operator==(
            const SensorDescriptorIdentity & other) const noexcept
    {
        return sensor_id == other.sensor_id && descriptor_hash == other.descriptor_hash;
    }

    bool SensorDescriptorIdentity::operator<(
            const SensorDescriptorIdentity & other) const noexcept
    {
        if(sensor_id != other.sensor_id) {
            return sensor_id < other.sensor_id;
        }
        return descriptor_hash < other.descriptor_hash;
    }

    bool VehicleRegistration::operator==(
            const VehicleRegistration & other) const noexcept
    {
        return identity == other.identity
               && registration_generation == other.registration_generation
               && components == other.components && sensors == other.sensors;
    }

    bool ResyncPrerequisites::ready() const noexcept
    {
        return link_ready && clock_ready && alignment_ready && map_ready;
    }

    bool ResyncPrerequisites::operator==(
            const ResyncPrerequisites & other) const noexcept
    {
        return link_ready == other.link_ready && clock_ready == other.clock_ready
               && alignment_ready == other.alignment_ready
               && map_ready == other.map_ready;
    }

    bool MemberRecord::operator==(const MemberRecord & other) const noexcept
    {
        return registration == other.registration && state == other.state
               && availability == other.availability
               && prerequisites == other.prerequisites;
    }

    bool MemberRecord::operator<(const MemberRecord & other) const noexcept
    {
        return registration.identity < other.registration.identity;
    }

    bool WhitelistEntry::operator==(const WhitelistEntry & other) const noexcept
    {
        return vehicle_id == other.vehicle_id && max_components == other.max_components
               && max_sensors == other.max_sensors;
    }

    bool WhitelistEntry::operator<(const WhitelistEntry & other) const noexcept
    {
        return vehicle_id < other.vehicle_id;
    }

    bool LinkDescriptor::operator==(const LinkDescriptor & other) const noexcept
    {
        return link_id == other.link_id && link_epoch == other.link_epoch
               && source == other.source && target == other.target
               && health == other.health && latency_budget_ns == other.latency_budget_ns
               && bandwidth_budget_bps == other.bandwidth_budget_bps
               && loss_budget_ppm == other.loss_budget_ppm;
    }

    bool LinkDescriptor::operator<(const LinkDescriptor & other) const noexcept
    {
        return std::tie(link_id, link_epoch, source, target)
               < std::tie(other.link_id, other.link_epoch, other.source, other.target);
    }

    bool GraphEdge::operator==(const GraphEdge & other) const noexcept
    {
        return graph == other.graph && link_id == other.link_id
               && link_epoch == other.link_epoch && source == other.source
               && target == other.target;
    }

    bool GraphEdge::operator<(const GraphEdge & other) const noexcept
    {
        return std::tie(graph, link_id, link_epoch, source, target)
               < std::tie(
                       other.graph, other.link_id, other.link_epoch,
                       other.source, other.target);
    }

    bool RouteHop::operator==(const RouteHop & other) const noexcept
    {
        return link_id == other.link_id && link_epoch == other.link_epoch;
    }

    bool RouteHop::operator<(const RouteHop & other) const noexcept
    {
        return std::tie(link_id, link_epoch) < std::tie(other.link_id, other.link_epoch);
    }

    bool RouteDescriptor::operator==(const RouteDescriptor & other) const noexcept
    {
        return route_id == other.route_id && graph == other.graph
               && source == other.source && target == other.target
               && topology_epoch == other.topology_epoch
               && route_epoch == other.route_epoch && ttl_hops == other.ttl_hops
               && validity_budget_ns == other.validity_budget_ns && hops == other.hops;
    }

    bool RouteDescriptor::operator<(const RouteDescriptor & other) const noexcept
    {
        return std::tie(route_id, graph, route_epoch, source, target)
               < std::tie(
                       other.route_id, other.graph, other.route_epoch,
                       other.source, other.target);
    }

    bool TopologySnapshot::operator==(const TopologySnapshot & other) const noexcept
    {
        return protocol_version == other.protocol_version && fleet_id == other.fleet_id
               && topology_epoch == other.topology_epoch && members == other.members
               && links == other.links && edges == other.edges && routes == other.routes;
    }

    MemberAvailability availability_for(MembershipState state) noexcept
    {
        switch(state) {
            case MembershipState::Ready:
                return MemberAvailability::Live;
            case MembershipState::Draining:
            case MembershipState::Lost:
                return MemberAvailability::Frozen;
            case MembershipState::Absent:
            case MembershipState::Joining:
            case MembershipState::Resyncing:
            case MembershipState::Quarantined:
                return MemberAvailability::Removed;
        }
        return MemberAvailability::Removed;
    }

    TopologyResult validate_topology_snapshot(
            const TopologySnapshot & snapshot,
            const TopologyLimits & limits)
    {
        if(snapshot.protocol_version != kTopologyProtocolVersion
           || !valid_text(snapshot.fleet_id, limits) || snapshot.topology_epoch == 0U) {
            return {TopologyStatus::RejectedInvalid, false,
                    "topology protocol, fleet identity, or epoch is invalid"};
        }
        if(snapshot.members.size() > limits.max_members
           || snapshot.links.size() > limits.max_links
           || snapshot.edges.size() > limits.max_edges
           || snapshot.routes.size() > limits.max_routes) {
            return {TopologyStatus::RejectedResourceLimit, false,
                    "topology snapshot exceeds configured collection limits"};
        }
        if(snapshot.members.size() > 1U
           && !strictly_sorted(
                   snapshot.members,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {TopologyStatus::RejectedInvalid, false,
                    "members are not in strict canonical order"};
        }
        std::set<std::string> vehicle_ids;
        for(const auto & member : snapshot.members) {
            const auto valid = validate_member(member, snapshot.fleet_id, limits);
            if(!valid) {
                return valid;
            }
            if(!vehicle_ids.insert(member.registration.identity.vehicle_id).second) {
                return {TopologyStatus::RejectedConflict, false,
                        "topology contains multiple sessions for one stable vehicle"};
            }
        }
        if(snapshot.links.size() > 1U
           && !strictly_sorted(
                   snapshot.links,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {TopologyStatus::RejectedInvalid, false,
                    "links are not in strict canonical order"};
        }
        std::set<std::string> link_ids;
        for(const auto & link : snapshot.links) {
            if(!valid_text(link.link_id, limits) || link.link_epoch == 0U
               || link.source == link.target || !valid_link_health(link.health)
               || link.loss_budget_ppm > 1'000'000U
               || find_ready_member(snapshot, link.source) == nullptr
               || find_ready_member(snapshot, link.target) == nullptr
               || !link_ids.insert(link.link_id).second) {
                return {TopologyStatus::RejectedInvalid, false,
                        "link identity, endpoints, health, or budget is invalid"};
            }
        }
        if(snapshot.edges.size() > 1U
           && !strictly_sorted(
                   snapshot.edges,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {TopologyStatus::RejectedInvalid, false,
                    "graph edges are not in strict canonical order"};
        }
        for(const auto & edge : snapshot.edges) {
            const RouteHop hop {edge.link_id, edge.link_epoch};
            const auto * link = find_link(snapshot, hop);
            if(!valid_graph(edge.graph) || link == nullptr || edge.source != link->source
               || edge.target != link->target) {
                return {TopologyStatus::RejectedInvalid, false,
                        "graph edge does not reference an exact committed link"};
            }
        }
        if(snapshot.routes.size() > 1U
           && !strictly_sorted(
                   snapshot.routes,
                   [](const auto & lhs, const auto & rhs) { return lhs < rhs; })) {
            return {TopologyStatus::RejectedInvalid, false,
                    "routes are not in strict canonical order"};
        }
        std::set<std::string> route_ids;
        for(const auto & route : snapshot.routes) {
            if(!valid_text(route.route_id, limits)
               || (route.graph != LogicalGraphKind::Control
                   && route.graph != LogicalGraphKind::Map)
               || route.topology_epoch != snapshot.topology_epoch
               || route.route_epoch == 0U || route.hops.empty()
               || route.hops.size() > limits.max_route_hops
               || route.ttl_hops < route.hops.size()
               || route.ttl_hops > limits.max_ttl_hops
               || route.validity_budget_ns == 0U
               || route.validity_budget_ns > limits.max_validity_budget_ns
               || find_ready_member(snapshot, route.source) == nullptr
               || find_ready_member(snapshot, route.target) == nullptr
               || !route_ids.insert(route.route_id).second) {
                return {TopologyStatus::RejectedRoute, false,
                        "route identity, graph, epoch, endpoints, or budget is invalid"};
            }
            VehicleIdentity current = route.source;
            for(const auto & hop : route.hops) {
                const auto * link = find_link(snapshot, hop);
                if(link == nullptr || link->source != current
                   || (link->health != LinkHealth::Up
                       && link->health != LinkHealth::Degraded)
                   || !has_graph_edge(snapshot, route.graph, *link)) {
                    return {TopologyStatus::RejectedRoute, false,
                            "route hop is disconnected, unhealthy, or outside its graph"};
                }
                current = link->target;
            }
            if(current != route.target) {
                return {TopologyStatus::RejectedRoute, false,
                        "route hop chain does not terminate at the declared target"};
            }
        }
        return {TopologyStatus::Applied, false, {}};
    }

}// namespace SwarmDataPlane
