#include "swarm_data_plane/PureRelay.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace SwarmDataPlane {

    namespace {

        const RouteDescriptor * find_route(
                const TopologySnapshot & topology,
                const std::string & route_id) noexcept
        {
            const auto found = std::find_if(
                    topology.routes.begin(), topology.routes.end(),
                    [&](const RouteDescriptor & route) {
                        return route.route_id == route_id;
                    });
            return found == topology.routes.end() ? nullptr : &*found;
        }

        const LinkDescriptor * find_link(
                const TopologySnapshot & topology,
                const RouteHop & hop) noexcept
        {
            const auto found = std::find_if(
                    topology.links.begin(), topology.links.end(),
                    [&](const LinkDescriptor & link) {
                        return link.link_id == hop.link_id
                               && link.link_epoch == hop.link_epoch;
                    });
            return found == topology.links.end() ? nullptr : &*found;
        }

        bool healthy_link(const LinkDescriptor * link) noexcept
        {
            return link != nullptr
                   && (link->health == LinkHealth::Up
                       || link->health == LinkHealth::Degraded);
        }

        void saturating_increment(std::uint64_t & value) noexcept
        {
            if(value != std::numeric_limits<std::uint64_t>::max()) {
                ++value;
            }
        }

    }// namespace

    PureRelay::PureRelay(
            VehicleIdentity identity,
            std::string route_id,
            std::uint16_t incoming_hop,
            bool terminal_output,
            DataPlaneLimits limits)
            : identity_(std::move(identity))
            , route_id_(std::move(route_id))
            , incoming_hop_(incoming_hop)
            , terminal_output_(terminal_output)
            , limits_(std::move(limits))
    {
        if(identity_.fleet_id.empty() || identity_.vehicle_id.empty()
           || identity_.session.boot_time_ns == 0U
           || !PerceptionMapUpdate::CanonicalCodec::validate_string(
                   route_id_, 128U, "relay route identity", false)
           || limits_.max_ttl_hops == 0U
           || limits_.max_validity_budget_ns == 0U) {
            throw std::invalid_argument("pure relay identity, route, or limits are invalid");
        }
    }

    PureRelayResult PureRelay::forward(
            const RuntimeSnapshotCache & snapshots,
            const RoutedMapUpdate & message,
            std::uint64_t local_elapsed_ns)
    {
        const auto reject = [&](PureRelayStatus status, std::string diagnostic,
                                WorkAdmissionStatus admission =
                                        WorkAdmissionStatus::NoAssignment) {
            count(status);
            return PureRelayResult {
                    status, admission, std::nullopt, std::move(diagnostic)};
        };

        if(snapshots.identity() != identity_ || !snapshots.aligned()
           || snapshots.topology() == nullptr) {
            return reject(
                    PureRelayStatus::RejectedSnapshot,
                    "relay runtime snapshots are missing, misaligned, or for another identity");
        }
        const auto admission = snapshots.service_admission(ServiceKind::Relay);
        if(!admission) {
            return reject(
                    PureRelayStatus::RejectedRole,
                    std::string("RelayService cannot accept work: ")
                            + work_admission_status_name(admission.status),
                    admission.status);
        }

        const auto & topology = *snapshots.topology();
        const auto * route = find_route(topology, route_id_);
        if(route == nullptr || route->graph != LogicalGraphKind::Map
           || route->topology_epoch != topology.topology_epoch
           || route->hops.empty()
           || (terminal_output_
                       ? static_cast<std::size_t>(incoming_hop_) + 1U
                                 != route->hops.size()
                       : static_cast<std::size_t>(incoming_hop_) + 1U
                                 >= route->hops.size())
           || message.route.route_epoch != route->route_epoch
           || message.route.hop_count != incoming_hop_
           || message.route.ttl_hops != route->ttl_hops
           || message.validity_budget_ns != route->validity_budget_ns) {
            return reject(
                    PureRelayStatus::RejectedRoute,
                    "message does not match the committed relay route binding");
        }

        VehicleIdentity current = route->source;
        for(std::size_t index = 0U; index <= incoming_hop_; ++index) {
            const auto * link = find_link(topology, route->hops[index]);
            if(!healthy_link(link) || link->source != current) {
                return reject(
                        PureRelayStatus::RejectedRoute,
                        "committed route is disconnected before the relay hop");
            }
            current = link->target;
        }
        if(current != identity_) {
            return reject(
                    PureRelayStatus::RejectedRoute,
                    "relay identity is not the target of the incoming route hop");
        }
        if(!terminal_output_) {
            const auto * outgoing = find_link(
                    topology,
                    route->hops[static_cast<std::size_t>(incoming_hop_) + 1U]);
            if(!healthy_link(outgoing) || outgoing->source != identity_) {
                return reject(
                        PureRelayStatus::RejectedRoute,
                        "relay identity is not the source of the outgoing route hop");
            }
        }

        auto forwarded = forward_routed_map_update(message, local_elapsed_ns, limits_);
        if(!forwarded) {
            const auto status = forwarded.status == EnvelopeValidationStatus::RejectedExpired
                                        ? PureRelayStatus::RejectedExpired
                                        : forwarded.status
                                                          == EnvelopeValidationStatus::RejectedRoute
                                                  ? PureRelayStatus::RejectedRoute
                                                  : PureRelayStatus::RejectedEnvelope;
            return reject(status, std::move(forwarded.diagnostic));
        }
        count(PureRelayStatus::Forwarded);
        return {
                PureRelayStatus::Forwarded,
                WorkAdmissionStatus::Allowed,
                std::move(forwarded.message),
                {}};
    }

    const VehicleIdentity & PureRelay::identity() const noexcept
    {
        return identity_;
    }

    const std::string & PureRelay::route_id() const noexcept
    {
        return route_id_;
    }

    std::uint16_t PureRelay::incoming_hop() const noexcept
    {
        return incoming_hop_;
    }

    bool PureRelay::terminal_output() const noexcept
    {
        return terminal_output_;
    }

    const PureRelayCounters & PureRelay::counters() const noexcept
    {
        return counters_;
    }

    void PureRelay::count(PureRelayStatus status) noexcept
    {
        switch(status) {
            case PureRelayStatus::Forwarded:
                saturating_increment(counters_.forwarded);
                return;
            case PureRelayStatus::RejectedSnapshot:
                saturating_increment(counters_.rejected_snapshot);
                return;
            case PureRelayStatus::RejectedRole:
                saturating_increment(counters_.rejected_role);
                return;
            case PureRelayStatus::RejectedRoute:
                saturating_increment(counters_.rejected_route);
                return;
            case PureRelayStatus::RejectedEnvelope:
                saturating_increment(counters_.rejected_envelope);
                return;
            case PureRelayStatus::RejectedExpired:
                saturating_increment(counters_.rejected_expired);
                return;
        }
    }

}// namespace SwarmDataPlane
