#ifndef SWARM_DATA_PLANE_PURE_RELAY_HPP
#define SWARM_DATA_PLANE_PURE_RELAY_HPP

#include "swarm_data_plane/RoutedMapValidator.hpp"
#include "swarm_data_plane/RuntimeSnapshotCache.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace SwarmDataPlane {

    enum class PureRelayStatus : std::uint8_t
    {
        Forwarded,
        RejectedSnapshot,
        RejectedRole,
        RejectedRoute,
        RejectedEnvelope,
        RejectedExpired
    };

    struct PureRelayResult {
        PureRelayStatus status = PureRelayStatus::RejectedSnapshot;
        WorkAdmissionStatus admission = WorkAdmissionStatus::NoAssignment;
        std::optional<RoutedMapUpdate> message;
        std::string diagnostic;

        explicit operator bool() const noexcept
        {
            return status == PureRelayStatus::Forwarded && message.has_value();
        }
    };

    struct PureRelayCounters {
        std::uint64_t forwarded = 0U;
        std::uint64_t rejected_snapshot = 0U;
        std::uint64_t rejected_role = 0U;
        std::uint64_t rejected_route = 0U;
        std::uint64_t rejected_envelope = 0U;
        std::uint64_t rejected_expired = 0U;
    };

    class PureRelay
    {
    public:
        PureRelay(
                VehicleIdentity identity,
                std::string route_id,
                std::uint16_t incoming_hop,
                bool terminal_output = false,
                DataPlaneLimits limits = {});

        PureRelayResult forward(
                const RuntimeSnapshotCache & snapshots,
                const RoutedMapUpdate & message,
                std::uint64_t local_elapsed_ns);

        const VehicleIdentity & identity() const noexcept;
        const std::string & route_id() const noexcept;
        std::uint16_t incoming_hop() const noexcept;
        bool terminal_output() const noexcept;
        const PureRelayCounters & counters() const noexcept;

    private:
        void count(PureRelayStatus status) noexcept;

        VehicleIdentity identity_;
        std::string route_id_;
        std::uint16_t incoming_hop_ = 0U;
        bool terminal_output_ = false;
        DataPlaneLimits limits_;
        PureRelayCounters counters_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_PURE_RELAY_HPP
