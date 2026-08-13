#ifndef SWARM_DATA_PLANE_ROUTED_RESYNC_HPP
#define SWARM_DATA_PLANE_ROUTED_RESYNC_HPP

#include "perception_map_update/ResyncStateMachine.hpp"
#include "swarm_data_plane/DataPlaneTypes.hpp"

#include <cstdint>
#include <string>

namespace SwarmDataPlane {

    struct RoutedResyncIntent {
        std::uint16_t protocol_version = kProtocolVersion;
        ProducerIdentity target_producer;
        std::uint64_t route_epoch = 0U;
        PerceptionMapUpdate::ResyncRequest request;
    };

    struct RoutedResyncAck {
        std::uint16_t protocol_version = kProtocolVersion;
        bool accepted = false;
        std::string correlation_id;
        ProducerIdentity target_producer;
        PerceptionMapUpdate::SourceIdentity current_source;
        std::uint64_t current_revision = 0U;
        std::string diagnostic;
    };

    class RoutedResyncLedger
    {
    public:
        RoutedResyncLedger(
                ProducerIdentity target_producer,
                std::uint64_t route_epoch,
                DataPlaneLimits data_plane_limits = {},
                PerceptionMapUpdate::MapUpdateLimits map_update_limits = {});

        RoutedResyncAck accept(
                const RoutedResyncIntent & intent,
                const PerceptionMapUpdate::SourceIdentity & current_source,
                std::uint64_t current_revision);
        std::size_t size() const noexcept;

    private:
        ProducerIdentity target_producer_;
        std::uint64_t route_epoch_ = 0U;
        DataPlaneLimits data_plane_limits_;
        PerceptionMapUpdate::ResyncRequestLedger ledger_;
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_ROUTED_RESYNC_HPP
