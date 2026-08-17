#include "swarm_data_plane/RoutedResync.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <utility>

namespace SwarmDataPlane {

    namespace {

        bool valid_producer(
                const ProducerIdentity & producer,
                const DataPlaneLimits & limits)
        {
            return producer.session.boot_time_ns != 0U
                   && static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                           producer.producer_id,
                           limits.max_producer_id_bytes,
                           "target_producer_id",
                           false));
        }

    }// namespace

    RoutedResyncLedger::RoutedResyncLedger(
            ProducerIdentity target_producer,
            std::uint64_t route_epoch,
            DataPlaneLimits data_plane_limits,
            PerceptionMapUpdate::MapUpdateLimits map_update_limits)
            : target_producer_(std::move(target_producer)),
              route_epoch_(route_epoch),
              data_plane_limits_(std::move(data_plane_limits)),
              ledger_(std::move(map_update_limits))
    {
    }

    RoutedResyncAck RoutedResyncLedger::accept(
            const RoutedResyncIntent & intent,
            const PerceptionMapUpdate::SourceIdentity & current_source,
            std::uint64_t current_revision,
            const PerceptionMapUpdate::VersionedContentDigest & current_content_identity)
    {
        const auto rejected = [&](std::string diagnostic) {
            return RoutedResyncAck {
                    kProtocolVersion,
                    false,
                    {},
                    target_producer_,
                    current_source,
                    current_revision,
                    current_content_identity,
                    std::move(diagnostic)};
        };
        if(intent.protocol_version != kProtocolVersion) {
            return rejected("routed resync protocol version mismatch");
        }
        if(!valid_producer(target_producer_, data_plane_limits_)
           || intent.target_producer != target_producer_) {
            return rejected("routed resync targets an unknown producer session");
        }
        if(route_epoch_ == 0U || intent.route_epoch != route_epoch_) {
            return rejected("routed resync targets a stale or unknown route epoch");
        }

        const auto response = ledger_.accept(
                intent.request, current_source, current_revision, current_content_identity);
        return {kProtocolVersion,
                response.accepted,
                response.correlation_id,
                target_producer_,
                response.current_source,
                response.current_revision,
                response.current_content_identity,
                response.diagnostic};
    }

    std::size_t RoutedResyncLedger::size() const noexcept
    {
        return ledger_.size();
    }

}// namespace SwarmDataPlane
