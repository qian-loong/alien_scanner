#ifndef SWARM_DATA_PLANE_DATA_PLANE_TYPES_HPP
#define SWARM_DATA_PLANE_DATA_PLANE_TYPES_HPP

#include "perception_core/types/identity.hpp"
#include "perception_map_update/MapUpdateTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace SwarmDataPlane {

    constexpr std::uint16_t kProtocolVersion = 1U;

    enum class LogicalPriority : std::uint8_t
    {
        Resync     = 1,
        MapKeyframe = 2,
        MapDelta   = 3,
        Summary    = 4,
        Diagnostic = 5
    };

    struct ProducerIdentity {
        std::string           producer_id;
        Perception::SessionID session {0U, 0U};

        bool operator==(const ProducerIdentity & other) const noexcept;
        bool operator!=(const ProducerIdentity & other) const noexcept;
        bool operator<(const ProducerIdentity & other) const noexcept;
    };

    struct OriginClock {
        std::string           domain;
        Perception::SessionID session {0U, 0U};
        std::uint64_t         time_ns = 0U;
    };

    struct RouteMetadata {
        std::uint64_t route_epoch = 0U;
        std::uint16_t hop_count   = 0U;
        std::uint16_t ttl_hops    = 0U;
    };

    struct RoutedMapUpdate {
        std::uint16_t protocol_version = kProtocolVersion;
        std::string message_id;
        ProducerIdentity producer;
        std::uint64_t sequence = 0U;
        std::string correlation_id;
        LogicalPriority priority = LogicalPriority::MapDelta;
        OriginClock origin;
        std::uint64_t validity_budget_ns = 0U;
        std::uint64_t accumulated_forwarding_ns = 0U;
        RouteMetadata route;
        std::uint64_t payload_bytes = 0U;
        PerceptionMapUpdate::Hash256 payload_hash {};
        std::shared_ptr<const PerceptionMapUpdate::MapUpdate> update;
    };

    struct DataPlaneLimits {
        std::size_t max_message_id_bytes = 128U;
        std::size_t max_producer_id_bytes = 128U;
        std::size_t max_correlation_id_bytes = 128U;
        std::size_t max_clock_domain_bytes = 64U;
        std::size_t max_aggregate_id_bytes = 128U;
        std::size_t max_contributors = 64U;
        std::size_t max_recent_messages = 256U;
        std::size_t max_retired_producers = 64U;
        std::uint16_t max_ttl_hops = 64U;
        std::uint64_t max_validity_budget_ns = 60'000'000'000U;
    };

    enum class EnvelopeValidationStatus : std::uint8_t
    {
        Valid,
        RejectedInvalid,
        RejectedExpired,
        RejectedRoute
    };

    struct EnvelopeValidationResult {
        EnvelopeValidationStatus status = EnvelopeValidationStatus::RejectedInvalid;
        std::string diagnostic;

        explicit operator bool() const noexcept
        {
            return status == EnvelopeValidationStatus::Valid;
        }
    };

    struct ForwardResult {
        EnvelopeValidationStatus status = EnvelopeValidationStatus::RejectedInvalid;
        std::optional<RoutedMapUpdate> message;
        std::string diagnostic;

        explicit operator bool() const noexcept { return message.has_value(); }
    };

}// namespace SwarmDataPlane

#endif// SWARM_DATA_PLANE_DATA_PLANE_TYPES_HPP
