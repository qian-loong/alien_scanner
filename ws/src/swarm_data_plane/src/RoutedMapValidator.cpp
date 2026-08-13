#include "swarm_data_plane/RoutedMapValidator.hpp"

#include "perception_map_update/CanonicalCodec.hpp"

#include <limits>

namespace SwarmDataPlane {

    namespace {

        bool valid_text(
                const std::string & value,
                std::size_t limit,
                bool allow_empty)
        {
            return static_cast<bool>(PerceptionMapUpdate::CanonicalCodec::validate_string(
                    value, limit, "data-plane string", allow_empty));
        }

        bool valid_priority(LogicalPriority priority) noexcept
        {
            switch(priority) {
                case LogicalPriority::Resync:
                case LogicalPriority::MapKeyframe:
                case LogicalPriority::MapDelta:
                case LogicalPriority::Summary:
                case LogicalPriority::Diagnostic:
                    return true;
            }
            return false;
        }

        LogicalPriority expected_priority(PerceptionMapUpdate::UpdateKind kind) noexcept
        {
            switch(kind) {
                case PerceptionMapUpdate::UpdateKind::Keyframe:
                case PerceptionMapUpdate::UpdateKind::Remove:
                    return LogicalPriority::MapKeyframe;
                case PerceptionMapUpdate::UpdateKind::Delta:
                    return LogicalPriority::MapDelta;
                case PerceptionMapUpdate::UpdateKind::Summary:
                    return LogicalPriority::Summary;
            }
            return LogicalPriority::Diagnostic;
        }

    }// namespace

    EnvelopeValidationResult validate_routed_map_update(
            const RoutedMapUpdate & message,
            const DataPlaneLimits & limits)
    {
        if(message.protocol_version != kProtocolVersion) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "data-plane protocol version mismatch"};
        }
        if(!valid_text(message.message_id, limits.max_message_id_bytes, false)
           || !valid_text(message.producer.producer_id, limits.max_producer_id_bytes, false)
           || !valid_text(
                   message.correlation_id, limits.max_correlation_id_bytes, true)
           || !valid_text(message.origin.domain, limits.max_clock_domain_bytes, false)) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "data-plane envelope contains an invalid or oversized string"};
        }
        if(message.producer.session.boot_time_ns == 0U || message.sequence == 0U
           || message.origin.session.boot_time_ns == 0U || message.origin.time_ns == 0U) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "producer, sequence, or origin clock identity is invalid"};
        }
        if(!valid_priority(message.priority) || message.update == nullptr) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "priority is invalid or map update payload is missing"};
        }
        if(message.validity_budget_ns == 0U
           || message.validity_budget_ns > limits.max_validity_budget_ns) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "validity budget is outside configured limits"};
        }
        if(message.accumulated_forwarding_ns >= message.validity_budget_ns) {
            return {EnvelopeValidationStatus::RejectedExpired,
                    "data-plane message validity budget is exhausted"};
        }
        if(message.route.route_epoch == 0U || message.route.ttl_hops == 0U
           || message.route.ttl_hops > limits.max_ttl_hops) {
            return {EnvelopeValidationStatus::RejectedRoute,
                    "route epoch or hop TTL is invalid"};
        }
        if(message.route.hop_count >= message.route.ttl_hops) {
            return {EnvelopeValidationStatus::RejectedRoute,
                    "route hop TTL is exhausted"};
        }
        if(message.payload_bytes != message.update->canonical_payload_bytes
           || message.payload_hash != message.update->update_hash) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "envelope payload size or hash differs from the C3 map update"};
        }
        if(message.correlation_id != message.update->correlation_id) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "envelope correlation differs from the C3 map update"};
        }
        if(message.priority != expected_priority(message.update->kind)) {
            return {EnvelopeValidationStatus::RejectedInvalid,
                    "logical priority does not match the map update kind"};
        }
        return {EnvelopeValidationStatus::Valid, {}};
    }

    ForwardResult forward_routed_map_update(
            const RoutedMapUpdate & message,
            std::uint64_t local_elapsed_ns,
            const DataPlaneLimits & limits)
    {
        const auto validation = validate_routed_map_update(message, limits);
        if(!validation) {
            return {validation.status, std::nullopt, validation.diagnostic};
        }
        if(local_elapsed_ns > std::numeric_limits<std::uint64_t>::max()
                                      - message.accumulated_forwarding_ns) {
            return {EnvelopeValidationStatus::RejectedExpired, std::nullopt,
                    "forwarding duration overflows the validity accumulator"};
        }
        const auto accumulated = message.accumulated_forwarding_ns + local_elapsed_ns;
        if(accumulated >= message.validity_budget_ns) {
            return {EnvelopeValidationStatus::RejectedExpired, std::nullopt,
                    "forwarding exhausted the message validity budget"};
        }
        if(message.route.hop_count == std::numeric_limits<std::uint16_t>::max()
           || static_cast<std::uint32_t>(message.route.hop_count) + 1U
                      >= message.route.ttl_hops) {
            return {EnvelopeValidationStatus::RejectedRoute, std::nullopt,
                    "forwarding would exhaust the route hop TTL"};
        }

        auto forwarded = message;
        forwarded.accumulated_forwarding_ns = accumulated;
        ++forwarded.route.hop_count;
        return {EnvelopeValidationStatus::Valid, std::move(forwarded), {}};
    }

}// namespace SwarmDataPlane
