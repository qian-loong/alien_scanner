#include "LogicalLinkAdapter.hpp"

#include "swarm_data_plane/RoutedMapValidator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace SwarmDataPlane::Test {

    bool LinkTraceEvent::operator==(const LinkTraceEvent & other) const noexcept
    {
        return kind == other.kind && reason == other.reason
               && message_id == other.message_id
               && physical_time_ns == other.physical_time_ns
               && observed_clock_time_ns == other.observed_clock_time_ns
               && clock_session == other.clock_session && queue_depth == other.queue_depth
               && queued_bytes == other.queued_bytes && payload_hash == other.payload_hash;
    }

    LogicalLinkAdapter::LogicalLinkAdapter(LogicalLinkConfig config)
            : config_(std::move(config)), random_(config_.seed)
    {
        if(config_.loss_per_million > 1'000'000U
           || config_.max_delay_ns < config_.min_delay_ns
           || config_.max_queue_messages == 0U || config_.max_queue_bytes == 0U) {
            throw std::invalid_argument("logical link configuration is invalid");
        }
    }

    std::uint64_t LogicalLinkAdapter::observed_clock_time(
            std::uint64_t physical_time_ns) const noexcept
    {
        const long double scaled = static_cast<long double>(physical_time_ns)
                                   * (1.0L
                                      + static_cast<long double>(
                                                clock_fault_.skew_parts_per_million)
                                                / 1'000'000.0L)
                                   + static_cast<long double>(clock_fault_.offset_ns)
                                   + static_cast<long double>(clock_fault_.jump_ns);
        if(scaled <= 0.0L) {
            return 0U;
        }
        if(scaled >= static_cast<long double>(
                             std::numeric_limits<std::uint64_t>::max())) {
            return std::numeric_limits<std::uint64_t>::max();
        }
        return static_cast<std::uint64_t>(scaled);
    }

    void LogicalLinkAdapter::record(
            LinkTraceKind kind,
            LinkFaultReason reason,
            const RoutedMapUpdate * message,
            std::uint64_t physical_time_ns)
    {
        LinkTraceEvent event;
        event.kind = kind;
        event.reason = reason;
        if(message != nullptr) {
            event.message_id = message->message_id;
            event.payload_hash = message->payload_hash;
        }
        event.physical_time_ns = physical_time_ns;
        event.observed_clock_time_ns = observed_clock_time(physical_time_ns);
        event.clock_session = clock_fault_.session;
        event.queue_depth = queue_.size();
        event.queued_bytes = queued_bytes_;
        trace_.push_back(std::move(event));
    }

    LinkSubmitResult LogicalLinkAdapter::drop(
            LinkFaultReason reason,
            const RoutedMapUpdate & message,
            std::uint64_t physical_time_ns)
    {
        record(LinkTraceKind::Dropped, reason, &message, physical_time_ns);
        return {false, reason};
    }

    LinkSubmitResult LogicalLinkAdapter::submit(
            const RoutedMapUpdate & message,
            std::uint64_t now_ns)
    {
        const auto validation = validate_routed_map_update(
                message, config_.data_plane_limits);
        if(!validation) {
            const auto reason = validation.status == EnvelopeValidationStatus::RejectedExpired
                                        ? LinkFaultReason::Expired
                                : validation.status
                                                  == EnvelopeValidationStatus::RejectedRoute
                                        ? LinkFaultReason::RouteLimit
                                        : LinkFaultReason::InvalidEnvelope;
            return drop(reason, message, now_ns);
        }
        if(partitioned_) {
            return drop(LinkFaultReason::Partitioned, message, now_ns);
        }
        if(config_.loss_per_million != 0U) {
            std::uniform_int_distribution<std::uint32_t> loss(0U, 999'999U);
            if(loss(random_) < config_.loss_per_million) {
                return drop(LinkFaultReason::RandomLoss, message, now_ns);
            }
        }
        const auto accounted_bytes = message.payload_bytes;
        if(queue_.size() >= config_.max_queue_messages) {
            return drop(LinkFaultReason::QueueMessageLimit, message, now_ns);
        }
        if(accounted_bytes > config_.max_queue_bytes - queued_bytes_) {
            return drop(LinkFaultReason::QueueByteLimit, message, now_ns);
        }

        std::uniform_int_distribution<std::uint64_t> delay(
                config_.min_delay_ns, config_.max_delay_ns);
        const auto propagation_delay = delay(random_);
        const auto transmission_start = std::max(now_ns, next_link_available_ns_);
        std::uint64_t transmission_duration = 0U;
        if(config_.bandwidth_bytes_per_second != 0U && accounted_bytes != 0U) {
            const long double duration = std::ceil(
                    static_cast<long double>(accounted_bytes) * 1'000'000'000.0L
                    / static_cast<long double>(config_.bandwidth_bytes_per_second));
            if(duration > static_cast<long double>(
                                  std::numeric_limits<std::uint64_t>::max())) {
                return drop(LinkFaultReason::QueueByteLimit, message, now_ns);
            }
            transmission_duration = static_cast<std::uint64_t>(duration);
        }
        if(transmission_duration
                   > std::numeric_limits<std::uint64_t>::max() - transmission_start
           || propagation_delay
                      > std::numeric_limits<std::uint64_t>::max()
                                - transmission_start - transmission_duration) {
            return drop(LinkFaultReason::Expired, message, now_ns);
        }
        next_link_available_ns_ = transmission_start + transmission_duration;
        const auto delivery_time = next_link_available_ns_ + propagation_delay;
        queue_.push_back({
                message,
                now_ns,
                delivery_time,
                accounted_bytes,
                next_insertion_order_++});
        queued_bytes_ += accounted_bytes;
        record(LinkTraceKind::Queued, LinkFaultReason::None, &message, now_ns);
        return {true, LinkFaultReason::None};
    }

    std::vector<RoutedMapUpdate> LogicalLinkAdapter::poll(std::uint64_t now_ns)
    {
        std::vector<RoutedMapUpdate> delivered;
        if(partitioned_) {
            return delivered;
        }
        std::stable_sort(
                queue_.begin(), queue_.end(), [](const QueuedMessage & left,
                                                 const QueuedMessage & right) {
                    if(left.delivery_time_ns != right.delivery_time_ns) {
                        return left.delivery_time_ns < right.delivery_time_ns;
                    }
                    return left.insertion_order < right.insertion_order;
                });
        while(!queue_.empty() && queue_.front().delivery_time_ns <= now_ns) {
            auto queued = std::move(queue_.front());
            queue_.erase(queue_.begin());
            queued_bytes_ -= queued.accounted_bytes;
            const auto elapsed = queued.delivery_time_ns - queued.enqueue_time_ns;
            auto forwarded = forward_routed_map_update(
                    queued.message, elapsed, config_.data_plane_limits);
            if(!forwarded) {
                const auto reason = forwarded.status
                                                    == EnvelopeValidationStatus::RejectedRoute
                                            ? LinkFaultReason::RouteLimit
                                            : LinkFaultReason::Expired;
                record(
                        LinkTraceKind::Dropped,
                        reason,
                        &queued.message,
                        queued.delivery_time_ns);
                continue;
            }
            record(
                    LinkTraceKind::Delivered,
                    LinkFaultReason::None,
                    &*forwarded.message,
                    queued.delivery_time_ns);
            delivered.push_back(std::move(*forwarded.message));
        }
        return delivered;
    }

    void LogicalLinkAdapter::set_partitioned(bool partitioned, std::uint64_t now_ns)
    {
        if(partitioned_ == partitioned) {
            return;
        }
        partitioned_ = partitioned;
        record(
                partitioned ? LinkTraceKind::LinkDown : LinkTraceKind::LinkUp,
                LinkFaultReason::None,
                nullptr,
                now_ns);
    }

    void LogicalLinkAdapter::set_clock_fault(
            LogicalClockFault fault,
            std::uint64_t now_ns)
    {
        if(fault.session.boot_time_ns == 0U) {
            throw std::invalid_argument("logical clock session must be non-zero");
        }
        clock_fault_ = fault;
        record(
                LinkTraceKind::ClockChanged,
                LinkFaultReason::None,
                nullptr,
                now_ns);
    }

    const std::vector<LinkTraceEvent> & LogicalLinkAdapter::trace() const noexcept
    {
        return trace_;
    }

    std::size_t LogicalLinkAdapter::queue_depth() const noexcept
    {
        return queue_.size();
    }

    std::uint64_t LogicalLinkAdapter::queued_bytes() const noexcept
    {
        return queued_bytes_;
    }

}// namespace SwarmDataPlane::Test
