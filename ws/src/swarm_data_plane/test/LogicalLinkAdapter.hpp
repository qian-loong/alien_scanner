#ifndef SWARM_DATA_PLANE_TEST_LOGICAL_LINK_ADAPTER_HPP
#define SWARM_DATA_PLANE_TEST_LOGICAL_LINK_ADAPTER_HPP

#include "swarm_data_plane/DataPlaneTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace SwarmDataPlane::Test {

    enum class LinkFaultReason : std::uint8_t
    {
        None,
        InvalidEnvelope,
        RandomLoss,
        Partitioned,
        QueueMessageLimit,
        QueueByteLimit,
        Expired,
        RouteLimit
    };

    enum class LinkTraceKind : std::uint8_t
    {
        Queued,
        Delivered,
        Dropped,
        LinkDown,
        LinkUp,
        ClockChanged
    };

    struct LogicalLinkConfig {
        std::uint64_t seed = 1U;
        std::uint32_t loss_per_million = 0U;
        std::uint64_t min_delay_ns = 0U;
        std::uint64_t max_delay_ns = 0U;
        std::uint64_t bandwidth_bytes_per_second = 0U;
        std::size_t max_queue_messages = 64U;
        std::uint64_t max_queue_bytes = 16U * 1024U * 1024U;
        DataPlaneLimits data_plane_limits;
    };

    struct LogicalClockFault {
        std::int64_t offset_ns = 0;
        std::int64_t skew_parts_per_million = 0;
        std::int64_t jump_ns = 0;
        Perception::SessionID session {1U, 1U};
    };

    struct LinkTraceEvent {
        LinkTraceKind kind = LinkTraceKind::Dropped;
        LinkFaultReason reason = LinkFaultReason::None;
        std::string message_id;
        std::uint64_t physical_time_ns = 0U;
        std::uint64_t observed_clock_time_ns = 0U;
        Perception::SessionID clock_session {0U, 0U};
        std::size_t queue_depth = 0U;
        std::uint64_t queued_bytes = 0U;
        PerceptionMapUpdate::Hash256 payload_hash {};

        bool operator==(const LinkTraceEvent & other) const noexcept;
    };

    struct LinkSubmitResult {
        bool accepted = false;
        LinkFaultReason reason = LinkFaultReason::None;
    };

    class LogicalLinkAdapter
    {
    public:
        explicit LogicalLinkAdapter(LogicalLinkConfig config);

        LinkSubmitResult submit(const RoutedMapUpdate & message, std::uint64_t now_ns);
        std::vector<RoutedMapUpdate> poll(std::uint64_t now_ns);
        void set_partitioned(bool partitioned, std::uint64_t now_ns);
        void set_clock_fault(LogicalClockFault fault, std::uint64_t now_ns);

        std::uint64_t observed_clock_time(std::uint64_t physical_time_ns) const noexcept;
        const std::vector<LinkTraceEvent> & trace() const noexcept;
        std::size_t queue_depth() const noexcept;
        std::uint64_t queued_bytes() const noexcept;

    private:
        struct QueuedMessage {
            RoutedMapUpdate message;
            std::uint64_t enqueue_time_ns = 0U;
            std::uint64_t delivery_time_ns = 0U;
            std::uint64_t accounted_bytes = 0U;
            std::uint64_t insertion_order = 0U;
        };

        void record(
                LinkTraceKind kind,
                LinkFaultReason reason,
                const RoutedMapUpdate * message,
                std::uint64_t physical_time_ns);
        LinkSubmitResult drop(
                LinkFaultReason reason,
                const RoutedMapUpdate & message,
                std::uint64_t physical_time_ns);

        LogicalLinkConfig config_;
        LogicalClockFault clock_fault_;
        std::mt19937_64 random_;
        bool partitioned_ = false;
        std::uint64_t next_link_available_ns_ = 0U;
        std::uint64_t next_insertion_order_ = 0U;
        std::uint64_t queued_bytes_ = 0U;
        std::vector<QueuedMessage> queue_;
        std::vector<LinkTraceEvent> trace_;
    };

}// namespace SwarmDataPlane::Test

#endif// SWARM_DATA_PLANE_TEST_LOGICAL_LINK_ADAPTER_HPP
