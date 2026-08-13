#include "LogicalLinkAdapter.hpp"
#include "TestFixtures.hpp"

#include "perception_map_update/MapUpdateProducer.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

namespace SwarmDataPlane::Test {

    namespace {

        std::vector<RoutedMapUpdate> messages(std::size_t count)
        {
            PerceptionMapUpdate::MapUpdateProducer producer;
            const auto prepared = producer.prepare(snapshot(
                    1U,
                    {{{0, 0, 0}, PerceptionMapUpdate::CellState::Free}}));
            EXPECT_TRUE(prepared.update.has_value()) << prepared.diagnostic;
            std::vector<RoutedMapUpdate> result;
            for(std::size_t index = 0U; index < count; ++index) {
                result.push_back(routed(
                        shared_update(prepared),
                        index + 1U,
                        "message-" + std::to_string(index + 1U)));
            }
            return result;
        }

        std::vector<LinkTraceEvent> run_seeded_trace()
        {
            LogicalLinkConfig config;
            config.seed = 42U;
            config.loss_per_million = 350'000U;
            config.min_delay_ns = 10U;
            config.max_delay_ns = 1'000U;
            config.bandwidth_bytes_per_second = 10'000'000U;
            config.max_queue_messages = 32U;
            LogicalLinkAdapter link(config);
            const auto input = messages(12U);
            for(const auto & message : input) {
                link.submit(message, 1'000U);
            }
            link.poll(10'000'000U);
            return link.trace();
        }

    }// namespace

    TEST(LogicalLinkAdapterTest, FixedSeedReplaysTheSameLossAndDeliveryTrace)
    {
        const auto first = run_seeded_trace();
        const auto second = run_seeded_trace();
        EXPECT_EQ(first, second);
        EXPECT_TRUE(std::any_of(
                first.begin(), first.end(), [](const LinkTraceEvent & event) {
                    return event.reason == LinkFaultReason::RandomLoss;
                }));
        EXPECT_TRUE(std::any_of(
                first.begin(), first.end(), [](const LinkTraceEvent & event) {
                    return event.kind == LinkTraceKind::Delivered;
                }));
    }

    TEST(LogicalLinkAdapterTest, PartitionAndQueueLimitsAreExplicitAndBounded)
    {
        LogicalLinkConfig config;
        config.seed = 7U;
        config.max_queue_messages = 2U;
        config.max_queue_bytes = 1U * 1024U * 1024U;
        config.min_delay_ns = 10U;
        config.max_delay_ns = 10U;
        LogicalLinkAdapter link(config);
        const auto input = messages(4U);

        link.set_partitioned(true, 100U);
        EXPECT_EQ(
                link.submit(input[0], 101U).reason,
                LinkFaultReason::Partitioned);
        link.set_partitioned(false, 102U);
        EXPECT_TRUE(link.submit(input[0], 103U).accepted);
        EXPECT_TRUE(link.submit(input[1], 104U).accepted);
        EXPECT_EQ(link.queue_depth(), 2U);
        EXPECT_EQ(
                link.submit(input[2], 105U).reason,
                LinkFaultReason::QueueMessageLimit);

        const auto delivered = link.poll(1'000U);
        ASSERT_EQ(delivered.size(), 2U);
        EXPECT_EQ(link.queue_depth(), 0U);
        EXPECT_EQ(link.queued_bytes(), 0U);
        for(const auto & message : delivered) {
            const auto original = std::find_if(
                    input.begin(), input.end(), [&](const RoutedMapUpdate & candidate) {
                        return candidate.message_id == message.message_id;
                    });
            ASSERT_NE(original, input.end());
            EXPECT_EQ(message.route.hop_count, 1U);
            EXPECT_EQ(message.update, original->update);
            EXPECT_EQ(message.payload_hash, original->payload_hash);
        }
    }

    TEST(LogicalLinkAdapterTest, ClockFaultsChangeDiagnosticsNotPayloadOrigin)
    {
        LogicalLinkConfig config;
        config.min_delay_ns = 20U;
        config.max_delay_ns = 20U;
        LogicalLinkAdapter link(config);
        LogicalClockFault fault;
        fault.offset_ns = 500;
        fault.skew_parts_per_million = 100'000;
        fault.jump_ns = -100;
        fault.session = {900U, 3U};
        link.set_clock_fault(fault, 1'000U);

        const auto input = messages(1U).front();
        ASSERT_TRUE(link.submit(input, 2'000U).accepted);
        const auto delivered = link.poll(3'000U);
        ASSERT_EQ(delivered.size(), 1U);
        EXPECT_EQ(delivered.front().origin.domain, input.origin.domain);
        EXPECT_EQ(delivered.front().origin.session, input.origin.session);
        EXPECT_EQ(delivered.front().origin.time_ns, input.origin.time_ns);
        EXPECT_EQ(delivered.front().payload_hash, input.payload_hash);
        ASSERT_FALSE(link.trace().empty());
        EXPECT_EQ(link.trace().front().kind, LinkTraceKind::ClockChanged);
        EXPECT_EQ(link.trace().front().clock_session, fault.session);
        EXPECT_NE(
                link.trace().back().observed_clock_time_ns,
                link.trace().back().physical_time_ns);
    }

}// namespace SwarmDataPlane::Test
