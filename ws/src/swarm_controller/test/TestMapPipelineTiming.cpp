#include "swarm_controller/MapPipelineTiming.hpp"

#include <gtest/gtest.h>

namespace SwarmController {

    TEST(MapPipelineTimingTest, BindsEveryDurationAndAgeToConsumedRevision)
    {
        const MapWorkerTiming worker {
                11.5, 0.4, 0.8, 12.8, 105'000'000'000LL};

        const MapPipelineTiming timing = makeMapPipelineTiming(
                7U, true, 100'000'000'000LL, 10.0, worker,
                13.0, 109'000'000'000LL);

        EXPECT_EQ(timing.revision, 7U);
        EXPECT_TRUE(timing.applied);
        EXPECT_DOUBLE_EQ(timing.queue_wait_seconds, 1.5);
        EXPECT_DOUBLE_EQ(timing.decode_seconds, 0.4);
        EXPECT_DOUBLE_EQ(timing.detect_seconds, 0.8);
        EXPECT_DOUBLE_EQ(timing.worker_seconds, 1.3);
        EXPECT_NEAR(timing.apply_wait_seconds, 0.2, 1.0e-12);
        EXPECT_DOUBLE_EQ(timing.total_latency_seconds, 3.0);
        EXPECT_DOUBLE_EQ(timing.receive_header_age_seconds, 5.0);
        EXPECT_DOUBLE_EQ(timing.consume_header_age_seconds, 9.0);
    }

    TEST(MapPipelineTimingTest, PreservesTimingForConsumedButRejectedResult)
    {
        const MapWorkerTiming worker {
                2.0, 0.1, 0.0, 2.2, 12'000'000'000LL};

        const MapPipelineTiming timing = makeMapPipelineTiming(
                3U, false, 10'000'000'000LL, 1.0, worker,
                2.5, 13'000'000'000LL);

        EXPECT_EQ(timing.revision, 3U);
        EXPECT_FALSE(timing.applied);
        EXPECT_DOUBLE_EQ(timing.total_latency_seconds, 1.5);
        EXPECT_DOUBLE_EQ(timing.consume_header_age_seconds, 3.0);
    }

    TEST(MapPipelineTimingTest, MarksClockRollbackAndInvalidSteadyOrderUnavailable)
    {
        const MapWorkerTiming worker {
                4.0, 0.1, 0.2, 3.5, 80'000'000'000LL};

        const MapPipelineTiming timing = makeMapPipelineTiming(
                9U, false, 90'000'000'000LL, 5.0, worker,
                3.0, 70'000'000'000LL);

        EXPECT_EQ(timing.queue_wait_seconds, -1.0);
        EXPECT_EQ(timing.worker_seconds, -1.0);
        EXPECT_EQ(timing.apply_wait_seconds, -1.0);
        EXPECT_EQ(timing.total_latency_seconds, -1.0);
        EXPECT_EQ(timing.receive_header_age_seconds, -1.0);
        EXPECT_EQ(timing.consume_header_age_seconds, -1.0);
    }

}// namespace SwarmController
