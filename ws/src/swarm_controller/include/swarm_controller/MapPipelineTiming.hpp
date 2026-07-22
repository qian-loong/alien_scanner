#ifndef SWARM_CONTROLLER_MAPPIPELINETIMING_HPP
#define SWARM_CONTROLLER_MAPPIPELINETIMING_HPP

#include <cmath>
#include <cstdint>

namespace SwarmController {

    struct MapWorkerTiming {
        double claim_seconds {-1.0};
        double decode_seconds {-1.0};
        double detect_seconds {-1.0};
        double complete_seconds {-1.0};
        std::int64_t receive_ros_ns {};
    };

    struct MapPipelineTiming {
        std::uint64_t revision {};
        bool applied {false};
        double queue_wait_seconds {-1.0};
        double decode_seconds {-1.0};
        double detect_seconds {-1.0};
        double worker_seconds {-1.0};
        double apply_wait_seconds {-1.0};
        double total_latency_seconds {-1.0};
        double receive_header_age_seconds {-1.0};
        double consume_header_age_seconds {-1.0};
    };

    inline double elapsedSeconds(const double start, const double finish)
    {
        if(!std::isfinite(start) || !std::isfinite(finish)
           || start < 0.0 || finish < start)
        {
            return -1.0;
        }
        return finish - start;
    }

    inline double rosAgeSeconds(const std::int64_t stamp_ns, const std::int64_t now_ns)
    {
        if(stamp_ns <= 0 || now_ns < stamp_ns) {
            return -1.0;
        }
        return static_cast<double>(now_ns - stamp_ns) * 1.0e-9;
    }

    inline MapPipelineTiming makeMapPipelineTiming(
            const std::uint64_t revision, const bool applied,
            const std::int64_t stamp_ns, const double received_seconds,
            const MapWorkerTiming & worker, const double consumed_seconds,
            const std::int64_t consumed_ros_ns)
    {
        MapPipelineTiming result;
        result.revision = revision;
        result.applied = applied;
        result.queue_wait_seconds = elapsedSeconds(
                received_seconds, worker.claim_seconds);
        result.decode_seconds = worker.decode_seconds;
        result.detect_seconds = worker.detect_seconds;
        result.worker_seconds = elapsedSeconds(
                worker.claim_seconds, worker.complete_seconds);
        result.apply_wait_seconds = elapsedSeconds(
                worker.complete_seconds, consumed_seconds);
        result.total_latency_seconds = elapsedSeconds(
                received_seconds, consumed_seconds);
        result.receive_header_age_seconds = rosAgeSeconds(
                stamp_ns, worker.receive_ros_ns);
        result.consume_header_age_seconds = rosAgeSeconds(
                stamp_ns, consumed_ros_ns);
        return result;
    }

}// namespace SwarmController

#endif// SWARM_CONTROLLER_MAPPIPELINETIMING_HPP
