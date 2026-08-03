#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER perception_local_map_stage

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./StageLatencyTracepoints.hpp"

#if !defined(PERCEPTION_LOCAL_MAP_STAGE_LATENCY_TRACEPOINTS_HPP) \
        || defined(TRACEPOINT_HEADER_MULTI_READ)
#define PERCEPTION_LOCAL_MAP_STAGE_LATENCY_TRACEPOINTS_HPP

#include <lttng/tracepoint.h>

#include <cstdint>

TRACEPOINT_EVENT_CLASS(
        perception_local_map_stage,
        stage_boundary,
        TP_ARGS(
                std::uint64_t, callback_id,
                std::uint64_t, revision,
                int, applied),
        TP_FIELDS(
                ctf_integer(std::uint64_t, callback_id, callback_id)
                ctf_integer(std::uint64_t, revision, revision)
                ctf_integer(int, applied, applied)))

#define PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(name) \
    TRACEPOINT_EVENT_INSTANCE(                              \
            perception_local_map_stage,                    \
            stage_boundary,                                \
            name,                                          \
            TP_ARGS(                                       \
                    std::uint64_t, callback_id,             \
                    std::uint64_t, revision,                \
                    int, applied))

PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(callback_begin)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(callback_end)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(mapper_apply_begin)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(mapper_apply_end)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(state_publication_begin)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(state_publication_end)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(read_transaction_begin)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(read_transaction_end)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(snapshot_serialization_begin)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(snapshot_serialization_end)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(snapshot_total_begin)
PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE(snapshot_total_end)

#undef PERCEPTION_LOCAL_MAP_STAGE_EVENT_INSTANCE

#endif// PERCEPTION_LOCAL_MAP_STAGE_LATENCY_TRACEPOINTS_HPP

#include <lttng/tracepoint-event.h>
