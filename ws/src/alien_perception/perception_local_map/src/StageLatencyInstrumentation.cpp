#include "StageLatencyInstrumentation.hpp"

#include "StageLatencyTracepoints.hpp"

#include <atomic>
#include <limits>

namespace PerceptionLocalMap::StageLatency {
    namespace {

        std::atomic<std::uint64_t> next_callback_id {1};
        thread_local std::uint64_t active_callback_id = 0;

        std::uint64_t allocate_callback_id() noexcept
        {
            auto value = next_callback_id.fetch_add(1, std::memory_order_relaxed);
            if(value != 0) {
                return value;
            }
            value = next_callback_id.fetch_add(1, std::memory_order_relaxed);
            return value == 0 ? std::numeric_limits<std::uint64_t>::max() : value;
        }

        void emit_stage(Stage stage, bool begin, std::uint64_t callback_id,
                        std::uint64_t revision) noexcept
        {
            switch(stage) {
                case Stage::MapperApply:
                    if(begin) {
                        tracepoint(
                                perception_local_map_stage, mapper_apply_begin,
                                callback_id, revision, -1);
                    }
                    else {
                        tracepoint(
                                perception_local_map_stage, mapper_apply_end,
                                callback_id, revision, -1);
                    }
                    return;
                case Stage::StatePublication:
                    if(begin) {
                        tracepoint(
                                perception_local_map_stage, state_publication_begin,
                                callback_id, revision, -1);
                    }
                    else {
                        tracepoint(
                                perception_local_map_stage, state_publication_end,
                                callback_id, revision, -1);
                    }
                    return;
                case Stage::ReadTransaction:
                    if(begin) {
                        tracepoint(
                                perception_local_map_stage, read_transaction_begin,
                                callback_id, revision, -1);
                    }
                    else {
                        tracepoint(
                                perception_local_map_stage, read_transaction_end,
                                callback_id, revision, -1);
                    }
                    return;
                case Stage::SnapshotSerialization:
                    if(begin) {
                        tracepoint(
                                perception_local_map_stage, snapshot_serialization_begin,
                                callback_id, revision, -1);
                    }
                    else {
                        tracepoint(
                                perception_local_map_stage, snapshot_serialization_end,
                                callback_id, revision, -1);
                    }
                    return;
                case Stage::SnapshotTotal:
                    if(begin) {
                        tracepoint(
                                perception_local_map_stage, snapshot_total_begin,
                                callback_id, revision, -1);
                    }
                    else {
                        tracepoint(
                                perception_local_map_stage, snapshot_total_end,
                                callback_id, revision, -1);
                    }
                    return;
            }
        }

    }// namespace

    CallbackScope::CallbackScope() noexcept
        : previous_callback_id_(active_callback_id)
        , callback_id_(allocate_callback_id())
    {
        active_callback_id = callback_id_;
        tracepoint(
                perception_local_map_stage, callback_begin,
                callback_id_, std::uint64_t {0}, 0);
    }

    CallbackScope::~CallbackScope() noexcept
    {
        tracepoint(
                perception_local_map_stage, callback_end,
                callback_id_, revision_, applied_ ? 1 : 0);
        active_callback_id = previous_callback_id_;
    }

    void CallbackScope::mark_applied(std::uint64_t revision) noexcept
    {
        applied_ = true;
        revision_ = revision;
    }

    StageScope::StageScope(Stage stage, std::uint64_t revision) noexcept
        : stage_(stage)
        , callback_id_(active_callback_id)
        , revision_(revision)
    {
        emit_stage(stage_, true, callback_id_, revision_);
    }

    StageScope::~StageScope() noexcept
    {
        emit_stage(stage_, false, callback_id_, revision_);
    }

}// namespace PerceptionLocalMap::StageLatency
