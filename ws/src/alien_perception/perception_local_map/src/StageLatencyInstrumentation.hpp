#ifndef PERCEPTION_LOCAL_MAP_STAGE_LATENCY_INSTRUMENTATION_HPP
#define PERCEPTION_LOCAL_MAP_STAGE_LATENCY_INSTRUMENTATION_HPP

#include <cstdint>

namespace PerceptionLocalMap::StageLatency {

    enum class Stage
    {
        MapperApply,
        StatePublication,
        ReadTransaction,
        SnapshotSerialization,
        SnapshotTotal,
    };

    class CallbackScope final
    {
    public:
        CallbackScope() noexcept;
        ~CallbackScope() noexcept;

        CallbackScope(const CallbackScope &) = delete;
        CallbackScope & operator=(const CallbackScope &) = delete;

        void mark_applied(std::uint64_t revision) noexcept;

    private:
        std::uint64_t previous_callback_id_ {0};
        std::uint64_t callback_id_ {0};
        std::uint64_t revision_ {0};
        bool applied_ {false};
    };

    class StageScope final
    {
    public:
        explicit StageScope(Stage stage, std::uint64_t revision = 0) noexcept;
        ~StageScope() noexcept;

        StageScope(const StageScope &) = delete;
        StageScope & operator=(const StageScope &) = delete;

    private:
        Stage stage_;
        std::uint64_t callback_id_ {0};
        std::uint64_t revision_ {0};
    };

}// namespace PerceptionLocalMap::StageLatency

#endif// PERCEPTION_LOCAL_MAP_STAGE_LATENCY_INSTRUMENTATION_HPP
