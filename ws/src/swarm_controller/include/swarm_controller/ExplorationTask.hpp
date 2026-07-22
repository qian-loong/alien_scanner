#ifndef SWARM_CONTROLLER_EXPLORATIONTASK_HPP
#define SWARM_CONTROLLER_EXPLORATIONTASK_HPP

#include "swarm_controller/Point3f.hpp"

#include <cstdint>

namespace SwarmController {

    enum class ExplorationTaskMode : std::uint8_t {
        LocalFallback = 0U,
        Assigned = 1U,
        Standby = 2U,
    };

    struct ExplorationTaskControl {
        std::int64_t        issued_time_ns {};
        std::int64_t        lease_duration_ns {};
        std::uint64_t       allocator_epoch {};
        std::uint64_t       revision {};
        std::uint64_t       task_id {};
        ExplorationTaskMode mode {ExplorationTaskMode::LocalFallback};
        Point3f             target {};
    };

    struct ExplorationTaskSnapshot {
        bool                   valid {false};
        ExplorationTaskControl control {};
        std::int64_t           expires_at_ns {};
    };

    struct TaskGuidance {
        bool                   valid {false};
        ExplorationTaskMode    mode {ExplorationTaskMode::LocalFallback};
        std::uint64_t          allocator_epoch {};
        std::uint64_t          revision {};
        std::uint64_t          task_id {};
        Point3f                target {};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_EXPLORATIONTASK_HPP
