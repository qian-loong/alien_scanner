#ifndef SWARM_CONTROLLER_PLANNINGSNAPSHOTGUARD_HPP
#define SWARM_CONTROLLER_PLANNINGSNAPSHOTGUARD_HPP

#include "swarm_controller/Point3f.hpp"
#include "swarm_controller/Pose3D.hpp"
#include "swarm_controller/ExplorationTask.hpp"

#include <cstdint>
#include <vector>

namespace SwarmController {

    struct PlanningSnapshot {
        Pose3D               pose {};
        std::uint64_t        map_epoch {};
        std::int64_t         map_stamp_ns {};
        double               monotonic_time_seconds {};
        std::vector<Point3f> active_peer_goals;
        bool                 task_valid {false};
        ExplorationTaskMode  task_mode {ExplorationTaskMode::LocalFallback};
        std::uint64_t        task_allocator_epoch {};
        std::uint64_t        task_revision {};
        std::uint64_t        task_id {};
        Point3f               task_target {};
    };

    struct PlanningSnapshotConfig {
        float  max_position_drift {0.10F};
        float  max_yaw_drift {0.10F};
        double max_snapshot_age_seconds {1.0};
    };

    struct PlanningSnapshotAssessment {
        bool   map_changed {false};
        bool   pose_changed {false};
        bool   active_peer_goals_changed {false};
        bool   task_changed {false};
        bool   age_exceeded {false};
        double age_seconds {};

        bool requiresRevalidation() const
        {
            return map_changed || pose_changed || active_peer_goals_changed || task_changed
                   || age_exceeded;
        }
    };

    class PlanningSnapshotGuard
    {
    public:
        explicit PlanningSnapshotGuard(PlanningSnapshotConfig config = {});

        PlanningSnapshotAssessment assess(
                const PlanningSnapshot & before,
                const PlanningSnapshot & after) const;

        const PlanningSnapshotConfig & config() const;

    private:
        PlanningSnapshotConfig config_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_PLANNINGSNAPSHOTGUARD_HPP
