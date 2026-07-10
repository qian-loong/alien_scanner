#ifndef SWARM_CONTROLLER_IEXPLORATIONSTRATEGY_HPP
#define SWARM_CONTROLLER_IEXPLORATIONSTRATEGY_HPP

#include "swarm_controller/Point3f.hpp"
#include "swarm_controller/Pose3D.hpp"

#include <optional>
#include <vector>

#include <octomap/OcTree.h>

namespace SwarmController {

    using FrontierClusterId = octomap::OcTreeKey;

    enum class GoalSelectionStatus {
        Success,
        InvalidInput,
        NoKnownFree,
        NoFrontier,
        NoSafeCandidate,
    };

    struct GoalSelectionRequest {
        Pose3D                         pose {};
        std::vector<FrontierClusterId> rejected_cluster_ids;
    };

    struct ExplorationGoal {
        Point3f           position {};
        FrontierClusterId cluster_id {};
        float             utility {};
        float             frontier_area {};
    };

    struct GoalSelectionResult {
        GoalSelectionStatus           status {GoalSelectionStatus::InvalidInput};
        std::optional<ExplorationGoal> goal;
    };

    class IExplorationStrategy
    {
    public:
        virtual ~IExplorationStrategy() = default;

        virtual GoalSelectionResult selectGoal(
                const GoalSelectionRequest & request, const octomap::OcTree & tree) const = 0;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_IEXPLORATIONSTRATEGY_HPP
