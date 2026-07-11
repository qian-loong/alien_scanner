#ifndef SWARM_CONTROLLER_IEXPLORATIONSTRATEGY_HPP
#define SWARM_CONTROLLER_IEXPLORATIONSTRATEGY_HPP

#include "swarm_controller/Point3f.hpp"
#include "swarm_controller/Pose3D.hpp"
#include "swarm_controller/ExplorationDiagnostics.hpp"

#include <optional>
#include <utility>
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

    struct ForwardHalfSpaceConstraint {
        Point3f origin {};
        float   yaw {};
        float   backward_margin {};
    };

    struct FixedAltitudeConstraint {
        float altitude {};
    };

    struct GoalSelectionRequest {
        GoalSelectionRequest() = default;

        GoalSelectionRequest(
                Pose3D input_pose, std::vector<FrontierClusterId> rejected_ids = {})
            : pose(input_pose)
            , rejected_cluster_ids(std::move(rejected_ids))
        {
        }

        Pose3D                                    pose {};
        std::vector<FrontierClusterId>            rejected_cluster_ids;
        std::optional<ForwardHalfSpaceConstraint> forward_half_space;
        std::optional<FixedAltitudeConstraint>    fixed_altitude;
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
                const GoalSelectionRequest & request, const octomap::OcTree & tree,
                ExplorationDiagnostics * diagnostics = nullptr) const = 0;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_IEXPLORATIONSTRATEGY_HPP
