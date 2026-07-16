#ifndef SWARM_CONTROLLER_FRONTIEREXPLORATIONSTRATEGY_HPP
#define SWARM_CONTROLLER_FRONTIEREXPLORATIONSTRATEGY_HPP

#include "swarm_controller/IExplorationStrategy.hpp"
#include "swarm_controller/KnownFreePathChecker.hpp"

namespace SwarmController {

    struct FrontierExplorationConfig {
        float forward_lookahead_min {0.8F};
        float forward_lookahead_max {2.0F};
        float forward_lateral_limit {0.5F};
        std::size_t forward_distance_samples {4U};
        std::size_t forward_lateral_samples {5U};

        float robot_radius {0.25F};
        float robot_half_height {0.15F};
        float safety_margin {0.25F};
        float vertical_margin {0.20F};

        float lateral_weight {0.60F};
        float heading_weight {0.25F};
        /// Soft penalty weight: utility -= w * Σ 1/(1+d_xy) over peers/goals.
        float dispersion_weight {0.35F};
        float task_progress_weight {1.0F};
        float task_min_progress {0.15F};
        float task_max_heading_error {1.05F};
        /// Hard filter radius (m, XY) against active peer goals only.
        float min_peer_goal_separation {0.8F};
    };

    class FrontierExplorationStrategy final : public IExplorationStrategy
    {
    public:
        explicit FrontierExplorationStrategy(FrontierExplorationConfig config = {});

        GoalSelectionResult selectGoal(
                const GoalSelectionRequest & request, const octomap::OcTree & tree,
                ExplorationDiagnostics * diagnostics = nullptr) const override;

    private:
        FrontierExplorationConfig config_;
        KnownFreePathChecker      path_checker_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_FRONTIEREXPLORATIONSTRATEGY_HPP
