#ifndef SWARM_CONTROLLER_FRONTIEREXPLORATIONSTRATEGY_HPP
#define SWARM_CONTROLLER_FRONTIEREXPLORATIONSTRATEGY_HPP

#include "swarm_controller/IExplorationStrategy.hpp"
#include "swarm_controller/KnownFreePathChecker.hpp"

namespace SwarmController {

    struct FrontierExplorationConfig {
        float planning_radius {5.5F};
        float min_goal_distance {0.8F};
        float max_goal_distance {4.0F};
        float goal_standoff {0.6F};
        float goal_search_radius {0.4F};

        float robot_radius {0.25F};
        float robot_half_height {0.15F};
        float safety_margin {0.25F};
        float vertical_margin {0.20F};

        float min_cluster_area {0.20F};
        float max_abs_frontier_normal_z {0.60F};

        float gain_weight {1.0F};
        float distance_weight {0.25F};
        float vertical_weight {0.30F};
        float heading_weight {0.15F};
        float gain_reference_area {0.20F};
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
