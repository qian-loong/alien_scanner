#ifndef SWARM_CONTROLLER_EXPLORATIONDIAGNOSTICS_HPP
#define SWARM_CONTROLLER_EXPLORATIONDIAGNOSTICS_HPP

#include "swarm_controller/Point3f.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <octomap/OcTreeKey.h>

namespace SwarmController {

    struct DebugFrontierFace {
        Point3f            position {};
        octomap::OcTreeKey cluster_id {};
    };

    struct DebugFrontierCluster {
        octomap::OcTreeKey id {};
        Point3f            position {};
        float              area {};
        bool               rejected {false};
    };

    struct ExplorationDiagnostics {
        std::size_t max_debug_faces {2000U};
        std::size_t max_debug_candidates {500U};

        std::size_t raw_candidate_count {};
        std::size_t unique_candidate_count {};
        std::size_t forward_filtered_count {};
        /// 固定规模前向候选流水线中 peer 硬过滤阶段的计数。
        std::size_t pre_peer_candidate_count {};
        std::size_t peer_goal_filtered_count {};
        std::size_t post_peer_candidate_count {};
        std::size_t task_filtered_count {};
        std::size_t segment_check_count {};
        double      selection_elapsed_seconds {};

        std::vector<DebugFrontierFace>    frontier_faces;
        std::vector<DebugFrontierCluster> frontier_clusters;
        std::vector<Point3f>              locally_safe_candidates;
        std::optional<Point3f>            selected_goal;
        std::optional<Point3f>            path_start;
        std::optional<Point3f>            path_goal;
        std::optional<Point3f>            first_blocked_position;
        std::string                       path_status;
        std::string                       current_body_status;
        std::string                       controller_state;
        std::string                       failure_reason;
        /// 最近一次策略返回的 GoalSelectionStatus 名称（由策略 finish() 写入）。
        std::string                       last_goal_status;

        void clear()
        {
            raw_candidate_count         = 0U;
            unique_candidate_count      = 0U;
            forward_filtered_count      = 0U;
            pre_peer_candidate_count    = 0U;
            peer_goal_filtered_count    = 0U;
            post_peer_candidate_count   = 0U;
            task_filtered_count         = 0U;
            segment_check_count         = 0U;
            selection_elapsed_seconds   = 0.0;
            frontier_faces.clear();
            frontier_clusters.clear();
            locally_safe_candidates.clear();
            selected_goal.reset();
            path_start.reset();
            path_goal.reset();
            first_blocked_position.reset();
            path_status.clear();
            current_body_status.clear();
            controller_state.clear();
            failure_reason.clear();
            last_goal_status.clear();
        }
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_EXPLORATIONDIAGNOSTICS_HPP
