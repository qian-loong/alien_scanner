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
        /// 当前位姿的 swept-body 包络已被 unknown/occupied 阻塞。
        /// 该状态必须进入净空恢复，不能用普通 yaw 重扫处理。
        StartBodyConflict,
        /// 存在候选（pre-peer > 0），但全部被有效 peer goal 硬分离掉（post-peer == 0）。
        /// 与 NoSafeCandidate 区分：应等待 peer 目标变化而非原地 yaw 重扫。
        PeerGoalConflict,
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
        /// 最近一段有效平移的 map-frame 航向。与入口 no-retreat 航向语义独立；
        /// yaw-only rescan 不得更新此值。
        std::optional<float>                      preferred_travel_yaw;
        std::optional<ForwardHalfSpaceConstraint> forward_half_space;
        std::optional<FixedAltitudeConstraint>    fixed_altitude;
        /// Other drones' map-frame positions (XY used for soft dispersion).
        std::vector<Point3f>                      peer_positions;
        /// Other drones' active exploration goals in map frame (XY hard separation).
        std::vector<Point3f>                      active_peer_goals;
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
