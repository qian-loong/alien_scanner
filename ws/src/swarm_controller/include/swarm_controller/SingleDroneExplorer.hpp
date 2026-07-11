#ifndef SWARM_CONTROLLER_SINGLEDRONEEXPLORER_HPP
#define SWARM_CONTROLLER_SINGLEDRONEEXPLORER_HPP

#include "swarm_controller/ExplorationDiagnostics.hpp"
#include "swarm_controller/IExplorationStrategy.hpp"
#include "swarm_controller/KnownFreePathChecker.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace SwarmController {

    enum class ExplorerState {
        WaitingForMap,
        Selecting,
        Moving,
        AwaitingFreshObservation,
        Rescanning,
        Stopping,
        HoveringFailure,
    };

    enum class MotionCommandType {
        None,
        MoveTo,
        Hold,
    };

    struct MotionCommand {
        MotionCommandType type {MotionCommandType::None};
        Pose3D            goal {};
    };

    struct ExplorerInput {
        Pose3D               pose {};
        Point3f              linear_velocity {};
        float                angular_velocity_z {};
        const octomap::OcTree * map {nullptr};
        std::uint64_t        observation_epoch {0U};
        std::int64_t         observation_stamp_ns {0};
        std::int64_t         odom_stamp_ns {0};
        double               monotonic_time_seconds {};
    };

    struct SingleDroneExplorerConfig {
        float  position_tolerance {0.20F};
        float  yaw_tolerance {0.15F};
        double motion_timeout_seconds {20.0};
        double hold_timeout_seconds {2.0};
        float  stopped_linear_speed_max {0.02F};
        float  stopped_angular_speed_max {0.03F};
        double map_stale_timeout_seconds {2.0};
        float  rescan_yaw_step {0.78539816339F};
        std::size_t rescan_max_steps {8U};
        std::size_t max_rejections_per_epoch {16U};
        bool        enforce_entry_forward_half_space {true};
        float       entry_backward_margin {0.10F};
        std::size_t max_debug_faces {2000U};
        std::size_t max_debug_candidates {500U};
        BodyEnvelopeConfig body_envelope {};
    };

    struct ExplorerTickResult {
        ExplorerState state {ExplorerState::WaitingForMap};
        MotionCommand command {};
    };

    class SingleDroneExplorer
    {
    public:
        SingleDroneExplorer(
                std::shared_ptr<IExplorationStrategy> strategy,
                SingleDroneExplorerConfig config = {});

        ExplorerTickResult tick(const ExplorerInput & input);

        ExplorerState state() const;
        const ExplorationDiagnostics & diagnostics() const;
        const std::vector<FrontierClusterId> & rejectedClusterIds() const;

    private:
        enum class StopDestination {
            Selecting,
            Failure,
        };

        /// input.monotonic + 本 tick 已流逝墙钟，避免长耗时 select 用过期时间启动运动超时。
        double effectiveNow(const ExplorerInput & input) const;
        ExplorerTickResult selectAndCommand(const ExplorerInput & input);
        ExplorerTickResult beginRescan(const ExplorerInput & input);
        ExplorerTickResult beginStopping(
                const ExplorerInput & input, StopDestination destination,
                const std::string & reason);
        bool isReached(const Pose3D & pose, const Pose3D & goal) const;
        bool isStopped(const ExplorerInput & input) const;
        bool isMapStale(const ExplorerInput & input) const;
        void setState(ExplorerState state);

        std::shared_ptr<IExplorationStrategy> strategy_;
        SingleDroneExplorerConfig            config_;
        KnownFreePathChecker                 path_checker_;
        ExplorerState                        state_ {ExplorerState::WaitingForMap};
        ExplorationDiagnostics               diagnostics_;
        std::vector<FrontierClusterId>        rejected_cluster_ids_;
        std::optional<Pose3D>                 entry_pose_;
        float                                 max_entry_forward_progress_ {};
        std::optional<Pose3D>                 active_goal_;
        std::optional<FrontierClusterId>       active_cluster_id_;
        std::optional<Pose3D>                 hold_pose_;
        StopDestination                       stop_destination_ {StopDestination::Selecting};
        std::uint64_t                         current_epoch_ {0U};
        std::uint64_t                         last_checked_path_epoch_ {0U};
        std::uint64_t                         event_epoch_ {0U};
        std::int64_t                          event_odom_stamp_ns_ {0};
        double                                last_observation_time_ {};
        double                                state_start_time_ {};
        std::size_t                           completed_rescan_steps_ {0U};
        bool                                  rescan_reached_ {false};
        std::chrono::steady_clock::time_point tick_wall_start_ {};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_SINGLEDRONEEXPLORER_HPP
