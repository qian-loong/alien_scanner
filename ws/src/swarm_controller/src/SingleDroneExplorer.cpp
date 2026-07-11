#include "swarm_controller/SingleDroneExplorer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr float PI = 3.14159265358979323846F;

        bool isFinite(const Pose3D & pose)
        {
            return std::isfinite(pose.position.x) && std::isfinite(pose.position.y)
                   && std::isfinite(pose.position.z) && std::isfinite(pose.yaw);
        }

        bool isFinite(const ExplorerInput & input)
        {
            return isFinite(input.pose) && std::isfinite(input.linear_velocity.x)
                   && std::isfinite(input.linear_velocity.y)
                   && std::isfinite(input.linear_velocity.z)
                   && std::isfinite(input.angular_velocity_z)
                   && std::isfinite(input.monotonic_time_seconds);
        }

        float shortestYawDistance(float lhs, float rhs)
        {
            return std::remainder(lhs - rhs, 2.0F * PI);
        }

        float normalizedYaw(float yaw)
        {
            return std::remainder(yaw, 2.0F * PI);
        }

        const char * stateName(ExplorerState state)
        {
            switch(state) {
                case ExplorerState::WaitingForMap:
                    return "WaitingForMap";
                case ExplorerState::Selecting:
                    return "Selecting";
                case ExplorerState::Moving:
                    return "Moving";
                case ExplorerState::AwaitingFreshObservation:
                    return "AwaitingFreshObservation";
                case ExplorerState::Rescanning:
                    return "Rescanning";
                case ExplorerState::Stopping:
                    return "Stopping";
                case ExplorerState::HoveringFailure:
                    return "HoveringFailure";
            }
            return "Unknown";
        }

        const char * pathStatusName(PathCheckStatus status)
        {
            switch(status) {
                case PathCheckStatus::Safe:
                    return "Safe";
                case PathCheckStatus::UnknownBlocked:
                    return "UnknownBlocked";
                case PathCheckStatus::OccupiedBlocked:
                    return "OccupiedBlocked";
                case PathCheckStatus::InvalidInput:
                    return "InvalidInput";
            }
            return "InvalidInput";
        }

    }// namespace

    SingleDroneExplorer::SingleDroneExplorer(
            std::shared_ptr<IExplorationStrategy> strategy, SingleDroneExplorerConfig config)
        : strategy_(std::move(strategy))
        , config_(config)
        , path_checker_(config_.body_envelope)
    {
        if(!strategy_) {
            throw std::invalid_argument("exploration strategy must not be null");
        }
        const bool finite = std::isfinite(config_.position_tolerance)
                            && std::isfinite(config_.yaw_tolerance)
                            && std::isfinite(config_.motion_timeout_seconds)
                            && std::isfinite(config_.hold_timeout_seconds)
                            && std::isfinite(config_.stopped_linear_speed_max)
                            && std::isfinite(config_.stopped_angular_speed_max)
                            && std::isfinite(config_.map_stale_timeout_seconds)
                            && std::isfinite(config_.rescan_yaw_step)
                            && std::isfinite(config_.entry_backward_margin);
        if(!finite || config_.position_tolerance <= 0.0F || config_.yaw_tolerance <= 0.0F
           || config_.motion_timeout_seconds <= 0.0 || config_.hold_timeout_seconds <= 0.0
           || config_.stopped_linear_speed_max < 0.0F
           || config_.stopped_angular_speed_max < 0.0F
           || config_.map_stale_timeout_seconds <= 0.0 || config_.rescan_yaw_step <= 0.0F
           || config_.rescan_max_steps == 0U || config_.max_rejections_per_epoch == 0U
           || config_.entry_backward_margin < 0.0F)
        {
            throw std::invalid_argument("invalid single-drone explorer config");
        }
        diagnostics_.max_debug_faces      = config_.max_debug_faces;
        diagnostics_.max_debug_candidates = config_.max_debug_candidates;
        setState(ExplorerState::WaitingForMap);
    }

    ExplorerTickResult SingleDroneExplorer::tick(const ExplorerInput & input)
    {
        tick_wall_start_ = std::chrono::steady_clock::now();
        if(!isFinite(input)) {
            diagnostics_.failure_reason = "invalid input";
            setState(ExplorerState::HoveringFailure);
            return {state_, {}};
        }

        if(input.observation_epoch > current_epoch_) {
            current_epoch_              = input.observation_epoch;
            last_observation_time_      = effectiveNow(input);
            rejected_cluster_ids_.clear();
        }
        if(!entry_pose_.has_value() && input.map != nullptr && current_epoch_ > 0U) {
            entry_pose_ = input.pose;
        }
        if(entry_pose_.has_value()) {
            const float dx = input.pose.position.x - entry_pose_->position.x;
            const float dy = input.pose.position.y - entry_pose_->position.y;
            const float progress =
                    std::cos(entry_pose_->yaw) * dx + std::sin(entry_pose_->yaw) * dy;
            max_entry_forward_progress_ = std::max(max_entry_forward_progress_, progress);
        }

        if(state_ == ExplorerState::WaitingForMap) {
            if(input.map == nullptr || current_epoch_ == 0U) {
                return {state_, {}};
            }
            setState(ExplorerState::Selecting);
            state_start_time_ = effectiveNow(input);
        }

        if(state_ == ExplorerState::Selecting) {
            if(input.map == nullptr) {
                setState(ExplorerState::WaitingForMap);
                return {state_, {}};
            }
            if(isMapStale(input)) {
                return beginStopping(input, StopDestination::Failure, "map stale while selecting");
            }
            if(!isStopped(input)) {
                return {state_, {}};
            }
            return selectAndCommand(input);
        }

        if(state_ == ExplorerState::Moving) {
            if(isMapStale(input)) {
                return beginStopping(input, StopDestination::Failure, "map stale while moving");
            }
            if(effectiveNow(input) - state_start_time_ > config_.motion_timeout_seconds) {
                return beginStopping(input, StopDestination::Selecting, "motion timeout");
            }

            if(input.map != nullptr && current_epoch_ > last_checked_path_epoch_
               && active_goal_.has_value())
            {
                const PathCheckResult result = path_checker_.checkSegment(
                        *input.map, input.pose.position, active_goal_->position);
                last_checked_path_epoch_     = current_epoch_;
                diagnostics_.path_start      = input.pose.position;
                diagnostics_.path_goal       = active_goal_->position;
                diagnostics_.path_status     = pathStatusName(result.status);
                diagnostics_.first_blocked_position = result.first_blocked_position;
                if(!result.safe()) {
                    if(active_cluster_id_.has_value()
                       && std::find(
                                  rejected_cluster_ids_.begin(), rejected_cluster_ids_.end(),
                                  *active_cluster_id_)
                                  == rejected_cluster_ids_.end())
                    {
                        rejected_cluster_ids_.push_back(*active_cluster_id_);
                    }
                    return beginStopping(
                            input, StopDestination::Selecting,
                            "remaining path blocked by fresh observation");
                }
            }

            if(active_goal_.has_value() && isReached(input.pose, *active_goal_)) {
                hold_pose_             = input.pose;
                event_epoch_           = current_epoch_;
                event_odom_stamp_ns_   = input.odom_stamp_ns;
                state_start_time_      = effectiveNow(input);
                setState(ExplorerState::AwaitingFreshObservation);
                return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
            }
            return {state_, {}};
        }

        if(state_ == ExplorerState::AwaitingFreshObservation) {
            if(isMapStale(input)) {
                return beginStopping(
                        input, StopDestination::Failure,
                        "map stale while awaiting post-arrival observation");
            }
            if(current_epoch_ > event_epoch_
               && input.observation_stamp_ns > event_odom_stamp_ns_)
            {
                completed_rescan_steps_ = 0U;
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
            }
            return {state_, {}};
        }

        if(state_ == ExplorerState::Rescanning) {
            if(isMapStale(input)) {
                return beginStopping(input, StopDestination::Failure, "map stale while rescanning");
            }
            if(effectiveNow(input) - state_start_time_ > config_.motion_timeout_seconds) {
                return beginStopping(input, StopDestination::Failure, "yaw rescan timeout");
            }

            if(!rescan_reached_ && active_goal_.has_value()
               && isReached(input.pose, *active_goal_))
            {
                rescan_reached_       = true;
                event_epoch_          = current_epoch_;
                event_odom_stamp_ns_  = input.odom_stamp_ns;
            }
            if(rescan_reached_ && current_epoch_ > event_epoch_
               && input.observation_stamp_ns > event_odom_stamp_ns_)
            {
                ++completed_rescan_steps_;
                rescan_reached_ = false;
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
            }
            return {state_, {}};
        }

        if(state_ == ExplorerState::Stopping) {
            if(isStopped(input) && hold_pose_.has_value()
               && isReached(input.pose, *hold_pose_))
            {
                if(stop_destination_ == StopDestination::Failure) {
                    setState(ExplorerState::HoveringFailure);
                } else {
                    setState(ExplorerState::Selecting);
                    state_start_time_ = effectiveNow(input);
                }
                return {state_, {}};
            }
            if(effectiveNow(input) - state_start_time_ > config_.hold_timeout_seconds) {
                diagnostics_.failure_reason = "hold confirmation timeout";
                setState(ExplorerState::HoveringFailure);
            }
            return {state_, {}};
        }

        return {state_, {}};
    }

    double SingleDroneExplorer::effectiveNow(const ExplorerInput & input) const
    {
        const double wall_elapsed = std::chrono::duration<double>(
                                            std::chrono::steady_clock::now() - tick_wall_start_)
                                            .count();
        return input.monotonic_time_seconds + wall_elapsed;
    }

    ExplorerTickResult SingleDroneExplorer::selectAndCommand(const ExplorerInput & input)
    {
        while(rejected_cluster_ids_.size() < config_.max_rejections_per_epoch) {
            GoalSelectionRequest request;
            request.pose                 = input.pose;
            request.rejected_cluster_ids = rejected_cluster_ids_;
            request.fixed_altitude = FixedAltitudeConstraint {input.pose.position.z};
            if(config_.enforce_entry_forward_half_space && entry_pose_.has_value()) {
                Point3f forward_plane_origin = entry_pose_->position;
                forward_plane_origin.x +=
                        std::cos(entry_pose_->yaw) * max_entry_forward_progress_;
                forward_plane_origin.y +=
                        std::sin(entry_pose_->yaw) * max_entry_forward_progress_;
                request.forward_half_space = ForwardHalfSpaceConstraint {
                        forward_plane_origin,
                        entry_pose_->yaw,
                        config_.entry_backward_margin,
                };
            }
            const GoalSelectionResult selection =
                    strategy_->selectGoal(request, *input.map, &diagnostics_);

            if(selection.status == GoalSelectionStatus::InvalidInput) {
                return beginStopping(input, StopDestination::Failure, "goal selection invalid");
            }
            if(selection.status != GoalSelectionStatus::Success || !selection.goal.has_value()) {
                return beginRescan(input);
            }

            Pose3D goal;
            goal.position = Point3f {
                    selection.goal->position.x,
                    selection.goal->position.y,
                    input.pose.position.z,
            };
            goal.yaw = std::atan2(
                    goal.position.y - input.pose.position.y,
                    goal.position.x - input.pose.position.x);

            const PathCheckResult path =
                    path_checker_.checkSegment(*input.map, input.pose.position, goal.position);
            diagnostics_.path_start             = input.pose.position;
            diagnostics_.path_goal              = goal.position;
            diagnostics_.path_status            = pathStatusName(path.status);
            diagnostics_.first_blocked_position = path.first_blocked_position;
            if(!path.safe()) {
                rejected_cluster_ids_.push_back(selection.goal->cluster_id);
                continue;
            }

            active_goal_               = goal;
            active_cluster_id_         = selection.goal->cluster_id;
            last_checked_path_epoch_   = current_epoch_;
            state_start_time_          = effectiveNow(input);
            diagnostics_.selected_goal = goal.position;
            setState(ExplorerState::Moving);
            return {state_, MotionCommand {MotionCommandType::MoveTo, goal}};
        }
        return beginRescan(input);
    }

    ExplorerTickResult SingleDroneExplorer::beginRescan(const ExplorerInput & input)
    {
        if(completed_rescan_steps_ >= config_.rescan_max_steps) {
            return beginStopping(input, StopDestination::Failure, "yaw rescan limit reached");
        }
        Pose3D goal = input.pose;
        goal.yaw    = normalizedYaw(input.pose.yaw + config_.rescan_yaw_step);
        active_goal_       = goal;
        active_cluster_id_.reset();
        rescan_reached_    = false;
        state_start_time_  = effectiveNow(input);
        setState(ExplorerState::Rescanning);
        return {state_, MotionCommand {MotionCommandType::MoveTo, goal}};
    }

    ExplorerTickResult SingleDroneExplorer::beginStopping(
            const ExplorerInput & input, StopDestination destination, const std::string & reason)
    {
        hold_pose_                   = input.pose;
        stop_destination_            = destination;
        state_start_time_            = effectiveNow(input);
        diagnostics_.failure_reason  = reason;
        setState(ExplorerState::Stopping);
        return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
    }

    bool SingleDroneExplorer::isReached(const Pose3D & pose, const Pose3D & goal) const
    {
        return std::hypot(
                       pose.position.x - goal.position.x,
                       pose.position.y - goal.position.y)
                       <= config_.position_tolerance
               && std::fabs(shortestYawDistance(pose.yaw, goal.yaw))
                          <= config_.yaw_tolerance;
    }

    bool SingleDroneExplorer::isStopped(const ExplorerInput & input) const
    {
        const float linear_speed = std::sqrt(
                input.linear_velocity.x * input.linear_velocity.x
                + input.linear_velocity.y * input.linear_velocity.y
                + input.linear_velocity.z * input.linear_velocity.z);
        return linear_speed <= config_.stopped_linear_speed_max
               && std::fabs(input.angular_velocity_z)
                          <= config_.stopped_angular_speed_max;
    }

    bool SingleDroneExplorer::isMapStale(const ExplorerInput & input) const
    {
        return current_epoch_ > 0U
               && effectiveNow(input) - last_observation_time_
                          > config_.map_stale_timeout_seconds;
    }

    void SingleDroneExplorer::setState(ExplorerState state)
    {
        state_                         = state;
        diagnostics_.controller_state = stateName(state_);
    }

    ExplorerState SingleDroneExplorer::state() const
    {
        return state_;
    }

    const ExplorationDiagnostics & SingleDroneExplorer::diagnostics() const
    {
        return diagnostics_;
    }

    const std::vector<FrontierClusterId> & SingleDroneExplorer::rejectedClusterIds() const
    {
        return rejected_cluster_ids_;
    }

}// namespace SwarmController
