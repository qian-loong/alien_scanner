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
            const auto points_finite = [](const std::vector<Point3f> & points) {
                return std::all_of(points.begin(), points.end(), [](const Point3f & point) {
                    return std::isfinite(point.x) && std::isfinite(point.y)
                           && std::isfinite(point.z);
                });
            };
            return isFinite(input.pose) && std::isfinite(input.linear_velocity.x)
                   && std::isfinite(input.linear_velocity.y)
                   && std::isfinite(input.linear_velocity.z)
                   && std::isfinite(input.angular_velocity_z)
                   && std::isfinite(input.monotonic_time_seconds)
                   && points_finite(input.peer_positions)
                   && points_finite(input.active_peer_goals)
                   && (!input.task_guidance.valid
                       || (input.task_guidance.mode
                                      != ExplorationTaskMode::Assigned
                           || (input.task_guidance.task_id != 0U
                               && std::isfinite(input.task_guidance.target.x)
                               && std::isfinite(input.task_guidance.target.y)
                               && std::isfinite(input.task_guidance.target.z))));
        }

        bool samePoints(const std::vector<Point3f> & lhs, const std::vector<Point3f> & rhs)
        {
            return lhs.size() == rhs.size()
                   && std::equal(
                           lhs.begin(), lhs.end(), rhs.begin(),
                           [](const Point3f & left, const Point3f & right) {
                               return left.x == right.x && left.y == right.y
                                      && left.z == right.z;
                           });
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
                case ExplorerState::WaitingForPeer:
                    return "WaitingForPeer";
                case ExplorerState::WaitingForTask:
                    return "WaitingForTask";
                case ExplorerState::ExplorationStalled:
                    return "ExplorationStalled";
                case ExplorerState::RecoveringClearance:
                    return "RecoveringClearance";
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
                            && std::isfinite(config_.peer_retry_interval_seconds)
                            && std::isfinite(config_.task_retry_interval_seconds)
                            && std::isfinite(config_.task_rescan_step_timeout_seconds)
                            && std::isfinite(config_.travel_heading_update_distance)
                            && std::isfinite(config_.clearance_recovery_timeout_seconds)
                            && std::isfinite(config_.rescan_yaw_step)
                            && std::isfinite(config_.entry_backward_margin);
        if(!finite || config_.position_tolerance <= 0.0F || config_.yaw_tolerance <= 0.0F
           || config_.motion_timeout_seconds <= 0.0 || config_.hold_timeout_seconds <= 0.0
           || config_.stopped_linear_speed_max < 0.0F
           || config_.stopped_angular_speed_max < 0.0F
           || config_.map_stale_timeout_seconds <= 0.0
           || config_.peer_retry_interval_seconds <= 0.0
           || config_.task_retry_interval_seconds <= 0.0
           || config_.task_rescan_step_timeout_seconds <= 0.0
           || config_.travel_heading_update_distance <= 0.0F
           || config_.clearance_recovery_timeout_seconds <= 0.0
           || config_.rescan_yaw_step <= 0.0F
           || config_.rescan_max_steps == 0U || config_.task_rescan_max_steps == 0U
           || config_.max_rejections_per_epoch == 0U
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

        synchronizeTaskRescanBudget(input);
        adoptObservation(input);
        if(!entry_pose_.has_value() && input.map != nullptr && current_epoch_ > 0U) {
            entry_pose_ = input.pose;
            preferred_travel_yaw_ = input.pose.yaw;
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
            if(input.task_guidance.valid
               && input.task_guidance.mode == ExplorationTaskMode::Standby)
            {
                return beginWaitingForTask(input);
            }
            return selectAndCommand(input);
        }

        if(state_ == ExplorerState::WaitingForPeer) {
            if(isMapStale(input)) {
                return beginStopping(
                        input, StopDestination::Failure,
                        "map stale while waiting for peer goals");
            }
            if(!isStopped(input) || !hold_pose_.has_value()
               || !isReached(input.pose, *hold_pose_))
            {
                return {state_, {}};
            }
            const bool goals_changed =
                    !samePoints(input.active_peer_goals, waiting_peer_goals_);
            const bool map_changed = current_epoch_ > event_epoch_;
            const bool retry_due = effectiveNow(input) - state_start_time_
                                   >= config_.peer_retry_interval_seconds;
            if(goals_changed || map_changed || retry_due) {
                diagnostics_.failure_reason.clear();
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
            }
            return {state_, {}};
        }

        if(state_ == ExplorerState::WaitingForTask) {
            if(isMapStale(input)) {
                return beginStopping(
                        input, StopDestination::Failure,
                        "map stale while waiting for task");
            }
            if(!isStopped(input) || !hold_pose_.has_value()
               || !isReached(input.pose, *hold_pose_))
            {
                return {state_, {}};
            }
            const bool task_changed = !input.task_guidance.valid
                                      || input.task_guidance.allocator_epoch
                                                 != waiting_task_epoch_
                                      || input.task_guidance.revision
                                                 != waiting_task_revision_
                                      || input.task_guidance.task_id != waiting_task_id_;
            const bool map_changed = current_epoch_ > event_epoch_;
            const bool retry_due = effectiveNow(input) - state_start_time_
                                   >= config_.task_retry_interval_seconds;
            if(task_changed || map_changed || retry_due) {
                diagnostics_.failure_reason.clear();
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
            }
            return {state_, {}};
        }

        if(state_ == ExplorerState::ExplorationStalled) {
            if(isMapStale(input)) {
                return beginStopping(
                        input, StopDestination::Failure,
                        "map stale while exploration is stalled");
            }
            if(!isStopped(input) || !hold_pose_.has_value()
               || !isReached(input.pose, *hold_pose_))
            {
                return {state_, {}};
            }
            if(input.task_guidance.valid
               && input.task_guidance.mode == ExplorationTaskMode::Standby)
            {
                return beginWaitingForTask(input);
            }
            if(input.task_guidance.valid
               && input.task_guidance.mode == ExplorationTaskMode::Assigned)
            {
                completed_rescan_steps_ = 0U;
                diagnostics_.failure_reason.clear();
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
                return {state_, {}};
            }
            if(input.map != nullptr && input.map->size() != stalled_map_node_count_) {
                completed_rescan_steps_ = 0U;
                diagnostics_.failure_reason.clear();
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
            }
            return {state_, {}};
        }

        if(state_ == ExplorerState::RecoveringClearance) {
            if(input.map == nullptr || isMapStale(input)) {
                return beginStopping(
                        input, StopDestination::Failure,
                        "map unavailable while recovering body clearance");
            }
            if(effectiveNow(input) - state_start_time_
               > config_.clearance_recovery_timeout_seconds)
            {
                return beginStopping(
                        input, StopDestination::Failure,
                        "body clearance recovery exhausted");
            }

            const PathCheckResult body =
                    path_checker_.checkBody(*input.map, input.pose.position);
            diagnostics_.current_body_status      = pathCheckStatusName(body.status);
            diagnostics_.path_start               = input.pose.position;
            diagnostics_.path_goal                = input.pose.position;
            diagnostics_.path_status              = pathCheckStatusName(body.status);
            diagnostics_.first_blocked_position   = body.first_blocked_position;
            if(body.status == PathCheckStatus::InvalidInput) {
                return beginStopping(
                        input, StopDestination::Failure,
                        "invalid body clearance recovery input");
            }
            if(body.safe()) {
                active_goal_.reset();
                completed_rescan_steps_ = 0U;
                diagnostics_.failure_reason.clear();
                if(!isStopped(input)) {
                    return beginStopping(input, StopDestination::Selecting, "");
                }
                setState(ExplorerState::Selecting);
                state_start_time_ = effectiveNow(input);
                return {state_, {}};
            }

            hold_pose_ = input.pose;
            return {state_, {}};
        }

        if(state_ == ExplorerState::Moving) {
            const bool assigned_now = input.task_guidance.valid
                                      && input.task_guidance.mode
                                                 == ExplorationTaskMode::Assigned;
            const bool standby_now = input.task_guidance.valid
                                     && input.task_guidance.mode
                                                == ExplorationTaskMode::Standby;
            const bool task_changed = standby_now
                                      || assigned_now != (active_task_id_ != 0U)
                                      || (assigned_now
                                          && (input.task_guidance.allocator_epoch
                                                      != active_task_epoch_
                                              || input.task_guidance.revision
                                                         != active_task_revision_
                                              || input.task_guidance.task_id
                                                         != active_task_id_));
            if(task_changed) {
                return beginStopping(
                        input, StopDestination::Selecting,
                        "assigned task changed while moving");
            }
            if(isMapStale(input)) {
                return beginStopping(input, StopDestination::Failure, "map stale while moving");
            }
            if(effectiveNow(input) - state_start_time_ > config_.motion_timeout_seconds) {
                updateTravelHeading(input.pose);
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
                diagnostics_.path_status     = pathCheckStatusName(result.status);
                diagnostics_.first_blocked_position = result.first_blocked_position;
                if(!result.safe()) {
                    updateTravelHeading(input.pose);
                    const PathCheckResult body =
                            path_checker_.checkBody(*input.map, input.pose.position);
                    diagnostics_.current_body_status = pathCheckStatusName(body.status);
                    if(active_cluster_id_.has_value()
                       && std::find(
                                  rejected_cluster_ids_.begin(), rejected_cluster_ids_.end(),
                                  *active_cluster_id_)
                                  == rejected_cluster_ids_.end())
                    {
                        rejected_cluster_ids_.push_back(*active_cluster_id_);
                    }
                    return beginStopping(
                            input,
                            body.status == PathCheckStatus::OccupiedBlocked
                                    ? StopDestination::ClearanceRecovery
                                    : StopDestination::Selecting,
                            body.status == PathCheckStatus::OccupiedBlocked
                                    ? "current body clearance blocked by fresh observation"
                                    : "remaining path blocked by fresh observation");
                }
                diagnostics_.current_body_status = "Safe";
            }

            if(active_goal_.has_value() && isReached(input.pose, *active_goal_)) {
                updateTravelHeading(input.pose);
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
            if(task_rescan_active_
               && (!input.task_guidance.valid
                   || input.task_guidance.mode != ExplorationTaskMode::Assigned
                   || input.task_guidance.allocator_epoch != active_task_epoch_
                   || input.task_guidance.revision != active_task_revision_
                   || input.task_guidance.task_id != active_task_id_))
            {
                return beginStopping(
                        input, StopDestination::Selecting,
                        "assigned task changed while rescanning");
            }
            if(isMapStale(input)) {
                return beginStopping(input, StopDestination::Failure, "map stale while rescanning");
            }
            const double rescan_timeout = task_rescan_active_
                                                  ? config_.task_rescan_step_timeout_seconds
                                                  : config_.motion_timeout_seconds;
            if(effectiveNow(input) - state_start_time_ > rescan_timeout) {
                if(task_rescan_active_) {
                    ++task_rescan_steps_;
                    return beginWaitingForTask(input);
                }
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
                if(task_rescan_active_) {
                    ++task_rescan_steps_;
                } else {
                    ++completed_rescan_steps_;
                }
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
                } else if(stop_destination_ == StopDestination::ClearanceRecovery) {
                    const PathCheckResult body = input.map != nullptr
                                                         ? path_checker_.checkBody(
                                                                 *input.map,
                                                                 input.pose.position)
                                                         : PathCheckResult {
                                                                   PathCheckStatus::InvalidInput,
                                                                   std::nullopt,
                                                           };
                    return beginClearanceRecovery(
                            input, body,
                            "current body clearance blocked by fresh observation");
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
            request.preferred_travel_yaw = preferred_travel_yaw_;
            request.fixed_altitude = FixedAltitudeConstraint {input.pose.position.z};
            request.peer_positions     = input.peer_positions;
            request.active_peer_goals  = input.active_peer_goals;
            if(input.task_guidance.valid
               && input.task_guidance.mode == ExplorationTaskMode::Assigned)
            {
                request.task_guidance = input.task_guidance;
                request.preferred_travel_yaw = input.pose.yaw;
            }
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
            if(selection.status == GoalSelectionStatus::PeerGoalConflict) {
                return beginWaitingForPeer(input);
            }
            if(selection.status == GoalSelectionStatus::TaskGuidanceUnavailable) {
                if(task_rescan_steps_ >= config_.task_rescan_max_steps) {
                    return beginWaitingForTask(input);
                }
                return beginRescan(input, true);
            }
            if(selection.status == GoalSelectionStatus::StartBodyConflict) {
                const PathCheckResult body =
                        path_checker_.checkBody(*input.map, input.pose.position);
                if(body.status == PathCheckStatus::UnknownBlocked) {
                    return beginRescan(input);
                }
                return beginClearanceRecovery(
                        input, body, "current body clearance conflict while selecting");
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
            diagnostics_.path_status            = pathCheckStatusName(path.status);
            diagnostics_.first_blocked_position = path.first_blocked_position;
            if(!path.safe()) {
                const PathCheckResult body =
                        path_checker_.checkBody(*input.map, input.pose.position);
                diagnostics_.current_body_status = pathCheckStatusName(body.status);
                if(!body.safe()) {
                    return beginClearanceRecovery(
                            input, body,
                            "current body clearance conflict before command");
                }
                rejected_cluster_ids_.push_back(selection.goal->cluster_id);
                continue;
            }

            diagnostics_.current_body_status = "Safe";
            active_goal_               = goal;
            active_goal_origin_        = input.pose.position;
            active_cluster_id_         = selection.goal->cluster_id;
            active_task_epoch_ = request.task_guidance.has_value()
                                         ? request.task_guidance->allocator_epoch
                                         : 0U;
            active_task_revision_ = request.task_guidance.has_value()
                                            ? request.task_guidance->revision
                                            : 0U;
            active_task_id_ = request.task_guidance.has_value()
                                      ? request.task_guidance->task_id
                                      : 0U;
            task_rescan_steps_ = 0U;
            last_checked_path_epoch_   = current_epoch_;
            state_start_time_          = effectiveNow(input);
            diagnostics_.selected_goal = goal.position;
            setState(ExplorerState::Moving);
            return {state_, MotionCommand {MotionCommandType::MoveTo, goal}};
        }
        return beginRescan(input);
    }

    ExplorerTickResult SingleDroneExplorer::beginRescan(
            const ExplorerInput & input, const bool for_task)
    {
        if(!for_task && completed_rescan_steps_ >= config_.rescan_max_steps) {
            return beginExplorationStalled(input);
        }
        Pose3D goal = input.pose;
        goal.yaw    = nextRescanYaw(input, for_task);
        active_goal_       = goal;
        active_goal_origin_.reset();
        active_cluster_id_.reset();
        rescan_reached_    = false;
        task_rescan_active_ = for_task;
        if(for_task) {
            active_task_epoch_ = input.task_guidance.allocator_epoch;
            active_task_revision_ = input.task_guidance.revision;
            active_task_id_ = input.task_guidance.task_id;
        } else {
            active_task_epoch_ = 0U;
            active_task_revision_ = 0U;
            active_task_id_ = 0U;
        }
        state_start_time_  = effectiveNow(input);
        setState(ExplorerState::Rescanning);
        return {state_, MotionCommand {MotionCommandType::MoveTo, goal}};
    }

    ExplorerTickResult SingleDroneExplorer::beginWaitingForPeer(const ExplorerInput & input)
    {
        hold_pose_                  = input.pose;
        active_goal_.reset();
        active_goal_origin_.reset();
        active_cluster_id_.reset();
        waiting_peer_goals_         = input.active_peer_goals;
        event_epoch_                = current_epoch_;
        state_start_time_           = effectiveNow(input);
        diagnostics_.failure_reason = "all candidates blocked by peer goals";
        setState(ExplorerState::WaitingForPeer);
        return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
    }

    ExplorerTickResult SingleDroneExplorer::beginWaitingForTask(
            const ExplorerInput & input)
    {
        hold_pose_                  = input.pose;
        active_goal_.reset();
        active_goal_origin_.reset();
        active_cluster_id_.reset();
        active_task_epoch_          = 0U;
        active_task_revision_       = 0U;
        active_task_id_             = 0U;
        waiting_task_epoch_ = input.task_guidance.valid
                                      ? input.task_guidance.allocator_epoch
                                      : 0U;
        waiting_task_revision_ = input.task_guidance.valid
                                         ? input.task_guidance.revision
                                         : 0U;
        waiting_task_id_ = input.task_guidance.valid
                                   ? input.task_guidance.task_id
                                   : 0U;
        event_epoch_                = current_epoch_;
        state_start_time_           = effectiveNow(input);
        diagnostics_.failure_reason =
                input.task_guidance.valid
                        && input.task_guidance.mode == ExplorationTaskMode::Standby
                ? "global task allocator requested standby"
                : "assigned task has no locally executable guidance";
        setState(ExplorerState::WaitingForTask);
        return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
    }

    ExplorerTickResult SingleDroneExplorer::beginExplorationStalled(
            const ExplorerInput & input)
    {
        hold_pose_                  = input.pose;
        active_goal_.reset();
        active_goal_origin_.reset();
        active_cluster_id_.reset();
        event_epoch_                = current_epoch_;
        stalled_map_node_count_     = input.map != nullptr ? input.map->size() : 0U;
        state_start_time_           = effectiveNow(input);
        diagnostics_.failure_reason =
                "no safe forward candidate after yaw rescan; sensor-limited hold";
        setState(ExplorerState::ExplorationStalled);
        return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
    }

    ExplorerTickResult SingleDroneExplorer::beginClearanceRecovery(
            const ExplorerInput & input, const PathCheckResult & conflict,
            const std::string & reason)
    {
        hold_pose_                        = input.pose;
        active_goal_.reset();
        active_goal_origin_.reset();
        active_cluster_id_.reset();
        state_start_time_                 = effectiveNow(input);
        diagnostics_.failure_reason      = reason;
        diagnostics_.current_body_status = pathCheckStatusName(conflict.status);
        diagnostics_.path_start          = input.pose.position;
        diagnostics_.path_goal           = input.pose.position;
        diagnostics_.path_status         = pathCheckStatusName(conflict.status);
        diagnostics_.first_blocked_position = conflict.first_blocked_position;
        setState(ExplorerState::RecoveringClearance);
        return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
    }

    ExplorerTickResult SingleDroneExplorer::beginStopping(
            const ExplorerInput & input, StopDestination destination, const std::string & reason)
    {
        hold_pose_                   = input.pose;
        active_goal_origin_.reset();
        stop_destination_            = destination;
        state_start_time_            = effectiveNow(input);
        diagnostics_.failure_reason  = reason;
        setState(ExplorerState::Stopping);
        return {state_, MotionCommand {MotionCommandType::Hold, *hold_pose_}};
    }

    bool SingleDroneExplorer::revalidatePendingResult(
            const ExplorerInput & input, ExplorerTickResult & result)
    {
        tick_wall_start_ = std::chrono::steady_clock::now();
        if(!isFinite(input) || input.map == nullptr || result.command.type == MotionCommandType::None) {
            return false;
        }
        synchronizeTaskRescanBudget(input);
        adoptObservation(input);

        if(result.command.type == MotionCommandType::Hold) {
            hold_pose_          = input.pose;
            result.command.goal = *hold_pose_;
            result.state        = state_;
            if(state_ == ExplorerState::WaitingForPeer) {
                waiting_peer_goals_ = input.active_peer_goals;
                event_epoch_        = current_epoch_;
            } else if(state_ == ExplorerState::ExplorationStalled) {
                stalled_map_node_count_ = input.map->size();
            } else if(state_ == ExplorerState::WaitingForTask) {
                waiting_task_epoch_ = input.task_guidance.valid
                                              ? input.task_guidance.allocator_epoch
                                              : 0U;
                waiting_task_revision_ = input.task_guidance.valid
                                                 ? input.task_guidance.revision
                                                 : 0U;
                waiting_task_id_ = input.task_guidance.valid
                                           ? input.task_guidance.task_id
                                           : 0U;
                event_epoch_ = current_epoch_;
            }
            return true;
        }

        if(state_ == ExplorerState::Moving && active_goal_.has_value()) {
            if(!activeTaskMatches(input)) {
                return false;
            }
            Pose3D goal = *active_goal_;
            goal.position.z = input.pose.position.z;
            goal.yaw = std::atan2(
                    goal.position.y - input.pose.position.y,
                    goal.position.x - input.pose.position.x);
            const PathCheckResult path =
                    path_checker_.checkSegment(*input.map, input.pose.position, goal.position);
            diagnostics_.path_start             = input.pose.position;
            diagnostics_.path_goal              = goal.position;
            diagnostics_.path_status            = pathCheckStatusName(path.status);
            diagnostics_.first_blocked_position = path.first_blocked_position;
            diagnostics_.current_body_status = pathCheckStatusName(
                    path_checker_.checkBody(*input.map, input.pose.position).status);
            if(!path.safe()) {
                return false;
            }
            active_goal_             = goal;
            active_goal_origin_      = input.pose.position;
            last_checked_path_epoch_ = current_epoch_;
            state_start_time_        = effectiveNow(input);
            result.command.goal = goal;
            result.state        = state_;
            return true;
        }

        if(state_ == ExplorerState::Rescanning) {
            if(!activeTaskMatches(input)) {
                return false;
            }
            const PathCheckResult body =
                    path_checker_.checkBody(*input.map, input.pose.position);
            diagnostics_.current_body_status = pathCheckStatusName(body.status);
            if(!body.safe()) {
                return false;
            }
            Pose3D goal = input.pose;
            goal.yaw    = nextRescanYaw(input, task_rescan_active_);
            active_goal_             = goal;
            last_checked_path_epoch_ = current_epoch_;
            state_start_time_        = effectiveNow(input);
            rescan_reached_          = false;
            result.command.goal      = goal;
            result.state             = state_;
            return true;
        }

        return false;
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

    void SingleDroneExplorer::synchronizeTaskRescanBudget(const ExplorerInput & input)
    {
        const bool assigned = input.task_guidance.valid
                              && input.task_guidance.mode == ExplorationTaskMode::Assigned;
        const std::uint64_t epoch = assigned ? input.task_guidance.allocator_epoch : 0U;
        const std::uint64_t revision = assigned ? input.task_guidance.revision : 0U;
        const std::uint64_t id = assigned ? input.task_guidance.task_id : 0U;
        if(epoch != task_rescan_epoch_ || revision != task_rescan_revision_
           || id != task_rescan_id_)
        {
            task_rescan_steps_ = 0U;
            task_rescan_epoch_ = epoch;
            task_rescan_revision_ = revision;
            task_rescan_id_ = id;
        }
    }

    bool SingleDroneExplorer::activeTaskMatches(const ExplorerInput & input) const
    {
        if(active_task_id_ == 0U) {
            return !input.task_guidance.valid
                   || input.task_guidance.mode == ExplorationTaskMode::LocalFallback;
        }
        return input.task_guidance.valid
               && input.task_guidance.mode == ExplorationTaskMode::Assigned
               && input.task_guidance.allocator_epoch == active_task_epoch_
               && input.task_guidance.revision == active_task_revision_
               && input.task_guidance.task_id == active_task_id_;
    }

    float SingleDroneExplorer::nextRescanYaw(
            const ExplorerInput & input, const bool for_task) const
    {
        if(for_task && input.task_guidance.valid
           && input.task_guidance.mode == ExplorationTaskMode::Assigned)
        {
            const float target_yaw = std::atan2(
                    input.task_guidance.target.y - input.pose.position.y,
                    input.task_guidance.target.x - input.pose.position.x);
            const float error = shortestYawDistance(target_yaw, input.pose.yaw);
            if(std::fabs(error) > config_.yaw_tolerance) {
                return normalizedYaw(
                        input.pose.yaw
                        + std::clamp(
                                  error, -config_.rescan_yaw_step,
                                  config_.rescan_yaw_step));
            }
        }
        return normalizedYaw(input.pose.yaw + config_.rescan_yaw_step);
    }

    void SingleDroneExplorer::updateTravelHeading(const Pose3D & pose)
    {
        if(!active_goal_origin_.has_value()) {
            return;
        }
        const float dx = pose.position.x - active_goal_origin_->x;
        const float dy = pose.position.y - active_goal_origin_->y;
        if(std::hypot(dx, dy) >= config_.travel_heading_update_distance) {
            preferred_travel_yaw_ = std::atan2(dy, dx);
        }
        active_goal_origin_.reset();
    }

    void SingleDroneExplorer::adoptObservation(const ExplorerInput & input)
    {
        if(input.observation_epoch > current_epoch_) {
            current_epoch_         = input.observation_epoch;
            last_observation_time_ = effectiveNow(input);
            rejected_cluster_ids_.clear();
        }
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
