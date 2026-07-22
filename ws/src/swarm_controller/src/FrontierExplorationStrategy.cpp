#include "swarm_controller/FrontierExplorationStrategy.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

namespace SwarmController {

    namespace {

        constexpr float SCORE_EPSILON = 1e-5F;

        struct KeyLess {
            bool operator()(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs) const
            {
                if(lhs[0] != rhs[0]) {
                    return lhs[0] < rhs[0];
                }
                if(lhs[1] != rhs[1]) {
                    return lhs[1] < rhs[1];
                }
                return lhs[2] < rhs[2];
            }
        };

        struct Candidate {
            Point3f            position {};
            octomap::OcTreeKey key {};
            float              utility {};
            float              forward_progress {};
            float              lateral_offset {};
            float              heading_change {};
        };

        bool isFinite(const Point3f & point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y)
                   && std::isfinite(point.z);
        }

        bool isFinite(const Pose3D & pose)
        {
            return isFinite(pose.position) && std::isfinite(pose.yaw);
        }

        bool containsKey(
                const std::vector<FrontierClusterId> & keys,
                const octomap::OcTreeKey & key)
        {
            return std::any_of(keys.begin(), keys.end(), [&](const FrontierClusterId & item) {
                return item == key;
            });
        }

        bool requestIsValid(const GoalSelectionRequest & request)
        {
            if(!isFinite(request.pose)) {
                return false;
            }
            if(request.preferred_travel_yaw.has_value()
               && !std::isfinite(*request.preferred_travel_yaw))
            {
                return false;
            }
            if(request.fixed_altitude.has_value()
               && !std::isfinite(request.fixed_altitude->altitude))
            {
                return false;
            }
            if(request.forward_half_space.has_value()) {
                const ForwardHalfSpaceConstraint & constraint = *request.forward_half_space;
                if(!isFinite(constraint.origin) || !std::isfinite(constraint.yaw)
                   || !std::isfinite(constraint.backward_margin)
                   || constraint.backward_margin < 0.0F)
                {
                    return false;
                }
            }
            const auto points_are_finite = [](const std::vector<Point3f> & points) {
                return std::all_of(
                        points.begin(), points.end(),
                        [](const Point3f & point) { return isFinite(point); });
            };
            return points_are_finite(request.peer_positions)
                   && points_are_finite(request.active_peer_goals)
                   && (!request.task_guidance.has_value()
                       || (request.task_guidance->valid
                           && request.task_guidance->mode
                                      == ExplorationTaskMode::Assigned
                           && request.task_guidance->task_id != 0U
                           && isFinite(request.task_guidance->target)));
        }

        void validateConfig(const FrontierExplorationConfig & config)
        {
            const std::array<float, 15> values {
                    config.forward_lookahead_min,
                    config.forward_lookahead_max,
                    config.forward_lateral_limit,
                    config.robot_radius,
                    config.robot_half_height,
                    config.safety_margin,
                    config.vertical_margin,
                    config.lateral_weight,
                    config.heading_weight,
                    config.dispersion_weight,
                    config.task_progress_weight,
                    config.task_min_progress,
                    config.task_max_heading_error,
                    config.min_peer_goal_separation,
                    static_cast<float>(config.forward_distance_samples),
            };
            if(!std::all_of(values.begin(), values.end(), [](const float value) {
                   return std::isfinite(value);
               })
               || config.forward_lookahead_min <= 0.0F
               || config.forward_lookahead_max < config.forward_lookahead_min
               || config.forward_lateral_limit < 0.0F
               || config.forward_distance_samples == 0U
               || config.forward_lateral_samples == 0U
               || config.robot_radius < 0.0F || config.robot_half_height < 0.0F
               || config.safety_margin < 0.0F || config.vertical_margin < 0.0F
               || config.lateral_weight < 0.0F || config.heading_weight < 0.0F
               || config.dispersion_weight < 0.0F
               || config.task_progress_weight < 0.0F
               || config.task_min_progress <= 0.0F
               || config.task_max_heading_error <= 0.0F
               || config.task_max_heading_error > 3.14159265358979323846F
               || config.min_peer_goal_separation < 0.0F)
            {
                throw std::invalid_argument("invalid forward local exploration config");
            }
        }

        float axisSample(
                const std::size_t index, const std::size_t count,
                const float minimum, const float maximum, const bool descending)
        {
            if(count == 1U) {
                return descending ? maximum : 0.5F * (minimum + maximum);
            }
            const float fraction = static_cast<float>(index)
                                   / static_cast<float>(count - 1U);
            return descending
                           ? maximum - fraction * (maximum - minimum)
                           : minimum + fraction * (maximum - minimum);
        }

        bool satisfiesForwardHalfSpace(
                const Point3f & candidate,
                const std::optional<ForwardHalfSpaceConstraint> & constraint)
        {
            if(!constraint.has_value()) {
                return true;
            }
            const float dx = candidate.x - constraint->origin.x;
            const float dy = candidate.y - constraint->origin.y;
            const float progress = std::cos(constraint->yaw) * dx
                                   + std::sin(constraint->yaw) * dy;
            return progress + constraint->backward_margin >= -SCORE_EPSILON;
        }

        float xyDistance(const Point3f & lhs, const Point3f & rhs)
        {
            return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
        }

        bool violatesPeerGoalSeparation(
                const Point3f & candidate, const GoalSelectionRequest & request,
                const float minimum_separation)
        {
            if(minimum_separation <= 0.0F
               || (request.task_guidance.has_value()
                   && request.task_guidance->mode == ExplorationTaskMode::Assigned))
            {
                return false;
            }
            return std::any_of(
                    request.active_peer_goals.begin(), request.active_peer_goals.end(),
                    [&](const Point3f & goal) {
                        return xyDistance(candidate, goal) + SCORE_EPSILON
                               < minimum_separation;
                    });
        }

        float peerDispersionPenalty(
                const Point3f & candidate, const GoalSelectionRequest & request)
        {
            const auto penalty = [&](const std::vector<Point3f> & points) {
                float total = 0.0F;
                for(const Point3f & point : points) {
                    total += 1.0F / (1.0F + xyDistance(candidate, point));
                }
                return total;
            };
            // Position and active goal intentionally contribute separately: avoid where a peer is
            // and where it is going.
            return penalty(request.peer_positions) + penalty(request.active_peer_goals);
        }

        bool betterCandidate(const Candidate & lhs, const Candidate & rhs)
        {
            if(std::fabs(lhs.utility - rhs.utility) > SCORE_EPSILON) {
                return lhs.utility > rhs.utility;
            }
            if(std::fabs(lhs.forward_progress - rhs.forward_progress) > SCORE_EPSILON) {
                return lhs.forward_progress > rhs.forward_progress;
            }
            if(std::fabs(std::fabs(lhs.lateral_offset) - std::fabs(rhs.lateral_offset))
               > SCORE_EPSILON)
            {
                return std::fabs(lhs.lateral_offset) < std::fabs(rhs.lateral_offset);
            }
            return KeyLess {}(lhs.key, rhs.key);
        }

        bool hasKnownFree(const octomap::OcTree & tree)
        {
            for(auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
                if(!tree.isNodeOccupied(*it)) {
                    return true;
                }
            }
            return false;
        }

    }// namespace

    FrontierExplorationStrategy::FrontierExplorationStrategy(FrontierExplorationConfig config)
        : config_(std::move(config))
        , path_checker_(BodyEnvelopeConfig {
                  config_.robot_radius,
                  config_.robot_half_height,
                  config_.safety_margin,
                  config_.vertical_margin,
                  0.5F,
          })
    {
        validateConfig(config_);
    }

    GoalSelectionResult FrontierExplorationStrategy::selectGoal(
            const GoalSelectionRequest & request, const octomap::OcTree & tree,
            ExplorationDiagnostics * diagnostics) const
    {
        const auto selection_start = std::chrono::steady_clock::now();
        if(diagnostics != nullptr) {
            diagnostics->clear();
        }
        const auto finish =
                [diagnostics, selection_start](
                        const GoalSelectionStatus status,
                        std::optional<ExplorationGoal> goal) {
                    if(diagnostics != nullptr) {
                        diagnostics->selection_elapsed_seconds =
                                std::chrono::duration<double>(
                                        std::chrono::steady_clock::now() - selection_start)
                                        .count();
                        switch(status) {
                            case GoalSelectionStatus::Success:
                                diagnostics->last_goal_status = "Success";
                                break;
                            case GoalSelectionStatus::InvalidInput:
                                diagnostics->last_goal_status = "InvalidInput";
                                break;
                            case GoalSelectionStatus::NoKnownFree:
                                diagnostics->last_goal_status = "NoKnownFree";
                                break;
                            case GoalSelectionStatus::NoFrontier:
                                diagnostics->last_goal_status = "NoFrontier";
                                break;
                            case GoalSelectionStatus::NoSafeCandidate:
                                diagnostics->last_goal_status = "NoSafeCandidate";
                                break;
                            case GoalSelectionStatus::TaskGuidanceUnavailable:
                                diagnostics->last_goal_status = "TaskGuidanceUnavailable";
                                break;
                            case GoalSelectionStatus::StartBodyConflict:
                                diagnostics->last_goal_status = "StartBodyConflict";
                                break;
                            case GoalSelectionStatus::PeerGoalConflict:
                                diagnostics->last_goal_status = "PeerGoalConflict";
                                break;
                        }
                    }
                    return GoalSelectionResult {status, std::move(goal)};
                };

        if(!requestIsValid(request)) {
            return finish(GoalSelectionStatus::InvalidInput, std::nullopt);
        }
        if(tree.size() == 0U) {
            return finish(GoalSelectionStatus::NoKnownFree, std::nullopt);
        }

        const PathCheckResult start_body = path_checker_.checkBody(tree, request.pose.position);
        if(diagnostics != nullptr) {
            diagnostics->current_body_status = pathCheckStatusName(start_body.status);
            if(!start_body.safe()) {
                diagnostics->path_start             = request.pose.position;
                diagnostics->path_goal              = request.pose.position;
                diagnostics->path_status            = pathCheckStatusName(start_body.status);
                diagnostics->first_blocked_position = start_body.first_blocked_position;
            }
        }
        if(!start_body.safe()) {
            return finish(GoalSelectionStatus::StartBodyConflict, std::nullopt);
        }
        if(!hasKnownFree(tree)) {
            return finish(GoalSelectionStatus::NoKnownFree, std::nullopt);
        }

        const float heading = request.preferred_travel_yaw.value_or(request.pose.yaw);
        const float forward_x = std::cos(heading);
        const float forward_y = std::sin(heading);
        const float left_x    = -forward_y;
        const float left_y    = forward_x;

        std::map<octomap::OcTreeKey, Candidate, KeyLess> unique_candidates;
        std::size_t pre_peer_candidate_count  = 0U;
        std::size_t post_peer_candidate_count = 0U;
        std::size_t task_filtered_count = 0U;
        const bool assigned = request.task_guidance.has_value();
        const float current_task_distance = assigned
                                                    ? xyDistance(
                                                              request.pose.position,
                                                              request.task_guidance->target)
                                                    : 0.0F;
        if(assigned && current_task_distance <= SCORE_EPSILON) {
            return finish(GoalSelectionStatus::TaskGuidanceUnavailable, std::nullopt);
        }
        const float task_direction_x = assigned
                                               ? (request.task_guidance->target.x
                                                  - request.pose.position.x)
                                                         / current_task_distance
                                               : 0.0F;
        const float task_direction_y = assigned
                                               ? (request.task_guidance->target.y
                                                  - request.pose.position.y)
                                                         / current_task_distance
                                               : 0.0F;
        const float task_heading = assigned
                                           ? std::atan2(task_direction_y, task_direction_x)
                                           : 0.0F;
        for(std::size_t distance_index = 0U;
            distance_index < config_.forward_distance_samples; ++distance_index)
        {
            const float forward = axisSample(
                    distance_index, config_.forward_distance_samples,
                    config_.forward_lookahead_min, config_.forward_lookahead_max, true);
            for(std::size_t lateral_index = 0U;
                lateral_index < config_.forward_lateral_samples; ++lateral_index)
            {
                const float lateral = axisSample(
                        lateral_index, config_.forward_lateral_samples,
                        -config_.forward_lateral_limit, config_.forward_lateral_limit, false);
                Point3f candidate {
                        request.pose.position.x + forward_x * forward + left_x * lateral,
                        request.pose.position.y + forward_y * forward + left_y * lateral,
                        request.fixed_altitude.has_value()
                                ? request.fixed_altitude->altitude
                                : request.pose.position.z,
                };
                if(!satisfiesForwardHalfSpace(candidate, request.forward_half_space)) {
                    if(diagnostics != nullptr) {
                        ++diagnostics->forward_filtered_count;
                    }
                    continue;
                }

                octomap::OcTreeKey key;
                if(!tree.coordToKeyChecked(
                           octomap::point3d(candidate.x, candidate.y, candidate.z), key)
                   || containsKey(request.rejected_cluster_ids, key))
                {
                    continue;
                }

                ++pre_peer_candidate_count;
                if(violatesPeerGoalSeparation(
                           candidate, request, config_.min_peer_goal_separation))
                {
                    if(diagnostics != nullptr) {
                        ++diagnostics->peer_goal_filtered_count;
                    }
                    continue;
                }
                ++post_peer_candidate_count;

                float task_progress = 0.0F;
                if(assigned) {
                    task_progress =
                            (candidate.x - request.pose.position.x) * task_direction_x
                            + (candidate.y - request.pose.position.y) * task_direction_y;
                    const float candidate_heading = std::atan2(
                            candidate.y - request.pose.position.y,
                            candidate.x - request.pose.position.x);
                    const float heading_error = std::fabs(std::atan2(
                            std::sin(candidate_heading - task_heading),
                            std::cos(candidate_heading - task_heading)));
                    const float required_progress = std::min(
                            config_.task_min_progress, current_task_distance);
                    if(task_progress + SCORE_EPSILON < required_progress
                       || heading_error > config_.task_max_heading_error)
                    {
                        ++task_filtered_count;
                        continue;
                    }
                }

                const PathCheckResult body = path_checker_.checkBody(tree, candidate);
                if(!body.safe()) {
                    continue;
                }
                if(diagnostics != nullptr) {
                    ++diagnostics->raw_candidate_count;
                }

                const float heading_change = std::fabs(std::atan2(lateral, forward));
                const float utility = forward
                                      - config_.lateral_weight * std::fabs(lateral)
                                      - config_.heading_weight * heading_change
                                      - config_.dispersion_weight
                                                * peerDispersionPenalty(candidate, request)
                                      + config_.task_progress_weight * task_progress;
                const Candidate draft {
                        candidate,
                        key,
                        utility,
                        forward,
                        lateral,
                        heading_change,
                };
                auto existing = unique_candidates.find(key);
                if(existing == unique_candidates.end()) {
                    unique_candidates.emplace(key, draft);
                } else if(betterCandidate(draft, existing->second)) {
                    existing->second = draft;
                }
            }
        }

        if(diagnostics != nullptr) {
            diagnostics->pre_peer_candidate_count  = pre_peer_candidate_count;
            diagnostics->post_peer_candidate_count = post_peer_candidate_count;
            diagnostics->task_filtered_count = task_filtered_count;
            diagnostics->unique_candidate_count = unique_candidates.size();
            for(const auto & [key, candidate] : unique_candidates) {
                (void) key;
                if(diagnostics->locally_safe_candidates.size()
                   >= diagnostics->max_debug_candidates)
                {
                    break;
                }
                diagnostics->locally_safe_candidates.push_back(candidate.position);
            }
        }

        if(unique_candidates.empty()) {
            if(assigned) {
                return finish(GoalSelectionStatus::TaskGuidanceUnavailable, std::nullopt);
            }
            if(pre_peer_candidate_count > 0U && post_peer_candidate_count == 0U) {
                return finish(GoalSelectionStatus::PeerGoalConflict, std::nullopt);
            }
            return finish(GoalSelectionStatus::NoSafeCandidate, std::nullopt);
        }

        std::vector<Candidate> ranked;
        ranked.reserve(unique_candidates.size());
        for(const auto & [key, candidate] : unique_candidates) {
            (void) key;
            ranked.push_back(candidate);
        }
        std::sort(ranked.begin(), ranked.end(), betterCandidate);

        for(const Candidate & candidate : ranked) {
            const PathCheckResult path = path_checker_.checkSegment(
                    tree, request.pose.position, candidate.position);
            if(diagnostics != nullptr) {
                ++diagnostics->segment_check_count;
                diagnostics->path_start             = request.pose.position;
                diagnostics->path_goal              = candidate.position;
                diagnostics->path_status            = pathCheckStatusName(path.status);
                diagnostics->first_blocked_position = path.first_blocked_position;
            }
            if(!path.safe()) {
                continue;
            }
            if(diagnostics != nullptr) {
                diagnostics->selected_goal = candidate.position;
            }
            return finish(
                    GoalSelectionStatus::Success,
                    ExplorationGoal {
                            candidate.position,
                            candidate.key,
                            candidate.utility,
                            0.0F,
                    });
        }
        return finish(
                assigned ? GoalSelectionStatus::TaskGuidanceUnavailable
                         : GoalSelectionStatus::NoSafeCandidate,
                std::nullopt);
    }

}// namespace SwarmController
