#include "swarm_controller/GlobalTaskAllocator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr float EPSILON = 1.0e-5F;
        constexpr std::int64_t INFEASIBLE_COST = std::numeric_limits<std::int64_t>::max() / 8;

        float xyDistance(const Point3f & lhs, const Point3f & rhs)
        {
            return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
        }

        float normalizedAngle(float value)
        {
            return std::atan2(std::sin(value), std::cos(value));
        }

        bool finitePose(const Pose3D & pose)
        {
            return std::isfinite(pose.position.x) && std::isfinite(pose.position.y)
                   && std::isfinite(pose.position.z) && std::isfinite(pose.yaw);
        }

        std::size_t overlapCount(
                const std::vector<FrontierColumnKey> & lhs,
                const std::vector<FrontierColumnKey> & rhs)
        {
            std::size_t count = 0U;
            auto left = lhs.begin();
            auto right = rhs.begin();
            while(left != lhs.end() && right != rhs.end()) {
                if(*left < *right) {
                    ++left;
                } else if(*right < *left) {
                    ++right;
                } else {
                    ++count;
                    ++left;
                    ++right;
                }
            }
            return count;
        }

        float directionSimilarity(const Point3f & lhs, const Point3f & rhs)
        {
            const float lhs_length = std::hypot(lhs.x, lhs.y);
            const float rhs_length = std::hypot(rhs.x, rhs.y);
            if(lhs_length <= EPSILON || rhs_length <= EPSILON) {
                return -1.0F;
            }
            return (lhs.x * rhs.x + lhs.y * rhs.y) / (lhs_length * rhs_length);
        }

        bool sameTarget(const Point3f & lhs, const Point3f & rhs, const float threshold)
        {
            return xyDistance(lhs, rhs) <= threshold
                   && std::fabs(lhs.z - rhs.z) <= threshold;
        }

        std::vector<std::size_t> hungarian(const std::vector<std::vector<std::int64_t>> & costs)
        {
            const std::size_t rows = costs.size();
            const std::size_t columns = rows == 0U ? 0U : costs.front().size();
            std::vector<std::int64_t> u(rows + 1U), v(columns + 1U);
            std::vector<std::size_t> p(columns + 1U), way(columns + 1U);
            for(std::size_t row = 1U; row <= rows; ++row) {
                p[0] = row;
                std::size_t column0 = 0U;
                std::vector<std::int64_t> minimum(columns + 1U, INFEASIBLE_COST);
                std::vector<bool> used(columns + 1U, false);
                do {
                    used[column0] = true;
                    const std::size_t row0 = p[column0];
                    std::int64_t delta = INFEASIBLE_COST;
                    std::size_t column1 = 0U;
                    for(std::size_t column = 1U; column <= columns; ++column) {
                        if(used[column]) {
                            continue;
                        }
                        const std::int64_t current = costs[row0 - 1U][column - 1U]
                                                     - u[row0] - v[column];
                        if(current < minimum[column]) {
                            minimum[column] = current;
                            way[column] = column0;
                        }
                        if(minimum[column] < delta) {
                            delta = minimum[column];
                            column1 = column;
                        }
                    }
                    for(std::size_t column = 0U; column <= columns; ++column) {
                        if(used[column]) {
                            u[p[column]] += delta;
                            v[column] -= delta;
                        } else {
                            minimum[column] -= delta;
                        }
                    }
                    column0 = column1;
                } while(p[column0] != 0U);
                do {
                    const std::size_t column1 = way[column0];
                    p[column0] = p[column1];
                    column0 = column1;
                } while(column0 != 0U);
            }
            std::vector<std::size_t> assignment(rows, columns);
            for(std::size_t column = 1U; column <= columns; ++column) {
                if(p[column] != 0U && p[column] <= rows) {
                    assignment[p[column] - 1U] = column - 1U;
                }
            }
            return assignment;
        }

        bool validConfig(const GlobalTaskAllocatorConfig & config)
        {
            return config.min_persistence_updates > 0U
                   && std::isfinite(config.min_persistence_seconds)
                   && config.min_persistence_seconds >= 0.0
                   && std::isfinite(config.track_match_distance)
                   && config.track_match_distance > 0.0F
                   && std::isfinite(config.min_direction_similarity)
                   && config.min_direction_similarity >= -1.0F
                   && config.min_direction_similarity <= 1.0F && config.max_tracks > 0U
                   && config.max_eligible_edges > 0U
                   && config.max_assignment_dimension > 0U
                   && std::isfinite(config.max_assignment_distance)
                   && config.max_assignment_distance > 0.0F
                   && std::isfinite(config.first_hop_distance)
                   && config.first_hop_distance > 0.0F
                   && config.first_hop_distance <= config.max_assignment_distance
                   && config.information_gain_scale >= 0
                   && config.distance_cost_scale >= 0
                   && config.heading_cost_scale >= 0
                   && config.owner_bonus >= 0
                   && config.switch_margin >= 0
                   && config.activation_min_regions >= 2U
                   && config.activation_min_drones >= 2U
                   && config.activation_updates > 0U
                   && std::isfinite(config.activation_seconds)
                   && config.activation_seconds >= 0.0
                   && std::isfinite(config.deactivation_grace_seconds)
                   && config.deactivation_grace_seconds >= 0.0
                   && std::isfinite(config.min_owner_progress)
                   && config.min_owner_progress > 0.0F
                   && std::isfinite(config.no_progress_timeout_seconds)
                   && config.no_progress_timeout_seconds > 0.0
                   && std::isfinite(config.failed_task_cooldown_seconds)
                   && config.failed_task_cooldown_seconds >= 0.0
                   && std::isfinite(config.target_update_threshold)
                   && config.target_update_threshold >= 0.0F;
        }

    }// namespace

    struct GlobalTaskAllocator::EligibleEdge {
        std::size_t drone_index {};
        std::size_t track_index {};
        std::int64_t utility {};
        Point3f first_hop {};
    };

    GlobalTaskAllocator::GlobalTaskAllocator(GlobalTaskAllocatorConfig config)
        : config_(std::move(config))
        , path_checker_(config_.body_envelope)
    {
        if(!validConfig(config_)) {
            throw std::invalid_argument("invalid global task allocator configuration");
        }
    }

    GlobalAllocationResult GlobalTaskAllocator::update(const GlobalAllocationInput & input)
    {
        if(!std::isfinite(input.monotonic_time_seconds)
           || input.monotonic_time_seconds < 0.0)
        {
            return fallbackResult(
                    input.drones, GlobalAllocationStatus::InvalidInput,
                    "invalid allocation time");
        }
        std::set<std::string> ids;
        for(const auto & drone : input.drones) {
            if(drone.id.empty() || !finitePose(drone.pose) || !ids.insert(drone.id).second) {
                return fallbackResult(
                        input.drones, GlobalAllocationStatus::InvalidInput,
                        "invalid or duplicate drone state");
            }
            auto & runtime = drone_runtime_[drone.id];
            if(drone.odom_fresh) {
                if(!runtime.entry_pose.has_value()) {
                    runtime.entry_pose = drone.pose;
                } else {
                    const float dx = drone.pose.position.x - runtime.entry_pose->position.x;
                    const float dy = drone.pose.position.y - runtime.entry_pose->position.y;
                    const float progress = std::cos(runtime.entry_pose->yaw) * dx
                                           + std::sin(runtime.entry_pose->yaw) * dy;
                    runtime.max_entry_forward_progress = std::max(
                            runtime.max_entry_forward_progress, progress);
                }
            }
        }

        if(!input.healthy) {
            coordination_mode_ = CoordinationMode::LocalFallback;
            activation_update_count_ = 0U;
            activation_start_time_ = -1.0;
            deactivation_start_time_ = -1.0;
            return fallbackResult(
                    input.drones, GlobalAllocationStatus::Accepted,
                    "allocator inputs unhealthy");
        }

        const auto previous_tracks = tracks_;
        const std::uint64_t previous_next_task_id = next_task_id_;
        const std::uint64_t previous_global_update_sequence =
                last_global_update_sequence_;
        std::string track_reason;
        if(!updateTracks(input, track_reason)) {
            tracks_ = previous_tracks;
            next_task_id_ = previous_next_task_id;
            last_global_update_sequence_ = previous_global_update_sequence;
            coordination_mode_ = CoordinationMode::LocalFallback;
            return fallbackResult(
                    input.drones, GlobalAllocationStatus::ResourceLimit, track_reason);
        }

        std::vector<DroneAllocationState> drones = input.drones;
        std::sort(drones.begin(), drones.end(), [](const auto & lhs, const auto & rhs) {
            return lhs.id < rhs.id;
        });
        std::vector<std::size_t> stable_tracks;
        for(std::size_t index = 0U; index < tracks_.size(); ++index) {
            if(tracks_[index].stable && tracks_[index].missed_updates <= config_.missed_update_grace) {
                stable_tracks.push_back(index);
            }
        }
        std::sort(stable_tracks.begin(), stable_tracks.end(), [this](const auto lhs, const auto rhs) {
            return tracks_[lhs].id < tracks_[rhs].id;
        });

        if(stable_tracks.size() + drones.size() > config_.max_assignment_dimension) {
            tracks_ = previous_tracks;
            next_task_id_ = previous_next_task_id;
            last_global_update_sequence_ = previous_global_update_sequence;
            coordination_mode_ = CoordinationMode::LocalFallback;
            return fallbackResult(
                    drones, GlobalAllocationStatus::ResourceLimit,
                    "assignment matrix dimension exceeded");
        }

        std::vector<EligibleEdge> edges;
        std::vector<std::vector<bool>> eligible(
                drones.size(), std::vector<bool>(stable_tracks.size(), false));
        std::vector<std::vector<std::int64_t>> utilities(
                drones.size(), std::vector<std::int64_t>(stable_tracks.size(), 0));
        for(std::size_t drone_index = 0U; drone_index < drones.size(); ++drone_index) {
            auto & runtime = drone_runtime_.at(drones[drone_index].id);
            for(std::size_t stable_index = 0U; stable_index < stable_tracks.size(); ++stable_index) {
                Track & track = tracks_[stable_tracks[stable_index]];
                if(runtime.failed_task_id == track.id
                   && input.monotonic_time_seconds < runtime.failed_until)
                {
                    continue;
                }
                Point3f first_hop;
                if(!pairEligible(drones[drone_index], track, runtime, first_hop)) {
                    continue;
                }
                eligible[drone_index][stable_index] = true;
                utilities[drone_index][stable_index] =
                        edgeUtility(drones[drone_index], track, runtime);
                edges.push_back(EligibleEdge {
                        drone_index, stable_index,
                        utilities[drone_index][stable_index], first_hop});
                if(edges.size() > config_.max_eligible_edges) {
                    tracks_ = previous_tracks;
                    next_task_id_ = previous_next_task_id;
                    last_global_update_sequence_ = previous_global_update_sequence;
                    coordination_mode_ = CoordinationMode::LocalFallback;
                    return fallbackResult(
                            drones, GlobalAllocationStatus::ResourceLimit,
                            "eligible edge limit exceeded");
                }
            }
        }

        const std::size_t columns = stable_tracks.size() + drones.size();
        std::vector<std::vector<std::int64_t>> costs(
                drones.size(), std::vector<std::int64_t>(columns, 0));
        std::int64_t max_utility = 0;
        for(const auto & edge : edges) {
            max_utility = std::max(max_utility, edge.utility);
        }
        for(std::size_t drone_index = 0U; drone_index < drones.size(); ++drone_index) {
            for(std::size_t track_index = 0U; track_index < stable_tracks.size(); ++track_index) {
                costs[drone_index][track_index] = eligible[drone_index][track_index]
                                                         ? max_utility
                                                                   - utilities[drone_index][track_index]
                                                         : INFEASIBLE_COST;
            }
            for(std::size_t dummy = stable_tracks.size(); dummy < columns; ++dummy) {
                costs[drone_index][dummy] = max_utility;
            }
        }
        const std::vector<std::size_t> matching = hungarian(costs);
        std::size_t cardinality = 0U;
        for(std::size_t drone_index = 0U; drone_index < matching.size(); ++drone_index) {
            if(matching[drone_index] < stable_tracks.size()
               && eligible[drone_index][matching[drone_index]])
            {
                ++cardinality;
            }
        }

        const bool activation_ready = stable_tracks.size() >= config_.activation_min_regions
                                      && cardinality >= config_.activation_min_drones;
        if(coordination_mode_ == CoordinationMode::LocalFallback) {
            if(activation_ready) {
                if(activation_update_count_ == 0U) {
                    activation_start_time_ = input.monotonic_time_seconds;
                }
                ++activation_update_count_;
                if(activation_update_count_ >= config_.activation_updates
                   && input.monotonic_time_seconds - activation_start_time_
                              >= config_.activation_seconds)
                {
                    coordination_mode_ = CoordinationMode::Coordinated;
                    deactivation_start_time_ = -1.0;
                }
            } else {
                activation_update_count_ = 0U;
                activation_start_time_ = -1.0;
            }
        } else if(!activation_ready) {
            if(deactivation_start_time_ < 0.0) {
                deactivation_start_time_ = input.monotonic_time_seconds;
            }
            if(input.monotonic_time_seconds - deactivation_start_time_
               >= config_.deactivation_grace_seconds)
            {
                coordination_mode_ = CoordinationMode::LocalFallback;
                activation_update_count_ = 0U;
                activation_start_time_ = -1.0;
            }
        } else {
            deactivation_start_time_ = -1.0;
        }

        GlobalAllocationResult result;
        result.status = GlobalAllocationStatus::Accepted;
        result.coordination_mode = coordination_mode_;
        result.eligible_edges = edges.size();
        result.matching_cardinality = cardinality;
        result.tracks = trackedRegions();
        result.assignments.reserve(drones.size());
        for(std::size_t drone_index = 0U; drone_index < drones.size(); ++drone_index) {
            ExplorationTaskMode mode = ExplorationTaskMode::LocalFallback;
            std::uint64_t task_id = 0U;
            Point3f target {};
            std::string reason = "coordination inactive";
            if(coordination_mode_ == CoordinationMode::Coordinated) {
                const std::size_t matched = matching[drone_index];
                if(matched < stable_tracks.size() && eligible[drone_index][matched]) {
                    const Track & track = tracks_[stable_tracks[matched]];
                    mode = ExplorationTaskMode::Assigned;
                    task_id = track.id;
                    target = track.region.representative;
                    reason = "unique global frontier assignment";
                } else {
                    const bool has_edge = std::any_of(
                            eligible[drone_index].begin(), eligible[drone_index].end(),
                            [](const bool value) { return value; });
                    if(has_edge) {
                        mode = ExplorationTaskMode::Standby;
                        reason = "eligible regions already owned";
                    } else {
                        reason = "no locally executable global task";
                    }
                }
            }
            DroneTaskAssignment assignment = semanticAssignment(
                    drones[drone_index].id, mode, task_id, target, reason);
            updateProgress(drones[drone_index], assignment, input.monotonic_time_seconds);
            result.assignments.push_back(std::move(assignment));
        }
        return result;
    }

    const GlobalTaskAllocatorConfig & GlobalTaskAllocator::config() const
    {
        return config_;
    }

    GlobalAllocationResult GlobalTaskAllocator::fallbackResult(
            const std::vector<DroneAllocationState> & drones,
            const GlobalAllocationStatus status, const std::string & reason)
    {
        GlobalAllocationResult result;
        result.status = status;
        result.coordination_mode = CoordinationMode::LocalFallback;
        result.reason = reason;
        result.tracks = trackedRegions();
        std::vector<DroneAllocationState> sorted = drones;
        std::sort(sorted.begin(), sorted.end(), [](const auto & lhs, const auto & rhs) {
            return lhs.id < rhs.id;
        });
        for(const auto & drone : sorted) {
            if(drone.id.empty()) {
                continue;
            }
            result.assignments.push_back(semanticAssignment(
                    drone.id, ExplorationTaskMode::LocalFallback, 0U, {}, reason));
        }
        return result;
    }

    bool GlobalTaskAllocator::updateTracks(
            const GlobalAllocationInput & input, std::string & reason)
    {
        if(input.global_update_sequence == last_global_update_sequence_) {
            return true;
        }
        std::vector<FrontierRegion> regions = input.regions;
        for(auto & region : regions) {
            std::sort(region.columns.begin(), region.columns.end());
        }
        std::sort(regions.begin(), regions.end(), [](const auto & lhs, const auto & rhs) {
            return lhs.stable_key < rhs.stable_key;
        });

        struct MatchCandidate {
            std::size_t track_index {};
            std::size_t region_index {};
            std::size_t overlap {};
            float distance {};
            float direction_similarity {};
        };
        std::vector<MatchCandidate> candidates;
        for(std::size_t track_index = 0U; track_index < tracks_.size(); ++track_index) {
            for(std::size_t region_index = 0U; region_index < regions.size(); ++region_index) {
                const std::size_t overlap = overlapCount(
                        tracks_[track_index].region.columns, regions[region_index].columns);
                const float distance = xyDistance(
                        tracks_[track_index].region.representative,
                        regions[region_index].representative);
                const float similarity = directionSimilarity(
                        tracks_[track_index].region.unknown_direction,
                        regions[region_index].unknown_direction);
                if(overlap > 0U
                   || (distance <= config_.track_match_distance
                       && similarity >= config_.min_direction_similarity))
                {
                    candidates.push_back(MatchCandidate {
                            track_index, region_index, overlap, distance, similarity});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end(), [this, &regions](const auto & lhs, const auto & rhs) {
            if(lhs.overlap != rhs.overlap) {
                return lhs.overlap > rhs.overlap;
            }
            if(std::fabs(lhs.distance - rhs.distance) > EPSILON) {
                return lhs.distance < rhs.distance;
            }
            if(std::fabs(lhs.direction_similarity - rhs.direction_similarity) > EPSILON) {
                return lhs.direction_similarity > rhs.direction_similarity;
            }
            if(tracks_[lhs.track_index].id != tracks_[rhs.track_index].id) {
                return tracks_[lhs.track_index].id < tracks_[rhs.track_index].id;
            }
            return regions[lhs.region_index].stable_key < regions[rhs.region_index].stable_key;
        });
        std::vector<bool> track_used(tracks_.size(), false);
        std::vector<bool> region_used(regions.size(), false);
        for(const auto & candidate : candidates) {
            if(track_used[candidate.track_index] || region_used[candidate.region_index]) {
                continue;
            }
            Track & track = tracks_[candidate.track_index];
            track.region = regions[candidate.region_index];
            ++track.persistence_updates;
            track.missed_updates = 0U;
            track.last_seen_time = input.monotonic_time_seconds;
            track.stable = track.persistence_updates >= config_.min_persistence_updates
                           && input.monotonic_time_seconds - track.first_seen_time
                                      >= config_.min_persistence_seconds;
            track_used[candidate.track_index] = true;
            region_used[candidate.region_index] = true;
        }
        for(std::size_t index = 0U; index < tracks_.size(); ++index) {
            if(!track_used[index]) {
                ++tracks_[index].missed_updates;
            }
        }
        for(std::size_t index = 0U; index < regions.size(); ++index) {
            if(region_used[index]) {
                continue;
            }
            if(tracks_.size() >= config_.max_tracks || next_task_id_ == 0U) {
                reason = "frontier track limit or id overflow";
                return false;
            }
            Track track;
            track.id = next_task_id_++;
            track.region = regions[index];
            track.persistence_updates = 1U;
            track.first_seen_time = input.monotonic_time_seconds;
            track.last_seen_time = input.monotonic_time_seconds;
            track.stable = config_.min_persistence_updates <= 1U
                           && config_.min_persistence_seconds <= 0.0;
            tracks_.push_back(std::move(track));
        }
        tracks_.erase(
                std::remove_if(
                        tracks_.begin(), tracks_.end(), [this](const Track & track) {
                            return track.missed_updates > config_.missed_update_grace;
                        }),
                tracks_.end());
        std::sort(tracks_.begin(), tracks_.end(), [](const Track & lhs, const Track & rhs) {
            return lhs.id < rhs.id;
        });
        last_global_update_sequence_ = input.global_update_sequence;
        return true;
    }

    std::vector<TrackedFrontierRegion> GlobalTaskAllocator::trackedRegions() const
    {
        std::vector<TrackedFrontierRegion> result;
        result.reserve(tracks_.size());
        for(const auto & track : tracks_) {
            result.push_back(TrackedFrontierRegion {
                    track.id, track.region, track.stable,
                    track.persistence_updates, track.missed_updates});
        }
        return result;
    }

    bool GlobalTaskAllocator::pairEligible(
            const DroneAllocationState & drone, const Track & track,
            DroneRuntime & runtime, Point3f & first_hop) const
    {
        if(!drone.odom_fresh || !drone.local_map_fresh || drone.local_map == nullptr) {
            return false;
        }
        const float dx = track.region.representative.x - drone.pose.position.x;
        const float dy = track.region.representative.y - drone.pose.position.y;
        const float distance = std::hypot(dx, dy);
        if(distance <= EPSILON || distance > config_.max_assignment_distance) {
            return false;
        }
        if(runtime.entry_pose.has_value()) {
            Point3f forward_plane_origin = runtime.entry_pose->position;
            forward_plane_origin.x += std::cos(runtime.entry_pose->yaw)
                                      * runtime.max_entry_forward_progress;
            forward_plane_origin.y += std::sin(runtime.entry_pose->yaw)
                                      * runtime.max_entry_forward_progress;
            const float entry_dx = track.region.representative.x - forward_plane_origin.x;
            const float entry_dy = track.region.representative.y - forward_plane_origin.y;
            const float progress = std::cos(runtime.entry_pose->yaw) * entry_dx
                                   + std::sin(runtime.entry_pose->yaw) * entry_dy;
            if(progress + config_.entry_backward_margin < -EPSILON) {
                return false;
            }
        }
        const float step = std::min(config_.first_hop_distance, distance);
        first_hop = Point3f {
                drone.pose.position.x + dx / distance * step,
                drone.pose.position.y + dy / distance * step,
                drone.pose.position.z,
        };
        if(!path_checker_.checkBody(*drone.local_map, drone.pose.position).safe()
           || !path_checker_.checkBody(*drone.local_map, first_hop).safe()
           || !path_checker_.checkSegment(
                      *drone.local_map, drone.pose.position, first_hop)
                       .safe())
        {
            return false;
        }
        return true;
    }

    std::int64_t GlobalTaskAllocator::edgeUtility(
            const DroneAllocationState & drone, const Track & track,
            const DroneRuntime & runtime) const
    {
        const float distance = xyDistance(drone.pose.position, track.region.representative);
        const float target_yaw = std::atan2(
                track.region.representative.y - drone.pose.position.y,
                track.region.representative.x - drone.pose.position.x);
        const float heading = std::fabs(normalizedAngle(target_yaw - drone.pose.yaw));
        const auto quantize = [](const double value, const std::int64_t scale) {
            const double scaled = value * static_cast<double>(scale);
            const double limited = std::clamp(
                    scaled,
                    static_cast<double>(std::numeric_limits<std::int64_t>::min() / 4),
                    static_cast<double>(std::numeric_limits<std::int64_t>::max() / 4));
            return static_cast<std::int64_t>(std::llround(limited));
        };
        std::int64_t utility = quantize(
                track.region.information_gain, config_.information_gain_scale);
        utility -= quantize(distance, config_.distance_cost_scale);
        utility -= quantize(heading, config_.heading_cost_scale);
        if(runtime.has_assignment
           && runtime.last_assignment.mode == ExplorationTaskMode::Assigned
           && runtime.last_assignment.task_id == track.id)
        {
            utility += config_.owner_bonus + config_.switch_margin;
        }
        return utility;
    }

    DroneTaskAssignment GlobalTaskAllocator::semanticAssignment(
            const std::string & drone_id, const ExplorationTaskMode mode,
            const std::uint64_t task_id, const Point3f & target,
            const std::string & reason)
    {
        DroneRuntime & runtime = drone_runtime_[drone_id];
        DroneTaskAssignment next;
        next.drone_id = drone_id;
        next.mode = mode;
        next.task_id = task_id;
        next.target = target;
        next.reason = reason;
        if(!runtime.has_assignment) {
            next.revision = 1U;
            runtime.has_assignment = true;
        } else {
            const bool changed = runtime.last_assignment.mode != mode
                                 || runtime.last_assignment.task_id != task_id
                                 || (mode == ExplorationTaskMode::Assigned
                                     && !sameTarget(
                                             runtime.last_assignment.target, target,
                                             config_.target_update_threshold));
            next.revision = runtime.last_assignment.revision + (changed ? 1U : 0U);
            if(!changed && mode == ExplorationTaskMode::Assigned) {
                next.target = runtime.last_assignment.target;
            }
        }
        runtime.last_assignment = next;
        return next;
    }

    void GlobalTaskAllocator::updateProgress(
            const DroneAllocationState & drone, const DroneTaskAssignment & assignment,
            const double now_seconds)
    {
        DroneRuntime & runtime = drone_runtime_.at(drone.id);
        if(assignment.mode != ExplorationTaskMode::Assigned) {
            runtime.progress_task_id = 0U;
            return;
        }
        if(runtime.progress_task_id != assignment.task_id) {
            runtime.progress_task_id = assignment.task_id;
            runtime.progress_start_pose = drone.pose;
            runtime.progress_target = assignment.target;
            const float dx = assignment.target.x - drone.pose.position.x;
            const float dy = assignment.target.y - drone.pose.position.y;
            const float length = std::hypot(dx, dy);
            runtime.progress_direction = length > EPSILON
                                                 ? Point3f {dx / length, dy / length, 0.0F}
                                                 : Point3f {};
            runtime.progress_start_time = now_seconds;
            return;
        }
        if(!sameTarget(
                   runtime.progress_target, assignment.target,
                   config_.target_update_threshold))
        {
            runtime.progress_start_pose = drone.pose;
            runtime.progress_target = assignment.target;
            const float target_dx = assignment.target.x - drone.pose.position.x;
            const float target_dy = assignment.target.y - drone.pose.position.y;
            const float target_length = std::hypot(target_dx, target_dy);
            runtime.progress_direction =
                    target_length > EPSILON
                            ? Point3f {target_dx / target_length, target_dy / target_length, 0.0F}
                            : Point3f {};
        }
        const float dx = drone.pose.position.x - runtime.progress_start_pose.position.x;
        const float dy = drone.pose.position.y - runtime.progress_start_pose.position.y;
        const float progress = dx * runtime.progress_direction.x
                               + dy * runtime.progress_direction.y;
        if(progress >= config_.min_owner_progress) {
            runtime.progress_start_pose = drone.pose;
            runtime.progress_target = assignment.target;
            const float target_dx = assignment.target.x - drone.pose.position.x;
            const float target_dy = assignment.target.y - drone.pose.position.y;
            const float target_length = std::hypot(target_dx, target_dy);
            runtime.progress_direction =
                    target_length > EPSILON
                            ? Point3f {target_dx / target_length, target_dy / target_length, 0.0F}
                            : Point3f {};
            runtime.progress_start_time = now_seconds;
            return;
        }
        if(now_seconds - runtime.progress_start_time > config_.no_progress_timeout_seconds) {
            runtime.failed_task_id = assignment.task_id;
            runtime.failed_until = now_seconds + config_.failed_task_cooldown_seconds;
            runtime.progress_task_id = 0U;
        }
    }

}// namespace SwarmController
