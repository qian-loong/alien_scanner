#include "swarm_controller/FrontierExplorationStrategy.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace SwarmController {

    namespace {

        constexpr float SCORE_EPSILON = 1.0e-6F;

        struct KeyLess {
            bool operator()(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs) const
            {
                for(unsigned int axis = 0; axis < 3U; ++axis) {
                    if(lhs[axis] != rhs[axis]) {
                        return lhs[axis] < rhs[axis];
                    }
                }
                return false;
            }
        };

        struct Direction {
            int dx;
            int dy;
            int dz;
        };

        constexpr std::array<Direction, 6> DIRECTIONS {{
                Direction {-1, 0, 0},
                Direction {1, 0, 0},
                Direction {0, -1, 0},
                Direction {0, 1, 0},
                Direction {0, 0, -1},
                Direction {0, 0, 1},
        }};

        struct FrontierFace {
            octomap::OcTreeKey key;
            Direction          direction;
        };

        struct FrontierCell {
            std::vector<Direction> directions;
        };

        struct FrontierCluster {
            FrontierClusterId       id;
            std::vector<FrontierFace> faces;
            float                   area {};
        };

        struct GoalCandidate {
            Point3f            position;
            octomap::OcTreeKey key;
            FrontierClusterId  cluster_id;
            float              utility;
            float              frontier_area;
            float              travel_distance;
        };

        bool isFinite(const Point3f & point)
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        bool isFinite(const Pose3D & pose)
        {
            return isFinite(pose.position) && std::isfinite(pose.yaw);
        }

        bool isValid(const GoalSelectionRequest & request)
        {
            if(!isFinite(request.pose)) {
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
            return !request.fixed_altitude.has_value()
                   || std::isfinite(request.fixed_altitude->altitude);
        }

        bool keysEqual(const octomap::OcTreeKey & lhs, const octomap::OcTreeKey & rhs)
        {
            return !KeyLess {}(lhs, rhs) && !KeyLess {}(rhs, lhs);
        }

        bool containsKey(
                const std::vector<FrontierClusterId> & keys, const octomap::OcTreeKey & candidate)
        {
            return std::any_of(keys.begin(), keys.end(), [&candidate](const auto & key) {
                return keysEqual(key, candidate);
            });
        }

        bool neighborKey(
                const octomap::OcTreeKey & key, const Direction & direction,
                octomap::OcTreeKey & neighbor)
        {
            neighbor = key;
            const std::array<int, 3> deltas {direction.dx, direction.dy, direction.dz};
            constexpr auto max_key = std::numeric_limits<octomap::key_type>::max();
            for(unsigned int axis = 0; axis < 3U; ++axis) {
                if(deltas[axis] < 0 && key[axis] == 0U) {
                    return false;
                }
                if(deltas[axis] > 0 && key[axis] == max_key) {
                    return false;
                }
                neighbor[axis] = static_cast<octomap::key_type>(
                        static_cast<int>(key[axis]) + deltas[axis]);
            }
            return true;
        }

        bool isFree(const octomap::OcTree & tree, const octomap::OcTreeKey & key)
        {
            const octomap::OcTreeNode * node = tree.search(key);
            return node != nullptr && !tree.isNodeOccupied(node);
        }

        Point3f toPoint3f(const octomap::point3d & point)
        {
            return Point3f {point.x(), point.y(), point.z()};
        }

        float distance(const Point3f & lhs, const Point3f & rhs)
        {
            const float dx = lhs.x - rhs.x;
            const float dy = lhs.y - rhs.y;
            const float dz = lhs.z - rhs.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        float squaredDistance(const Point3f & lhs, const Point3f & rhs)
        {
            const float dx = lhs.x - rhs.x;
            const float dy = lhs.y - rhs.y;
            const float dz = lhs.z - rhs.z;
            return dx * dx + dy * dy + dz * dz;
        }

        bool keyRange(
                const octomap::OcTree & tree, const Point3f & center, const float radius,
                octomap::OcTreeKey & min_key, octomap::OcTreeKey & max_key)
        {
            return tree.coordToKeyChecked(
                           octomap::point3d(
                                   center.x - radius, center.y - radius, center.z - radius),
                           min_key)
                   && tree.coordToKeyChecked(
                           octomap::point3d(
                                   center.x + radius, center.y + radius, center.z + radius),
                           max_key);
        }

        template<typename Callback>
        void forEachKey(
                const octomap::OcTreeKey & min_key, const octomap::OcTreeKey & max_key,
                Callback callback)
        {
            for(std::uint32_t x = min_key[0]; x <= max_key[0]; ++x) {
                for(std::uint32_t y = min_key[1]; y <= max_key[1]; ++y) {
                    for(std::uint32_t z = min_key[2]; z <= max_key[2]; ++z) {
                        callback(octomap::OcTreeKey {
                                static_cast<octomap::key_type>(x),
                                static_cast<octomap::key_type>(y),
                                static_cast<octomap::key_type>(z),
                        });
                    }
                }
            }
        }

        template<typename Callback>
        void forEachKnownFreeKey(
                const octomap::OcTree & tree, const Point3f & center, const float radius,
                const octomap::OcTreeKey & window_min_key,
                const octomap::OcTreeKey & window_max_key, Callback callback)
        {
            const octomap::point3d min_coord {
                    center.x - radius, center.y - radius, center.z - radius};
            const octomap::point3d max_coord {
                    center.x + radius, center.y + radius, center.z + radius};
            const double resolution = tree.getResolution();

            for(auto it = tree.begin_leafs_bbx(min_coord, max_coord),
                     end = tree.end_leafs_bbx();
                it != end; ++it)
            {
                if(tree.isNodeOccupied(*it)) {
                    continue;
                }

                const octomap::point3d leaf_center = it.getCoordinate();
                const double          half_size   = tree.getNodeSize(it.getDepth()) * 0.5;
                octomap::OcTreeKey     leaf_min_key;
                octomap::OcTreeKey     leaf_max_key;
                if(!tree.coordToKeyChecked(
                           octomap::point3d(
                                   leaf_center.x() - half_size + resolution * 0.5,
                                   leaf_center.y() - half_size + resolution * 0.5,
                                   leaf_center.z() - half_size + resolution * 0.5),
                           leaf_min_key)
                   || !tree.coordToKeyChecked(
                           octomap::point3d(
                                   leaf_center.x() + half_size - resolution * 0.5,
                                   leaf_center.y() + half_size - resolution * 0.5,
                                   leaf_center.z() + half_size - resolution * 0.5),
                           leaf_max_key))
                {
                    continue;
                }

                bool intersects_window = true;
                for(unsigned int axis = 0; axis < 3U; ++axis) {
                    if(leaf_max_key[axis] < window_min_key[axis]
                       || leaf_min_key[axis] > window_max_key[axis])
                    {
                        intersects_window = false;
                        break;
                    }
                    leaf_min_key[axis] = std::max(leaf_min_key[axis], window_min_key[axis]);
                    leaf_max_key[axis] = std::min(leaf_max_key[axis], window_max_key[axis]);
                }
                if(intersects_window) {
                    forEachKey(leaf_min_key, leaf_max_key, callback);
                }
            }
        }

        std::vector<octomap::OcTreeKey> nearbyFreeKeys(
                const octomap::OcTree & tree, const Point3f & nominal, const float radius)
        {
            octomap::OcTreeKey min_key;
            octomap::OcTreeKey max_key;
            if(!keyRange(tree, nominal, radius, min_key, max_key)) {
                return {};
            }

            std::vector<octomap::OcTreeKey> keys;
            forEachKey(min_key, max_key, [&](const octomap::OcTreeKey & key) {
                const Point3f point = toPoint3f(tree.keyToCoord(key));
                if(distance(point, nominal) <= radius + SCORE_EPSILON && isFree(tree, key)) {
                    keys.push_back(key);
                }
            });
            std::sort(keys.begin(), keys.end(), [&tree, &nominal](const auto & lhs, const auto & rhs) {
                const float lhs_distance =
                        squaredDistance(toPoint3f(tree.keyToCoord(lhs)), nominal);
                const float rhs_distance =
                        squaredDistance(toPoint3f(tree.keyToCoord(rhs)), nominal);
                if(lhs_distance != rhs_distance) {
                    return lhs_distance < rhs_distance;
                }
                return KeyLess {}(lhs, rhs);
            });
            return keys;
        }

        float headingAlignment(const Pose3D & pose, const Point3f & candidate)
        {
            const float dx      = candidate.x - pose.position.x;
            const float dy      = candidate.y - pose.position.y;
            const float xy_norm = std::sqrt(dx * dx + dy * dy);
            if(xy_norm <= SCORE_EPSILON) {
                return 0.0F;
            }
            return (std::cos(pose.yaw) * dx + std::sin(pose.yaw) * dy) / xy_norm;
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
            const float forward =
                    std::cos(constraint->yaw) * dx + std::sin(constraint->yaw) * dy;
            return forward + constraint->backward_margin >= -SCORE_EPSILON;
        }

        const char * pathStatusName(const PathCheckStatus status)
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

        float score(
                const FrontierCluster & cluster, const Point3f & candidate, const Pose3D & pose,
                const FrontierExplorationConfig & config)
        {
            const float gain =
                    std::log1p(cluster.area / std::max(config.gain_reference_area, SCORE_EPSILON));
            return config.gain_weight * gain
                   - config.distance_weight * distance(candidate, pose.position)
                   - config.vertical_weight * std::fabs(candidate.z - pose.position.z)
                   + config.heading_weight * headingAlignment(pose, candidate);
        }

        bool betterCandidate(const GoalCandidate & candidate, const GoalCandidate & current)
        {
            if(candidate.utility != current.utility) {
                return candidate.utility > current.utility;
            }
            if(candidate.frontier_area != current.frontier_area) {
                return candidate.frontier_area > current.frontier_area;
            }
            if(candidate.travel_distance != current.travel_distance) {
                return candidate.travel_distance < current.travel_distance;
            }
            if(!keysEqual(candidate.cluster_id, current.cluster_id)) {
                return KeyLess {}(candidate.cluster_id, current.cluster_id);
            }
            return KeyLess {}(candidate.key, current.key);
        }

        void validateConfig(const FrontierExplorationConfig & config)
        {
            const std::array<float, 15> finite_values {
                    config.planning_radius,
                    config.min_goal_distance,
                    config.max_goal_distance,
                    config.goal_standoff,
                    config.goal_search_radius,
                    config.robot_radius,
                    config.robot_half_height,
                    config.safety_margin,
                    config.vertical_margin,
                    config.min_cluster_area,
                    config.max_abs_frontier_normal_z,
                    config.gain_weight,
                    config.distance_weight,
                    config.vertical_weight,
                    config.heading_weight,
            };
            if(!std::all_of(finite_values.begin(), finite_values.end(), [](const float value) {
                   return std::isfinite(value);
               })
               || !std::isfinite(config.gain_reference_area))
            {
                throw std::invalid_argument("frontier exploration config must be finite");
            }

            const float horizontal_clearance = config.robot_radius + config.safety_margin;
            if(config.min_goal_distance < 0.0F
               || config.max_goal_distance <= config.min_goal_distance
               || config.goal_standoff <= 0.0F || config.goal_search_radius < 0.0F
               || config.robot_radius < 0.0F || config.robot_half_height < 0.0F
               || config.safety_margin < 0.0F || config.vertical_margin < 0.0F
               || config.min_cluster_area <= 0.0F || config.max_abs_frontier_normal_z < 0.0F
               || config.max_abs_frontier_normal_z > 1.0F || config.gain_weight < 0.0F
               || config.distance_weight < 0.0F || config.vertical_weight < 0.0F
               || config.heading_weight < 0.0F || config.gain_reference_area <= 0.0F
               || config.planning_radius
                          < config.max_goal_distance + config.goal_standoff
                                    + std::max(horizontal_clearance, config.goal_search_radius))
            {
                throw std::invalid_argument("invalid frontier exploration config");
            }
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
                    }
                    return GoalSelectionResult {status, std::move(goal)};
                };
        if(!isValid(request)) {
            return finish(GoalSelectionStatus::InvalidInput, std::nullopt);
        }

        octomap::OcTreeKey min_key;
        octomap::OcTreeKey max_key;
        if(!keyRange(tree, request.pose.position, config_.planning_radius, min_key, max_key)) {
            return finish(GoalSelectionStatus::InvalidInput, std::nullopt);
        }

        std::map<octomap::OcTreeKey, FrontierCell, KeyLess> frontier_cells;
        std::size_t                                        raw_frontier_faces = 0U;
        bool                                               has_known_free     = false;
        forEachKnownFreeKey(
                tree, request.pose.position, config_.planning_radius, min_key, max_key,
                [&](const octomap::OcTreeKey & key) {
                    const Point3f cell_center = toPoint3f(tree.keyToCoord(key));
                    if(distance(cell_center, request.pose.position) > config_.planning_radius) {
                        return;
                    }

                    has_known_free = true;
                    for(const Direction & direction : DIRECTIONS) {
                        octomap::OcTreeKey neighbor;
                        if(!neighborKey(key, direction, neighbor)
                           || tree.search(neighbor) != nullptr)
                        {
                            continue;
                        }
                        ++raw_frontier_faces;
                        if(std::fabs(static_cast<float>(direction.dz))
                           <= config_.max_abs_frontier_normal_z)
                        {
                            frontier_cells[key].directions.push_back(direction);
                        }
                    }
                });

        if(!has_known_free) {
            return finish(GoalSelectionStatus::NoKnownFree, std::nullopt);
        }
        if(raw_frontier_faces == 0U) {
            return finish(GoalSelectionStatus::NoFrontier, std::nullopt);
        }

        std::map<octomap::OcTreeKey, bool, KeyLess> visited;
        std::vector<FrontierCluster>                 clusters;
        const float face_area =
                static_cast<float>(tree.getResolution() * tree.getResolution());
        for(const auto & [seed_key, seed_cell] : frontier_cells) {
            (void) seed_cell;
            if(visited[seed_key]) {
                continue;
            }

            FrontierCluster cluster;
            cluster.id = seed_key;
            std::queue<octomap::OcTreeKey> pending;
            pending.push(seed_key);
            visited[seed_key] = true;
            while(!pending.empty()) {
                const octomap::OcTreeKey key = pending.front();
                pending.pop();

                const FrontierCell & cell = frontier_cells.at(key);
                for(const Direction & direction : cell.directions) {
                    cluster.faces.push_back(FrontierFace {key, direction});
                }

                for(const Direction & direction : DIRECTIONS) {
                    octomap::OcTreeKey neighbor;
                    if(neighborKey(key, direction, neighbor)
                       && frontier_cells.find(neighbor) != frontier_cells.end()
                       && !visited[neighbor])
                    {
                        visited[neighbor] = true;
                        pending.push(neighbor);
                    }
                }
            }

            cluster.area = static_cast<float>(cluster.faces.size()) * face_area;
            if(cluster.area + SCORE_EPSILON >= config_.min_cluster_area) {
                if(diagnostics != nullptr) {
                    if(diagnostics->frontier_clusters.size()
                       < diagnostics->max_debug_candidates)
                    {
                        diagnostics->frontier_clusters.push_back(DebugFrontierCluster {
                                cluster.id,
                                toPoint3f(tree.keyToCoord(cluster.id)),
                                cluster.area,
                                containsKey(request.rejected_cluster_ids, cluster.id),
                        });
                    }
                    for(const FrontierFace & face : cluster.faces) {
                        if(diagnostics->frontier_faces.size()
                           >= diagnostics->max_debug_faces)
                        {
                            break;
                        }
                        diagnostics->frontier_faces.push_back(DebugFrontierFace {
                                toPoint3f(tree.keyToCoord(face.key)),
                                cluster.id,
                        });
                    }
                }
                clusters.push_back(std::move(cluster));
            }
        }

        std::map<octomap::OcTreeKey, bool, KeyLess>          body_safety_cache;
        std::map<octomap::OcTreeKey, GoalCandidate, KeyLess> unique_candidates;
        Pose3D scoring_pose = request.pose;
        if(request.forward_half_space.has_value()) {
            // 主动重扫会改变当前 yaw；部署方向约束存在时，评分朝向必须保持稳定，
            // 否则候选会跟随扫描 yaw 绕入口旋转。
            scoring_pose.yaw = request.forward_half_space->yaw;
        }
        for(const FrontierCluster & cluster : clusters) {
            if(containsKey(request.rejected_cluster_ids, cluster.id)) {
                continue;
            }

            for(const FrontierFace & face : cluster.faces) {
                const Point3f frontier = toPoint3f(tree.keyToCoord(face.key));
                const Point3f nominal {
                        frontier.x - static_cast<float>(face.direction.dx) * config_.goal_standoff,
                        frontier.y - static_cast<float>(face.direction.dy) * config_.goal_standoff,
                        frontier.z - static_cast<float>(face.direction.dz) * config_.goal_standoff,
                };

                for(const octomap::OcTreeKey & candidate_key :
                    nearbyFreeKeys(tree, nominal, config_.goal_search_radius))
                {
                    Point3f candidate = toPoint3f(tree.keyToCoord(candidate_key));
                    if(request.fixed_altitude.has_value()) {
                        candidate.z = request.fixed_altitude->altitude;
                    }
                    octomap::OcTreeKey effective_key;
                    if(!tree.coordToKeyChecked(
                               octomap::point3d(candidate.x, candidate.y, candidate.z),
                               effective_key))
                    {
                        continue;
                    }
                    const float  goal_distance = distance(candidate, request.pose.position);
                    if(goal_distance + SCORE_EPSILON < config_.min_goal_distance
                       || goal_distance > config_.max_goal_distance + SCORE_EPSILON)
                    {
                        continue;
                    }
                    if(!satisfiesForwardHalfSpace(candidate, request.forward_half_space)) {
                        if(diagnostics != nullptr) {
                            ++diagnostics->forward_filtered_count;
                        }
                        continue;
                    }

                    auto body_safety = body_safety_cache.find(effective_key);
                    if(body_safety == body_safety_cache.end()) {
                        body_safety = body_safety_cache
                                              .emplace(
                                                      effective_key,
                                                      path_checker_
                                                              .checkBody(tree, candidate)
                                                              .safe())
                                              .first;
                    }
                    if(!body_safety->second)
                    {
                        continue;
                    }
                    if(diagnostics != nullptr) {
                        ++diagnostics->raw_candidate_count;
                    }

                    const GoalCandidate goal {
                            candidate,
                            effective_key,
                            cluster.id,
                            score(cluster, candidate, scoring_pose, config_),
                            cluster.area,
                            goal_distance,
                    };
                    auto existing = unique_candidates.find(effective_key);
                    if(existing == unique_candidates.end()) {
                        unique_candidates.emplace(effective_key, goal);
                    } else if(betterCandidate(goal, existing->second)) {
                        existing->second = goal;
                    }
                }
            }
        }

        if(diagnostics != nullptr) {
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
            return finish(GoalSelectionStatus::NoSafeCandidate, std::nullopt);
        }

        std::vector<GoalCandidate> ranked_candidates;
        ranked_candidates.reserve(unique_candidates.size());
        for(const auto & [key, candidate] : unique_candidates) {
            (void) key;
            ranked_candidates.push_back(candidate);
        }
        std::sort(
                ranked_candidates.begin(), ranked_candidates.end(),
                [](const GoalCandidate & lhs, const GoalCandidate & rhs) {
                    return betterCandidate(lhs, rhs);
                });

        for(const GoalCandidate & candidate : ranked_candidates) {
            const PathCheckResult path = path_checker_.checkSegment(
                    tree, request.pose.position, candidate.position);
            if(diagnostics != nullptr) {
                ++diagnostics->segment_check_count;
                diagnostics->path_start             = request.pose.position;
                diagnostics->path_goal              = candidate.position;
                diagnostics->path_status            = pathStatusName(path.status);
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
                            candidate.cluster_id,
                            candidate.utility,
                            candidate.frontier_area,
                    });
        }
        return finish(GoalSelectionStatus::NoSafeCandidate, std::nullopt);
    }

}// namespace SwarmController
