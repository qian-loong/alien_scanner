#include "swarm_controller/GlobalFrontierDetector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr float EPSILON = 1.0e-5F;
        using Direction = std::pair<int, int>;
        constexpr std::array<Direction, 8U> NEIGHBOR_DIRECTIONS {
                Direction {1, 0}, Direction {1, 1}, Direction {0, 1}, Direction {-1, 1},
                Direction {-1, 0}, Direction {-1, -1}, Direction {0, -1}, Direction {1, -1},
        };
        constexpr std::array<Direction, 4U> FRONTIER_DIRECTIONS {
                Direction {1, 0}, Direction {0, 1}, Direction {-1, 0}, Direction {0, -1},
        };

        struct ColumnSample {
            std::vector<Point3f>    free_points;
            std::array<std::vector<Point3f>, 4U> frontier_points;
            std::array<std::set<std::uint16_t>, 4U> frontier_z_keys;
            std::array<std::size_t, 4U> unknown_votes {};
            std::size_t             unknown_count {};
        };

        struct RegionAccumulator {
            FrontierColumnKey        stable_key {};
            std::vector<FrontierColumnKey> columns;
            Point3f                  representative {};
            Point3f                  direction {};
            float                    information_gain {};
            float                    area {};
            float                    span {};
            float                    consistency {};
        };

        bool finiteConfig(const GlobalFrontierDetectorConfig & config)
        {
            const bool base_valid =
                    std::isfinite(config.resolution) && config.resolution > 0.0
                   && config.column_stride_voxels > 0U && config.min_z_layers > 0U
                   && std::isfinite(config.min_z_span) && config.min_z_span > 0.0F
                   && std::isfinite(config.support_depth) && config.support_depth > 0.0F
                   && std::isfinite(config.support_width) && config.support_width > 0.0F
                   && config.min_columns > 0U && std::isfinite(config.min_area)
                   && config.min_area > 0.0F && std::isfinite(config.min_span)
                   && config.min_span > 0.0F
                   && std::isfinite(config.min_direction_consistency)
                   && config.min_direction_consistency > 0.0F
                   && config.min_direction_consistency <= 1.0F
                   && config.max_frontier_columns > 0U
                   && config.max_scanned_free_voxels > 0U
                   && config.max_support_samples_per_column > 0U
                   && config.max_columns_per_region > 0U && config.max_regions > 0U;
            if(!base_valid) {
                return false;
            }
            const double depth_samples = std::max(
                    2.0, std::ceil(config.support_depth / config.resolution));
            const double width_samples =
                    2.0 * std::ceil(0.5 * config.support_width / config.resolution) + 1.0;
            const double height_samples =
                    2.0 * std::ceil(0.5 * config.min_z_span / config.resolution) + 1.0;
            const double limit = static_cast<double>(
                    config.max_support_samples_per_column);
            return depth_samples <= limit && width_samples <= limit
                   && height_samples <= limit
                   && depth_samples * width_samples <= limit
                   && depth_samples * width_samples * height_samples <= limit;
        }

        std::int64_t columnIndex(const std::uint16_t key, const std::size_t stride)
        {
            return static_cast<std::int64_t>(key) / static_cast<std::int64_t>(stride);
        }

        FrontierColumnKey columnKey(
                const octomap::OcTreeKey & key, const std::size_t stride)
        {
            return FrontierColumnKey {
                    columnIndex(key.k[0], stride), columnIndex(key.k[1], stride)};
        }

        bool supportAt(
                const octomap::OcTree & tree, const octomap::point3d & center,
                const float inward_x, const float inward_y,
                const GlobalFrontierDetectorConfig & config)
        {
            octomap::OcTreeKey center_key;
            if(!tree.coordToKeyChecked(center, center_key)) {
                return false;
            }
            const int normal_x = static_cast<int>(std::lround(inward_x));
            const int normal_y = static_cast<int>(std::lround(inward_y));
            const int lateral_x = -normal_y;
            const int lateral_y = normal_x;
            const std::size_t depth_samples = std::max<std::size_t>(
                    2U, static_cast<std::size_t>(std::ceil(
                            config.support_depth / static_cast<float>(config.resolution))));
            const std::int64_t half_width_steps = static_cast<std::int64_t>(std::ceil(
                    0.5F * config.support_width
                    / static_cast<float>(config.resolution)));
            const std::int64_t half_height_steps = static_cast<std::int64_t>(std::ceil(
                    0.5F * config.min_z_span
                    / static_cast<float>(config.resolution)));
            const auto max_key = static_cast<std::int64_t>(
                    std::numeric_limits<octomap::key_type>::max());
            for(std::size_t depth_index = 1U; depth_index <= depth_samples; ++depth_index) {
                for(std::int64_t lateral_offset = -half_width_steps;
                    lateral_offset <= half_width_steps; ++lateral_offset)
                {
                    const std::int64_t x = static_cast<std::int64_t>(center_key[0])
                                           + normal_x * static_cast<std::int64_t>(depth_index)
                                           + lateral_x * lateral_offset;
                    const std::int64_t y = static_cast<std::int64_t>(center_key[1])
                                           + normal_y * static_cast<std::int64_t>(depth_index)
                                           + lateral_y * lateral_offset;
                    for(std::int64_t vertical_offset = -half_height_steps;
                        vertical_offset <= half_height_steps; ++vertical_offset)
                    {
                        const std::int64_t z = static_cast<std::int64_t>(center_key[2])
                                               + vertical_offset;
                        if(x < 0 || y < 0 || z < 0
                           || x > max_key || y > max_key || z > max_key)
                        {
                            return false;
                        }
                        const octomap::OcTreeKey key {
                                static_cast<octomap::key_type>(x),
                                static_cast<octomap::key_type>(y),
                                static_cast<octomap::key_type>(z)};
                        const auto * node = tree.search(key);
                        if(node == nullptr || tree.isNodeOccupied(node)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        FrontierRegion makeRegion(
                const std::vector<FrontierColumnKey> & columns,
                const std::map<FrontierColumnKey, ColumnSample> & samples,
                const float column_size, const std::size_t max_columns_per_region)
        {
            FrontierRegion result;
            result.columns = columns;
            result.stable_key = columns.front();
            Point3f min_point {
                    std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), 0.0F};
            Point3f max_point {
                    std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), 0.0F};
            Point3f sum {};
            std::array<std::size_t, 4U> votes {};
            std::size_t total_unknown = 0U;
            std::size_t point_count = 0U;
            if(result.columns.size() > max_columns_per_region) {
                result.columns.resize(max_columns_per_region);
            }
            for(const auto & key : result.columns) {
                const auto & sample = samples.at(key);
                for(const Point3f & point : sample.free_points) {
                    sum.x += point.x;
                    sum.y += point.y;
                    sum.z += point.z;
                    min_point.x = std::min(min_point.x, point.x);
                    min_point.y = std::min(min_point.y, point.y);
                    max_point.x = std::max(max_point.x, point.x);
                    max_point.y = std::max(max_point.y, point.y);
                    ++point_count;
                }
                for(std::size_t index = 0U; index < votes.size(); ++index) {
                    votes[index] += sample.unknown_votes[index];
                    total_unknown += sample.unknown_votes[index];
                }
            }
            if(point_count > 0U) {
                result.representative = {
                        sum.x / static_cast<float>(point_count),
                        sum.y / static_cast<float>(point_count),
                        sum.z / static_cast<float>(point_count),
                };
            }
            const auto dominant = static_cast<std::size_t>(std::distance(
                    votes.begin(), std::max_element(votes.begin(), votes.end())));
            result.unknown_direction = Point3f {
                    static_cast<float>(FRONTIER_DIRECTIONS[dominant].first),
                    static_cast<float>(FRONTIER_DIRECTIONS[dominant].second),
                    0.0F,
            };
            result.information_gain = static_cast<float>(total_unknown);
            result.area = static_cast<float>(result.columns.size()) * column_size * column_size;
            result.horizontal_span = std::hypot(
                    max_point.x - min_point.x, max_point.y - min_point.y);
            result.direction_consistency = total_unknown == 0U
                                                   ? 0.0F
                                                   : static_cast<float>(votes[dominant])
                                                             / static_cast<float>(total_unknown);
            return result;
        }

    }// namespace

    GlobalFrontierDetector::GlobalFrontierDetector(GlobalFrontierDetectorConfig config)
        : config_(std::move(config))
    {
        if(!finiteConfig(config_)) {
            throw std::invalid_argument("invalid global frontier detector configuration");
        }
    }

    FrontierDetectionResult GlobalFrontierDetector::detect(const octomap::OcTree & tree) const
    {
        FrontierDetectionResult result;
        if(std::fabs(tree.getResolution() - config_.resolution) > 1.0e-5) {
            result.reason = "tree resolution does not match detector resolution";
            return result;
        }

        std::map<FrontierColumnKey, ColumnSample> samples;
        for(auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
            if(tree.isNodeOccupied(*it)) {
                continue;
            }
            ++result.scanned_free_voxels;
            if(result.scanned_free_voxels > config_.max_scanned_free_voxels) {
                result.status = FrontierDetectionStatus::ResourceLimit;
                result.reason = "scanned free voxel limit exceeded";
                return result;
            }
            const auto key = it.getKey();
            const auto column = columnKey(key, config_.column_stride_voxels);
            const octomap::point3d center = tree.keyToCoord(key);
            auto [sample_it, inserted] = samples.try_emplace(column);
            if(inserted && samples.size() > config_.max_frontier_columns) {
                result.status = FrontierDetectionStatus::ResourceLimit;
                result.reason = "sampled column limit exceeded";
                return result;
            }
            ColumnSample & sample = sample_it->second;
            sample.free_points.push_back(Point3f {
                    static_cast<float>(center.x()), static_cast<float>(center.y()),
                    static_cast<float>(center.z())});
            for(std::size_t direction = 0U;
                direction < FRONTIER_DIRECTIONS.size();
                ++direction)
            {
                const auto & [dx, dy] = FRONTIER_DIRECTIONS[direction];
                octomap::OcTreeKey neighbor_key;
                if(!tree.coordToKeyChecked(
                           octomap::point3d(
                                   center.x() + static_cast<double>(dx) * config_.resolution,
                                   center.y() + static_cast<double>(dy) * config_.resolution,
                                   center.z()),
                           neighbor_key))
                {
                    continue;
                }
                if(tree.search(neighbor_key) == nullptr) {
                    ++sample.unknown_votes[direction];
                    ++sample.unknown_count;
                    sample.frontier_points[direction].push_back(Point3f {
                            static_cast<float>(center.x()),
                            static_cast<float>(center.y()),
                            static_cast<float>(center.z())});
                    sample.frontier_z_keys[direction].insert(key.k[2]);
                }
            }
        }

        std::map<FrontierColumnKey, ColumnSample> supported;
        for(const auto & [column, sample] : samples) {
            if(sample.unknown_count == 0U) {
                continue;
            }
            std::array<std::size_t, 4U> directions {0U, 1U, 2U, 3U};
            std::sort(directions.begin(), directions.end(), [&](const auto lhs, const auto rhs) {
                if(sample.unknown_votes[lhs] != sample.unknown_votes[rhs]) {
                    return sample.unknown_votes[lhs] > sample.unknown_votes[rhs];
                }
                return lhs < rhs;
            });
            bool has_vertical_candidate = false;
            bool accepted_direction = false;
            for(const std::size_t direction : directions) {
                const auto & frontier_points = sample.frontier_points[direction];
                if(frontier_points.empty()
                   || sample.frontier_z_keys[direction].size() < config_.min_z_layers)
                {
                    continue;
                }
                const auto [minimum_z, maximum_z] = std::minmax_element(
                        frontier_points.begin(), frontier_points.end(),
                        [](const Point3f & lhs, const Point3f & rhs) {
                            return lhs.z < rhs.z;
                        });
                if(maximum_z->z - minimum_z->z + EPSILON < config_.min_z_span) {
                    continue;
                }
                has_vertical_candidate = true;
                const float length = std::hypot(
                        static_cast<float>(FRONTIER_DIRECTIONS[direction].first),
                        static_cast<float>(FRONTIER_DIRECTIONS[direction].second));
                const float inward_x =
                        -static_cast<float>(FRONTIER_DIRECTIONS[direction].first) / length;
                const float inward_y =
                        -static_cast<float>(FRONTIER_DIRECTIONS[direction].second) / length;
                auto median_z = sample.frontier_z_keys[direction].begin();
                std::advance(
                        median_z,
                        static_cast<std::ptrdiff_t>(
                                sample.frontier_z_keys[direction].size() / 2U));
                octomap::OcTreeKey center_key;
                if(!tree.coordToKeyChecked(
                           octomap::point3d(
                                   frontier_points.front().x,
                                   frontier_points.front().y,
                                   frontier_points.front().z),
                           center_key))
                {
                    continue;
                }
                center_key[2] = *median_z;
                const octomap::point3d center = tree.keyToCoord(center_key);
                if(!supportAt(
                           tree, center,
                           inward_x, inward_y, config_))
                {
                    continue;
                }
                ColumnSample supported_sample = sample;
                for(std::size_t other = 0U; other < supported_sample.unknown_votes.size(); ++other) {
                    if(other != direction) {
                        supported_sample.unknown_votes[other] = 0U;
                    }
                }
                supported_sample.unknown_count = sample.unknown_votes[direction];
                supported.emplace(column, std::move(supported_sample));
                accepted_direction = true;
                break;
            }
            if(!accepted_direction) {
                if(has_vertical_candidate) {
                    ++result.support_rejected_columns;
                } else {
                    ++result.vertical_rejected_columns;
                }
                continue;
            }
            if(supported.size() > config_.max_frontier_columns) {
                result.status = FrontierDetectionStatus::ResourceLimit;
                result.reason = "supported frontier column limit exceeded";
                return result;
            }
        }
        result.raw_columns = samples.size();
        result.supported_columns = supported.size();
        if(supported.empty()) {
            result.status = FrontierDetectionStatus::Empty;
            return result;
        }

        std::set<FrontierColumnKey> remaining;
        for(const auto & [column, sample] : supported) {
            (void) sample;
            remaining.insert(column);
        }
        const float column_size = static_cast<float>(
                config_.column_stride_voxels * config_.resolution);
        while(!remaining.empty()) {
            const FrontierColumnKey seed = *remaining.begin();
            remaining.erase(remaining.begin());
            std::vector<FrontierColumnKey> component {seed};
            std::queue<FrontierColumnKey> pending;
            pending.push(seed);
            while(!pending.empty()) {
                const FrontierColumnKey current = pending.front();
                pending.pop();
                for(const auto & [dx, dy] : NEIGHBOR_DIRECTIONS) {
                    const FrontierColumnKey neighbor {
                            current.x + static_cast<std::int64_t>(dx),
                            current.y + static_cast<std::int64_t>(dy)};
                    const auto it = remaining.find(neighbor);
                    if(it == remaining.end()) {
                        continue;
                    }
                    remaining.erase(it);
                    component.push_back(neighbor);
                    pending.push(neighbor);
                }
            }
            std::sort(component.begin(), component.end());
            if(component.size() > config_.max_columns_per_region) {
                result.status = FrontierDetectionStatus::ResourceLimit;
                result.reason = "frontier region column limit exceeded";
                result.regions.clear();
                return result;
            }
            FrontierRegion region = makeRegion(
                    component, supported, column_size, config_.max_columns_per_region);
            if(region.columns.size() < config_.min_columns || region.area + EPSILON < config_.min_area
               || region.horizontal_span + EPSILON < config_.min_span
               || region.direction_consistency + EPSILON < config_.min_direction_consistency)
            {
                continue;
            }
            result.regions.push_back(std::move(region));
            if(result.regions.size() > config_.max_regions) {
                result.status = FrontierDetectionStatus::ResourceLimit;
                result.reason = "frontier region limit exceeded";
                result.regions.clear();
                return result;
            }
        }
        std::sort(
                result.regions.begin(), result.regions.end(),
                [](const FrontierRegion & lhs, const FrontierRegion & rhs) {
                    return lhs.stable_key < rhs.stable_key;
                });
        result.status = result.regions.empty() ? FrontierDetectionStatus::Empty
                                                : FrontierDetectionStatus::Accepted;
        return result;
    }

    const GlobalFrontierDetectorConfig & GlobalFrontierDetector::config() const
    {
        return config_;
    }

}// namespace SwarmController
