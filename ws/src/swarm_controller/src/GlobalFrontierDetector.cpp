#include "swarm_controller/GlobalFrontierDetector.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr float EPSILON = 1.0e-5F;
        using SteadyClock = std::chrono::steady_clock;
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
            std::array<std::map<octomap::key_type, octomap::OcTreeKey>, 4U>
                    frontier_anchor_keys;
            std::array<std::size_t, 4U> unknown_votes {};
            std::size_t             unknown_count {};
        };

        struct SupportCheckResult {
            FrontierSupportFailure failure {FrontierSupportFailure::None};
            std::uint64_t  attempted_samples {};
            std::size_t    depth_index {};
            std::size_t    depth_samples {};
            bool           has_failure_position {};
            Point3f        failure_position {};

            bool passed() const
            {
                return failure == FrontierSupportFailure::None;
            }
        };

        struct TraceCollector {
            FrontierDetectionTrace * trace {};
            std::size_t support_samples {};
            std::size_t geometry_elements {};
            const GlobalFrontierDetectorConfig * config {};

            bool enabled() const
            {
                return trace != nullptr;
            }

            void addSupportSample(
                    FrontierSupportAttemptTrace & attempt, const Point3f & position,
                    const FrontierTraceSampleState state)
            {
                if(!enabled()) {
                    return;
                }
                if(support_samples >= config->max_trace_support_samples) {
                    trace->truncated = true;
                    return;
                }
                attempt.samples.push_back(
                        FrontierSupportSampleTrace {position, state});
                ++support_samples;
            }

            template<typename T>
            bool copyGeometry(
                    std::vector<T> & target, const std::vector<T> & source)
            {
                if(!enabled()) {
                    return false;
                }
                if(source.empty()) {
                    return true;
                }
                const std::size_t available =
                        geometry_elements >= config->max_trace_geometry_elements
                                ? 0U
                                : config->max_trace_geometry_elements
                                          - geometry_elements;
                const std::size_t count = std::min(available, source.size());
                target.assign(source.begin(), source.begin() + count);
                geometry_elements += count;
                if(count != source.size()) {
                    trace->truncated = true;
                }
                return count == source.size();
            }

            bool reserveGeometryElement()
            {
                if(!enabled()) {
                    return false;
                }
                if(geometry_elements >= config->max_trace_geometry_elements) {
                    trace->truncated = true;
                    return false;
                }
                ++geometry_elements;
                return true;
            }
        };

        void saturatingAdd(std::uint64_t & target, const std::uint64_t value)
        {
            const auto maximum = std::numeric_limits<std::uint64_t>::max();
            target = value > maximum - target ? maximum : target + value;
        }

        void saturatingIncrement(std::uint64_t & target)
        {
            saturatingAdd(target, 1U);
        }

        std::size_t sizeAlias(const std::uint64_t value)
        {
            const auto maximum = std::numeric_limits<std::size_t>::max();
            return value > static_cast<std::uint64_t>(maximum)
                           ? maximum
                           : static_cast<std::size_t>(value);
        }

        double secondsBetween(
                const SteadyClock::time_point start,
                const SteadyClock::time_point end)
        {
            return std::chrono::duration<double>(end - start).count();
        }

        std::optional<std::size_t> checkedSupportDepthSamples(
                const GlobalFrontierDetectorConfig & config)
        {
            if(!std::isfinite(config.support_depth) || config.support_depth <= 0.0
               || !std::isfinite(config.resolution) || config.resolution <= 0.0
               || config.max_support_samples_per_column == 0U)
            {
                return std::nullopt;
            }
            const long double depth_samples = std::ceil(
                    static_cast<long double>(config.support_depth)
                    / static_cast<long double>(config.resolution));
            const long double configured_limit = static_cast<long double>(
                    config.max_support_samples_per_column);
            const long double size_limit = static_cast<long double>(
                    std::numeric_limits<std::size_t>::max());
            if(!std::isfinite(depth_samples) || depth_samples < 1.0L
               || depth_samples > configured_limit || depth_samples >= size_limit)
            {
                return std::nullopt;
            }
            return static_cast<std::size_t>(depth_samples);
        }

        bool finiteConfig(const GlobalFrontierDetectorConfig & config)
        {
            const bool base_valid =
                    std::isfinite(config.resolution) && config.resolution > 0.0
                   && config.column_stride_voxels > 0U && config.min_z_layers > 0U
                   && std::isfinite(config.min_z_span) && config.min_z_span > 0.0F
                   && std::isfinite(config.support_depth) && config.support_depth > 0.0
                   && config.min_columns > 0U && std::isfinite(config.min_area)
                   && config.min_area > 0.0F && std::isfinite(config.min_span)
                   && config.min_span > 0.0F
                   && std::isfinite(config.min_direction_consistency)
                   && config.min_direction_consistency > 0.0F
                   && config.min_direction_consistency <= 1.0F
                   && config.max_frontier_columns > 0U
                   && config.max_scanned_free_voxels > 0U
                   && config.max_support_samples_per_column > 0U
                   && config.max_columns_per_region > 0U && config.max_regions > 0U
                    && config.max_trace_candidates > 0U
                    && config.max_trace_support_samples > 0U
                    && config.max_trace_components > 0U
                    && config.max_trace_geometry_elements > 0U;
            return base_valid;
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

        SupportCheckResult supportAt(
                const octomap::OcTree & tree, const octomap::OcTreeKey & anchor_key,
                const int inward_x, const int inward_y,
                const std::size_t depth_samples,
                TraceCollector * trace_collector,
                FrontierSupportAttemptTrace * attempt_trace)
        {
            SupportCheckResult result;
            const auto max_key = static_cast<std::int64_t>(
                    std::numeric_limits<octomap::key_type>::max());
            result.depth_samples = depth_samples;
            for(std::size_t depth_index = 1U; depth_index <= depth_samples; ++depth_index) {
                saturatingIncrement(result.attempted_samples);
                result.depth_index = depth_index;
                result.has_failure_position = true;
                const std::int64_t x = static_cast<std::int64_t>(anchor_key[0])
                                       + inward_x * static_cast<std::int64_t>(depth_index);
                const std::int64_t y = static_cast<std::int64_t>(anchor_key[1])
                                       + inward_y * static_cast<std::int64_t>(depth_index);
                const std::int64_t z = static_cast<std::int64_t>(anchor_key[2]);
                if(x < 0 || y < 0 || z < 0
                   || x > max_key || y > max_key || z > max_key)
                {
                    const Point3f failure_position {
                            static_cast<float>(tree.keyToCoord(
                                    std::clamp<std::int64_t>(x, 0, max_key))),
                            static_cast<float>(tree.keyToCoord(
                                    std::clamp<std::int64_t>(y, 0, max_key))),
                            static_cast<float>(tree.keyToCoord(
                                    std::clamp<std::int64_t>(z, 0, max_key)))};
                    result.failure_position = failure_position;
                    if(trace_collector != nullptr && attempt_trace != nullptr) {
                        trace_collector->addSupportSample(
                                *attempt_trace, failure_position,
                                FrontierTraceSampleState::OutOfBounds);
                    }
                    result.failure = FrontierSupportFailure::OutOfBounds;
                    return result;
                }
                const octomap::OcTreeKey key {
                        static_cast<octomap::key_type>(x),
                        static_cast<octomap::key_type>(y),
                        static_cast<octomap::key_type>(z)};
                const auto sample = tree.keyToCoord(key);
                const Point3f sample_position {sample.x(), sample.y(), sample.z()};
                const auto * node = tree.search(key);
                if(node == nullptr) {
                    result.failure_position = sample_position;
                    if(trace_collector != nullptr && attempt_trace != nullptr) {
                        trace_collector->addSupportSample(
                                *attempt_trace, sample_position,
                                FrontierTraceSampleState::Unknown);
                    }
                    result.failure = FrontierSupportFailure::Unknown;
                    return result;
                }
                if(tree.isNodeOccupied(node)) {
                    result.failure_position = sample_position;
                    if(trace_collector != nullptr && attempt_trace != nullptr) {
                        trace_collector->addSupportSample(
                                *attempt_trace, sample_position,
                                FrontierTraceSampleState::Occupied);
                    }
                    result.failure = FrontierSupportFailure::Occupied;
                    return result;
                }
                if(trace_collector != nullptr && attempt_trace != nullptr) {
                    trace_collector->addSupportSample(
                            *attempt_trace, sample_position,
                            FrontierTraceSampleState::Free);
                }
            }
            result.has_failure_position = false;
            return result;
        }

        void recordSupportFailure(
                FrontierDetectionDiagnostics & diagnostics,
                const SupportCheckResult & failure)
        {
            switch(failure.failure) {
                case FrontierSupportFailure::Unknown:
                    saturatingIncrement(diagnostics.support_rejected_unknown);
                    break;
                case FrontierSupportFailure::Occupied:
                    saturatingIncrement(diagnostics.support_rejected_occupied);
                    break;
                case FrontierSupportFailure::OutOfBounds:
                    saturatingIncrement(diagnostics.support_rejected_out_of_bounds);
                    break;
                case FrontierSupportFailure::None:
                    return;
            }
            if(!failure.has_failure_position || failure.depth_samples == 0U) {
                saturatingIncrement(diagnostics.support_failure_position_unavailable);
                return;
            }
            const std::size_t depth_bucket = std::min<std::size_t>(
                    FrontierDetectionDiagnostics::SUPPORT_DEPTH_BUCKETS - 1U,
                    (failure.depth_index - 1U)
                            * FrontierDetectionDiagnostics::SUPPORT_DEPTH_BUCKETS
                            / failure.depth_samples);
            saturatingIncrement(diagnostics.support_failure_depth_octiles[depth_bucket]);
        }

        std::size_t componentSizeBucket(const std::size_t size)
        {
            if(size <= 1U) {
                return 0U;
            }
            if(size <= 3U) {
                return 1U;
            }
            if(size <= 7U) {
                return 2U;
            }
            if(size <= 11U) {
                return 3U;
            }
            if(size <= 23U) {
                return 4U;
            }
            return 5U;
        }

        struct RegionComputation {
            FrontierRegion region;
            std::array<std::size_t, 4U> direction_votes {};
            Point3f xy_minimum {};
            Point3f xy_maximum {};
        };

        RegionComputation makeRegion(
                const std::vector<FrontierColumnKey> & columns,
                const std::map<FrontierColumnKey, ColumnSample> & samples,
                const float column_size, const std::size_t max_columns_per_region)
        {
            RegionComputation computation;
            FrontierRegion & result = computation.region;
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
                computation.xy_minimum = min_point;
                computation.xy_maximum = max_point;
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
            computation.direction_votes = votes;
            return computation;
        }

    }// namespace

    GlobalFrontierDetector::GlobalFrontierDetector(GlobalFrontierDetectorConfig config)
        : config_(std::move(config))
    {
        const auto support_depth_samples = checkedSupportDepthSamples(config_);
        if(!finiteConfig(config_) || !support_depth_samples.has_value()) {
            throw std::invalid_argument("invalid global frontier detector configuration");
        }
        support_depth_samples_ = *support_depth_samples;
    }

    FrontierDetectionResult GlobalFrontierDetector::detect(const octomap::OcTree & tree) const
    {
        return detectImpl(tree, nullptr);
    }

    TracedFrontierDetectionResult GlobalFrontierDetector::detectWithTrace(
            const octomap::OcTree & tree) const
    {
        TracedFrontierDetectionResult traced;
        traced.result = detectImpl(tree, &traced.trace);
        return traced;
    }

    FrontierDetectionResult GlobalFrontierDetector::detectImpl(
            const octomap::OcTree & tree, FrontierDetectionTrace * trace) const
    {
        FrontierDetectionResult result;
        TraceCollector trace_collector {trace, 0U, 0U, &config_};
        const auto total_start = config_.collect_stage_timings
                                         ? SteadyClock::now()
                                         : SteadyClock::time_point {};
        auto finish = [&](const FrontierDetectionStatus status, std::string reason,
                          const bool complete) {
            result.status = status;
            result.reason = std::move(reason);
            result.diagnostics.complete = complete;
            result.raw_columns = sizeAlias(result.diagnostics.sampled_free_columns);
            result.scanned_free_voxels = sizeAlias(result.diagnostics.scanned_free_voxels);
            result.supported_columns = sizeAlias(result.diagnostics.support_passed_columns);
            result.vertical_rejected_columns =
                    sizeAlias(result.diagnostics.vertical_rejected_columns);
            std::uint64_t support_rejected = result.diagnostics.support_rejected_unknown;
            saturatingAdd(
                    support_rejected, result.diagnostics.support_rejected_occupied);
            saturatingAdd(
                    support_rejected, result.diagnostics.support_rejected_out_of_bounds);
            result.support_rejected_columns = sizeAlias(support_rejected);
            if(config_.collect_stage_timings) {
                result.diagnostics.timings.total_seconds =
                        secondsBetween(total_start, SteadyClock::now());
            }
            return result;
        };
        if(std::fabs(tree.getResolution() - config_.resolution) > 1.0e-5) {
            return finish(
                    FrontierDetectionStatus::Invalid,
                    "tree resolution does not match detector resolution", false);
        }

        std::map<FrontierColumnKey, ColumnSample> samples;
        const auto leaf_scan_start = config_.collect_stage_timings
                                             ? SteadyClock::now()
                                             : SteadyClock::time_point {};
        for(auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
            if(tree.isNodeOccupied(*it)) {
                continue;
            }
            saturatingIncrement(result.diagnostics.scanned_free_voxels);
            if(result.diagnostics.scanned_free_voxels > config_.max_scanned_free_voxels) {
                if(config_.collect_stage_timings) {
                    result.diagnostics.timings.leaf_scan_seconds =
                            secondsBetween(leaf_scan_start, SteadyClock::now());
                }
                result.diagnostics.sampled_free_columns = samples.size();
                return finish(
                        FrontierDetectionStatus::ResourceLimit,
                        "scanned free voxel limit exceeded", false);
            }
            const auto key = it.getKey();
            const auto column = columnKey(key, config_.column_stride_voxels);
            const octomap::point3d center = tree.keyToCoord(key);
            auto [sample_it, inserted] = samples.try_emplace(column);
            result.diagnostics.sampled_free_columns = samples.size();
            if(inserted && samples.size() > config_.max_frontier_columns) {
                if(config_.collect_stage_timings) {
                    result.diagnostics.timings.leaf_scan_seconds =
                            secondsBetween(leaf_scan_start, SteadyClock::now());
                }
                return finish(
                        FrontierDetectionStatus::ResourceLimit,
                        "sampled column limit exceeded", false);
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
                    auto & anchors = sample.frontier_anchor_keys[direction];
                    const auto [anchor, inserted] = anchors.try_emplace(key.k[2], key);
                    if(!inserted
                       && (key.k[0] < anchor->second.k[0]
                           || (key.k[0] == anchor->second.k[0]
                               && key.k[1] < anchor->second.k[1])))
                    {
                        anchor->second = key;
                    }
                }
            }
        }
        if(config_.collect_stage_timings) {
            result.diagnostics.timings.leaf_scan_seconds =
                    secondsBetween(leaf_scan_start, SteadyClock::now());
        }

        std::map<FrontierColumnKey, ColumnSample> supported;
        for(const auto & [column, sample] : samples) {
            if(sample.unknown_count == 0U) {
                continue;
            }
            saturatingIncrement(result.diagnostics.unknown_neighbor_candidate_columns);
            FrontierCandidateTrace * candidate_trace = nullptr;
            if(trace != nullptr) {
                if(trace->candidates.size() < config_.max_trace_candidates) {
                    trace->candidates.push_back(FrontierCandidateTrace {});
                    candidate_trace = &trace->candidates.back();
                    candidate_trace->key = column;
                    trace_collector.copyGeometry(
                            candidate_trace->column_points, sample.free_points);
                    if(!sample.free_points.empty()) {
                        Point3f sum {};
                        for(const Point3f & free_point : sample.free_points) {
                            sum.x += free_point.x;
                            sum.y += free_point.y;
                            sum.z += free_point.z;
                        }
                        const float count = static_cast<float>(sample.free_points.size());
                        candidate_trace->center = Point3f {
                                sum.x / count, sum.y / count, sum.z / count};
                    }
                } else {
                    trace->truncated = true;
                }
            }
            auto vertical_segment_start = config_.collect_stage_timings
                                                  ? SteadyClock::now()
                                                  : SteadyClock::time_point {};
            std::array<std::size_t, 4U> directions {0U, 1U, 2U, 3U};
            std::sort(directions.begin(), directions.end(), [&](const auto lhs, const auto rhs) {
                if(sample.unknown_votes[lhs] != sample.unknown_votes[rhs]) {
                    return sample.unknown_votes[lhs] > sample.unknown_votes[rhs];
                }
                return lhs < rhs;
            });
            bool has_vertical_candidate = false;
            bool accepted_direction = false;
            std::optional<SupportCheckResult> first_support_failure;
            for(const std::size_t direction : directions) {
                const auto & frontier_points = sample.frontier_points[direction];
                const auto & frontier_anchor_keys =
                        sample.frontier_anchor_keys[direction];
                if(frontier_points.empty()
                   || frontier_anchor_keys.size() < config_.min_z_layers)
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
                const int inward_x = -FRONTIER_DIRECTIONS[direction].first;
                const int inward_y = -FRONTIER_DIRECTIONS[direction].second;
                const Point3f unknown_direction {
                        static_cast<float>(FRONTIER_DIRECTIONS[direction].first),
                        static_cast<float>(FRONTIER_DIRECTIONS[direction].second), 0.0F};
                FrontierSupportAttemptTrace * attempt_trace = nullptr;
                if(candidate_trace != nullptr) {
                    candidate_trace->vertical_passed = true;
                    candidate_trace->support_attempts.push_back(
                            FrontierSupportAttemptTrace {});
                    attempt_trace = &candidate_trace->support_attempts.back();
                    attempt_trace->unknown_direction = unknown_direction;
                    attempt_trace->inward_direction =
                            Point3f {static_cast<float>(inward_x),
                                     static_cast<float>(inward_y), 0.0F};
                    trace_collector.copyGeometry(
                            attempt_trace->column_points, frontier_points);
                }
                auto anchor = frontier_anchor_keys.begin();
                std::advance(
                        anchor,
                        static_cast<std::ptrdiff_t>(
                                frontier_anchor_keys.size() / 2U));
                const octomap::OcTreeKey anchor_key = anchor->second;
                const octomap::point3d anchor_position = tree.keyToCoord(anchor_key);
                SupportCheckResult support_result;
                if(attempt_trace != nullptr) {
                    attempt_trace->anchor = Point3f {
                            anchor_position.x(), anchor_position.y(), anchor_position.z()};
                }
                if(config_.collect_stage_timings) {
                    const auto support_start = SteadyClock::now();
                    result.diagnostics.timings.vertical_seconds +=
                            secondsBetween(vertical_segment_start, support_start);
                    support_result = supportAt(
                            tree, anchor_key, inward_x, inward_y,
                            support_depth_samples_,
                            trace == nullptr ? nullptr : &trace_collector,
                            attempt_trace);
                    const auto support_end = SteadyClock::now();
                    result.diagnostics.timings.support_seconds +=
                            secondsBetween(support_start, support_end);
                    vertical_segment_start = support_end;
                } else {
                    support_result = supportAt(
                            tree, anchor_key, inward_x, inward_y,
                            support_depth_samples_,
                            trace == nullptr ? nullptr : &trace_collector,
                            attempt_trace);
                }
                if(attempt_trace != nullptr) {
                    attempt_trace->failure = support_result.failure;
                    attempt_trace->has_first_failure_position =
                            support_result.has_failure_position;
                    attempt_trace->first_failure_position =
                            support_result.failure_position;
                }
                saturatingAdd(
                        result.diagnostics.support_samples_attempted,
                        support_result.attempted_samples);
                if(!support_result.passed()) {
                    if(!first_support_failure.has_value()) {
                        first_support_failure = support_result;
                    }
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
                if(candidate_trace != nullptr) {
                    candidate_trace->support_passed = true;
                }
                if(attempt_trace != nullptr) {
                    attempt_trace->selected = true;
                }
                accepted_direction = true;
                break;
            }
            if(config_.collect_stage_timings) {
                result.diagnostics.timings.vertical_seconds +=
                        secondsBetween(vertical_segment_start, SteadyClock::now());
            }
            if(!accepted_direction) {
                if(has_vertical_candidate) {
                    saturatingIncrement(result.diagnostics.vertical_passed_columns);
                    if(first_support_failure.has_value()) {
                        recordSupportFailure(result.diagnostics, *first_support_failure);
                    } else {
                        SupportCheckResult unavailable;
                        unavailable.failure = FrontierSupportFailure::OutOfBounds;
                        recordSupportFailure(result.diagnostics, unavailable);
                    }
                } else {
                    saturatingIncrement(result.diagnostics.vertical_rejected_columns);
                }
                continue;
            }
            saturatingIncrement(result.diagnostics.vertical_passed_columns);
            saturatingIncrement(result.diagnostics.support_passed_columns);
            if(supported.size() > config_.max_frontier_columns) {
                return finish(
                        FrontierDetectionStatus::ResourceLimit,
                        "supported frontier column limit exceeded", false);
            }
        }
        if(supported.empty()) {
            return finish(FrontierDetectionStatus::Empty, {}, true);
        }

        const auto component_start = config_.collect_stage_timings
                                             ? SteadyClock::now()
                                             : SteadyClock::time_point {};
        auto finishComponentTiming = [&]() {
            if(config_.collect_stage_timings) {
                result.diagnostics.timings.component_seconds =
                        secondsBetween(component_start, SteadyClock::now());
            }
        };
        std::set<FrontierColumnKey> remaining;
        for(const auto & [column, sample] : supported) {
            (void) sample;
            remaining.insert(column);
        }
        const float column_size = static_cast<float>(
                config_.column_stride_voxels * config_.resolution);
        std::size_t component_index = 0U;
        while(!remaining.empty()) {
            const std::size_t current_component_index = component_index++;
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
            saturatingIncrement(result.diagnostics.components_built);
            saturatingIncrement(
                    result.diagnostics.component_size_buckets[
                            componentSizeBucket(component.size())]);
            if(component.size() > config_.max_columns_per_region) {
                result.regions.clear();
                finishComponentTiming();
                return finish(
                        FrontierDetectionStatus::ResourceLimit,
                        "frontier region column limit exceeded", false);
            }
            RegionComputation computation = makeRegion(
                    component, supported, column_size, config_.max_columns_per_region);
            FrontierRegion & region = computation.region;
            FrontierComponentTrace component_trace;
            bool collect_component_trace = false;
            if(trace != nullptr) {
                if(trace->components.size() >= config_.max_trace_components) {
                    trace->truncated = true;
                } else {
                    collect_component_trace = true;
                    component_trace.component_index = current_component_index;
                    component_trace.stable_key = region.stable_key;
                    component_trace.exact_column_count = component.size();
                    component_trace.representative = region.representative;
                    component_trace.unknown_direction = region.unknown_direction;
                    component_trace.xy_minimum = computation.xy_minimum;
                    component_trace.xy_maximum = computation.xy_maximum;
                    component_trace.direction_votes = computation.direction_votes;
                    component_trace.information_gain = region.information_gain;
                    component_trace.area = region.area;
                    component_trace.horizontal_span = region.horizontal_span;
                    component_trace.direction_consistency =
                            region.direction_consistency;
                    component_trace.columns_complete = trace_collector.copyGeometry(
                            component_trace.columns, component);
                    component_trace.edges_complete = true;
                    const std::set<FrontierColumnKey> component_keys(
                            component.begin(), component.end());
                    for(const FrontierColumnKey & current : component) {
                        for(const auto & [dx, dy] : NEIGHBOR_DIRECTIONS) {
                            const FrontierColumnKey neighbor {
                                    current.x + static_cast<std::int64_t>(dx),
                                    current.y + static_cast<std::int64_t>(dy)};
                            if(current < neighbor
                               && component_keys.find(neighbor)
                                          != component_keys.end())
                            {
                                if(trace_collector.reserveGeometryElement()) {
                                    component_trace.edges.push_back(
                                            FrontierComponentEdge {current, neighbor});
                                } else {
                                    component_trace.edges_complete = false;
                                }
                            }
                        }
                    }
                }
            }
            auto appendComponentTrace = [&]() {
                if(trace == nullptr) {
                    return;
                }
                if(!collect_component_trace) {
                    return;
                }
                trace->components.push_back(std::move(component_trace));
            };
            if(region.columns.size() < config_.min_columns) {
                saturatingIncrement(
                        result.diagnostics.component_primary_rejected_columns);
                component_trace.rejection = FrontierComponentRejection::Columns;
                appendComponentTrace();
                continue;
            }
            if(region.area + EPSILON < config_.min_area) {
                saturatingIncrement(result.diagnostics.component_primary_rejected_area);
                component_trace.rejection = FrontierComponentRejection::Area;
                appendComponentTrace();
                continue;
            }
            if(region.horizontal_span + EPSILON < config_.min_span) {
                saturatingIncrement(result.diagnostics.component_primary_rejected_span);
                component_trace.rejection = FrontierComponentRejection::Span;
                appendComponentTrace();
                continue;
            }
            if(region.direction_consistency + EPSILON
               < config_.min_direction_consistency)
            {
                saturatingIncrement(
                        result.diagnostics.component_primary_rejected_direction);
                component_trace.rejection = FrontierComponentRejection::Direction;
                appendComponentTrace();
                continue;
            }
            saturatingIncrement(result.diagnostics.components_accepted);
            appendComponentTrace();
            result.regions.push_back(std::move(region));
            if(result.regions.size() > config_.max_regions) {
                result.regions.clear();
                finishComponentTiming();
                return finish(
                        FrontierDetectionStatus::ResourceLimit,
                        "frontier region limit exceeded", false);
            }
        }
        std::sort(
                result.regions.begin(), result.regions.end(),
                [](const FrontierRegion & lhs, const FrontierRegion & rhs) {
                    return lhs.stable_key < rhs.stable_key;
                });
        finishComponentTiming();
        return finish(
                result.regions.empty() ? FrontierDetectionStatus::Empty
                                       : FrontierDetectionStatus::Accepted,
                {}, true);
    }

    const GlobalFrontierDetectorConfig & GlobalFrontierDetector::config() const
    {
        return config_;
    }

}// namespace SwarmController
