#ifndef SWARM_CONTROLLER_GLOBALFRONTIERDETECTOR_HPP
#define SWARM_CONTROLLER_GLOBALFRONTIERDETECTOR_HPP

#include "swarm_controller/Point3f.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

namespace SwarmController {

    struct FrontierColumnKey {
        std::int64_t x {};
        std::int64_t y {};

        bool operator==(const FrontierColumnKey & other) const
        {
            return x == other.x && y == other.y;
        }

        bool operator<(const FrontierColumnKey & other) const
        {
            return x < other.x || (x == other.x && y < other.y);
        }
    };

    struct GlobalFrontierDetectorConfig {
        double      resolution {0.1};
        std::size_t column_stride_voxels {2U};
        std::size_t min_z_layers {5U};
        float       min_z_span {0.4F};
        double      support_depth {0.8};
        std::size_t min_columns {12U};
        float       min_area {0.48F};
        float       min_span {0.6F};
        float       min_direction_consistency {0.65F};
        std::size_t max_frontier_columns {250'000U};
        std::size_t max_scanned_free_voxels {2'000'000U};
        std::size_t max_support_samples_per_column {10'000U};
        std::size_t max_columns_per_region {50'000U};
        std::size_t max_regions {64U};
        std::size_t max_trace_candidates {10'000U};
        std::size_t max_trace_support_samples {100'000U};
        std::size_t max_trace_components {10'000U};
        std::size_t max_trace_geometry_elements {500'000U};
        bool        collect_stage_timings {false};
    };

    struct FrontierRegion {
        FrontierColumnKey        stable_key {};
        std::vector<FrontierColumnKey> columns;
        Point3f                  representative {};
        Point3f                  unknown_direction {};
        float                    information_gain {};
        float                    area {};
        float                    horizontal_span {};
        float                    direction_consistency {};
    };

    enum class FrontierDetectionStatus {
        Accepted,
        Empty,
        Invalid,
        ResourceLimit,
    };

    struct FrontierDetectionTimings {
        double leaf_scan_seconds {};
        double vertical_seconds {};
        double support_seconds {};
        double component_seconds {};
        double total_seconds {};
    };

    struct FrontierDetectionDiagnostics {
        static constexpr std::size_t SUPPORT_DEPTH_BUCKETS = 8U;
        static constexpr std::size_t COMPONENT_SIZE_BUCKETS = 6U;

        bool          complete {};
        std::uint64_t scanned_free_voxels {};
        std::uint64_t sampled_free_columns {};
        std::uint64_t unknown_neighbor_candidate_columns {};
        std::uint64_t vertical_passed_columns {};
        std::uint64_t vertical_rejected_columns {};
        std::uint64_t support_passed_columns {};
        std::uint64_t support_rejected_unknown {};
        std::uint64_t support_rejected_occupied {};
        std::uint64_t support_rejected_out_of_bounds {};
        std::uint64_t support_samples_attempted {};
        std::uint64_t support_failure_position_unavailable {};
        std::array<std::uint64_t, SUPPORT_DEPTH_BUCKETS> support_failure_depth_octiles {};
        std::uint64_t components_built {};
        std::array<std::uint64_t, COMPONENT_SIZE_BUCKETS> component_size_buckets {};
        std::uint64_t component_primary_rejected_columns {};
        std::uint64_t component_primary_rejected_area {};
        std::uint64_t component_primary_rejected_span {};
        std::uint64_t component_primary_rejected_direction {};
        std::uint64_t components_accepted {};
        FrontierDetectionTimings timings;
    };

    struct FrontierDetectionResult {
        FrontierDetectionStatus status {FrontierDetectionStatus::Invalid};
        std::vector<FrontierRegion> regions;
        FrontierDetectionDiagnostics diagnostics;
        // Compatibility aliases retained while diagnostics consumers migrate.
        std::size_t raw_columns {};
        std::size_t scanned_free_voxels {};
        std::size_t supported_columns {};
        std::size_t vertical_rejected_columns {};
        std::size_t support_rejected_columns {};
        std::string reason;

        bool accepted() const
        {
            return status == FrontierDetectionStatus::Accepted
                   || status == FrontierDetectionStatus::Empty;
        }
    };

    enum class FrontierTraceSampleState {
        Free,
        Unknown,
        Occupied,
        OutOfBounds,
    };

    struct FrontierSupportSampleTrace {
        Point3f                  position {};
        FrontierTraceSampleState state {FrontierTraceSampleState::Free};
    };

    enum class FrontierSupportFailure {
        None,
        Unknown,
        Occupied,
        OutOfBounds,
    };

    struct FrontierSupportAttemptTrace {
        Point3f unknown_direction {};
        Point3f inward_direction {};
        Point3f anchor {};
        std::vector<Point3f> column_points;
        std::vector<FrontierSupportSampleTrace> samples;
        FrontierSupportFailure failure {FrontierSupportFailure::None};
        bool has_first_failure_position {};
        Point3f first_failure_position {};
        bool selected {};

        bool passed() const
        {
            return failure == FrontierSupportFailure::None;
        }
    };

    struct FrontierCandidateTrace {
        FrontierColumnKey key {};
        Point3f           center {};
        std::vector<Point3f> column_points;
        bool              vertical_passed {};
        bool              support_passed {};
        std::vector<FrontierSupportAttemptTrace> support_attempts;
    };

    enum class FrontierComponentRejection {
        None,
        Columns,
        Area,
        Span,
        Direction,
    };

    struct FrontierComponentEdge {
        FrontierColumnKey first {};
        FrontierColumnKey second {};
    };

    struct FrontierComponentTrace {
        std::size_t                     component_index {};
        FrontierColumnKey               stable_key {};
        std::size_t                     exact_column_count {};
        std::vector<FrontierColumnKey> columns;
        std::vector<FrontierComponentEdge> edges;
        Point3f                         representative {};
        Point3f                         unknown_direction {};
        Point3f                         xy_minimum {};
        Point3f                         xy_maximum {};
        std::array<std::size_t, 4U>     direction_votes {};
        float                           information_gain {};
        float                           area {};
        float                           horizontal_span {};
        float                           direction_consistency {};
        FrontierComponentRejection      rejection {FrontierComponentRejection::None};
        bool                            columns_complete {};
        bool                            edges_complete {};
    };

    struct FrontierDetectionTrace {
        bool                                truncated {};
        std::vector<FrontierCandidateTrace> candidates;
        std::vector<FrontierComponentTrace> components;
    };

    struct TracedFrontierDetectionResult {
        FrontierDetectionResult result;
        FrontierDetectionTrace  trace;
    };

    class GlobalFrontierDetector
    {
    public:
        explicit GlobalFrontierDetector(GlobalFrontierDetectorConfig config = {});

        FrontierDetectionResult detect(const octomap::OcTree & tree) const;
        TracedFrontierDetectionResult detectWithTrace(const octomap::OcTree & tree) const;
        const GlobalFrontierDetectorConfig & config() const;

    private:
        FrontierDetectionResult detectImpl(
                const octomap::OcTree & tree, FrontierDetectionTrace * trace) const;

        GlobalFrontierDetectorConfig config_;
        std::size_t support_depth_samples_ {};
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_GLOBALFRONTIERDETECTOR_HPP
