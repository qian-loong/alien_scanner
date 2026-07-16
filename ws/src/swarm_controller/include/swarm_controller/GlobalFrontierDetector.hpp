#ifndef SWARM_CONTROLLER_GLOBALFRONTIERDETECTOR_HPP
#define SWARM_CONTROLLER_GLOBALFRONTIERDETECTOR_HPP

#include "swarm_controller/Point3f.hpp"

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
        float       support_depth {0.8F};
        float       support_width {1.0F};
        std::size_t min_columns {12U};
        float       min_area {0.48F};
        float       min_span {0.6F};
        float       min_direction_consistency {0.65F};
        std::size_t max_frontier_columns {250'000U};
        std::size_t max_scanned_free_voxels {2'000'000U};
        std::size_t max_support_samples_per_column {10'000U};
        std::size_t max_columns_per_region {50'000U};
        std::size_t max_regions {64U};
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

    struct FrontierDetectionResult {
        FrontierDetectionStatus status {FrontierDetectionStatus::Invalid};
        std::vector<FrontierRegion> regions;
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

    class GlobalFrontierDetector
    {
    public:
        explicit GlobalFrontierDetector(GlobalFrontierDetectorConfig config = {});

        FrontierDetectionResult detect(const octomap::OcTree & tree) const;
        const GlobalFrontierDetectorConfig & config() const;

    private:
        GlobalFrontierDetectorConfig config_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_GLOBALFRONTIERDETECTOR_HPP
