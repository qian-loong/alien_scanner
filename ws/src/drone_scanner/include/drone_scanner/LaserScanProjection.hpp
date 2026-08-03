#ifndef DRONE_SCANNER_LASERSCANPROJECTION_HPP
#define DRONE_SCANNER_LASERSCANPROJECTION_HPP

#include "drone_scanner/FakeLidar.hpp"
#include "drone_scanner/Point3f.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace DroneScanner {

    struct LaserScanProjectionConfig {
        std::size_t beam_count {360U};
        float       range_min_m {0.1F};
        float       range_max_m {30.0F};
        double      scan_rate_hz {10.0};
    };

    struct ProjectedLaserScan {
        float              angle_min_rad {0.0F};
        float              angle_max_rad {0.0F};
        float              angle_increment_rad {0.0F};
        float              range_min_m {0.0F};
        float              range_max_m {0.0F};
        float              scan_time_s {0.0F};
        std::vector<float> ranges;
    };

    class LaserScanProjection
    {
    public:
        explicit LaserScanProjection(LaserScanProjectionConfig config);

        const LaserScanProjectionConfig & config() const noexcept;
        ProjectedLaserScan metadata() const;
        std::size_t fakeLidarIndex(std::size_t scan_index) const;
        Point3f bodyDirection(std::size_t scan_index) const;

        std::optional<ProjectedLaserScan> project(
                const std::vector<LidarReturn> & returns,
                std::string * diagnostic = nullptr) const;

    private:
        LaserScanProjectionConfig config_;
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_LASERSCANPROJECTION_HPP
