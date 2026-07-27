#include "drone_scanner/LaserScanProjection.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace DroneScanner {

    namespace {

        constexpr double kPi = 3.14159265358979323846;

        void set_diagnostic(std::string * diagnostic, const std::string & value)
        {
            if(diagnostic != nullptr) {
                *diagnostic = value;
            }
        }

    }// namespace

    LaserScanProjection::LaserScanProjection(LaserScanProjectionConfig config)
        : config_(config)
    {
        if(config_.beam_count < 2U || (config_.beam_count % 2U) != 0U) {
            throw std::invalid_argument("LaserScanProjection requires an even beam_count >= 2");
        }
        if(!std::isfinite(config_.range_min_m) || !std::isfinite(config_.range_max_m)
           || config_.range_min_m < 0.0F || config_.range_min_m >= config_.range_max_m) {
            throw std::invalid_argument(
                    "LaserScanProjection requires finite 0 <= range_min < range_max");
        }
        if(!std::isfinite(config_.scan_rate_hz) || config_.scan_rate_hz <= 0.0) {
            throw std::invalid_argument("LaserScanProjection requires a positive scan_rate_hz");
        }
    }

    const LaserScanProjectionConfig & LaserScanProjection::config() const noexcept
    {
        return config_;
    }

    ProjectedLaserScan LaserScanProjection::metadata() const
    {
        ProjectedLaserScan result;
        result.angle_min_rad = static_cast<float>(-kPi);
        result.angle_increment_rad = static_cast<float>(
                (2.0 * kPi) / static_cast<double>(config_.beam_count));
        result.angle_max_rad = result.angle_min_rad
                               + static_cast<float>(config_.beam_count - 1U)
                                         * result.angle_increment_rad;
        result.range_min_m = config_.range_min_m;
        result.range_max_m = config_.range_max_m;
        result.scan_time_s = static_cast<float>(1.0 / config_.scan_rate_hz);
        return result;
    }

    std::size_t LaserScanProjection::fakeLidarIndex(std::size_t scan_index) const
    {
        if(scan_index >= config_.beam_count) {
            throw std::out_of_range("LaserScanProjection scan index is out of range");
        }
        return (scan_index + config_.beam_count / 2U) % config_.beam_count;
    }

    Point3f LaserScanProjection::bodyDirection(std::size_t scan_index) const
    {
        if(scan_index >= config_.beam_count) {
            throw std::out_of_range("LaserScanProjection scan index is out of range");
        }
        const auto scan = metadata();
        const double theta = static_cast<double>(scan.angle_min_rad)
                             + static_cast<double>(scan_index)
                                       * static_cast<double>(scan.angle_increment_rad);
        return Point3f {
                0.0F,
                static_cast<float>(std::cos(theta)),
                static_cast<float>(std::sin(theta))};
    }

    std::optional<ProjectedLaserScan> LaserScanProjection::project(
            const std::vector<LidarReturn> & returns,
            std::string * diagnostic) const
    {
        if(returns.size() != config_.beam_count) {
            set_diagnostic(diagnostic, "FakeLidar return count does not match beam_count");
            return std::nullopt;
        }

        ProjectedLaserScan result = metadata();
        result.ranges.resize(config_.beam_count);
        for(std::size_t scan_index = 0U; scan_index < config_.beam_count; ++scan_index) {
            const auto & value = returns[fakeLidarIndex(scan_index)];
            if(!value.hit) {
                result.ranges[scan_index] = std::numeric_limits<float>::infinity();
                continue;
            }
            if(!std::isfinite(value.range)
               || value.range < config_.range_min_m
               || value.range > config_.range_max_m) {
                set_diagnostic(diagnostic, "FakeLidar hit range is outside the frozen scan bounds");
                return std::nullopt;
            }
            result.ranges[scan_index] = value.range;
        }
        set_diagnostic(diagnostic, {});
        return result;
    }

}// namespace DroneScanner
