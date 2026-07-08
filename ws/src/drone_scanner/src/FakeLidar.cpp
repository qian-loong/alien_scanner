#include "drone_scanner/FakeLidar.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace DroneScanner {

    namespace {

        constexpr float kPi = 3.14159265358979323846F;

        CaveWorld::Point3 lidarBeamToMap(float cos_theta, float sin_theta, float yaw)
        {
            const float cy = std::cos(yaw);
            const float sy = std::sin(yaw);
            // lidar 系 beam (0, cos θ, sin θ)，绕 map z 旋转 yaw
            return CaveWorld::Point3 {-sy * cos_theta, cy * cos_theta, sin_theta};
        }

        float gaussianNoise(std::uint32_t seed, int beam_index, float std_dev)
        {
            std::mt19937                    rng(seed + static_cast<std::uint32_t>(beam_index) * 2654435761U);
            std::normal_distribution<float> dist(0.0F, std_dev);
            return dist(rng);
        }

    }// namespace

    FakeLidar::FakeLidar(std::shared_ptr<CaveWorld::ICaveField> field, FakeLidarConfig config)
        : field_(std::move(field))
        , config_(config)
    {
        if(!field_) {
            throw std::invalid_argument("FakeLidar: ICaveField must not be null");
        }
        config_.num_beams = std::max(1, config_.num_beams);
        if(config_.max_range <= 0.0F) {
            config_.max_range = 1.0F;
        }
    }

    std::vector<LidarPoint> FakeLidar::scan(const Pose3D & lidar_pose_in_map) const
    {
        std::vector<LidarPoint> hits;
        hits.reserve(static_cast<std::size_t>(config_.num_beams));

        const CaveWorld::Point3 origin {lidar_pose_in_map.x, lidar_pose_in_map.y, lidar_pose_in_map.z};

        for(int beam_index = 0; beam_index < config_.num_beams; ++beam_index) {
            const float theta = 2.0F * kPi * static_cast<float>(beam_index)
                                / static_cast<float>(config_.num_beams);
            const float cos_theta = std::cos(theta);
            const float sin_theta = std::sin(theta);
            const CaveWorld::Point3 dir_map =
                    lidarBeamToMap(cos_theta, sin_theta, lidar_pose_in_map.yaw);

            float hit_dist = 0.0F;
            if(!field_->raycast(origin, dir_map, config_.max_range, hit_dist)) {
                continue;
            }

            if(config_.range_noise_std > 0.0F && config_.noise_seed != 0U) {
                hit_dist += gaussianNoise(config_.noise_seed, beam_index, config_.range_noise_std);
            }
            hit_dist = std::clamp(hit_dist, 0.0F, config_.max_range);

            hits.push_back(LidarPoint {
                    0.0F,
                    hit_dist * cos_theta,
                    hit_dist * sin_theta,
            });
        }

        return hits;
    }

}// namespace DroneScanner
