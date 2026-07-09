#include "drone_scanner/FakeLidar.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace DroneScanner {

    namespace {

        constexpr float kPi = 3.14159265358979323846F;

        /// lidar 系 beam：θ 在俯仰后的环上；(pitch=0) → (0, cos θ, sin θ)。
        /// 绕 +Y 旋转 ring_pitch：Ry(φ)·(0, cosθ, sinθ) = (sinθ·sinφ, cosθ, sinθ·cosφ)。
        void lidarBeamDirection(float cos_theta, float sin_theta, float ring_pitch, float & out_x, float & out_y, float & out_z)
        {
            const float sin_pitch = std::sin(ring_pitch);
            const float cos_pitch = std::cos(ring_pitch);
            out_x = sin_theta * sin_pitch;
            out_y = cos_theta;
            out_z = sin_theta * cos_pitch;
        }

        CaveWorld::Point3 lidarBeamToMap(float lx, float ly, float lz, float yaw)
        {
            const float cy = std::cos(yaw);
            const float sy = std::sin(yaw);
            return CaveWorld::Point3 {cy * lx - sy * ly, sy * lx + cy * ly, lz};
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
        const float             ring_pitch = config_.ring_pitch_rad;

        for(int beam_index = 0; beam_index < config_.num_beams; ++beam_index) {
            const float theta = 2.0F * kPi * static_cast<float>(beam_index)
                                / static_cast<float>(config_.num_beams);
            const float cos_theta = std::cos(theta);
            const float sin_theta = std::sin(theta);

            float lx = 0.0F;
            float ly = 0.0F;
            float lz = 0.0F;
            lidarBeamDirection(cos_theta, sin_theta, ring_pitch, lx, ly, lz);
            const CaveWorld::Point3 dir_map = lidarBeamToMap(lx, ly, lz, lidar_pose_in_map.yaw);

            float hit_dist = 0.0F;
            if(!field_->raycast(origin, dir_map, config_.max_range, hit_dist)) {
                continue;
            }

            if(config_.range_noise_std > 0.0F && config_.noise_seed != 0U) {
                hit_dist += gaussianNoise(config_.noise_seed, beam_index, config_.range_noise_std);
            }
            hit_dist = std::clamp(hit_dist, 0.0F, config_.max_range);

            hits.push_back(LidarPoint {
                    hit_dist * lx,
                    hit_dist * ly,
                    hit_dist * lz,
            });
        }

        return hits;
    }

}// namespace DroneScanner
