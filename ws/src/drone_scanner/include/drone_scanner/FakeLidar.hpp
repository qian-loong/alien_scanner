#ifndef DRONE_SCANNER_FAKELIDAR_HPP
#define DRONE_SCANNER_FAKELIDAR_HPP

#include "cave_world/ICaveField.hpp"
#include "drone_scanner/Pose3D.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace DroneScanner {

    struct FakeLidarConfig {
        int           num_beams {360};
        float         max_range {30.0F};
        float         range_noise_std {0.0F};
        std::uint32_t noise_seed {0U}; ///< 非零且 range_noise_std>0 时用于确定性噪声
        /// 扫描平面绕机体 +Y 前倾角 (rad)。0 = 纯 YZ 垂直环；>0 时 beam 带 +X，消除正前盲区。
        float         ring_pitch_rad {0.0F};
    };

    /// lidar_link 系下的命中点（REP-103：x 前、y 左、z 上）。
    struct LidarPoint {
        float x {};
        float y {};
        float z {};
    };

    /// 每束 beam 的 endpoint。hit=false 时 endpoint 是 max_range 虚点，仅表示沿途 free。
    struct LidarReturn {
        float x {};
        float y {};
        float z {};
        float range {};
        bool  hit {false};
    };

    /// 注入 ICaveField，在可俯仰的垂直 360° 环上 raycast（pitch=0 时环平面 ⊥ 前进 +x）。
    class FakeLidar
    {
    public:
        FakeLidar(std::shared_ptr<CaveWorld::ICaveField> field, FakeLidarConfig config);

        /// @param lidar_pose_in_map LiDAR 原点在 map 下的位姿（当前仅 yaw 参与 beam 旋转）。
        std::vector<LidarPoint> scan(const Pose3D & lidar_pose_in_map) const;
        std::vector<LidarReturn> scanReturns(const Pose3D & lidar_pose_in_map) const;

    private:
        std::shared_ptr<CaveWorld::ICaveField> field_;
        FakeLidarConfig                        config_;
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_FAKELIDAR_HPP
