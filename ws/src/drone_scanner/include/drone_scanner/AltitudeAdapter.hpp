#ifndef DRONE_SCANNER_ALTITUDEADAPTER_HPP
#define DRONE_SCANNER_ALTITUDEADAPTER_HPP

#include "drone_scanner/FakeLidar.hpp"

#include <vector>

namespace DroneScanner {

    struct AltitudeAdaptConfig {
        float target_fraction {0.5F};     ///< 0=贴底，1=贴顶；默认走廊中带
        float min_clearance {0.35F};      ///< 相对顶/底至少保留的净空 (m)
        float max_vertical_speed {0.6F};  ///< |vz| 上限 (m/s)，按 dt 限幅
        float band_ema_alpha {0.25F};     ///< 顶/底估计 EMA；越大越跟新帧
        float min_band_height {0.8F};     ///< 顶底间距过小则视为无效
        float vertical_dot_min {0.65F};   ///< |hit.z|/range 下限，筛近垂向 beam
        /// 与 FakeLidar::ring_pitch_rad 一致；用于校验 vertical_dot_min < cos(pitch)
        float ring_pitch_rad {0.0F};
    };

    struct AltitudeBand {
        bool  valid {false};
        float floor_z {0.0F};    ///< map 系：最近下方障碍
        float ceiling_z {0.0F};  ///< map 系：最近上方障碍
    };

    /// 由当前帧 lidar 命中点估计局部顶/底，并给出安全飞行高度（不读洞穴真值）。
    /// 顶/底 = 上/下方向**最近**障碍（通用启发式；钟乳石等复杂场景完备策略为未来特性）。
    /// EMA 仅应在「新扫描帧」上调用 estimateBand；限速用 adaptZ(dt)。
    class AltitudeAdapter
    {
    public:
        explicit AltitudeAdapter(AltitudeAdaptConfig config = {});

        /// 几何是否可用：vertical_dot_min < cos(ring_pitch)。失败时 estimateBand 恒 invalid。
        bool geometryCompatible() const
        {
            return geometry_ok_;
        }

        /// @param hits lidar_link 系命中点
        /// @param scan_origin_z 该帧扫描时的机体 map/odom z（禁止用后续 adapted z 解释旧 hits）
        AltitudeBand estimateBand(const std::vector<LidarPoint> & hits, float scan_origin_z);

        /// 在 band 内取目标高度，并相对 current_z 按 dt 限速。
        float adaptZ(const AltitudeBand & band, float current_z, float dt_seconds) const;

        /// 仅用于单测：estimateBand + adaptZ（生产路径应拆开，避免旧帧重复 EMA）
        float adaptFromHits(const std::vector<LidarPoint> & hits, float scan_origin_z, float current_z, float dt_seconds);

        void reset();

        const AltitudeAdaptConfig & config() const
        {
            return config_;
        }

    private:
        AltitudeAdaptConfig config_;
        bool                geometry_ok_ {true};
        bool                has_filtered_band_ {false};
        float               filtered_floor_z_ {0.0F};
        float               filtered_ceiling_z_ {0.0F};
    };

}// namespace DroneScanner

#endif// DRONE_SCANNER_ALTITUDEADAPTER_HPP
