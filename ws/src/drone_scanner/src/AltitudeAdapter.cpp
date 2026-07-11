#include "drone_scanner/AltitudeAdapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace DroneScanner {

    AltitudeAdapter::AltitudeAdapter(AltitudeAdaptConfig config)
        : config_(config)
    {
        config_.target_fraction = std::clamp(config_.target_fraction, 0.0F, 1.0F);
        if(config_.min_clearance < 0.0F) {
            config_.min_clearance = 0.0F;
        }
        if(config_.max_vertical_speed <= 0.0F) {
            config_.max_vertical_speed = 0.3F;
        }
        config_.band_ema_alpha = std::clamp(config_.band_ema_alpha, 0.0F, 1.0F);
        if(config_.min_band_height <= 0.0F) {
            config_.min_band_height = 0.1F;
        }
        config_.vertical_dot_min = std::clamp(config_.vertical_dot_min, 0.0F, 1.0F);

        // 近垂向筛选要求 |sinθ|·cos(pitch) ≥ vertical_dot_min，故必须 vertical_dot_min < cos(pitch)
        const float cos_pitch = std::cos(config_.ring_pitch_rad);
        geometry_ok_          = config_.vertical_dot_min < (cos_pitch - 1e-3F);
    }

    void AltitudeAdapter::reset()
    {
        has_filtered_band_  = false;
        filtered_floor_z_   = 0.0F;
        filtered_ceiling_z_ = 0.0F;
    }

    AltitudeBand AltitudeAdapter::estimateBand(const std::vector<LidarPoint> & hits, float scan_origin_z)
    {
        AltitudeBand band;
        if(!geometry_ok_) {
            return band;
        }

        // 最近上方障碍 = 最小正 hit.z；最近下方 = 最大负 hit.z（最靠近机体）
        float nearest_up_z   = std::numeric_limits<float>::infinity();
        float nearest_down_z = -std::numeric_limits<float>::infinity();
        int   n_up           = 0;
        int   n_down         = 0;

        for(const LidarPoint & hit : hits) {
            const float range = std::sqrt(hit.x * hit.x + hit.y * hit.y + hit.z * hit.z);
            if(range < 1e-3F) {
                continue;
            }
            const float vertical_dot = std::abs(hit.z) / range;
            if(vertical_dot < config_.vertical_dot_min) {
                continue;
            }
            if(hit.z > 1e-4F) {
                nearest_up_z = std::min(nearest_up_z, hit.z);
                ++n_up;
            } else if(hit.z < -1e-4F) {
                nearest_down_z = std::max(nearest_down_z, hit.z);
                ++n_down;
            }
        }

        // 上下成对：任一侧缺失则 invalid（避免单侧侧壁伪造成 band）
        if(n_up == 0 || n_down == 0) {
            return band;
        }

        const float raw_ceiling = scan_origin_z + nearest_up_z;
        const float raw_floor   = scan_origin_z + nearest_down_z;
        if((raw_ceiling - raw_floor) < config_.min_band_height) {
            return band;
        }

        if(!has_filtered_band_) {
            filtered_floor_z_   = raw_floor;
            filtered_ceiling_z_ = raw_ceiling;
            has_filtered_band_  = true;
        } else {
            const float a       = config_.band_ema_alpha;
            filtered_floor_z_   = a * raw_floor + (1.0F - a) * filtered_floor_z_;
            filtered_ceiling_z_ = a * raw_ceiling + (1.0F - a) * filtered_ceiling_z_;
        }

        band.floor_z   = filtered_floor_z_;
        band.ceiling_z = filtered_ceiling_z_;
        if((band.ceiling_z - band.floor_z) < config_.min_band_height) {
            return AltitudeBand {};
        }
        band.valid = true;
        return band;
    }

    float AltitudeAdapter::adaptZ(const AltitudeBand & band, float current_z, float dt_seconds) const
    {
        if(!band.valid) {
            return current_z;
        }

        const float lo = band.floor_z + config_.min_clearance;
        const float hi = band.ceiling_z - config_.min_clearance;

        // 已在顶/底净空内则保持高度，避免入口等不对称截面被强行拉向中带后正反馈下坠。
        if(lo <= hi && current_z >= lo && current_z <= hi) {
            return current_z;
        }

        const float height = band.ceiling_z - band.floor_z;
        float       target = band.floor_z + config_.target_fraction * height;
        if(lo <= hi) {
            target = std::clamp(target, lo, hi);
        } else {
            target = 0.5F * (band.floor_z + band.ceiling_z);
        }

        const float dt     = std::max(dt_seconds, 1e-3F);
        const float max_dz = config_.max_vertical_speed * dt;
        const float delta  = std::clamp(target - current_z, -max_dz, max_dz);
        return current_z + delta;
    }

    float AltitudeAdapter::adaptFromHits(
            const std::vector<LidarPoint> & hits, float scan_origin_z, float current_z, float dt_seconds)
    {
        return adaptZ(estimateBand(hits, scan_origin_z), current_z, dt_seconds);
    }

}// namespace DroneScanner
