#ifndef CAVE_WORLD_PROCEDURALCAVEFIELD_HPP
#define CAVE_WORLD_PROCEDURALCAVEFIELD_HPP

#include "ICaveField.hpp"

#include <cstdint>
#include <vector>

namespace CaveWorld {

    struct ProceduralCaveFieldConfig {
        float length {40.0F};        ///< 入口到 junction 的 trunk 长度 (m)，经典 Y 专用
        float branch_length {20.0F}; ///< 自 junction 向外的每条岔臂长度 (m)
        float base_radius {2.5F};    ///< 隧道基础半径 (m)
        int   n_segments {200};      ///< 每条臂的中轴线分段数；端点数为 n_segments + 1
        bool  branch {true};         ///< true = 经典 Y（trunk + 两岔臂）；false = 仅直 trunk
        float branch_angle {0.55F};  ///< 两岔臂相对 trunk 切线的半角 (rad)
        float chamber_at {0.5F};     ///< 溶洞中心在 trunk 上的轴向比例 [0, 1]
        float chamber_scale {3.0F};  ///< 溶洞半径相对 base_radius 的放大倍数
        float noise_scale {0.4F};    ///< 洞壁噪声强度
        int   density {400};         ///< 每米环向/表面采样密度（sampleSurface 用）

        std::uint32_t seed {0};///< 随机种子；相同 seed 应得到确定性结果
    };

    class ProceduralCaveField : public ICaveField
    {
    public:
        explicit ProceduralCaveField(const ProceduralCaveFieldConfig & config);

        bool                isSolid(float x, float y, float z) const override;
        bool                raycast(const Point3 & origin, const Point3 & dir, float max_range, float & out_dist) const override;
        std::vector<Point3> sampleSurface() const override;

    private:
        void  buildCenterline();
        void  buildArmCenterline(
                const Point3 & start, float dir_x, float dir_y, float dir_z, float arm_length, int n_segments,
                float phase_y, float phase_z, std::vector<Point3> & out_centerline, float & out_length) const;
        float radiusProfile(float t) const;
        float wallNoise(float wx, float wy, float wz) const;
        float localRadius(float t, float wx, float wy, float wz) const;

        bool closestOnPolyline(
                const std::vector<Point3> & polyline, float polyline_length, float x, float y, float z,
                float & out_t, float & out_ax, float & out_ay, float & out_az, float & out_dist_sq) const;

        void samplePolylineSurface(
                const std::vector<Point3> & polyline, float polyline_length, float radius_scale, bool apply_chamber,
                std::vector<Point3> & out_points) const;

        bool isExteriorShellPoint(
                float px, float py, float pz, float axis_x, float axis_y, float axis_z) const;

        void appendSurfaceRing(
                float cx, float cy, float cz, float tx, float ty, float tz, float radius, int points_per_ring,
                std::vector<Point3> & out_points) const;

        ProceduralCaveFieldConfig config_;
        std::vector<Point3>       trunk_centerline_;
        std::vector<Point3>       left_branch_centerline_;
        std::vector<Point3>       right_branch_centerline_;
        float                     trunk_length_ {0.0F};
        float                     branch_arm_length_ {0.0F};
    };

}// namespace CaveWorld

#endif// CAVE_WORLD_PROCEDURALCAVEFIELD_HPP
