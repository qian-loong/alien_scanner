#ifndef CAVE_WORLD_TREECAVEFIELD_HPP
#define CAVE_WORLD_TREECAVEFIELD_HPP

#include "ICaveField.hpp"

#include <cstdint>
#include <vector>

namespace CaveWorld {

    /// 草图拓扑：入口→A；A→B 有两条通道（直连 + 左侧外环）构成环；
    /// B→出口1；A→右廊→C；C→出口2 / 出口3。共 1 入口、3 出口、1 环。
    struct TreeCaveFieldConfig {
        float approach_length {12.0F}; ///< 入口到分叉 A 的接入段 (m)
        float base_radius {2.5F};      ///< 管体基础半径 (m)
        int   n_segments {160};        ///< 接入段分段数；其余臂按比例取
        int   density {400};           ///< sampleSurface 环向密度
        float noise_scale {0.4F};      ///< 洞壁噪声
        std::uint32_t seed {42U};      ///< 形状确定性种子（基础地图 seed=42）

        float loop_yaw {0.50F};            ///< A→B 直连相对入口切线偏角 (rad，左正)
        float loop_direct_length {16.0F};  ///< A→B 直连长度 (m)
        float loop_bulge {12.0F};          ///< 外环相对直连中点的左侧鼓出 (m)
        float exit1_length {14.0F};        ///< B→出口1 长度 (m)

        float right_yaw {-0.12F};            ///< A→C 右廊偏角 (rad)
        float right_corridor_length {10.0F}; ///< A→C 右廊长度 (m)
        float exit_yaw_spread {0.35F};       ///< 出口2/3 相对右廊展开半角 (rad)
        float exit_arm_length {14.0F};       ///< 出口2/3 长度 (m)

        float vertical_step {-0.10F}; ///< 各臂相对上一层 pitch (rad，负=下沉)
        float asymmetry {0.22F};      ///< seed 角度/长度扰动 [0,1]

        bool  chamber_on_approach {false}; ///< 接入段是否启用溶洞（默认关，避免入口膨大）
        float chamber_at {0.55F};          ///< 溶洞中心（仅 chamber_on_approach 时）
        float chamber_scale {2.2F};        ///< 溶洞放大倍数
    };

    class TreeCaveField : public ICaveField
    {
    public:
        explicit TreeCaveField(const TreeCaveFieldConfig & config);

        bool                isSolid(float x, float y, float z) const override;
        bool                raycast(const Point3 & origin, const Point3 & dir, float max_range, float & out_dist) const override;
        std::vector<Point3> sampleSurface() const override;

    private:
        struct Arm {
            std::vector<Point3> centerline;
            float               length {0.0F};
            bool                apply_chamber {false};
        };

        void buildArms();
        void pushArm(
                const Point3 & start, float dir_x, float dir_y, float dir_z, float arm_length, int n_segments,
                bool apply_chamber, float phase_y, float phase_z);
        void pushArmBezier(const Point3 & start, const Point3 & ctrl, const Point3 & end, int n_segments);

        float unitHash(int ix, int iy, int iz) const;
        float unitHash01(int ix, int iy, int iz) const;

        void buildArmCenterline(
                const Point3 & start, float dir_x, float dir_y, float dir_z, float arm_length, int n_segments,
                float phase_y, float phase_z, std::vector<Point3> & out_centerline, float & out_length) const;
        void branchDirection(
                float parent_x, float parent_y, float parent_z, float yaw, float pitch, float & out_x, float & out_y,
                float & out_z) const;
        bool tangentAtPolylineEnd(
                const std::vector<Point3> & polyline, float & out_x, float & out_y, float & out_z) const;

        float radiusProfile(float t, bool apply_chamber) const;
        float wallNoise(float wx, float wy, float wz) const;
        float localRadius(float t, bool apply_chamber, float wx, float wy, float wz) const;

        bool closestOnPolyline(
                const std::vector<Point3> & polyline, float polyline_length, float x, float y, float z,
                float & out_t, float & out_ax, float & out_ay, float & out_az, float & out_dist_sq) const;

        void samplePolylineSurface(
                const std::vector<Point3> & polyline, float polyline_length, bool apply_chamber,
                std::vector<Point3> & out_points) const;

        bool isExteriorShellPoint(
                float px, float py, float pz, float axis_x, float axis_y, float axis_z) const;

        void appendSurfaceRing(
                float cx, float cy, float cz, float tx, float ty, float tz, float radius, int points_per_ring,
                std::vector<Point3> & out_points) const;

        TreeCaveFieldConfig config_;
        std::vector<Arm>    arms_;
    };

}// namespace CaveWorld

#endif// CAVE_WORLD_TREECAVEFIELD_HPP
