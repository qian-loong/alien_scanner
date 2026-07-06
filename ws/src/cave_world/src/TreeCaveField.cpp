#include "cave_world/TreeCaveField.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace CaveWorld {

    namespace {

        float clamp01(float v)
        {
            return std::max(0.0F, std::min(1.0F, v));
        }

        float length3(float x, float y, float z)
        {
            return std::sqrt(x * x + y * y + z * z);
        }

        float dot3(float ax, float ay, float az, float bx, float by, float bz)
        {
            return ax * bx + ay * by + az * bz;
        }

        std::uint32_t hashMix(std::uint32_t seed, int ix, int iy, int iz)
        {
            std::uint32_t h = seed;
            h ^= static_cast<std::uint32_t>(ix) * 73856093U;
            h ^= static_cast<std::uint32_t>(iy) * 19349663U;
            h ^= static_cast<std::uint32_t>(iz) * 83492791U;
            h ^= h >> 16;
            h *= 0x7feb352dU;
            h ^= h >> 15;
            h *= 0x846ca68bU;
            h ^= h >> 16;
            return h;
        }

        float hashToUnit(std::uint32_t seed, int ix, int iy, int iz)
        {
            const std::uint32_t h = hashMix(seed, ix, iy, iz);
            return static_cast<float>(h & 0x00FFFFFFU) / static_cast<float>(0x01000000U) * 2.0F - 1.0F;
        }

        float smoothStep(float t)
        {
            return t * t * (3.0F - 2.0F * t);
        }

        float valueNoise(std::uint32_t seed, float x, float y, float z)
        {
            const float fx = std::floor(x);
            const float fy = std::floor(y);
            const float fz = std::floor(z);
            const int   ix = static_cast<int>(fx);
            const int   iy = static_cast<int>(fy);
            const int   iz = static_cast<int>(fz);
            const float tx = smoothStep(x - fx);
            const float ty = smoothStep(y - fy);
            const float tz = smoothStep(z - fz);

            const float c000 = hashToUnit(seed, ix, iy, iz);
            const float c100 = hashToUnit(seed, ix + 1, iy, iz);
            const float c010 = hashToUnit(seed, ix, iy + 1, iz);
            const float c110 = hashToUnit(seed, ix + 1, iy + 1, iz);
            const float c001 = hashToUnit(seed, ix, iy, iz + 1);
            const float c101 = hashToUnit(seed, ix + 1, iy, iz + 1);
            const float c011 = hashToUnit(seed, ix, iy + 1, iz + 1);
            const float c111 = hashToUnit(seed, ix + 1, iy + 1, iz + 1);

            const float x00 = c000 + (c100 - c000) * tx;
            const float x10 = c010 + (c110 - c010) * tx;
            const float x01 = c001 + (c101 - c001) * tx;
            const float x11 = c011 + (c111 - c011) * tx;
            const float y0  = x00 + (x10 - x00) * ty;
            const float y1  = x01 + (x11 - x01) * ty;
            return y0 + (y1 - y0) * tz;
        }

        void rotateAroundZ(float tx, float ty, float tz, float angle, float & out_x, float & out_y, float & out_z)
        {
            const float c = std::cos(angle);
            const float s = std::sin(angle);
            out_x         = c * tx - s * ty;
            out_y         = s * tx + c * ty;
            out_z         = tz;
        }

        bool normalize3(float & x, float & y, float & z)
        {
            const float len = length3(x, y, z);
            if(len < 1e-5F) {
                return false;
            }
            const float inv = 1.0F / len;
            x *= inv;
            y *= inv;
            z *= inv;
            return true;
        }

    }// namespace

    TreeCaveField::TreeCaveField(const TreeCaveFieldConfig & config)
        : config_(config)
    {
        buildArms();
    }

    float TreeCaveField::unitHash(int ix, int iy, int iz) const
    {
        return hashToUnit(config_.seed + 911U, ix, iy, iz);
    }

    float TreeCaveField::unitHash01(int ix, int iy, int iz) const
    {
        return unitHash(ix, iy, iz) * 0.5F + 0.5F;
    }

    void TreeCaveField::branchDirection(
            float parent_x, float parent_y, float parent_z, float yaw, float pitch, float & out_x, float & out_y,
            float & out_z) const
    {
        float lx = 0.0F;
        float ly = 0.0F;
        float lz = 0.0F;
        rotateAroundZ(parent_x, parent_y, parent_z, yaw, lx, ly, lz);

        const float cp = std::cos(pitch);
        const float sp = std::sin(pitch);
        out_x          = lx * cp;
        out_y          = ly * cp;
        out_z          = lz * cp + sp;
        if(!normalize3(out_x, out_y, out_z)) {
            out_x = parent_x;
            out_y = parent_y;
            out_z = parent_z;
        }
    }

    void TreeCaveField::buildArmCenterline(
            const Point3 & start, float dir_x, float dir_y, float dir_z, float arm_length, int n_segments,
            float phase_y, float phase_z, std::vector<Point3> & out_centerline, float & out_length) const
    {
        out_centerline.clear();
        if(arm_length <= 0.0F || n_segments <= 0) {
            out_length = 0.0F;
            return;
        }

        const float segment_length = arm_length / static_cast<float>(n_segments);
        const float wobble_y0      = std::sin(phase_y);
        const float wobble_z0      = std::sin(phase_z);
        const float wobble_z1      = std::sin(phase_y * 0.5F);
        out_centerline.reserve(static_cast<std::size_t>(n_segments) + 1U);
        for(int i = 0; i <= n_segments; ++i) {
            const float s = static_cast<float>(i) * segment_length;
            out_centerline.push_back(Point3 {
                    start.x + dir_x * s + 1.5F * (std::sin(s * 0.15F + phase_y) - wobble_y0),
                    start.y + dir_y * s + 1.0F * (std::sin(s * 0.11F + phase_z) - wobble_z0),
                    start.z + dir_z * s + 0.6F * (std::sin(s * 0.09F + phase_y * 0.5F) - wobble_z1)});
        }
        out_length = arm_length;
    }

    bool TreeCaveField::tangentAtPolylineEnd(
            const std::vector<Point3> & polyline, float & out_x, float & out_y, float & out_z) const
    {
        if(polyline.size() < 2U) {
            return false;
        }
        const Point3 & a = polyline[polyline.size() - 2U];
        const Point3 & b = polyline.back();
        out_x            = b.x - a.x;
        out_y            = b.y - a.y;
        out_z            = b.z - a.z;
        return normalize3(out_x, out_y, out_z);
    }

    void TreeCaveField::pushArm(
            const Point3 & start, float dir_x, float dir_y, float dir_z, float arm_length, int n_segments,
            bool apply_chamber, float phase_y, float phase_z)
    {
        Arm arm;
        buildArmCenterline(start, dir_x, dir_y, dir_z, arm_length, n_segments, phase_y, phase_z, arm.centerline, arm.length);
        arm.apply_chamber = apply_chamber;
        arms_.push_back(std::move(arm));
    }

    void TreeCaveField::pushArmBezier(const Point3 & start, const Point3 & ctrl, const Point3 & end, int n_segments)
    {
        const int segments = std::max(n_segments, 2);
        Arm       arm;
        arm.centerline.reserve(static_cast<std::size_t>(segments) + 1U);

        Point3 prev {};
        float  total = 0.0F;
        for(int i = 0; i <= segments; ++i) {
            const float u  = static_cast<float>(i) / static_cast<float>(segments);
            const float u1 = 1.0F - u;
            const Point3 p {
                    u1 * u1 * start.x + 2.0F * u1 * u * ctrl.x + u * u * end.x,
                    u1 * u1 * start.y + 2.0F * u1 * u * ctrl.y + u * u * end.y,
                    u1 * u1 * start.z + 2.0F * u1 * u * ctrl.z + u * u * end.z};
            if(i > 0) {
                total += length3(p.x - prev.x, p.y - prev.y, p.z - prev.z);
            }
            arm.centerline.push_back(p);
            prev = p;
        }

        arm.length        = total;
        arm.apply_chamber = false;
        if(total > 1e-3F) {
            arms_.push_back(std::move(arm));
        }
    }

    void TreeCaveField::buildArms()
    {
        arms_.clear();

        const int   n       = std::max(config_.n_segments, 1);
        const float phase_y = static_cast<float>(config_.seed) * 0.0011F;
        const float phase_z = static_cast<float>(config_.seed) * 0.0019F + 0.8F;
        const float jitter  = config_.asymmetry;

        const int corridor_segments = std::max(n / 2, 10);
        const int exit_segments     = std::max(n / 3, 8);

        // 入口 → 分叉 A
        pushArm(
                Point3 {0.0F, 0.0F, 0.0F}, 1.0F, 0.0F, 0.0F, config_.approach_length, n,
                config_.chamber_on_approach, phase_y, phase_z);

        const Point3 junction_a = arms_.front().centerline.back();
        float        trunk_tx   = 1.0F;
        float        trunk_ty   = 0.0F;
        float        trunk_tz   = 0.0F;
        if(!tangentAtPolylineEnd(arms_.front().centerline, trunk_tx, trunk_ty, trunk_tz)) {
            return;
        }

        // A → B 直连（环的内侧边）
        const float loop_yaw   = config_.loop_yaw + unitHash(1, 0, 0) * jitter * 0.30F;
        const float loop_pitch = config_.vertical_step + unitHash(1, 1, 0) * jitter * 0.15F;
        const float direct_len = config_.loop_direct_length * (0.90F + 0.20F * unitHash01(1, 2, 0));

        float ab_dx = 0.0F;
        float ab_dy = 0.0F;
        float ab_dz = 0.0F;
        branchDirection(trunk_tx, trunk_ty, trunk_tz, loop_yaw, loop_pitch, ab_dx, ab_dy, ab_dz);
        pushArm(junction_a, ab_dx, ab_dy, ab_dz, direct_len, corridor_segments, false, phase_y + 0.6F, phase_z + 0.4F);

        const Point3 junction_b = arms_.back().centerline.back();
        float        b_tx       = ab_dx;
        float        b_ty       = ab_dy;
        float        b_tz       = ab_dz;
        if(!tangentAtPolylineEnd(arms_.back().centerline, b_tx, b_ty, b_tz)) {
            return;
        }

        // A → B 外环（相对直连向左侧鼓出，与直连围成环）
        float lateral_x = -ab_dy;
        float lateral_y = ab_dx;
        float lateral_z = 0.0F;
        if(!normalize3(lateral_x, lateral_y, lateral_z)) {
            lateral_x = 0.0F;
            lateral_y = 1.0F;
            lateral_z = 0.0F;
        }

        // 二次 Bezier 中点偏移为控制点偏移的一半，故控制点用 2×bulge
        const float  bulge = config_.loop_bulge * (0.90F + 0.20F * unitHash01(2, 0, 0));
        const Point3 loop_ctrl {
                (junction_a.x + junction_b.x) * 0.5F + lateral_x * bulge * 2.0F,
                (junction_a.y + junction_b.y) * 0.5F + lateral_y * bulge * 2.0F,
                (junction_a.z + junction_b.z) * 0.5F + unitHash(2, 1, 0) * 1.2F};
        pushArmBezier(junction_a, loop_ctrl, junction_b, corridor_segments);

        // B → 出口 1
        const float exit1_yaw   = unitHash(3, 0, 0) * jitter * 0.30F;
        const float exit1_pitch = config_.vertical_step + unitHash(3, 1, 0) * jitter * 0.12F;
        const float exit1_len   = config_.exit1_length * (0.90F + 0.20F * unitHash01(3, 2, 0));

        float e1_dx = 0.0F;
        float e1_dy = 0.0F;
        float e1_dz = 0.0F;
        branchDirection(b_tx, b_ty, b_tz, exit1_yaw, exit1_pitch, e1_dx, e1_dy, e1_dz);
        pushArm(junction_b, e1_dx, e1_dy, e1_dz, exit1_len, exit_segments, false, phase_y + 1.1F, phase_z + 0.7F);

        // A → C 右廊
        const float right_yaw   = config_.right_yaw + unitHash(4, 0, 0) * jitter * 0.25F;
        const float right_pitch = config_.vertical_step * 0.85F + unitHash(4, 1, 0) * jitter * 0.12F;
        const float right_len   = config_.right_corridor_length * (0.90F + 0.20F * unitHash01(4, 2, 0));

        float ac_dx = 0.0F;
        float ac_dy = 0.0F;
        float ac_dz = 0.0F;
        branchDirection(trunk_tx, trunk_ty, trunk_tz, right_yaw, right_pitch, ac_dx, ac_dy, ac_dz);
        pushArm(junction_a, ac_dx, ac_dy, ac_dz, right_len, corridor_segments, false, phase_y + 0.9F, phase_z + 0.5F);

        const Point3 junction_c = arms_.back().centerline.back();
        float        c_tx       = ac_dx;
        float        c_ty       = ac_dy;
        float        c_tz       = ac_dz;
        if(!tangentAtPolylineEnd(arms_.back().centerline, c_tx, c_ty, c_tz)) {
            return;
        }

        // C → 出口 2 / 出口 3
        const float spread     = config_.exit_yaw_spread * (0.85F + 0.30F * unitHash01(5, 0, 0));
        const float exit_pitch = config_.vertical_step * 1.10F + unitHash(5, 1, 0) * jitter * 0.10F;
        const float exit_len   = config_.exit_arm_length * (0.90F + 0.20F * unitHash01(5, 2, 0));

        float e2_dx = 0.0F;
        float e2_dy = 0.0F;
        float e2_dz = 0.0F;
        float e3_dx = 0.0F;
        float e3_dy = 0.0F;
        float e3_dz = 0.0F;
        branchDirection(c_tx, c_ty, c_tz, spread, exit_pitch, e2_dx, e2_dy, e2_dz);
        branchDirection(
                c_tx, c_ty, c_tz, -spread * (0.75F + 0.25F * unitHash01(5, 3, 0)), exit_pitch * 1.05F, e3_dx,
                e3_dy, e3_dz);

        pushArm(junction_c, e2_dx, e2_dy, e2_dz, exit_len, exit_segments, false, phase_y + 1.4F, phase_z + 0.8F);
        pushArm(
                junction_c, e3_dx, e3_dy, e3_dz, exit_len * (0.92F + 0.12F * unitHash01(5, 4, 0)), exit_segments,
                false, phase_y + 1.8F, phase_z + 1.0F);
    }

    float TreeCaveField::radiusProfile(float t, bool apply_chamber) const
    {
        if(!apply_chamber) {
            return config_.base_radius;
        }
        const float clamped_t = clamp01(t);
        const float dt        = clamped_t - config_.chamber_at;
        const float bump      = std::exp(-dt * dt * 18.0F);
        return config_.base_radius * (1.0F + bump * (config_.chamber_scale - 1.0F));
    }

    float TreeCaveField::wallNoise(float wx, float wy, float wz) const
    {
        const float scale = 0.35F;
        return valueNoise(config_.seed + 29U, wx * scale, wy * scale, wz * scale);
    }

    float TreeCaveField::localRadius(float t, bool apply_chamber, float wx, float wy, float wz) const
    {
        return radiusProfile(t, apply_chamber) + config_.noise_scale * wallNoise(wx, wy, wz);
    }

    bool TreeCaveField::closestOnPolyline(
            const std::vector<Point3> & polyline, float polyline_length, float x, float y, float z,
            float & out_t, float & out_ax, float & out_ay, float & out_az, float & out_dist_sq) const
    {
        if(polyline.size() < 2U || polyline_length <= 0.0F) {
            return false;
        }

        bool  found        = false;
        float best_dist_sq = 0.0F;
        float best_t       = 0.0F;
        float best_ax      = 0.0F;
        float best_ay      = 0.0F;
        float best_az      = 0.0F;

        float traversed = 0.0F;
        for(std::size_t i = 0; i + 1U < polyline.size(); ++i) {
            const Point3 & a          = polyline[i];
            const Point3 & b          = polyline[i + 1U];
            const float    sx         = b.x - a.x;
            const float    sy         = b.y - a.y;
            const float    sz         = b.z - a.z;
            const float    seg_len_sq = sx * sx + sy * sy + sz * sz;
            if(seg_len_sq < 1e-8F) {
                continue;
            }

            const float seg_len = std::sqrt(seg_len_sq);
            const float u =
                    std::max(0.0F, std::min(1.0F, dot3(x - a.x, y - a.y, z - a.z, sx, sy, sz) / seg_len_sq));
            const float ax      = a.x + sx * u;
            const float ay      = a.y + sy * u;
            const float az      = a.z + sz * u;
            const float dx      = x - ax;
            const float dy      = y - ay;
            const float dz      = z - az;
            const float dist_sq = dx * dx + dy * dy + dz * dz;
            const float t       = (traversed + seg_len * u) / polyline_length;

            if(!found || dist_sq < best_dist_sq) {
                found        = true;
                best_dist_sq = dist_sq;
                best_t       = t;
                best_ax      = ax;
                best_ay      = ay;
                best_az      = az;
            }
            traversed += seg_len;
        }

        if(!found) {
            return false;
        }

        out_t       = best_t;
        out_ax      = best_ax;
        out_ay      = best_ay;
        out_az      = best_az;
        out_dist_sq = best_dist_sq;
        return true;
    }

    bool TreeCaveField::isExteriorShellPoint(
            float px, float py, float pz, float axis_x, float axis_y, float axis_z) const
    {
        const float ox    = px - axis_x;
        const float oy    = py - axis_y;
        const float oz    = pz - axis_z;
        const float o_len = length3(ox, oy, oz);
        if(o_len < 1e-5F) {
            return false;
        }

        const float inv        = 1.0F / o_len;
        const float filter_eps = std::max(0.08F, config_.base_radius * 0.04F);
        return isSolid(px + ox * inv * filter_eps, py + oy * inv * filter_eps, pz + oz * inv * filter_eps);
    }

    void TreeCaveField::appendSurfaceRing(
            float cx, float cy, float cz, float tx, float ty, float tz, float radius, int points_per_ring,
            std::vector<Point3> & out_points) const
    {
        float ref_x = 0.0F;
        float ref_y = 0.0F;
        float ref_z = 1.0F;
        if(std::fabs(dot3(tx, ty, tz, ref_x, ref_y, ref_z)) > 0.95F) {
            ref_x = 1.0F;
            ref_y = 0.0F;
            ref_z = 0.0F;
        }

        float       nx    = ty * ref_z - tz * ref_y;
        float       ny    = tz * ref_x - tx * ref_z;
        float       nz    = tx * ref_y - ty * ref_x;
        const float n_len = length3(nx, ny, nz);
        if(n_len < 1e-6F) {
            return;
        }
        nx /= n_len;
        ny /= n_len;
        nz /= n_len;

        float       bx    = ty * nz - tz * ny;
        float       by    = tz * nx - tx * nz;
        float       bz    = tx * ny - ty * nx;
        const float b_len = length3(bx, by, bz);
        if(b_len < 1e-6F) {
            return;
        }
        bx /= b_len;
        by /= b_len;
        bz /= b_len;

        for(int j = 0; j < points_per_ring; ++j) {
            const float angle =
                    (2.0F * 3.14159265358979323846F) * static_cast<float>(j) / static_cast<float>(points_per_ring);
            const float cxa = std::cos(angle);
            const float cya = std::sin(angle);
            const float px  = cx + radius * (cxa * nx + cya * bx);
            const float py  = cy + radius * (cxa * ny + cya * by);
            const float pz  = cz + radius * (cxa * nz + cya * bz);
            if(isExteriorShellPoint(px, py, pz, cx, cy, cz)) {
                out_points.push_back(Point3 {px, py, pz});
            }
        }
    }

    bool TreeCaveField::isSolid(float x, float y, float z) const
    {
        for(const Arm & arm : arms_) {
            if(arm.centerline.size() < 2U || arm.length <= 0.0F) {
                continue;
            }

            float t       = 0.0F;
            float ax      = 0.0F;
            float ay      = 0.0F;
            float az      = 0.0F;
            float dist_sq = 0.0F;
            if(!closestOnPolyline(arm.centerline, arm.length, x, y, z, t, ax, ay, az, dist_sq)) {
                continue;
            }

            const float radius = localRadius(t, arm.apply_chamber, ax, ay, az);
            if(dist_sq <= radius * radius) {
                return false;
            }
        }
        return true;
    }

    bool TreeCaveField::raycast(
            const Point3 & origin, const Point3 & dir, float max_range, float & out_dist) const
    {
        const float dir_len = length3(dir.x, dir.y, dir.z);
        if(dir_len < 1e-6F || max_range <= 0.0F) {
            return false;
        }

        const float     inv_len = 1.0F / dir_len;
        const float     dx      = dir.x * inv_len;
        const float     dy      = dir.y * inv_len;
        const float     dz      = dir.z * inv_len;
        constexpr float kStep   = 0.08F;

        for(float t = kStep; t <= max_range; t += kStep) {
            const float px = origin.x + dx * t;
            const float py = origin.y + dy * t;
            const float pz = origin.z + dz * t;
            if(isSolid(px, py, pz)) {
                out_dist = t;
                return true;
            }
        }
        return false;
    }

    void TreeCaveField::samplePolylineSurface(
            const std::vector<Point3> & polyline, float polyline_length, bool apply_chamber,
            std::vector<Point3> & out_points) const
    {
        if(polyline.size() < 2U || polyline_length <= 0.0F) {
            return;
        }

        const int points_per_ring = std::max(16, config_.density / 10);
        float     traversed       = 0.0F;
        for(std::size_t i = 0; i + 1U < polyline.size(); ++i) {
            const Point3 & a       = polyline[i];
            const Point3 & b       = polyline[i + 1U];
            const float    sx      = b.x - a.x;
            const float    sy      = b.y - a.y;
            const float    sz      = b.z - a.z;
            const float    seg_len = length3(sx, sy, sz);
            if(seg_len < 1e-6F) {
                continue;
            }

            const float inv_seg = 1.0F / seg_len;
            const float tx      = sx * inv_seg;
            const float ty      = sy * inv_seg;
            const float tz      = sz * inv_seg;

            const float mid_arc   = traversed + seg_len * 0.5F;
            const float mid_x     = (a.x + b.x) * 0.5F;
            const float mid_y     = (a.y + b.y) * 0.5F;
            const float mid_z     = (a.z + b.z) * 0.5F;
            const float profile_t = mid_arc / polyline_length;
            const float radius    = localRadius(profile_t, apply_chamber, mid_x, mid_y, mid_z);

            appendSurfaceRing(mid_x, mid_y, mid_z, tx, ty, tz, radius, points_per_ring, out_points);
            traversed += seg_len;
        }
    }

    std::vector<Point3> TreeCaveField::sampleSurface() const
    {
        std::vector<Point3> points;
        for(const Arm & arm : arms_) {
            samplePolylineSurface(arm.centerline, arm.length, arm.apply_chamber, points);
        }
        return points;
    }

}// namespace CaveWorld
