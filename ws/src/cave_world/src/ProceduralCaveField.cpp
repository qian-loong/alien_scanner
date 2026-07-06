#include "cave_world/ProceduralCaveField.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace CaveWorld {

    namespace {

        constexpr float kTwoPi = 2.0F * 3.14159265358979323846F;

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

    }// namespace

    bool ProceduralCaveField::isExteriorShellPoint(
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

    void ProceduralCaveField::appendSurfaceRing(
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
            const float cxa   = std::cos(angle);
            const float cya   = std::sin(angle);
            const float px    = cx + radius * (cxa * nx + cya * bx);
            const float py    = cy + radius * (cxa * ny + cya * by);
            const float pz    = cz + radius * (cxa * nz + cya * bz);
            if(isExteriorShellPoint(px, py, pz, cx, cy, cz)) {
                out_points.push_back(Point3 {px, py, pz});
            }
        }
    }

    ProceduralCaveField::ProceduralCaveField(const ProceduralCaveFieldConfig & config)
        : config_(config)
    {
        buildCenterline();
    }

    void ProceduralCaveField::buildArmCenterline(
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

    void ProceduralCaveField::buildCenterline()
    {
        const int   n       = std::max(config_.n_segments, 1);
        const float phase_y = static_cast<float>(config_.seed) * 0.0013F;
        const float phase_z = static_cast<float>(config_.seed) * 0.0017F + 1.2F;

        left_branch_centerline_.clear();
        right_branch_centerline_.clear();
        branch_arm_length_ = 0.0F;

        buildArmCenterline(
                Point3 {0.0F, 0.0F, 0.0F}, 1.0F, 0.0F, 0.0F, config_.length, n, phase_y, phase_z,
                trunk_centerline_, trunk_length_);

        if(!config_.branch || trunk_centerline_.size() < 2U) {
            return;
        }

        const Point3 & junction = trunk_centerline_.back();
        const Point3 & prev     = trunk_centerline_[trunk_centerline_.size() - 2U];
        const float    tx       = junction.x - prev.x;
        const float    ty       = junction.y - prev.y;
        const float    tz       = junction.z - prev.z;
        const float    t_len    = length3(tx, ty, tz);
        if(t_len < 1e-4F) {
            return;
        }

        const float inv_t = 1.0F / t_len;
        const float ntx   = tx * inv_t;
        const float nty   = ty * inv_t;
        const float ntz   = tz * inv_t;

        float left_x  = 0.0F;
        float left_y  = 0.0F;
        float left_z  = 0.0F;
        float right_x = 0.0F;
        float right_y = 0.0F;
        float right_z = 0.0F;
        rotateAroundZ(ntx, nty, ntz, config_.branch_angle, left_x, left_y, left_z);
        rotateAroundZ(ntx, nty, ntz, -config_.branch_angle, right_x, right_y, right_z);

        const int branch_segments = std::max(n / 2, 8);
        buildArmCenterline(
                junction, left_x, left_y, left_z, config_.branch_length, branch_segments, phase_y + 0.7F,
                phase_z + 0.4F, left_branch_centerline_, branch_arm_length_);
        buildArmCenterline(
                junction, right_x, right_y, right_z, config_.branch_length, branch_segments, phase_y + 1.1F,
                phase_z + 0.9F, right_branch_centerline_, branch_arm_length_);
    }

    float ProceduralCaveField::radiusProfile(float t) const
    {
        const float clamped_t = clamp01(t);
        const float dt        = clamped_t - config_.chamber_at;
        const float bump      = std::exp(-dt * dt * 18.0F);
        return config_.base_radius * (1.0F + bump * (config_.chamber_scale - 1.0F));
    }

    float ProceduralCaveField::wallNoise(float wx, float wy, float wz) const
    {
        const float scale = 0.35F;
        return valueNoise(config_.seed + 17U, wx * scale, wy * scale, wz * scale);
    }

    float ProceduralCaveField::localRadius(float t, float wx, float wy, float wz) const
    {
        return radiusProfile(t) + config_.noise_scale * wallNoise(wx, wy, wz);
    }

    bool ProceduralCaveField::closestOnPolyline(
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

    bool ProceduralCaveField::isSolid(float x, float y, float z) const
    {
        auto check_polyline = [&](const std::vector<Point3> & polyline, float polyline_length,
                                  float radius_scale, bool apply_chamber) -> bool {
            float t       = 0.0F;
            float ax      = 0.0F;
            float ay      = 0.0F;
            float az      = 0.0F;
            float dist_sq = 0.0F;
            if(!closestOnPolyline(polyline, polyline_length, x, y, z, t, ax, ay, az, dist_sq)) {
                return true;
            }
            const float profile_t = apply_chamber ? t : 0.0F;
            const float radius    = localRadius(profile_t, ax, ay, az) * radius_scale;
            return dist_sq > radius * radius;
        };

        if(!check_polyline(trunk_centerline_, trunk_length_, 1.0F, true)) {
            return false;
        }
        if(!left_branch_centerline_.empty() && branch_arm_length_ > 0.0F) {
            if(!check_polyline(left_branch_centerline_, branch_arm_length_, 1.0F, false)) {
                return false;
            }
        }
        if(!right_branch_centerline_.empty() && branch_arm_length_ > 0.0F) {
            if(!check_polyline(right_branch_centerline_, branch_arm_length_, 1.0F, false)) {
                return false;
            }
        }
        return true;
    }

    bool ProceduralCaveField::raycast(
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

    void ProceduralCaveField::samplePolylineSurface(
            const std::vector<Point3> & polyline, float polyline_length, float radius_scale, bool apply_chamber,
            std::vector<Point3> & out_points) const
    {
        if(polyline.size() < 2U || polyline_length <= 0.0F) {
            return;
        }

        const int points_per_ring = std::max(16, config_.density / 10);
        out_points.reserve(
                out_points.size() + (polyline.size() - 1U) * static_cast<std::size_t>(points_per_ring));

        float traversed = 0.0F;
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

            const float mid_arc = traversed + seg_len * 0.5F;
            const float mid_x   = (a.x + b.x) * 0.5F;
            const float mid_y   = (a.y + b.y) * 0.5F;
            const float mid_z   = (a.z + b.z) * 0.5F;
            const float profile_t = apply_chamber ? (mid_arc / polyline_length) : 0.0F;
            const float radius    = localRadius(profile_t, mid_x, mid_y, mid_z) * radius_scale;

            appendSurfaceRing(mid_x, mid_y, mid_z, tx, ty, tz, radius, points_per_ring, out_points);
            traversed += seg_len;
        }
    }

    std::vector<Point3> ProceduralCaveField::sampleSurface() const
    {
        std::vector<Point3> points;
        samplePolylineSurface(trunk_centerline_, trunk_length_, 1.0F, true, points);

        if(!left_branch_centerline_.empty() && branch_arm_length_ > 0.0F) {
            samplePolylineSurface(left_branch_centerline_, branch_arm_length_, 1.0F, false, points);
        }
        if(!right_branch_centerline_.empty() && branch_arm_length_ > 0.0F) {
            samplePolylineSurface(right_branch_centerline_, branch_arm_length_, 1.0F, false, points);
        }
        return points;
    }

}// namespace CaveWorld
