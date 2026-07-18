#include "swarm_controller/FrontierGeometryDemo.hpp"
#include "swarm_controller/GlobalFrontierDetector.hpp"
#include "swarm_controller/OctoMapBuilder.hpp"

#include "cave_world/ProceduralCaveField.hpp"
#include "drone_scanner/FakeLidar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

namespace SwarmController {

    namespace {

        constexpr float PI = 3.14159265358979323846F;
        constexpr float REAR_TUNNEL_MARGIN = 0.8F;
        constexpr std::array<const char *, 5U> COMPOSED_STAGES {
                "standard_tunnel_geometry", "single_ring", "bootstrap_yaw_sweep",
                "validated_observation_hop", "accumulated_frontier",
        };
        constexpr DemoColor WHITE {0.92F, 0.95F, 0.97F, 0.85F};
        constexpr DemoColor RED {1.0F, 0.25F, 0.22F, 0.95F};
        constexpr DemoColor GREEN {0.25F, 0.88F, 0.34F, 0.85F};
        constexpr DemoColor BLUE {0.25F, 0.55F, 1.0F, 0.90F};
        constexpr DemoColor CYAN {0.10F, 0.82F, 0.88F, 0.52F};
        constexpr DemoColor YELLOW {1.0F, 0.84F, 0.20F, 0.82F};
        constexpr DemoColor ORANGE {1.0F, 0.38F, 0.10F, 0.92F};
        constexpr DemoColor MAGENTA {0.82F, 0.32F, 1.0F, 0.92F};
        constexpr DemoColor GRAY {0.36F, 0.42F, 0.46F, 0.30F};
        constexpr DemoColor DARK_GRAY {0.20F, 0.24F, 0.27F, 0.32F};

        Point3f point(const float x, const float y, const float z)
        {
            return Point3f {x, y, z};
        }

        Point3f add(const Point3f & lhs, const Point3f & rhs)
        {
            return point(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
        }

        Point3f multiply(const Point3f & value, const float scalar)
        {
            return point(value.x * scalar, value.y * scalar, value.z * scalar);
        }

        bool finitePositive(const double value)
        {
            return std::isfinite(value) && value > 0.0F;
        }

        bool validMode(const std::string & mode)
        {
            constexpr std::array<const char *, 6U> MODES {
                    "combined", "ring_geometry", "voxel_observation",
                    "vertical_columns", "support_evidence", "component_fragmentation",
            };
            return std::any_of(MODES.begin(), MODES.end(), [&](const char * candidate) {
                return mode == candidate;
            });
        }

        bool validStage(const std::string & stage)
        {
            return std::any_of(
                    COMPOSED_STAGES.begin(), COMPOSED_STAGES.end(),
                    [&](const char * candidate) {
                        return stage == candidate;
                    });
        }

        void addCube(
                DemoScene & scene, const std::string & ns, const Point3f & center,
                const Point3f & scale, const DemoColor color)
        {
            scene.cubes.push_back(DemoCube {
                    ns, scene.nextId(ns), center, scale, color});
        }

        void addSphere(
                DemoScene & scene, const std::string & ns, const Point3f & center,
                const float diameter, const DemoColor color)
        {
            scene.spheres.push_back(DemoSphere {
                    ns, scene.nextId(ns), center, diameter, color});
        }

        void addLine(
                DemoScene & scene, const std::string & ns, const Point3f & start,
                const Point3f & end, const float width, const DemoColor color)
        {
            scene.lines.push_back(DemoLine {
                    ns, scene.nextId(ns), start, end, width, color});
        }

        void addArrow(
                DemoScene & scene, const std::string & ns, const Point3f & start,
                const Point3f & end, const DemoColor color, const float scale = 1.0F)
        {
            scene.arrows.push_back(DemoArrow {
                    ns, scene.nextId(ns), start, end,
                    0.035F * scale, 0.10F * scale, 0.16F * scale, color});
        }

        void addText(
                DemoScene & scene, const std::string & ns, const Point3f & position,
                std::string text, const DemoColor color = WHITE,
                const float height = 0.18F)
        {
            scene.texts.push_back(DemoText {
                    ns, scene.nextId(ns), position, std::move(text), height, color});
        }

        void addTriangle(
                DemoScene & scene, const std::string & ns, const Point3f & first,
                const Point3f & second, const Point3f & third, const DemoColor color)
        {
            scene.triangles.push_back(DemoTriangle {
                    ns, scene.nextId(ns), first, second, third, color});
        }

        void addWireBox(
                DemoScene & scene, const std::string & ns, const Point3f & center,
                const Point3f & scale, const DemoColor color, const float width = 0.018F)
        {
            const float hx = 0.5F * scale.x;
            const float hy = 0.5F * scale.y;
            const float hz = 0.5F * scale.z;
            const std::array<Point3f, 8U> corners {
                    point(center.x - hx, center.y - hy, center.z - hz),
                    point(center.x + hx, center.y - hy, center.z - hz),
                    point(center.x + hx, center.y + hy, center.z - hz),
                    point(center.x - hx, center.y + hy, center.z - hz),
                    point(center.x - hx, center.y - hy, center.z + hz),
                    point(center.x + hx, center.y - hy, center.z + hz),
                    point(center.x + hx, center.y + hy, center.z + hz),
                    point(center.x - hx, center.y + hy, center.z + hz),
            };
            constexpr std::array<std::pair<std::size_t, std::size_t>, 12U> EDGES {
                    std::pair<std::size_t, std::size_t> {0U, 1U}, {1U, 2U},
                    {2U, 3U}, {3U, 0U}, {4U, 5U}, {5U, 6U},
                    {6U, 7U}, {7U, 4U}, {0U, 4U}, {1U, 5U},
                    {2U, 6U}, {3U, 7U},
            };
            for(const auto & [first, second] : EDGES) {
                addLine(scene, ns, corners[first], corners[second], width, color);
            }
        }

        Point3f pitchedDirection(
                const float phi, const float sin_pitch, const float cos_pitch)
        {
            const float plane_vertical = std::sin(phi);
            return point(
                    plane_vertical * sin_pitch,
                    std::cos(phi),
                    plane_vertical * cos_pitch);
        }

        float cylinderHitDistance(const Point3f & direction, const float radius)
        {
            const float radial = std::hypot(direction.y, direction.z);
            return radial <= std::numeric_limits<float>::epsilon()
                            ? 0.0F
                            : radius / radial;
        }

        struct CapturedScan {
            Point3f origin {};
            std::vector<RayReturn> returns;
        };

        std::vector<RayReturn> mapReturns(
                const std::vector<DroneScanner::LidarReturn> & lidar_returns,
                const Point3f & origin, const float yaw)
        {
            const float cos_yaw = std::cos(yaw);
            const float sin_yaw = std::sin(yaw);
            std::vector<RayReturn> result;
            result.reserve(lidar_returns.size());
            for(const DroneScanner::LidarReturn & lidar_return : lidar_returns) {
                result.push_back(RayReturn {
                        Point3f {
                                origin.x + cos_yaw * lidar_return.x
                                        - sin_yaw * lidar_return.y,
                                origin.y + sin_yaw * lidar_return.x
                                        + cos_yaw * lidar_return.y,
                                origin.z + lidar_return.z},
                        lidar_return.range,
                        lidar_return.hit});
            }
            return result;
        }

        CapturedScan insertYawSweep(
                const DroneScanner::FakeLidar & lidar, OctoMapBuilder & builder,
                const Point3f & position, const std::size_t yaw_steps,
                std::size_t & scan_frames, std::size_t & ray_returns)
        {
            CapturedScan focus;
            focus.origin = position;
            for(std::size_t step = 0U; step < yaw_steps; ++step) {
                const float yaw = 2.0F * PI * static_cast<float>(step)
                                  / static_cast<float>(yaw_steps);
                const DroneScanner::Pose3D pose {
                        position.x, position.y, position.z, yaw};
                std::vector<RayReturn> returns = mapReturns(
                        lidar.scanReturns(pose), position, yaw);
                builder.insertScan(position, returns);
                ++scan_frames;
                ray_returns += returns.size();
                if(step == (3U * yaw_steps) / 4U) {
                    focus.returns = std::move(returns);
                }
            }
            return focus;
        }

        std::size_t selectedRayIndex(
                const float selected_phi_degrees, const std::size_t ray_count)
        {
            float normalized = std::fmod(selected_phi_degrees, 360.0F);
            if(normalized < 0.0F) {
                normalized += 360.0F;
            }
            const float scaled = normalized / 360.0F * static_cast<float>(ray_count);
            return static_cast<std::size_t>(std::lround(scaled)) % ray_count;
        }

        struct TreeCounts {
            std::size_t known {};
            std::size_t occupied {};
        };

        TreeCounts treeCounts(const octomap::OcTree & tree)
        {
            TreeCounts counts;
            for(auto leaf = tree.begin_leafs(), end = tree.end_leafs(); leaf != end; ++leaf) {
                ++counts.known;
                if(tree.isNodeOccupied(*leaf)) {
                    ++counts.occupied;
                }
            }
            return counts;
        }

    }// namespace

    struct FrontierGeometryDemo::DetectorReplay {
        std::shared_ptr<CaveWorld::ProceduralCaveField> field;
        CapturedScan initial_focus_scan;
        CapturedScan final_focus_scan;
        octomap::OcTree bootstrap_tree;
        octomap::OcTree accumulated_tree;
        TreeCounts single_ring_counts;
        TreeCounts bootstrap_counts;
        std::size_t bootstrap_scan_frames {};
        std::size_t bootstrap_ray_returns {};
        Point3f hop_goal {};
        DemoPipelineSummary summary;
    };

    void DemoScene::clear()
    {
        cubes.clear();
        spheres.clear();
        lines.clear();
        arrows.clear();
        triangles.clear();
        texts.clear();
        pipeline.reset();
        counters_.clear();
    }

    std::int32_t DemoScene::nextId(const std::string & ns)
    {
        const auto found = std::find_if(
                counters_.begin(), counters_.end(), [&](const NamespaceCounter & counter) {
                    return counter.ns == ns;
                });
        if(found != counters_.end()) {
            return found->next++;
        }
        counters_.push_back(NamespaceCounter {ns, 1});
        return 0;
    }

    FrontierGeometryDemo::FrontierGeometryDemo(FrontierGeometryDemoConfig config)
        : config_(std::move(config))
    {
        if(!validMode(config_.mode) || !validStage(config_.stage)
           || !finitePositive(config_.tunnel_radius)
           || !finitePositive(config_.tunnel_length)
           || !std::isfinite(config_.tunnel_center_z)
           || !std::isfinite(config_.pitch_degrees)
           || !std::isfinite(config_.selected_phi_degrees)
           || std::fabs(config_.pitch_degrees) >= 80.0F
           || config_.tunnel_radius
                              * std::fabs(std::tan(
                                      config_.pitch_degrees * PI / 180.0F))
                      > std::min(REAR_TUNNEL_MARGIN, config_.tunnel_length)
           || config_.tunnel_ring_segments < 12U || config_.ray_count < 8U
           || config_.yaw_steps < 4U || !finitePositive(config_.lidar_max_range)
           || !std::isfinite(config_.initial_x)
           || !std::isfinite(config_.initial_y)
           || !finitePositive(config_.hop_distance)
           || !finitePositive(config_.voxel_resolution)
           || config_.voxel_radial_samples < 2U
           || config_.voxel_angular_samples < 6U
           || config_.voxel_thickness_layers == 0U
           || !finitePositive(config_.voxel_visual_scale)
           || config_.column_stride_voxels == 0U || config_.min_z_layers == 0U
           || !finitePositive(config_.min_z_span)
           || !finitePositive(config_.support_depth)
           || config_.min_component_columns == 0U
           || config_.max_trace_candidates == 0U
           || config_.max_trace_support_samples == 0U
           || config_.max_trace_components == 0U
           || config_.max_trace_geometry_elements == 0U)
        {
            throw std::invalid_argument("invalid frontier geometry demo configuration");
        }
    }

    DemoScene FrontierGeometryDemo::buildScene() const
    {
        DemoScene scene;
        const bool combined = config_.mode == "combined";
        if(config_.mode == "ring_geometry") {
            appendRingGeometry(scene);
        }
        if(config_.mode == "voxel_observation") {
            appendRingGeometry(scene);
            appendVoxelObservation(scene);
        }
        if(combined) {
            if(config_.stage == "standard_tunnel_geometry") {
                appendRingGeometry(scene);
            } else {
                const DetectorReplay replay = buildDetectorReplay();
                appendDetectorFrontierStage(scene, replay, config_.stage);
            }
        } else if(config_.mode == "vertical_columns"
                  || config_.mode == "support_evidence"
                  || config_.mode == "component_fragmentation") {
            appendFrontierPipeline(scene);
        }
        appendSceneTitle(scene, config_.stage);
        return scene;
    }

    std::vector<DemoStageScene> FrontierGeometryDemo::buildStageScenes() const
    {
        if(config_.mode != "combined") {
            return std::vector<DemoStageScene> {
                    DemoStageScene {config_.stage, buildScene()}};
        }

        const DetectorReplay replay = buildDetectorReplay();
        std::vector<DemoStageScene> stages;
        stages.reserve(COMPOSED_STAGES.size());
        for(const char * stage : COMPOSED_STAGES) {
            DemoScene scene;
            if(std::string(stage) == "standard_tunnel_geometry") {
                appendRingGeometry(scene);
            } else {
                appendDetectorFrontierStage(scene, replay, stage);
            }
            appendSceneTitle(scene, stage);
            stages.push_back(DemoStageScene {stage, std::move(scene)});
        }
        return stages;
    }

    void FrontierGeometryDemo::appendSceneTitle(
            DemoScene & scene, const std::string & stage) const
    {
        if(!config_.show_labels) {
            return;
        }

        std::string title = "SYNTHETIC FRONTIER GEOMETRY DEMO";
        if(config_.mode == "combined") {
            const auto found = std::find(
                    COMPOSED_STAGES.begin(), COMPOSED_STAGES.end(), stage);
            const std::size_t index = static_cast<std::size_t>(
                    std::distance(COMPOSED_STAGES.begin(), found));
            title = "STAGE " + std::to_string(index) + "/4: ";
            if(stage == "standard_tunnel_geometry") {
                title += "STANDARD TUNNEL GEOMETRY";
            } else if(stage == "single_ring") {
                title += "SINGLE RING";
            } else if(stage == "bootstrap_yaw_sweep") {
                title += "BOOTSTRAP YAW SWEEP";
            } else if(stage == "validated_observation_hop") {
                title += "VALIDATED OBSERVATION HOP";
            } else {
                title += "ACCUMULATED FRONTIER";
            }
        }
        addText(
                scene, "scene_title",
                point(0.0F, 0.0F, config_.tunnel_radius + 0.75F),
                std::move(title), WHITE, 0.26F);
    }

    const FrontierGeometryDemoConfig & FrontierGeometryDemo::config() const
    {
        return config_;
    }

    void FrontierGeometryDemo::appendAxes(
            DemoScene & scene, const Point3f & origin, const float scale) const
    {
        addArrow(scene, "coordinate_axes_x", origin, add(origin, point(scale, 0.0F, 0.0F)), RED);
        addArrow(scene, "coordinate_axes_y", origin, add(origin, point(0.0F, scale, 0.0F)), GREEN);
        addArrow(scene, "coordinate_axes_z", origin, add(origin, point(0.0F, 0.0F, scale)), BLUE);
        if(config_.show_labels) {
            addText(scene, "axis_labels", add(origin, point(scale + 0.1F, 0.0F, 0.0F)), "+X Forward", RED);
            addText(scene, "axis_labels", add(origin, point(0.0F, scale + 0.1F, 0.0F)), "+Y Left", GREEN);
            addText(scene, "axis_labels", add(origin, point(0.0F, 0.0F, scale + 0.1F)), "+Z Up", BLUE);
        }
    }

    void FrontierGeometryDemo::appendRingGeometry(DemoScene & scene) const
    {
        const float pitch = config_.pitch_degrees * PI / 180.0F;
        const float sin_pitch = std::sin(pitch);
        const float cos_pitch = std::cos(pitch);
        const Point3f origin = point(0.0F, 0.0F, config_.tunnel_center_z);
        const Point3f plane_y = point(0.0F, 1.0F, 0.0F);
        const Point3f plane_z = point(sin_pitch, 0.0F, cos_pitch);
        const Point3f plane_normal = point(cos_pitch, 0.0F, -sin_pitch);

        appendAxes(scene, origin, 1.35F);

        const std::size_t ring_count = 5U;
        for(std::size_t ring = 0U; ring < ring_count; ++ring) {
            const float x = -REAR_TUNNEL_MARGIN
                            + (config_.tunnel_length + REAR_TUNNEL_MARGIN)
                                      * static_cast<float>(ring)
                                      / static_cast<float>(ring_count - 1U);
            for(std::size_t index = 0U; index < config_.tunnel_ring_segments; ++index) {
                const float first_angle = 2.0F * PI * static_cast<float>(index)
                                          / static_cast<float>(config_.tunnel_ring_segments);
                const float second_angle = 2.0F * PI * static_cast<float>(index + 1U)
                                           / static_cast<float>(config_.tunnel_ring_segments);
                const Point3f first = point(
                        x,
                        config_.tunnel_radius * std::cos(first_angle),
                        config_.tunnel_center_z
                                + config_.tunnel_radius * std::sin(first_angle));
                const Point3f second = point(
                        x,
                        config_.tunnel_radius * std::cos(second_angle),
                        config_.tunnel_center_z
                                + config_.tunnel_radius * std::sin(second_angle));
                addLine(scene, "tunnel", first, second, 0.018F, DemoColor {0.60F, 0.67F, 0.71F, 0.48F});
                if(ring + 1U < ring_count && index % 6U == 0U) {
                    const float next_x = -0.8F
                                         + (config_.tunnel_length + 0.8F)
                                                   * static_cast<float>(ring + 1U)
                                                   / static_cast<float>(ring_count - 1U);
                    addLine(
                            scene, "tunnel", first,
                            point(next_x, first.y, first.z), 0.012F,
                            DemoColor {0.48F, 0.56F, 0.61F, 0.34F});
                }
            }
        }
        addArrow(
                scene, "tunnel_axis", point(-0.6F, 0.0F, config_.tunnel_center_z),
                point(config_.tunnel_length, 0.0F, config_.tunnel_center_z), RED, 1.15F);

        if(config_.show_reference_geometry) {
            for(std::size_t index = 0U; index < config_.tunnel_ring_segments; ++index) {
                if(index % 2U != 0U) {
                    continue;
                }
                const float first_angle = 2.0F * PI * static_cast<float>(index)
                                          / static_cast<float>(config_.tunnel_ring_segments);
                const float second_angle = 2.0F * PI * static_cast<float>(index + 1U)
                                           / static_cast<float>(config_.tunnel_ring_segments);
                addLine(
                        scene, "normal_scan_plane",
                        point(0.0F, config_.tunnel_radius * std::cos(first_angle),
                              config_.tunnel_center_z
                                      + config_.tunnel_radius * std::sin(first_angle)),
                        point(0.0F, config_.tunnel_radius * std::cos(second_angle),
                              config_.tunnel_center_z
                                      + config_.tunnel_radius * std::sin(second_angle)),
                        0.025F, WHITE);
            }
        }

        const float plane_extent = 1.12F * config_.tunnel_radius;
        const Point3f corner_a = add(origin, add(multiply(plane_y, plane_extent), multiply(plane_z, plane_extent)));
        const Point3f corner_b = add(origin, add(multiply(plane_y, -plane_extent), multiply(plane_z, plane_extent)));
        const Point3f corner_c = add(origin, add(multiply(plane_y, -plane_extent), multiply(plane_z, -plane_extent)));
        const Point3f corner_d = add(origin, add(multiply(plane_y, plane_extent), multiply(plane_z, -plane_extent)));
        addTriangle(scene, "pitched_scan_plane", corner_a, corner_b, corner_c, CYAN);
        addTriangle(scene, "pitched_scan_plane", corner_a, corner_c, corner_d, CYAN);

        Point3f previous_hit {};
        bool has_previous = false;
        for(std::size_t index = 0U; index <= config_.tunnel_ring_segments; ++index) {
            const float phi = 2.0F * PI * static_cast<float>(index)
                              / static_cast<float>(config_.tunnel_ring_segments);
            const Point3f direction = pitchedDirection(phi, sin_pitch, cos_pitch);
            const Point3f hit = add(
                    origin,
                    multiply(direction, cylinderHitDistance(direction, config_.tunnel_radius)));
            if(has_previous) {
                addLine(scene, "pitched_scan_ring", previous_hit, hit, 0.045F, DemoColor {0.08F, 0.90F, 0.96F, 0.95F});
            }
            previous_hit = hit;
            has_previous = true;
        }

        for(std::size_t index = 0U; index < config_.ray_count; ++index) {
            const float phi = 2.0F * PI * static_cast<float>(index)
                              / static_cast<float>(config_.ray_count);
            if(!config_.show_full_ring && std::sin(phi) < 0.0F) {
                continue;
            }
            const Point3f direction = pitchedDirection(phi, sin_pitch, cos_pitch);
            const Point3f hit = add(
                    origin,
                    multiply(direction, cylinderHitDistance(direction, config_.tunnel_radius)));
            addLine(scene, "scan_rays", origin, hit, 0.012F, YELLOW);
            addSphere(scene, "scan_hits", hit, 0.10F, RED);
        }

        addArrow(
                scene, "scan_plane_normal", origin,
                add(origin, multiply(plane_normal, 1.65F)), MAGENTA, 1.1F);
        addLine(scene, "pitch_reference", origin, add(origin, point(0.0F, 0.0F, 1.6F)), 0.025F, WHITE);
        addLine(scene, "pitch_reference", origin, add(origin, multiply(plane_z, 1.6F)), 0.035F, DemoColor {0.10F, 0.85F, 0.92F, 0.95F});
        Point3f arc_previous = add(origin, point(0.0F, 0.0F, 0.72F));
        for(std::size_t index = 1U; index <= 12U; ++index) {
            const float angle = pitch * static_cast<float>(index) / 12.0F;
            const Point3f arc = add(origin, point(0.72F * std::sin(angle), 0.0F, 0.72F * std::cos(angle)));
            addLine(scene, "pitch_angle", arc_previous, arc, 0.025F, YELLOW);
            arc_previous = arc;
        }

        addCube(scene, "drone", origin, point(0.65F, 0.38F, 0.20F), DemoColor {0.18F, 0.24F, 0.28F, 1.0F});
        constexpr std::array<Point3f, 4U> ARM_ENDS {
                Point3f {0.42F, 0.42F, 0.06F}, Point3f {0.42F, -0.42F, 0.06F},
                Point3f {-0.42F, 0.42F, 0.06F}, Point3f {-0.42F, -0.42F, 0.06F},
        };
        for(const Point3f & arm_end : ARM_ENDS) {
            const Point3f end = add(origin, arm_end);
            addLine(scene, "drone", origin, end, 0.055F, WHITE);
            addSphere(scene, "drone", end, 0.18F, DemoColor {0.42F, 0.50F, 0.55F, 0.95F});
        }

        const Point3f check_origin = point(0.0F, -4.2F, config_.tunnel_center_z);
        addArrow(scene, "orthographic_axes", check_origin, add(check_origin, point(1.8F, 0.0F, 0.0F)), RED);
        addArrow(scene, "orthographic_axes", check_origin, add(check_origin, point(0.0F, 0.0F, 1.8F)), BLUE);
        addLine(scene, "orthographic_check", add(check_origin, multiply(plane_z, -1.5F)), add(check_origin, multiply(plane_z, 1.5F)), 0.045F, DemoColor {0.08F, 0.90F, 0.96F, 0.95F});

        if(config_.show_labels) {
            addText(scene, "ring_labels", add(origin, point(0.2F, 0.0F, config_.tunnel_radius + 0.3F)), "PITCHED 2D LIDAR RING", CYAN, 0.22F);
            addText(scene, "ring_labels", add(origin, point(1.7F, 0.0F, 0.9F)), "pitch = " + std::to_string(static_cast<int>(std::lround(config_.pitch_degrees))) + " deg", YELLOW);
            addText(scene, "ring_labels", add(origin, multiply(plane_normal, 1.85F)), "scan-plane normal n", MAGENTA);
            addText(scene, "ring_labels", add(check_origin, point(0.2F, 0.0F, 2.05F)), "X-Z ORTHOGRAPHIC CHECK", WHITE, 0.20F);
        }
    }

    void FrontierGeometryDemo::appendVoxelObservation(DemoScene & scene) const
    {
        const float pitch = config_.pitch_degrees * PI / 180.0F;
        const float sin_pitch = std::sin(pitch);
        const float cos_pitch = std::cos(pitch);
        const Point3f origin = point(0.0F, 0.0F, config_.tunnel_center_z);
        const Point3f normal = point(cos_pitch, 0.0F, -sin_pitch);
        const float cube_size = config_.voxel_resolution * 0.72F * config_.voxel_visual_scale;

        for(std::size_t angle_index = 0U;
            angle_index <= config_.voxel_angular_samples;
            ++angle_index)
        {
            const float phi = PI * static_cast<float>(angle_index)
                              / static_cast<float>(config_.voxel_angular_samples);
            const Point3f direction = pitchedDirection(phi, sin_pitch, cos_pitch);
            const float hit_distance = cylinderHitDistance(direction, config_.tunnel_radius);
            for(std::size_t layer = 0U;
                layer < config_.voxel_thickness_layers;
                ++layer)
            {
                const float centered_layer = static_cast<float>(layer)
                                             - 0.5F * static_cast<float>(
                                                     config_.voxel_thickness_layers - 1U);
                const Point3f layer_offset = multiply(
                        normal, centered_layer * config_.voxel_resolution);
                for(std::size_t radial = 1U; radial < config_.voxel_radial_samples; ++radial) {
                    const float distance = hit_distance * static_cast<float>(radial)
                                           / static_cast<float>(config_.voxel_radial_samples);
                    addCube(
                            scene, "voxel_free",
                            add(origin, add(multiply(direction, distance), layer_offset)),
                            point(cube_size, cube_size, cube_size),
                            DemoColor {0.18F, 0.82F, 0.88F, 0.36F});
                }
                addCube(
                        scene, "voxel_occupied",
                        add(origin, add(multiply(direction, hit_distance), layer_offset)),
                        point(cube_size * 1.12F, cube_size * 1.12F, cube_size * 1.12F), RED);
            }
            if(config_.show_unknown && angle_index % 2U == 0U) {
                addCube(
                        scene, "voxel_unknown",
                        add(origin, multiply(
                                            direction,
                                            hit_distance + 1.4F * config_.voxel_resolution)),
                        point(cube_size, cube_size, cube_size), DARK_GRAY);
            }
        }

        addLine(
                scene, "voxel_cut_diameter",
                add(origin, point(0.0F, -config_.tunnel_radius, 0.0F)),
                add(origin, point(0.0F, config_.tunnel_radius, 0.0F)),
                0.025F, WHITE);
        if(config_.show_labels) {
            addText(
                    scene, "voxel_labels",
                    point(0.5F, 0.0F, config_.tunnel_center_z + 1.35F),
                    "PITCHED SCAN VOXEL SLICE", CYAN, 0.21F);
            addText(
                    scene, "voxel_labels",
                    point(0.2F, -config_.tunnel_radius - 0.35F, config_.tunnel_center_z),
                    "diameter = visualization cut, not a wall", WHITE, 0.14F);
            addText(scene, "voxel_legend", point(0.0F, 2.55F, config_.tunnel_center_z + 0.65F), "FREE", GREEN, 0.15F);
            addText(scene, "voxel_legend", point(0.0F, 2.55F, config_.tunnel_center_z + 0.35F), "HIT / OCCUPIED", RED, 0.15F);
            addText(scene, "voxel_legend", point(0.0F, 2.55F, config_.tunnel_center_z + 0.05F), "UNKNOWN OUTSIDE SLICE", GRAY, 0.15F);
        }
    }

    void FrontierGeometryDemo::appendFrontierPipeline(DemoScene & scene) const
    {
        const float column_size = config_.voxel_resolution
                                  * static_cast<float>(config_.column_stride_voxels);
        const float layer_height = config_.voxel_resolution;

        auto addColumn = [&](const Point3f & base, const std::size_t layers,
                             const std::string & ns, const DemoColor color) {
            const float first_z = base.z
                                  - 0.5F * layer_height * static_cast<float>(layers - 1U);
            for(std::size_t layer = 0U; layer < layers; ++layer) {
                addCube(
                        scene, ns,
                        point(base.x, base.y, first_z + static_cast<float>(layer) * layer_height),
                        point(0.88F * column_size, 0.88F * column_size,
                              0.82F * layer_height),
                        color);
            }
        };

        const bool show_vertical = config_.mode == "combined"
                                   || config_.mode == "vertical_columns";
        const bool show_support = config_.mode == "combined"
                                  || config_.mode == "support_evidence";
        const bool show_components = config_.mode == "combined"
                                     || config_.mode == "component_fragmentation";

        if(show_vertical) {
        const std::size_t span_layers = static_cast<std::size_t>(std::ceil(
                                                config_.min_z_span
                                                / config_.voxel_resolution))
                                        + 1U;
        const std::size_t passed_layers = std::max(
                                                  config_.min_z_layers, span_layers)
                                          + 2U;
        const std::size_t rejected_layers = config_.min_z_layers > 2U
                                                    ? config_.min_z_layers - 2U
                                                    : 1U;
        const Point3f vertical_pass = point(6.0F, 3.0F, 0.0F);
        const Point3f vertical_reject = point(6.7F, 3.0F, 0.0F);
        addColumn(vertical_pass, passed_layers, "vertical_pass", YELLOW);
        addColumn(vertical_reject, rejected_layers, "vertical_reject", BLUE);
        addArrow(scene, "frontier_unknown_direction", add(vertical_pass, point(0.0F, 0.0F, 0.45F)), add(vertical_pass, point(0.55F, 0.0F, 0.45F)), RED, 0.75F);
        if(config_.show_labels) {
            addText(scene, "vertical_labels", add(vertical_pass, point(0.0F, 0.0F, 0.85F)), "VERTICAL PASS: " + std::to_string(passed_layers) + " layers", YELLOW, 0.16F);
            addText(scene, "vertical_labels", add(vertical_reject, point(0.0F, 0.0F, 0.55F)), "VERTICAL REJECT: " + std::to_string(rejected_layers) + " layers", BLUE, 0.16F);
            addText(scene, "vertical_labels", point(6.0F, 3.0F, 1.25F), "COLUMN BASE = " + std::to_string(column_size) + " x " + std::to_string(column_size) + " m", WHITE, 0.15F);
        }
        }

        if(show_support) {
            enum class SupportExample {
                Passed,
                Unknown,
                Occupied,
            };
            const std::array<std::pair<Point3f, SupportExample>, 3U> support_examples {
                    std::pair<Point3f, SupportExample> {
                            point(1.1F, 3.0F, 0.0F), SupportExample::Passed},
                    {point(1.1F, 4.35F, 0.0F), SupportExample::Unknown},
                    {point(1.1F, 5.70F, 0.0F), SupportExample::Occupied},
            };
            const std::size_t support_samples = static_cast<std::size_t>(std::ceil(
                    config_.support_depth / config_.voxel_resolution));
            const std::size_t unknown_failure_depth =
                    std::min<std::size_t>(1U, support_samples - 1U);
            const std::size_t occupied_failure_depth =
                    std::min<std::size_t>(3U, support_samples - 1U);
            for(const auto & [column, example] : support_examples) {
                const DemoColor status_color = example == SupportExample::Passed
                                                       ? GREEN
                                                       : (example == SupportExample::Unknown
                                                                  ? ORANGE
                                                                  : MAGENTA);
                const std::string status_ns = example == SupportExample::Passed
                                                      ? "support_pass"
                                                      : (example == SupportExample::Unknown
                                                                 ? "support_reject_unknown"
                                                                 : "support_reject_occupied");
                const Point3f anchor = column;
                addColumn(column, 7U, status_ns, status_color);
                addSphere(scene, "support_anchor", anchor, 0.14F, MAGENTA);
                addArrow(
                        scene, "support_unknown_direction", anchor,
                        add(anchor, point(0.55F, 0.0F, 0.0F)), RED, 0.72F);
                addArrow(
                        scene, "support_inward_direction", anchor,
                        add(anchor, point(-config_.support_depth, 0.0F, 0.0F)),
                        GREEN, 0.72F);

                Point3f last_sample = anchor;
                for(std::size_t depth = 0U; depth < support_samples; ++depth) {
                    const Point3f sample = add(
                            anchor,
                            point(-config_.voxel_resolution
                                          * static_cast<float>(depth + 1U),
                                  0.0F, 0.0F));
                    last_sample = sample;
                    const bool unknown_failure = example == SupportExample::Unknown
                                                 && depth == unknown_failure_depth;
                    const bool occupied_failure = example == SupportExample::Occupied
                                                  && depth == occupied_failure_depth;
                    DemoColor sample_color {0.35F, 0.92F, 0.70F, 0.34F};
                    std::string sample_ns = "support_known_samples";
                    if(unknown_failure) {
                        sample_color = ORANGE;
                        sample_ns = "support_first_failure_unknown";
                    } else if(occupied_failure) {
                        sample_color = MAGENTA;
                        sample_ns = "support_first_failure_occupied";
                    }
                    const float sample_size = 0.52F * config_.voxel_resolution;
                    addCube(
                            scene, sample_ns, sample,
                            point(sample_size, sample_size, sample_size), sample_color);
                    if(unknown_failure || occupied_failure) {
                        break;
                    }
                }
                addLine(
                        scene, "support_evidence_ray", anchor, last_sample, 0.026F,
                        status_color);
                if(config_.show_unknown) {
                    for(int offset = -1; offset <= 1; ++offset) {
                        addCube(
                                scene, "support_unknown_space",
                                add(column,
                                    point(0.25F, 0.22F * static_cast<float>(offset),
                                          0.0F)),
                                point(column_size, column_size, column_size), DARK_GRAY);
                    }
                }
                if(config_.show_labels) {
                    const std::string label = example == SupportExample::Passed
                                                      ? "SUPPORT PASSED"
                                                      : (example == SupportExample::Unknown
                                                                 ? "SUPPORT REJECTED: UNKNOWN"
                                                                 : "SUPPORT REJECTED: OCCUPIED");
                    addText(
                            scene, "support_labels",
                            add(column, point(-0.4F, 0.0F, 0.92F)), label,
                            status_color, 0.16F);
                }
            }
        }

        if(show_components) {
        struct ComponentColumn {
            int x {};
            int y {};
            std::size_t component {};
        };
        const std::array<ComponentColumn, 17U> component_columns {
                ComponentColumn {0, 0, 0U}, {1, 0, 0U}, {2, 0, 0U}, {3, 0, 0U},
                {6, 0, 1U}, {7, 0, 1U}, {8, 0, 1U},
                {0, 3, 2U}, {1, 3, 2U},
                {3, 3, 3U}, {5, 3, 4U}, {7, 3, 5U}, {9, 3, 6U},
                {0, 6, 7U}, {2, 6, 8U}, {4, 6, 9U}, {6, 6, 10U},
        };
        constexpr std::array<DemoColor, 11U> COMPONENT_COLORS {
                DemoColor {0.20F, 0.85F, 0.35F, 0.88F},
                DemoColor {0.12F, 0.72F, 0.88F, 0.88F},
                DemoColor {0.92F, 0.72F, 0.18F, 0.88F},
                DemoColor {0.93F, 0.26F, 0.26F, 0.90F},
                DemoColor {0.82F, 0.34F, 0.82F, 0.90F},
                DemoColor {0.95F, 0.48F, 0.18F, 0.90F},
                DemoColor {0.40F, 0.58F, 0.95F, 0.90F},
                DemoColor {0.88F, 0.28F, 0.42F, 0.90F},
                DemoColor {0.46F, 0.82F, 0.72F, 0.90F},
                DemoColor {0.72F, 0.62F, 0.24F, 0.90F},
                DemoColor {0.62F, 0.42F, 0.92F, 0.90F},
        };
        const Point3f component_origin = point(3.0F, 3.0F, 0.0F);
        const float component_spacing = 1.22F * column_size;
        std::array<std::vector<Point3f>, 11U> component_points;
        for(const ComponentColumn & entry : component_columns) {
            const Point3f center = add(
                    component_origin,
                    point(component_spacing * static_cast<float>(entry.x),
                          component_spacing * static_cast<float>(entry.y), 0.0F));
            addColumn(
                    center, 7U,
                    entry.component >= 3U ? "component_singletons" : "component_columns",
                    COMPONENT_COLORS[entry.component]);
            component_points[entry.component].push_back(center);
        }
        for(std::size_t component = 0U; component < component_points.size(); ++component) {
            auto & points = component_points[component];
            std::sort(points.begin(), points.end(), [](const Point3f & lhs, const Point3f & rhs) {
                return lhs.x < rhs.x || (lhs.x == rhs.x && lhs.y < rhs.y);
            });
            for(std::size_t index = 1U; index < points.size(); ++index) {
                addLine(
                        scene, "component_links",
                        add(points[index - 1U], point(0.0F, 0.0F, 0.52F)),
                        add(points[index], point(0.0F, 0.0F, 0.52F)),
                        0.028F, COMPONENT_COLORS[component]);
            }
        }
        if(config_.show_labels) {
            const bool all_rejected = config_.min_component_columns > 4U;
            addText(scene, "component_labels", add(component_origin, point(0.35F, 0.0F, 0.95F)), "COMPONENT A: 4 COLUMNS", COMPONENT_COLORS[0U], 0.15F);
            addText(scene, "component_labels", add(component_origin, point(1.65F, 0.0F, 0.95F)), "COMPONENT B: 3 COLUMNS", COMPONENT_COLORS[1U], 0.15F);
            addText(scene, "component_labels", add(component_origin, point(0.15F, 0.75F, 0.95F)), "COMPONENT C: 2 COLUMNS", COMPONENT_COLORS[2U], 0.15F);
            addText(scene, "component_labels", add(component_origin, point(1.0F, 1.45F, 0.95F)), "8 SINGLETON COMPONENTS", RED, 0.15F);
            addText(scene, "component_labels", add(component_origin, point(0.7F, -0.45F, 1.35F)), "17 COLUMNS -> 11 COMPONENTS", WHITE, 0.20F);
            addText(scene, "component_labels", add(component_origin, point(0.7F, -0.45F, 1.05F)), all_rejected ? "ALL COMPONENTS REJECTED" : "LARGE COMPONENTS PASS", all_rejected ? ORANGE : GREEN, 0.17F);
        }
        }
    }

    FrontierGeometryDemo::DetectorReplay
    FrontierGeometryDemo::buildDetectorReplay() const
    {
        CaveWorld::ProceduralCaveFieldConfig cave_config;
        cave_config.length = std::max(config_.tunnel_length, config_.initial_x + config_.hop_distance + 4.0F);
        cave_config.base_radius = config_.tunnel_radius;
        cave_config.n_segments = std::max(
                40, static_cast<int>(std::ceil(cave_config.length * 10.0F)));
        cave_config.branch = false;
        cave_config.chamber_scale = 1.0F;
        cave_config.noise_scale = 0.12F;
        cave_config.density = 30;
        cave_config.seed = config_.cave_seed;
        auto field = std::make_shared<CaveWorld::ProceduralCaveField>(cave_config);

        DroneScanner::FakeLidarConfig lidar_config;
        lidar_config.num_beams = static_cast<int>(config_.ray_count);
        lidar_config.max_range = config_.lidar_max_range;
        lidar_config.ring_pitch_rad = config_.pitch_degrees * PI / 180.0F;
        const DroneScanner::FakeLidar lidar(field, lidar_config);

        OctoMapBuilder builder(config_.voxel_resolution);
        DemoPipelineSummary summary;
        summary.initial_position =
                point(config_.initial_x, config_.initial_y, config_.tunnel_center_z);
        CapturedScan initial_focus_scan = insertYawSweep(
                lidar, builder, summary.initial_position, config_.yaw_steps,
                summary.scan_frames, summary.ray_returns);
        octomap::OcTree bootstrap_tree(builder.tree());
        const TreeCounts bootstrap_counts = treeCounts(bootstrap_tree);
        OctoMapBuilder single_ring_builder(config_.voxel_resolution);
        single_ring_builder.insertScan(
                initial_focus_scan.origin, initial_focus_scan.returns);
        const TreeCounts single_ring_counts = treeCounts(single_ring_builder.tree());
        const std::size_t bootstrap_scan_frames = summary.scan_frames;
        const std::size_t bootstrap_ray_returns = summary.ray_returns;
        CapturedScan final_focus_scan = initial_focus_scan;
        summary.observation_epochs = 1U;

        const Point3f hop_goal = add(
                summary.initial_position, point(config_.hop_distance, 0.0F, 0.0F));
        const KnownFreePathChecker path_checker;
        summary.hop_result = path_checker.checkSegment(
                builder.tree(), summary.initial_position, hop_goal);
        summary.final_position = summary.initial_position;
        if(summary.hop_result.safe()) {
            summary.final_position = hop_goal;
            final_focus_scan = insertYawSweep(
                    lidar, builder, summary.final_position, config_.yaw_steps,
                    summary.scan_frames, summary.ray_returns);
            summary.observation_epochs = 2U;
        }

        GlobalFrontierDetectorConfig detector_config;
        detector_config.resolution = config_.voxel_resolution;
        detector_config.column_stride_voxels = config_.column_stride_voxels;
        detector_config.min_z_layers = config_.min_z_layers;
        detector_config.min_z_span = config_.min_z_span;
        detector_config.support_depth = config_.support_depth;
        detector_config.max_trace_candidates = config_.max_trace_candidates;
        detector_config.max_trace_support_samples =
                config_.max_trace_support_samples;
        detector_config.max_trace_components = config_.max_trace_components;
        detector_config.max_trace_geometry_elements =
                config_.max_trace_geometry_elements;
        summary.detection =
                GlobalFrontierDetector(detector_config).detectWithTrace(builder.tree());
        summary.known_voxels = builder.knownCount();
        summary.occupied_voxels = builder.occupiedCount();
        octomap::OcTree accumulated_tree(builder.tree());

        return DetectorReplay {
                std::move(field), std::move(initial_focus_scan),
                std::move(final_focus_scan), std::move(bootstrap_tree),
                std::move(accumulated_tree), single_ring_counts, bootstrap_counts,
                bootstrap_scan_frames, bootstrap_ray_returns, hop_goal,
                std::move(summary)};
    }

    void FrontierGeometryDemo::appendDetectorFrontierStage(
            DemoScene & scene, const DetectorReplay & replay,
            const std::string & stage) const
    {
        const DemoPipelineSummary & summary = replay.summary;
        const TracedFrontierDetectionResult & detected = summary.detection;
        const bool accumulated = stage == "accumulated_frontier";
        const bool show_map = stage != "single_ring";
        const bool show_hop = stage == "validated_observation_hop"
                               || accumulated;
        const CapturedScan & focus_scan =
                accumulated ? replay.final_focus_scan : replay.initial_focus_scan;
        const octomap::OcTree & display_tree =
                accumulated ? replay.accumulated_tree : replay.bootstrap_tree;
        DemoPipelineSummary stage_summary = summary;
        if(stage == "single_ring") {
            stage_summary.final_position = summary.initial_position;
            stage_summary.observation_epochs = 0U;
            stage_summary.scan_frames = 1U;
            stage_summary.ray_returns = replay.initial_focus_scan.returns.size();
            stage_summary.known_voxels = replay.single_ring_counts.known;
            stage_summary.occupied_voxels = replay.single_ring_counts.occupied;
            stage_summary.hop_result = PathCheckResult {};
            stage_summary.detection = TracedFrontierDetectionResult {};
        } else if(stage == "bootstrap_yaw_sweep") {
            stage_summary.final_position = summary.initial_position;
            stage_summary.observation_epochs = 1U;
            stage_summary.scan_frames = replay.bootstrap_scan_frames;
            stage_summary.ray_returns = replay.bootstrap_ray_returns;
            stage_summary.known_voxels = replay.bootstrap_counts.known;
            stage_summary.occupied_voxels = replay.bootstrap_counts.occupied;
            stage_summary.hop_result = PathCheckResult {};
            stage_summary.detection = TracedFrontierDetectionResult {};
        } else if(stage == "validated_observation_hop") {
            stage_summary.observation_epochs = 1U;
            stage_summary.scan_frames = replay.bootstrap_scan_frames;
            stage_summary.ray_returns = replay.bootstrap_ray_returns;
            stage_summary.known_voxels = replay.bootstrap_counts.known;
            stage_summary.occupied_voxels = replay.bootstrap_counts.occupied;
            stage_summary.detection = TracedFrontierDetectionResult {};
        }
        scene.pipeline = std::move(stage_summary);
        if(accumulated && detected.trace.truncated && config_.show_labels) {
            addText(
                    scene, "trace_status",
                    add(summary.final_position, point(0.0F, 0.0F, 0.75F)),
                    "TRACE TRUNCATED", MAGENTA, 0.17F);
        }

        appendAxes(scene, summary.initial_position, 0.8F);
        if(config_.show_reference_geometry) {
            const std::vector<CaveWorld::Point3> surface = replay.field->sampleSurface();
            const std::size_t surface_stride =
                    std::max<std::size_t>(1U, surface.size() / 1'200U);
            for(std::size_t index = 0U; index < surface.size(); index += surface_stride) {
                addSphere(
                        scene, "tunnel",
                        point(surface[index].x, surface[index].y, surface[index].z),
                        0.045F, GRAY);
            }
        }

        addCube(
                scene, "drone", summary.initial_position,
                point(0.28F, 0.22F, 0.12F), BLUE);
        if(show_hop && summary.observation_epochs == 2U) {
            addCube(
                    scene, "drone", summary.final_position,
                    point(0.28F, 0.22F, 0.12F), GREEN);
        }
        if(show_hop) {
            addLine(
                    scene, "known_free_hop", summary.initial_position,
                    replay.hop_goal, 0.045F,
                    summary.hop_result.safe() ? GREEN : RED);
            if(summary.hop_result.first_blocked_position.has_value()) {
                addSphere(
                        scene, "known_free_hop_blocked",
                        *summary.hop_result.first_blocked_position, 0.18F, RED);
            }
        }

        for(const RayReturn & ray : focus_scan.returns) {
            addLine(
                    scene, "scan_rays", focus_scan.origin, ray.endpoint, 0.012F,
                    ray.hit ? CYAN : GRAY);
            if(ray.hit) {
                addSphere(scene, "scan_hits", ray.endpoint, 0.065F, RED);
            }
        }

        if(focus_scan.returns.empty()) {
            if(config_.show_labels) {
                addText(
                        scene, "pipeline_labels",
                        add(focus_scan.origin, point(0.0F, 0.0F, 0.5F)),
                        "NO SCAN RETURNS", ORANGE, 0.16F);
            }
            return;
        }
        const RayReturn & selected_return = focus_scan.returns[
                selectedRayIndex(config_.selected_phi_degrees, focus_scan.returns.size())];
        const Point3f selected_endpoint = selected_return.endpoint;
        scene.pipeline->selected_endpoint = selected_endpoint;
        scene.pipeline->selected_return_hit = selected_return.hit;

        const float map_voxel_size = 0.72F * config_.voxel_resolution;
        if(show_map) {
            const Point3f map_display_center =
                    accumulated ? selected_endpoint : summary.initial_position;
            const float map_display_radius =
                    accumulated
                            ? 2.0F
                            : config_.lidar_max_range + config_.voxel_resolution;
            for(auto leaf = display_tree.begin_leafs(), end = display_tree.end_leafs();
                leaf != end; ++leaf)
            {
                const Point3f center {
                        static_cast<float>(leaf.getX()), static_cast<float>(leaf.getY()),
                        static_cast<float>(leaf.getZ())};
                if(std::fabs(center.x - map_display_center.x) > map_display_radius
                   || std::fabs(center.y - map_display_center.y) > map_display_radius
                   || std::fabs(center.z - map_display_center.z) > map_display_radius)
                {
                    continue;
                }
                const bool occupied = display_tree.isNodeOccupied(*leaf);
                const octomap::OcTreeKey key = leaf.getKey();
                const std::uint64_t visual_hash =
                        static_cast<std::uint64_t>(key[0]) * 73'856'093ULL
                        ^ static_cast<std::uint64_t>(key[1]) * 19'349'663ULL
                        ^ static_cast<std::uint64_t>(key[2]) * 83'492'791ULL;
                if((occupied && visual_hash % 2U != 0U)
                   || (!occupied && visual_hash % 8U != 0U))
                {
                    continue;
                }
                addCube(
                        scene,
                        occupied ? "voxel_occupied" : "voxel_free",
                        center,
                        point(map_voxel_size, map_voxel_size, map_voxel_size),
                        occupied ? RED
                                 : DemoColor {0.18F, 0.82F, 0.88F, 0.24F});
            }
        }
        if(!accumulated) {
            return;
        }

        addSphere(
                scene, "frontier_selected_endpoint", selected_endpoint,
                0.20F, ORANGE);

        constexpr float analysis_radius = 1.5F;
        addWireBox(
                scene, "analysis_window", selected_endpoint,
                point(2.0F * analysis_radius, 2.0F * analysis_radius, 2.0F),
                WHITE, 0.025F);

        const FrontierCandidateTrace * selected_candidate = nullptr;
        float nearest_distance_squared = analysis_radius * analysis_radius;
        for(const FrontierCandidateTrace & candidate : detected.trace.candidates) {
            const float dx = candidate.center.x - selected_endpoint.x;
            const float dy = candidate.center.y - selected_endpoint.y;
            const float dz = candidate.center.z - selected_endpoint.z;
            const float distance_squared = dx * dx + dy * dy + dz * dz;
            if(distance_squared <= nearest_distance_squared) {
                nearest_distance_squared = distance_squared;
                selected_candidate = &candidate;
            }
        }
        const std::size_t accepted_regions = detected.result.regions.size();
        const std::string global_status =
                "GLOBAL: " + std::to_string(accepted_regions)
                + " ACCEPTED REGION" + (accepted_regions == 1U ? "" : "S");
        if(selected_candidate == nullptr) {
            if(config_.show_labels) {
                addText(
                        scene, "pipeline_labels",
                        add(selected_endpoint, point(0.0F, 0.0F, 0.40F)),
                        (detected.trace.truncated
                                 ? "LOCAL: NO RECORDED CANDIDATE (TRACE TRUNCATED)"
                                 : "LOCAL: NO FRONTIER CANDIDATE")
                                + std::string("\n") + global_status,
                        detected.trace.truncated ? MAGENTA : ORANGE, 0.16F);
            }
            return;
        }
        scene.pipeline->focused_candidate = selected_candidate->key;

        const float voxel_size = 0.82F * config_.voxel_resolution;
        for(const Point3f & candidate_point : selected_candidate->column_points) {
            addCube(
                    scene, "frontier_candidate", candidate_point,
                    point(voxel_size, voxel_size, voxel_size), YELLOW);
        }
        addLine(
                scene, "pipeline_connector", selected_endpoint,
                selected_candidate->center, 0.028F, WHITE);

        const FrontierSupportAttemptTrace * selected_attempt = nullptr;
        const auto selected_attempt_it = std::find_if(
                selected_candidate->support_attempts.begin(),
                selected_candidate->support_attempts.end(),
                [](const FrontierSupportAttemptTrace & attempt) {
                    return attempt.selected;
                });
        if(selected_attempt_it != selected_candidate->support_attempts.end()) {
            selected_attempt = &*selected_attempt_it;
            scene.pipeline->focused_attempt_index = static_cast<std::size_t>(
                    std::distance(
                            selected_candidate->support_attempts.begin(),
                            selected_attempt_it));
        } else if(!selected_candidate->support_attempts.empty()) {
            selected_attempt = &selected_candidate->support_attempts.front();
            scene.pipeline->focused_attempt_index = 0U;
        }

        if(selected_attempt != nullptr) {
            addSphere(
                    scene, "support_anchor", selected_attempt->anchor, 0.14F, MAGENTA);
            if(config_.show_unknown) {
                for(const Point3f & candidate_point : selected_attempt->column_points) {
                    addCube(
                            scene, "voxel_unknown",
                            add(candidate_point,
                                multiply(
                                        selected_attempt->unknown_direction,
                                        config_.voxel_resolution)),
                            point(map_voxel_size, map_voxel_size, map_voxel_size),
                            DARK_GRAY);
                }
            }
            addArrow(
                    scene, "frontier_unknown_direction", selected_attempt->anchor,
                    add(selected_attempt->anchor,
                        multiply(selected_attempt->unknown_direction, 0.55F)),
                    RED, 0.72F);
            addArrow(
                    scene, "support_inward_direction", selected_attempt->anchor,
                    add(selected_attempt->anchor,
                        multiply(
                                selected_attempt->inward_direction,
                                config_.support_depth)),
                    GREEN, 0.72F);

            if(!selected_attempt->samples.empty()) {
                addLine(
                        scene, "support_evidence_ray", selected_attempt->anchor,
                        selected_attempt->samples.back().position, 0.026F,
                        selected_attempt->passed() ? GREEN : ORANGE);
            }
            for(const FrontierSupportSampleTrace & sample : selected_attempt->samples) {
                DemoColor color = DemoColor {0.35F, 0.92F, 0.70F, 0.38F};
                std::string ns = "support_known_samples";
                if(sample.state == FrontierTraceSampleState::Unknown) {
                    color = ORANGE;
                    ns = "support_unknown_samples";
                } else if(sample.state == FrontierTraceSampleState::Occupied) {
                    color = RED;
                    ns = "support_occupied_samples";
                } else if(sample.state == FrontierTraceSampleState::OutOfBounds) {
                    color = MAGENTA;
                    ns = "support_out_of_bounds_samples";
                }
                addCube(
                        scene, ns, sample.position,
                        point(
                                0.52F * config_.voxel_resolution,
                                0.52F * config_.voxel_resolution,
                                0.52F * config_.voxel_resolution),
                        color);
            }
        }

        auto candidateForKey = [&](const FrontierColumnKey & key)
                -> const FrontierCandidateTrace * {
            const auto found = std::find_if(
                    detected.trace.candidates.begin(),
                    detected.trace.candidates.end(),
                    [&](const FrontierCandidateTrace & candidate) {
                        return candidate.key == key;
                    });
            return found == detected.trace.candidates.end() ? nullptr : &*found;
        };
        for(const FrontierComponentTrace & component : detected.trace.components) {
            const bool intersects_window = std::any_of(
                    component.columns.begin(), component.columns.end(),
                    [&](const FrontierColumnKey & key) {
                        const auto * candidate = candidateForKey(key);
                        if(candidate == nullptr) {
                            return false;
                        }
                        const float dx = candidate->center.x - selected_endpoint.x;
                        const float dy = candidate->center.y - selected_endpoint.y;
                        const float dz = candidate->center.z - selected_endpoint.z;
                        return dx * dx + dy * dy + dz * dz
                               <= analysis_radius * analysis_radius;
                    });
            if(!intersects_window) {
                continue;
            }
            const bool accepted =
                    component.rejection == FrontierComponentRejection::None;
            for(const FrontierColumnKey & key : component.columns) {
                const auto * candidate = candidateForKey(key);
                if(candidate == nullptr) {
                    continue;
                }
                const std::string ns = component.columns.size() == 1U
                                               ? "component_singletons"
                                               : "component_columns";
                for(const Point3f & column_point : candidate->column_points) {
                    addCube(
                            scene, ns, column_point,
                            point(voxel_size, voxel_size, voxel_size),
                            component.columns.size() == 1U
                                    ? MAGENTA
                                    : (accepted ? GREEN : ORANGE));
                }
            }
            const float link_z = component.representative.z
                                 + 0.5F * config_.min_z_span;
            for(const FrontierComponentEdge & edge : component.edges) {
                const auto * first = candidateForKey(edge.first);
                const auto * second = candidateForKey(edge.second);
                if(first != nullptr && second != nullptr) {
                    addLine(
                            scene, "component_links",
                            point(first->center.x, first->center.y, link_z),
                            point(second->center.x, second->center.y, link_z),
                            0.025F, GREEN);
                }
            }
            addSphere(
                    scene, "region_decision", component.representative,
                    0.28F, accepted ? GREEN : ORANGE);
        }

        if(config_.show_labels) {
            constexpr float LABEL_LATERAL_OFFSET = 1.15F;
            addText(
                    scene, "pipeline_labels",
                    add(
                            selected_endpoint,
                            point(0.0F, -LABEL_LATERAL_OFFSET, 0.25F)),
                    selected_return.hit
                            ? "SELECTED HIT ENDPOINT"
                            : "SELECTED MAX-RANGE ENDPOINT",
                    ORANGE, 0.15F);
            addText(
                    scene, "frontier_scope_status",
                    add(
                            selected_endpoint,
                            point(0.0F, -LABEL_LATERAL_OFFSET, 0.65F)),
                    "LOCAL: FRONTIER CANDIDATE\n" + global_status,
                    WHITE, 0.14F);
            addText(
                    scene, "pipeline_labels",
                    add(
                            selected_candidate->center,
                            point(0.0F, LABEL_LATERAL_OFFSET, 0.65F)),
                    selected_candidate->vertical_passed
                            ? (selected_candidate->support_passed
                                       ? "SUPPORT PASSED"
                                       : "SUPPORT REJECTED")
                            : "VERTICAL REJECTED",
                    selected_candidate->support_passed ? GREEN : ORANGE, 0.16F);
            addText(
                    scene, "pipeline_labels",
                    add(summary.initial_position, point(0.0F, 0.0F, -0.55F)),
                    std::string("HOP: ") + pathCheckStatusName(summary.hop_result.status),
                    summary.hop_result.safe() ? GREEN : RED, 0.14F);
        }
    }

}// namespace SwarmController
