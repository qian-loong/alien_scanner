#ifndef SWARM_CONTROLLER_FRONTIERGEOMETRYDEMO_HPP
#define SWARM_CONTROLLER_FRONTIERGEOMETRYDEMO_HPP

#include "swarm_controller/GlobalFrontierDetector.hpp"
#include "swarm_controller/KnownFreePathChecker.hpp"
#include "swarm_controller/Point3f.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace SwarmController {

    struct DemoColor {
        float r {1.0F};
        float g {1.0F};
        float b {1.0F};
        float a {1.0F};
    };

    struct DemoCube {
        std::string ns;
        std::int32_t id {};
        Point3f center {};
        Point3f scale {};
        DemoColor color {};
    };

    struct DemoSphere {
        std::string ns;
        std::int32_t id {};
        Point3f center {};
        float diameter {0.1F};
        DemoColor color {};
    };

    struct DemoLine {
        std::string ns;
        std::int32_t id {};
        Point3f start {};
        Point3f end {};
        float width {0.01F};
        DemoColor color {};
    };

    struct DemoArrow {
        std::string ns;
        std::int32_t id {};
        Point3f start {};
        Point3f end {};
        float shaft_diameter {0.025F};
        float head_diameter {0.08F};
        float head_length {0.12F};
        DemoColor color {};
    };

    struct DemoTriangle {
        std::string ns;
        std::int32_t id {};
        Point3f first {};
        Point3f second {};
        Point3f third {};
        DemoColor color {};
    };

    struct DemoText {
        std::string ns;
        std::int32_t id {};
        Point3f position {};
        std::string text;
        float height {0.12F};
        DemoColor color {};
    };

    struct DemoPipelineSummary {
        Point3f initial_position {};
        Point3f final_position {};
        std::size_t observation_epochs {};
        std::size_t scan_frames {};
        std::size_t ray_returns {};
        std::size_t known_voxels {};
        std::size_t occupied_voxels {};
        Point3f selected_endpoint {};
        bool selected_return_hit {};
        std::optional<FrontierColumnKey> focused_candidate;
        std::optional<std::size_t> focused_attempt_index;
        PathCheckResult hop_result;
        TracedFrontierDetectionResult detection;
    };

    struct DemoScene {
        std::vector<DemoCube> cubes;
        std::vector<DemoSphere> spheres;
        std::vector<DemoLine> lines;
        std::vector<DemoArrow> arrows;
        std::vector<DemoTriangle> triangles;
        std::vector<DemoText> texts;
        std::optional<DemoPipelineSummary> pipeline;

        void clear();
        std::int32_t nextId(const std::string & ns);

    private:
        struct NamespaceCounter {
            std::string ns;
            std::int32_t next {};
        };
        std::vector<NamespaceCounter> counters_;
    };

    struct DemoStageScene {
        std::string stage;
        DemoScene scene;
    };

    struct FrontierGeometryDemoConfig {
        std::string mode {"combined"};
        std::string stage {"accumulated_frontier"};
        float tunnel_radius {2.0F};
        float tunnel_length {8.0F};
        float tunnel_center_z {0.0F};
        std::uint32_t cave_seed {42U};
        float pitch_degrees {20.0F};
        float selected_phi_degrees {0.0F};
        std::size_t tunnel_ring_segments {72U};
        std::size_t ray_count {144U};
        std::size_t yaw_steps {144U};
        float lidar_max_range {3.0F};
        float initial_x {1.0F};
        float initial_y {0.0F};
        float hop_distance {0.8F};
        bool show_full_ring {true};
        float voxel_resolution {0.1F};
        std::size_t voxel_radial_samples {8U};
        std::size_t voxel_angular_samples {24U};
        std::size_t voxel_thickness_layers {3U};
        float voxel_visual_scale {1.0F};
        std::size_t column_stride_voxels {2U};
        std::size_t min_z_layers {5U};
        float min_z_span {0.4F};
        double support_depth {0.8};
        std::size_t min_component_columns {12U};
        std::size_t max_trace_candidates {10'000U};
        std::size_t max_trace_support_samples {100'000U};
        std::size_t max_trace_components {10'000U};
        std::size_t max_trace_geometry_elements {500'000U};
        bool show_labels {true};
        bool show_unknown {true};
        bool show_reference_geometry {true};
    };

    class FrontierGeometryDemo
    {
    public:
        explicit FrontierGeometryDemo(FrontierGeometryDemoConfig config = {});

        DemoScene buildScene() const;
        std::vector<DemoStageScene> buildStageScenes() const;
        const FrontierGeometryDemoConfig & config() const;

    private:
        struct DetectorReplay;

        void appendAxes(DemoScene & scene, const Point3f & origin, float scale) const;
        void appendRingGeometry(DemoScene & scene) const;
        void appendVoxelObservation(DemoScene & scene) const;
        DetectorReplay buildDetectorReplay() const;
        void appendDetectorFrontierStage(
                DemoScene & scene, const DetectorReplay & replay,
                const std::string & stage) const;
        void appendSceneTitle(DemoScene & scene, const std::string & stage) const;
        void appendFrontierPipeline(DemoScene & scene) const;

        FrontierGeometryDemoConfig config_;
    };

}// namespace SwarmController

#endif// SWARM_CONTROLLER_FRONTIERGEOMETRYDEMO_HPP
