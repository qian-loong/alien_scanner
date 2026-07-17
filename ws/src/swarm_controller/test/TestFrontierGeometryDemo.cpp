#include "swarm_controller/FrontierGeometryDemo.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace SwarmController {

    namespace {

        std::size_t cubeCount(const DemoScene & scene, const std::string & ns)
        {
            return static_cast<std::size_t>(std::count_if(
                    scene.cubes.begin(), scene.cubes.end(),
                    [&](const DemoCube & cube) { return cube.ns == ns; }));
        }

        std::size_t lineCount(const DemoScene & scene, const std::string & ns)
        {
            return static_cast<std::size_t>(std::count_if(
                    scene.lines.begin(), scene.lines.end(),
                    [&](const DemoLine & line) { return line.ns == ns; }));
        }

        std::size_t sphereCount(const DemoScene & scene, const std::string & ns)
        {
            return static_cast<std::size_t>(std::count_if(
                    scene.spheres.begin(), scene.spheres.end(),
                    [&](const DemoSphere & sphere) { return sphere.ns == ns; }));
        }

        bool hasText(const DemoScene & scene, const std::string & content)
        {
            return std::any_of(
                    scene.texts.begin(), scene.texts.end(),
                    [&](const DemoText & text) {
                        return text.text.find(content) != std::string::npos;
                    });
        }

        DemoLine firstLine(const DemoScene & scene, const std::string & ns)
        {
            const auto found = std::find_if(
                    scene.lines.begin(), scene.lines.end(),
                    [&](const DemoLine & line) { return line.ns == ns; });
            EXPECT_NE(found, scene.lines.end());
            return *found;
        }

        DemoArrow firstArrow(const DemoScene & scene, const std::string & ns)
        {
            const auto found = std::find_if(
                    scene.arrows.begin(), scene.arrows.end(),
                    [&](const DemoArrow & arrow) { return arrow.ns == ns; });
            EXPECT_NE(found, scene.arrows.end());
            return *found;
        }

        DemoSphere firstSphere(const DemoScene & scene, const std::string & ns)
        {
            const auto found = std::find_if(
                    scene.spheres.begin(), scene.spheres.end(),
                    [&](const DemoSphere & sphere) { return sphere.ns == ns; });
            EXPECT_NE(found, scene.spheres.end());
            return *found;
        }

    }// namespace

    TEST(FrontierGeometryDemoTest, RejectsInvalidConfiguration)
    {
        FrontierGeometryDemoConfig config;
        config.pitch_degrees = 90.0F;
        EXPECT_THROW({ FrontierGeometryDemo demo(config); }, std::invalid_argument);

        config = FrontierGeometryDemoConfig {};
        config.mode = "unsupported";
        EXPECT_THROW({ FrontierGeometryDemo demo(config); }, std::invalid_argument);

        config = FrontierGeometryDemoConfig {};
        config.stage = "unsupported";
        EXPECT_THROW({ FrontierGeometryDemo demo(config); }, std::invalid_argument);

        config = FrontierGeometryDemoConfig {};
        config.pitch_degrees = 45.0F;
        EXPECT_THROW({ FrontierGeometryDemo demo(config); }, std::invalid_argument);
    }

    TEST(FrontierGeometryDemoTest, PitchedRingSatisfiesCylinderAndPlaneEquations)
    {
        FrontierGeometryDemoConfig config;
        config.mode = "ring_geometry";
        config.pitch_degrees = 20.0F;
        const DemoScene scene = FrontierGeometryDemo(config).buildScene();
        const float tangent = std::tan(
                config.pitch_degrees * 3.14159265358979323846F / 180.0F);

        std::size_t checked_points = 0U;
        for(const DemoLine & line : scene.lines) {
            if(line.ns != "pitched_scan_ring") {
                continue;
            }
            for(const Point3f & point : {line.start, line.end}) {
                EXPECT_NEAR(
                        point.y * point.y
                                + (point.z - config.tunnel_center_z)
                                          * (point.z - config.tunnel_center_z),
                        config.tunnel_radius * config.tunnel_radius, 2.0e-3F);
                EXPECT_NEAR(
                        point.x,
                        (point.z - config.tunnel_center_z) * tangent, 2.0e-3F);
                ++checked_points;
            }
        }
        EXPECT_GT(checked_points, 100U);
        EXPECT_EQ(scene.triangles.size(), 2U);
        EXPECT_TRUE(hasText(scene, "X-Z ORTHOGRAPHIC CHECK"));
        EXPECT_FALSE(hasText(scene, "tan("));
    }

    TEST(FrontierGeometryDemoTest, VoxelObservationContainsExplicitStates)
    {
        FrontierGeometryDemoConfig config;
        config.mode = "voxel_observation";
        const DemoScene scene = FrontierGeometryDemo(config).buildScene();

        EXPECT_GT(cubeCount(scene, "voxel_free"), 0U);
        EXPECT_GT(cubeCount(scene, "voxel_occupied"), 0U);
        EXPECT_GT(cubeCount(scene, "voxel_unknown"), 0U);
        EXPECT_GT(lineCount(scene, "voxel_cut_diameter"), 0U);
        EXPECT_EQ(cubeCount(scene, "component_columns"), 0U);
    }

    TEST(FrontierGeometryDemoTest, SupportModeShowsAllThreeOutcomesOnly)
    {
        FrontierGeometryDemoConfig config;
        config.mode = "support_envelope";
        const DemoScene scene = FrontierGeometryDemo(config).buildScene();

        EXPECT_EQ(cubeCount(scene, "support_pass"), 7U);
        EXPECT_EQ(cubeCount(scene, "support_reject_unknown"), 7U);
        EXPECT_EQ(cubeCount(scene, "support_reject_occupied"), 7U);
        EXPECT_EQ(cubeCount(scene, "support_first_failure_unknown"), 1U);
        EXPECT_EQ(cubeCount(scene, "support_first_failure_occupied"), 1U);
        EXPECT_EQ(sphereCount(scene, "support_anchor"), 3U);
        EXPECT_EQ(cubeCount(scene, "vertical_pass"), 0U);
        EXPECT_EQ(cubeCount(scene, "component_columns"), 0U);
    }

    TEST(FrontierGeometryDemoTest, ComponentModeBuildsSeventeenColumnsAndElevenComponents)
    {
        FrontierGeometryDemoConfig config;
        config.mode = "component_fragmentation";
        const DemoScene scene = FrontierGeometryDemo(config).buildScene();
        const std::size_t layer_count = 7U;
        const std::size_t column_count =
                (cubeCount(scene, "component_columns")
                 + cubeCount(scene, "component_singletons"))
                / layer_count;

        EXPECT_EQ(column_count, 17U);
        EXPECT_EQ(cubeCount(scene, "component_singletons") / layer_count, 8U);
        EXPECT_EQ(lineCount(scene, "component_links"), 6U);
        EXPECT_TRUE(hasText(scene, "17 COLUMNS -> 11 COMPONENTS"));
        EXPECT_TRUE(hasText(scene, "ALL COMPONENTS REJECTED"));
    }

    TEST(FrontierGeometryDemoTest, CombinedSceneUsesTheRealObservationPipeline)
    {
        const FrontierGeometryDemoConfig config;
        const DemoScene scene = FrontierGeometryDemo(config).buildScene();

        ASSERT_TRUE(scene.pipeline.has_value());
        const DemoPipelineSummary & pipeline = *scene.pipeline;
        EXPECT_EQ(pipeline.hop_result.status, PathCheckStatus::Safe);
        EXPECT_EQ(pipeline.observation_epochs, 2U);
        EXPECT_EQ(pipeline.scan_frames, 2U * config.yaw_steps);
        EXPECT_EQ(
                pipeline.ray_returns,
                pipeline.scan_frames * config.ray_count);
        EXPECT_GT(pipeline.known_voxels, 0U);
        EXPECT_GT(pipeline.occupied_voxels, 0U);
        EXPECT_EQ(
                pipeline.detection.result.status,
                FrontierDetectionStatus::Accepted);
        EXPECT_FALSE(pipeline.detection.result.regions.empty());
        EXPECT_FALSE(pipeline.detection.trace.candidates.empty());
        EXPECT_FALSE(pipeline.detection.trace.components.empty());
        EXPECT_FALSE(pipeline.detection.trace.truncated);

        EXPECT_GT(sphereCount(scene, "tunnel"), 0U);
        EXPECT_EQ(lineCount(scene, "scan_rays"), config.ray_count);
        EXPECT_GT(cubeCount(scene, "voxel_free"), 0U);
        EXPECT_GT(cubeCount(scene, "voxel_occupied"), 0U);
        EXPECT_LT(
                cubeCount(scene, "voxel_free")
                        + cubeCount(scene, "voxel_occupied"),
                pipeline.known_voxels);
        EXPECT_GT(cubeCount(scene, "frontier_candidate"), 0U);
        EXPECT_GT(lineCount(scene, "analysis_window"), 0U);
        EXPECT_GT(lineCount(scene, "pipeline_connector"), 0U);
        EXPECT_GT(
                lineCount(scene, "support_pass_envelope")
                        + lineCount(scene, "support_reject_envelope"),
                0U);
        EXPECT_EQ(sphereCount(scene, "support_anchor"), 1U);
        EXPECT_GT(cubeCount(scene, "component_columns"), 0U);
        EXPECT_GT(sphereCount(scene, "region_decision"), 0U);
        EXPECT_FALSE(hasText(scene, "threshold:"));
        EXPECT_FALSE(hasText(scene, "min_columns"));
    }

    TEST(FrontierGeometryDemoTest, MarkerGeometryMapsToTheFocusedTraceAttempt)
    {
        const DemoScene scene = FrontierGeometryDemo {}.buildScene();
        ASSERT_TRUE(scene.pipeline.has_value());
        ASSERT_TRUE(scene.pipeline->focused_candidate.has_value());
        ASSERT_TRUE(scene.pipeline->focused_attempt_index.has_value());

        const auto candidate = std::find_if(
                scene.pipeline->detection.trace.candidates.begin(),
                scene.pipeline->detection.trace.candidates.end(),
                [&](const FrontierCandidateTrace & trace) {
                    return trace.key == *scene.pipeline->focused_candidate;
                });
        ASSERT_NE(candidate, scene.pipeline->detection.trace.candidates.end());
        ASSERT_LT(
                *scene.pipeline->focused_attempt_index,
                candidate->support_attempts.size());
        const FrontierSupportAttemptTrace & attempt = candidate->support_attempts[
                *scene.pipeline->focused_attempt_index];

        const DemoLine connector = firstLine(scene, "pipeline_connector");
        const DemoArrow unknown = firstArrow(scene, "frontier_unknown_direction");
        const DemoArrow inward = firstArrow(scene, "support_inward_direction");
        const DemoSphere anchor = firstSphere(scene, "support_anchor");

        EXPECT_FLOAT_EQ(connector.end.x, candidate->center.x);
        EXPECT_FLOAT_EQ(connector.end.y, candidate->center.y);
        EXPECT_FLOAT_EQ(connector.end.z, candidate->center.z);
        EXPECT_FLOAT_EQ(unknown.start.x, attempt.anchor.x);
        EXPECT_FLOAT_EQ(unknown.start.y, attempt.anchor.y);
        EXPECT_FLOAT_EQ(unknown.start.z, attempt.anchor.z);
        EXPECT_FLOAT_EQ(inward.start.x, attempt.anchor.x);
        EXPECT_FLOAT_EQ(inward.start.y, attempt.anchor.y);
        EXPECT_FLOAT_EQ(inward.start.z, attempt.anchor.z);
        EXPECT_FLOAT_EQ(anchor.center.x, attempt.anchor.x);
        EXPECT_FLOAT_EQ(anchor.center.y, attempt.anchor.y);
        EXPECT_FLOAT_EQ(anchor.center.z, attempt.anchor.z);
        EXPECT_FLOAT_EQ(anchor.diameter, 0.14F);
        EXPECT_GT(
                (unknown.end.x - unknown.start.x) * attempt.unknown_direction.x
                        + (unknown.end.y - unknown.start.y)
                                  * attempt.unknown_direction.y,
                0.0F);
        EXPECT_GT(
                (inward.end.x - inward.start.x) * attempt.inward_direction.x
                        + (inward.end.y - inward.start.y)
                                  * attempt.inward_direction.y,
                0.0F);

        const std::string envelope_ns = attempt.passed()
                                                ? "support_pass_envelope"
                                                : "support_reject_envelope";
        std::vector<Point3f> envelope_points;
        for(const DemoLine & line : scene.lines) {
            if(line.ns == envelope_ns) {
                envelope_points.push_back(line.start);
                envelope_points.push_back(line.end);
            }
        }
        ASSERT_EQ(envelope_points.size(), 24U);
        const Point3f lateral {
                -attempt.inward_direction.y, attempt.inward_direction.x, 0.0F};
        float min_depth = std::numeric_limits<float>::max();
        float max_depth = std::numeric_limits<float>::lowest();
        float min_lateral = std::numeric_limits<float>::max();
        float max_lateral = std::numeric_limits<float>::lowest();
        float min_vertical = std::numeric_limits<float>::max();
        float max_vertical = std::numeric_limits<float>::lowest();
        for(const Point3f & envelope_point : envelope_points) {
            const Point3f relative {
                    envelope_point.x - attempt.anchor.x,
                    envelope_point.y - attempt.anchor.y,
                    envelope_point.z - attempt.anchor.z};
            const float depth = relative.x * attempt.inward_direction.x
                                + relative.y * attempt.inward_direction.y;
            const float lateral_offset = relative.x * lateral.x
                                         + relative.y * lateral.y;
            min_depth = std::min(min_depth, depth);
            max_depth = std::max(max_depth, depth);
            min_lateral = std::min(min_lateral, lateral_offset);
            max_lateral = std::max(max_lateral, lateral_offset);
            min_vertical = std::min(min_vertical, relative.z);
            max_vertical = std::max(max_vertical, relative.z);
        }
        EXPECT_NEAR(min_depth, 0.0F, 1.0e-5F);
        EXPECT_NEAR(max_depth, FrontierGeometryDemoConfig {}.support_depth, 1.0e-5F);
        EXPECT_NEAR(
                min_lateral,
                -0.5F * FrontierGeometryDemoConfig {}.support_width, 1.0e-5F);
        EXPECT_NEAR(
                max_lateral,
                0.5F * FrontierGeometryDemoConfig {}.support_width, 1.0e-5F);
        EXPECT_NEAR(
                min_vertical,
                -0.5F * FrontierGeometryDemoConfig {}.min_z_span, 1.0e-5F);
        EXPECT_NEAR(
                max_vertical,
                0.5F * FrontierGeometryDemoConfig {}.min_z_span, 1.0e-5F);

        const std::size_t sample_markers =
                cubeCount(scene, "support_known_samples")
                + cubeCount(scene, "support_unknown_samples")
                + cubeCount(scene, "support_occupied_samples")
                + cubeCount(scene, "support_out_of_bounds_samples");
        EXPECT_EQ(sample_markers, attempt.samples.size());
        for(const FrontierSupportSampleTrace & sample : attempt.samples) {
            std::string ns = "support_known_samples";
            if(sample.state == FrontierTraceSampleState::Unknown) {
                ns = "support_unknown_samples";
            } else if(sample.state == FrontierTraceSampleState::Occupied) {
                ns = "support_occupied_samples";
            } else if(sample.state == FrontierTraceSampleState::OutOfBounds) {
                ns = "support_out_of_bounds_samples";
            }
            EXPECT_TRUE(std::any_of(
                    scene.cubes.begin(), scene.cubes.end(),
                    [&](const DemoCube & cube) {
                        return cube.ns == ns
                               && cube.center.x == sample.position.x
                               && cube.center.y == sample.position.y
                               && cube.center.z == sample.position.z;
                    }));
        }
    }

    TEST(FrontierGeometryDemoTest, SelectedPhiOnlyChangesTheFocusedView)
    {
        FrontierGeometryDemoConfig first_config;
        first_config.selected_phi_degrees = 0.0F;
        FrontierGeometryDemoConfig second_config = first_config;
        second_config.selected_phi_degrees = 180.0F;

        const DemoScene first = FrontierGeometryDemo(first_config).buildScene();
        const DemoScene second = FrontierGeometryDemo(second_config).buildScene();
        ASSERT_TRUE(first.pipeline.has_value());
        ASSERT_TRUE(second.pipeline.has_value());
        const DemoPipelineSummary & first_pipeline = *first.pipeline;
        const DemoPipelineSummary & second_pipeline = *second.pipeline;

        EXPECT_EQ(first_pipeline.observation_epochs, second_pipeline.observation_epochs);
        EXPECT_EQ(first_pipeline.scan_frames, second_pipeline.scan_frames);
        EXPECT_EQ(first_pipeline.ray_returns, second_pipeline.ray_returns);
        EXPECT_EQ(first_pipeline.known_voxels, second_pipeline.known_voxels);
        EXPECT_EQ(first_pipeline.occupied_voxels, second_pipeline.occupied_voxels);
        EXPECT_EQ(first_pipeline.hop_result.status, second_pipeline.hop_result.status);
        EXPECT_EQ(
                first_pipeline.detection.result.status,
                second_pipeline.detection.result.status);
        EXPECT_EQ(
                first_pipeline.detection.result.reason,
                second_pipeline.detection.result.reason);
        ASSERT_EQ(
                first_pipeline.detection.result.regions.size(),
                second_pipeline.detection.result.regions.size());
        for(std::size_t index = 0U;
            index < first_pipeline.detection.result.regions.size(); ++index)
        {
            EXPECT_EQ(
                    first_pipeline.detection.result.regions[index].stable_key,
                    second_pipeline.detection.result.regions[index].stable_key);
            EXPECT_EQ(
                    first_pipeline.detection.result.regions[index].columns,
                    second_pipeline.detection.result.regions[index].columns);
        }
        EXPECT_EQ(
                first_pipeline.detection.trace.candidates.size(),
                second_pipeline.detection.trace.candidates.size());
        EXPECT_EQ(
                first_pipeline.detection.trace.components.size(),
                second_pipeline.detection.trace.components.size());
        EXPECT_NE(
                first_pipeline.selected_endpoint.x,
                second_pipeline.selected_endpoint.x);
    }

    TEST(FrontierGeometryDemoTest, StageSelectsSnapshotsFromOnePipeline)
    {
        FrontierGeometryDemoConfig config;

        const std::vector<DemoStageScene> stages =
                FrontierGeometryDemo(config).buildStageScenes();
        ASSERT_EQ(stages.size(), 5U);
        EXPECT_EQ(stages[0U].stage, "standard_tunnel_geometry");
        EXPECT_EQ(stages[1U].stage, "single_ring");
        EXPECT_EQ(stages[2U].stage, "bootstrap_yaw_sweep");
        EXPECT_EQ(stages[3U].stage, "validated_observation_hop");
        EXPECT_EQ(stages[4U].stage, "accumulated_frontier");

        const DemoScene & standard_tunnel = stages[0U].scene;
        EXPECT_FALSE(standard_tunnel.pipeline.has_value());
        EXPECT_GT(lineCount(standard_tunnel, "tunnel"), 0U);
        EXPECT_GT(lineCount(standard_tunnel, "normal_scan_plane"), 0U);
        EXPECT_GT(lineCount(standard_tunnel, "pitched_scan_ring"), 0U);
        EXPECT_EQ(standard_tunnel.triangles.size(), 2U);
        EXPECT_EQ(cubeCount(standard_tunnel, "voxel_free"), 0U);

        const DemoScene & single_ring = stages[1U].scene;
        ASSERT_TRUE(single_ring.pipeline.has_value());
        EXPECT_GT(lineCount(single_ring, "scan_rays"), 0U);
        EXPECT_EQ(cubeCount(single_ring, "voxel_free"), 0U);
        EXPECT_EQ(lineCount(single_ring, "known_free_hop"), 0U);
        EXPECT_EQ(cubeCount(single_ring, "frontier_candidate"), 0U);
        EXPECT_EQ(single_ring.pipeline->observation_epochs, 0U);
        EXPECT_EQ(single_ring.pipeline->scan_frames, 1U);
        EXPECT_EQ(
                single_ring.pipeline->detection.result.status,
                FrontierDetectionStatus::Invalid);
        EXPECT_EQ(
                single_ring.pipeline->hop_result.status,
                PathCheckStatus::InvalidInput);

        const DemoScene & bootstrap = stages[2U].scene;
        ASSERT_TRUE(bootstrap.pipeline.has_value());
        EXPECT_GT(cubeCount(bootstrap, "voxel_free"), 0U);
        EXPECT_EQ(lineCount(bootstrap, "known_free_hop"), 0U);
        EXPECT_EQ(cubeCount(bootstrap, "frontier_candidate"), 0U);
        EXPECT_EQ(bootstrap.pipeline->observation_epochs, 1U);
        EXPECT_EQ(bootstrap.pipeline->scan_frames, config.yaw_steps);
        EXPECT_EQ(
                bootstrap.pipeline->detection.result.status,
                FrontierDetectionStatus::Invalid);
        EXPECT_EQ(
                bootstrap.pipeline->hop_result.status,
                PathCheckStatus::InvalidInput);
        EXPECT_GT(
                bootstrap.pipeline->known_voxels,
                single_ring.pipeline->known_voxels);

        float minimum_display_x = std::numeric_limits<float>::max();
        float maximum_display_x = std::numeric_limits<float>::lowest();
        for(const DemoCube & cube : bootstrap.cubes) {
            if(cube.ns == "voxel_free" || cube.ns == "voxel_occupied") {
                minimum_display_x = std::min(minimum_display_x, cube.center.x);
                maximum_display_x = std::max(maximum_display_x, cube.center.x);
            }
        }
        EXPECT_LT(minimum_display_x, config.initial_x);
        EXPECT_GT(maximum_display_x, config.initial_x);

        const DemoScene & validated = stages[3U].scene;
        ASSERT_TRUE(validated.pipeline.has_value());
        EXPECT_GT(cubeCount(validated, "voxel_free"), 0U);
        EXPECT_EQ(lineCount(validated, "known_free_hop"), 1U);
        EXPECT_EQ(cubeCount(validated, "frontier_candidate"), 0U);
        EXPECT_EQ(validated.pipeline->observation_epochs, 1U);
        EXPECT_EQ(validated.pipeline->scan_frames, config.yaw_steps);
        EXPECT_EQ(
                validated.pipeline->known_voxels,
                bootstrap.pipeline->known_voxels);
        EXPECT_EQ(
                validated.pipeline->detection.result.status,
                FrontierDetectionStatus::Invalid);
        EXPECT_NE(
                validated.pipeline->hop_result.status,
                PathCheckStatus::InvalidInput);
        ASSERT_TRUE(validated.pipeline->hop_result.safe());
        EXPECT_FLOAT_EQ(
                validated.pipeline->final_position.x,
                config.initial_x + config.hop_distance);
        EXPECT_FLOAT_EQ(validated.pipeline->final_position.y, config.initial_y);

        const DemoScene & accumulated = stages[4U].scene;
        ASSERT_TRUE(accumulated.pipeline.has_value());
        EXPECT_EQ(accumulated.pipeline->observation_epochs, 2U);
        EXPECT_NE(
                accumulated.pipeline->detection.result.status,
                FrontierDetectionStatus::Invalid);
        EXPECT_FALSE(accumulated.pipeline->detection.result.regions.empty());
        EXPECT_EQ(sphereCount(accumulated, "frontier_selected_endpoint"), 1U);
        EXPECT_EQ(sphereCount(accumulated, "frontier_selected_hit"), 0U);
    }

    TEST(FrontierGeometryDemoTest, FocusCoversNoCandidateAndRejectionStates)
    {
        FrontierGeometryDemoConfig no_candidate_config;
        no_candidate_config.selected_phi_degrees = 72.0F;
        const DemoScene no_candidate =
                FrontierGeometryDemo(no_candidate_config).buildScene();
        ASSERT_TRUE(no_candidate.pipeline.has_value());
        EXPECT_FALSE(no_candidate.pipeline->focused_candidate.has_value());
        EXPECT_TRUE(hasText(no_candidate, "NO FRONTIER CANDIDATE"));
        EXPECT_TRUE(hasText(
                no_candidate,
                "GLOBAL: " + std::to_string(
                                      no_candidate.pipeline->detection.result.regions.size())));
        EXPECT_TRUE(hasText(no_candidate, "ACCEPTED REGION"));

        FrontierGeometryDemoConfig vertical_config;
        vertical_config.selected_phi_degrees = 180.0F;
        const DemoScene vertical = FrontierGeometryDemo(vertical_config).buildScene();
        ASSERT_TRUE(vertical.pipeline.has_value());
        ASSERT_TRUE(vertical.pipeline->focused_candidate.has_value());
        const auto vertical_candidate = std::find_if(
                vertical.pipeline->detection.trace.candidates.begin(),
                vertical.pipeline->detection.trace.candidates.end(),
                [&](const FrontierCandidateTrace & candidate) {
                    return candidate.key == *vertical.pipeline->focused_candidate;
                });
        ASSERT_NE(
                vertical_candidate,
                vertical.pipeline->detection.trace.candidates.end());
        EXPECT_FALSE(vertical_candidate->vertical_passed);
        EXPECT_TRUE(hasText(vertical, "VERTICAL REJECTED"));

        const DemoScene support = FrontierGeometryDemo {}.buildScene();
        ASSERT_TRUE(support.pipeline.has_value());
        ASSERT_TRUE(support.pipeline->focused_candidate.has_value());
        const auto support_candidate = std::find_if(
                support.pipeline->detection.trace.candidates.begin(),
                support.pipeline->detection.trace.candidates.end(),
                [&](const FrontierCandidateTrace & candidate) {
                    return candidate.key == *support.pipeline->focused_candidate;
                });
        ASSERT_NE(
                support_candidate,
                support.pipeline->detection.trace.candidates.end());
        EXPECT_TRUE(support_candidate->vertical_passed);
        EXPECT_FALSE(support_candidate->support_passed);
        EXPECT_FALSE(support_candidate->support_attempts.empty());
        EXPECT_GT(lineCount(support, "support_reject_envelope"), 0U);
        EXPECT_TRUE(std::any_of(
                support.pipeline->detection.trace.components.begin(),
                support.pipeline->detection.trace.components.end(),
                [](const FrontierComponentTrace & component) {
                    return component.rejection == FrontierComponentRejection::None;
                }));
        EXPECT_TRUE(std::any_of(
                support.pipeline->detection.trace.components.begin(),
                support.pipeline->detection.trace.components.end(),
                [](const FrontierComponentTrace & component) {
                    return component.rejection != FrontierComponentRejection::None;
                }));
    }

    TEST(FrontierGeometryDemoTest, TraceTruncationIsVisibleAndDoesNotChangeRegions)
    {
        const DemoScene baseline = FrontierGeometryDemo {}.buildScene();
        FrontierGeometryDemoConfig truncated_config;
        truncated_config.max_trace_candidates = 1U;
        truncated_config.max_trace_support_samples = 1U;
        truncated_config.max_trace_components = 1U;
        truncated_config.max_trace_geometry_elements = 1U;
        const DemoScene truncated =
                FrontierGeometryDemo(truncated_config).buildScene();

        ASSERT_TRUE(baseline.pipeline.has_value());
        ASSERT_TRUE(truncated.pipeline.has_value());
        EXPECT_TRUE(truncated.pipeline->detection.trace.truncated);
        EXPECT_TRUE(hasText(truncated, "TRACE TRUNCATED"));
        if(!truncated.pipeline->focused_candidate.has_value()) {
            EXPECT_TRUE(hasText(truncated, "NO RECORDED CANDIDATE"));
            EXPECT_FALSE(hasText(truncated, "NO FRONTIER CANDIDATE IN WINDOW"));
        }
        EXPECT_EQ(
                baseline.pipeline->detection.result.status,
                truncated.pipeline->detection.result.status);
        ASSERT_EQ(
                baseline.pipeline->detection.result.regions.size(),
                truncated.pipeline->detection.result.regions.size());
        for(std::size_t index = 0U;
            index < baseline.pipeline->detection.result.regions.size(); ++index)
        {
            EXPECT_EQ(
                    baseline.pipeline->detection.result.regions[index].stable_key,
                    truncated.pipeline->detection.result.regions[index].stable_key);
        }
    }

}// namespace SwarmController
