#include "swarm_controller/FrontierComponentAuditReplay.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#ifndef SWARM_CONTROLLER_TEST_FIXTURE_DIR
#error "SWARM_CONTROLLER_TEST_FIXTURE_DIR must be defined"
#endif

namespace SwarmController {
    namespace {

        std::filesystem::path fixturePath(const char * filename)
        {
            return std::filesystem::path {SWARM_CONTROLLER_TEST_FIXTURE_DIR}
                   / filename;
        }

        ComponentAuditSnapshot loadFixture(
                const FrontierComponentAuditReplay & replay)
        {
            return replay.loadSnapshot(
                    fixturePath("frontier_component_audit_frame3_components.csv"),
                    fixturePath("frontier_component_audit_frame3_membership.csv"));
        }

        const ComponentAuditStageScene & stageByName(
                const std::vector<ComponentAuditStageScene> & stages,
                const std::string & name)
        {
            const auto found = std::find_if(
                    stages.begin(), stages.end(), [&](const auto & stage) {
                        return stage.stage == name;
                    });
            if(found == stages.end()) {
                throw std::runtime_error("missing stage: " + name);
            }
            return *found;
        }

        const ComponentAuditPointLayer & pointLayerByNamespace(
                const ComponentAuditScene & scene, const std::string & ns,
                const std::int32_t id = 0)
        {
            const auto found = std::find_if(
                    scene.point_layers.begin(), scene.point_layers.end(),
                    [&](const auto & layer) {
                        return layer.ns == ns && layer.id == id;
                    });
            if(found == scene.point_layers.end()) {
                throw std::runtime_error("missing point layer: " + ns);
            }
            return *found;
        }

        bool hasNamespace(
                const ComponentAuditScene & scene, const std::string & ns)
        {
            return std::any_of(
                           scene.boxes.begin(), scene.boxes.end(),
                           [&](const auto & item) { return item.ns == ns; })
                   || std::any_of(
                           scene.point_layers.begin(), scene.point_layers.end(),
                           [&](const auto & item) { return item.ns == ns; })
                   || std::any_of(
                           scene.line_layers.begin(), scene.line_layers.end(),
                           [&](const auto & item) { return item.ns == ns; })
                   || std::any_of(
                           scene.arrows.begin(), scene.arrows.end(),
                           [&](const auto & item) { return item.ns == ns; })
                   || std::any_of(
                           scene.texts.begin(), scene.texts.end(),
                           [&](const auto & item) { return item.ns == ns; });
        }

        TEST(FrontierComponentAuditReplayTest, LoadsCompleteFrameThreeSnapshot)
        {
            const FrontierComponentAuditReplay replay;
            const ComponentAuditSnapshot snapshot = loadFixture(replay);

            EXPECT_EQ(snapshot.frame_index, 3U);
            EXPECT_EQ(snapshot.bag_timestamp_ns, 1784164312999738429ULL);
            EXPECT_EQ(snapshot.map_stamp_ns, 1784164310186407576ULL);
            ASSERT_EQ(snapshot.components.size(), 108U);
            EXPECT_EQ(snapshot.components[0].exact_column_count, 2220U);
            EXPECT_EQ(
                    snapshot.components[0].rejection,
                    ComponentAuditRejection::Direction);
            EXPECT_EQ(snapshot.components[0].columns.size(), 2220U);
            EXPECT_EQ(snapshot.components[86].exact_column_count, 15U);
            EXPECT_EQ(
                    snapshot.components[86].rejection,
                    ComponentAuditRejection::None);
            EXPECT_EQ(snapshot.components[107].columns.size(), 1U);
        }

        TEST(FrontierComponentAuditReplayTest, RecomputesConservationAndGapAudit)
        {
            const FrontierComponentAuditReplay replay;
            const ComponentAuditSnapshot snapshot = loadFixture(replay);
            const ComponentAuditAnalysis analysis = replay.analyze(snapshot);

            EXPECT_EQ(analysis.total_components, 108U);
            EXPECT_EQ(analysis.total_columns, 2519U);
            EXPECT_EQ(
                    analysis.component_counts[static_cast<std::size_t>(
                            ComponentAuditRejection::Columns)],
                    103U);
            EXPECT_EQ(
                    analysis.component_counts[static_cast<std::size_t>(
                            ComponentAuditRejection::Direction)],
                    4U);
            EXPECT_EQ(
                    analysis.component_counts[static_cast<std::size_t>(
                            ComponentAuditRejection::None)],
                    1U);
            EXPECT_EQ(
                    analysis.column_counts[static_cast<std::size_t>(
                            ComponentAuditRejection::Columns)],
                    242U);
            EXPECT_EQ(
                    analysis.column_counts[static_cast<std::size_t>(
                            ComponentAuditRejection::Direction)],
                    2262U);
            EXPECT_EQ(
                    analysis.column_counts[static_cast<std::size_t>(
                            ComponentAuditRejection::None)],
                    15U);

            ASSERT_EQ(analysis.one_column_gap_pairs.size(), 122U);
            ASSERT_EQ(analysis.beneficial_gap_pairs.size(), 6U);
            const ComponentAuditGapPair & selected =
                    analysis.beneficial_gap_pairs.front();
            EXPECT_EQ(selected.first_component_index, 26U);
            EXPECT_EQ(selected.second_component_index, 37U);
            EXPECT_EQ(selected.discrete_gap_columns, 1U);
            EXPECT_EQ(selected.merged_decision.exact_column_count, 16U);
            EXPECT_NEAR(selected.merged_decision.area, 0.64F, 1.0e-5F);
            EXPECT_NEAR(
                    selected.merged_decision.direction_consistency,
                    0.770115F, 1.0e-5F);
            EXPECT_EQ(
                    selected.merged_decision.rejection,
                    ComponentAuditRejection::None);

            EXPECT_EQ(analysis.radius_two_groups.size(), 20U);
            EXPECT_EQ(analysis.baseline_accepted_components, 1U);
            EXPECT_EQ(analysis.radius_two_accepted_groups, 0U);
            const auto group = std::find_if(
                    analysis.radius_two_groups.begin(),
                    analysis.radius_two_groups.end(), [](const auto & candidate) {
                        return std::find(
                                       candidate.component_indices.begin(),
                                       candidate.component_indices.end(), 86U)
                               != candidate.component_indices.end();
                    });
            ASSERT_NE(group, analysis.radius_two_groups.end());
            EXPECT_EQ(group->component_indices.size(), 82U);
            EXPECT_EQ(
                    group->merged_decision.rejection,
                    ComponentAuditRejection::Direction);
        }

        TEST(FrontierComponentAuditReplayTest, BuildsFourIndependentAuditStages)
        {
            const FrontierComponentAuditReplay replay;
            const auto stages = replay.buildStageScenes(loadFixture(replay));

            ASSERT_EQ(stages.size(), 4U);
            EXPECT_EQ(stages[0].stage, "audit_overview");
            EXPECT_EQ(stages[1].stage, "component_rejection");
            EXPECT_EQ(stages[2].stage, "direction_evidence");
            EXPECT_EQ(stages[3].stage, "gap_counterfactual");

            const auto & overview = stageByName(stages, "audit_overview").scene;
            EXPECT_TRUE(hasNamespace(overview, "audit_component_count_bar"));
            EXPECT_TRUE(hasNamespace(overview, "audit_column_mass_bar"));
            EXPECT_TRUE(hasNamespace(overview, "audit_thresholds"));

            const auto & rejection =
                    stageByName(stages, "component_rejection").scene;
            EXPECT_EQ(
                    pointLayerByNamespace(
                            rejection, "audit_component_accepted")
                            .points.size(),
                    15U);
            EXPECT_EQ(
                    pointLayerByNamespace(
                            rejection,
                            "audit_component_min_columns_rejected", 0)
                            .points.size(),
                    10U);
            EXPECT_EQ(
                    pointLayerByNamespace(
                            rejection,
                            "audit_component_min_columns_rejected", 1)
                            .points.size(),
                    6U);
            EXPECT_EQ(
                    pointLayerByNamespace(
                            rejection, "audit_min_columns_threshold")
                            .points.size(),
                    12U);

            const auto & direction =
                    stageByName(stages, "direction_evidence").scene;
            EXPECT_EQ(
                    pointLayerByNamespace(
                            direction, "audit_direction_rejected_component")
                            .points.size(),
                    13U);
            EXPECT_EQ(direction.arrows.size(), 4U);

            const auto & gap =
                    stageByName(stages, "gap_counterfactual").scene;
            EXPECT_EQ(
                    pointLayerByNamespace(gap, "audit_gap_component_a")
                            .points.size(),
                    7U);
            EXPECT_EQ(
                    pointLayerByNamespace(gap, "audit_gap_component_b")
                            .points.size(),
                    9U);
            EXPECT_TRUE(hasNamespace(gap, "audit_one_column_gap"));
            EXPECT_TRUE(hasNamespace(gap, "audit_pair_counterfactual_bounds"));
            EXPECT_TRUE(hasNamespace(gap, "audit_radius_two_result"));
        }

        TEST(FrontierComponentAuditReplayTest, RejectsIncompleteMembership)
        {
            const auto unique = std::to_string(
                    std::chrono::steady_clock::now().time_since_epoch().count());
            const std::filesystem::path directory =
                    std::filesystem::temp_directory_path()
                    / ("frontier-component-audit-" + unique);
            ASSERT_TRUE(std::filesystem::create_directory(directory));
            const std::filesystem::path component_copy =
                    directory / "components.csv";
            const std::filesystem::path membership_copy =
                    directory / "membership.csv";
            ASSERT_TRUE(std::filesystem::copy_file(
                    fixturePath("frontier_component_audit_frame3_components.csv"),
                    component_copy));

            std::ifstream source(fixturePath(
                    "frontier_component_audit_frame3_membership.csv"));
            ASSERT_TRUE(source.good());
            const std::string content {
                    std::istreambuf_iterator<char>(source),
                    std::istreambuf_iterator<char>()};
            const std::size_t final_row = content.find_last_of('\n', content.size() - 2U);
            ASSERT_NE(final_row, std::string::npos);
            std::ofstream truncated(membership_copy, std::ios::binary);
            truncated.write(
                    content.data(), static_cast<std::streamsize>(final_row + 1U));
            truncated.close();
            ASSERT_TRUE(truncated.good());

            const FrontierComponentAuditReplay replay;
            EXPECT_THROW(
                    replay.loadSnapshot(component_copy, membership_copy),
                    std::runtime_error);
            std::error_code cleanup_error;
            std::filesystem::remove_all(directory, cleanup_error);
            EXPECT_FALSE(cleanup_error);
        }

        TEST(FrontierComponentAuditReplayTest, RejectsInvalidConfiguration)
        {
            FrontierComponentAuditReplayConfig config;
            config.thresholds.min_direction_consistency = 1.1F;
            EXPECT_THROW(
                    FrontierComponentAuditReplay replay(config),
                    std::invalid_argument);
        }

    }// namespace
}// namespace SwarmController
