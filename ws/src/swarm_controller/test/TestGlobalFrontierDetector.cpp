#include "swarm_controller/GlobalFrontierDetector.hpp"
#include "swarm_controller/KnownFreePathChecker.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace SwarmController {

    namespace {

        void setFree(octomap::OcTree & tree, const float x, const float y, const float z)
        {
            tree.updateNode(octomap::point3d(x, y, z), false);
        }

        void setOccupied(octomap::OcTree & tree, const float x, const float y, const float z)
        {
            tree.updateNode(octomap::point3d(x, y, z), true);
        }

        void fillKeyBox(
                octomap::OcTree & tree, const Point3f & minimum,
                const Point3f & maximum, const bool occupied)
        {
            const auto min_key = tree.coordToKey(minimum.x, minimum.y, minimum.z);
            const auto max_key = tree.coordToKey(maximum.x, maximum.y, maximum.z);
            for(std::uint32_t x = min_key[0]; x <= max_key[0]; ++x) {
                for(std::uint32_t y = min_key[1]; y <= max_key[1]; ++y) {
                    for(std::uint32_t z = min_key[2]; z <= max_key[2]; ++z) {
                        tree.updateNode(
                                octomap::OcTreeKey {
                                        static_cast<octomap::key_type>(x),
                                        static_cast<octomap::key_type>(y),
                                        static_cast<octomap::key_type>(z)},
                                occupied, true);
                    }
                }
            }
            tree.updateInnerOccupancy();
        }

        void addCorridor(
                octomap::OcTree & tree, const int x_min, const int x_max,
                const int y_min, const int y_max, const int z_min, const int z_max)
        {
            for(int x = x_min; x <= x_max; ++x) {
                for(int y = y_min; y <= y_max; ++y) {
                    for(int z = z_min; z <= z_max; ++z) {
                        setFree(
                                tree, 0.1F * static_cast<float>(x),
                                0.1F * static_cast<float>(y),
                                0.1F * static_cast<float>(z));
                    }
                }
            }
            tree.updateInnerOccupancy();
        }

        GlobalFrontierDetectorConfig testConfig()
        {
            GlobalFrontierDetectorConfig config;
            config.column_stride_voxels = 1U;
            config.min_columns = 4U;
            config.min_area = 0.05F;
            config.min_span = 0.3F;
            config.min_direction_consistency = 0.3F;
            config.support_depth = 0.2;
            config.max_frontier_columns = 10'000U;
            return config;
        }

        std::uint64_t supportRejected(const FrontierDetectionDiagnostics & diagnostics)
        {
            return diagnostics.support_rejected_unknown
                   + diagnostics.support_rejected_occupied
                   + diagnostics.support_rejected_out_of_bounds;
        }

        std::size_t traceGeometryElements(const FrontierDetectionTrace & trace)
        {
            std::size_t count = 0U;
            for(const FrontierCandidateTrace & candidate : trace.candidates) {
                count += candidate.column_points.size();
                for(const FrontierSupportAttemptTrace & attempt :
                    candidate.support_attempts)
                {
                    count += attempt.column_points.size();
                }
            }
            for(const FrontierComponentTrace & component : trace.components) {
                count += component.columns.size();
                count += component.edges.size();
            }
            return count;
        }

        void expectCompleteConservation(const FrontierDetectionResult & result)
        {
            const auto & diagnostics = result.diagnostics;
            ASSERT_TRUE(diagnostics.complete);
            EXPECT_EQ(
                    diagnostics.vertical_passed_columns
                            + diagnostics.vertical_rejected_columns,
                    diagnostics.unknown_neighbor_candidate_columns);
            EXPECT_EQ(
                    diagnostics.support_passed_columns + supportRejected(diagnostics),
                    diagnostics.vertical_passed_columns);
            EXPECT_EQ(
                    std::accumulate(
                            diagnostics.component_size_buckets.begin(),
                            diagnostics.component_size_buckets.end(), std::uint64_t {0U}),
                    diagnostics.components_built);
            EXPECT_EQ(
                    diagnostics.component_primary_rejected_columns
                            + diagnostics.component_primary_rejected_area
                            + diagnostics.component_primary_rejected_span
                            + diagnostics.component_primary_rejected_direction
                            + diagnostics.components_accepted,
                    diagnostics.components_built);
            EXPECT_EQ(result.raw_columns, diagnostics.sampled_free_columns);
            EXPECT_EQ(result.scanned_free_voxels, diagnostics.scanned_free_voxels);
            EXPECT_EQ(result.supported_columns, diagnostics.support_passed_columns);
            EXPECT_EQ(
                    result.vertical_rejected_columns,
                    diagnostics.vertical_rejected_columns);
            EXPECT_EQ(result.support_rejected_columns, supportRejected(diagnostics));
        }

        void addKeyColumn(
                octomap::OcTree & tree, const octomap::key_type x,
                const octomap::key_type y, const octomap::key_type z_center)
        {
            for(int offset = -3; offset <= 3; ++offset) {
                tree.updateNode(
                        octomap::OcTreeKey {
                                x, y,
                                static_cast<octomap::key_type>(
                                        static_cast<int>(z_center) + offset)},
                        false, true);
            }
        }

        void addKeySupportRay(
                octomap::OcTree & tree, const octomap::key_type candidate_x,
                const octomap::key_type y, const octomap::key_type z_center)
        {
            for(int depth = 1; depth <= 2; ++depth) {
                tree.updateNode(
                        octomap::OcTreeKey {
                                static_cast<octomap::key_type>(
                                        static_cast<int>(candidate_x) - depth),
                                y, z_center},
                        false, true);
            }
        }

        void expectSameRegions(
                const FrontierDetectionResult & lhs,
                const FrontierDetectionResult & rhs,
                const bool compare_timings = true)
        {
            ASSERT_EQ(lhs.status, rhs.status);
            ASSERT_EQ(lhs.reason, rhs.reason);
            ASSERT_EQ(lhs.regions.size(), rhs.regions.size());
            for(std::size_t index = 0U; index < lhs.regions.size(); ++index) {
                const auto & left = lhs.regions[index];
                const auto & right = rhs.regions[index];
                EXPECT_EQ(left.stable_key, right.stable_key);
                EXPECT_EQ(left.columns, right.columns);
                EXPECT_FLOAT_EQ(left.representative.x, right.representative.x);
                EXPECT_FLOAT_EQ(left.representative.y, right.representative.y);
                EXPECT_FLOAT_EQ(left.representative.z, right.representative.z);
                EXPECT_FLOAT_EQ(left.unknown_direction.x, right.unknown_direction.x);
                EXPECT_FLOAT_EQ(left.unknown_direction.y, right.unknown_direction.y);
                EXPECT_FLOAT_EQ(left.unknown_direction.z, right.unknown_direction.z);
                EXPECT_FLOAT_EQ(left.information_gain, right.information_gain);
                EXPECT_FLOAT_EQ(left.area, right.area);
                EXPECT_FLOAT_EQ(left.horizontal_span, right.horizontal_span);
                EXPECT_FLOAT_EQ(
                        left.direction_consistency, right.direction_consistency);
            }
            EXPECT_EQ(lhs.raw_columns, rhs.raw_columns);
            EXPECT_EQ(lhs.scanned_free_voxels, rhs.scanned_free_voxels);
            EXPECT_EQ(lhs.supported_columns, rhs.supported_columns);
            EXPECT_EQ(lhs.vertical_rejected_columns, rhs.vertical_rejected_columns);
            EXPECT_EQ(lhs.support_rejected_columns, rhs.support_rejected_columns);

            const auto & left = lhs.diagnostics;
            const auto & right = rhs.diagnostics;
            EXPECT_EQ(left.complete, right.complete);
            EXPECT_EQ(left.scanned_free_voxels, right.scanned_free_voxels);
            EXPECT_EQ(left.sampled_free_columns, right.sampled_free_columns);
            EXPECT_EQ(
                    left.unknown_neighbor_candidate_columns,
                    right.unknown_neighbor_candidate_columns);
            EXPECT_EQ(left.vertical_passed_columns, right.vertical_passed_columns);
            EXPECT_EQ(left.vertical_rejected_columns, right.vertical_rejected_columns);
            EXPECT_EQ(left.support_passed_columns, right.support_passed_columns);
            EXPECT_EQ(left.support_rejected_unknown, right.support_rejected_unknown);
            EXPECT_EQ(left.support_rejected_occupied, right.support_rejected_occupied);
            EXPECT_EQ(
                    left.support_rejected_out_of_bounds,
                    right.support_rejected_out_of_bounds);
            EXPECT_EQ(left.support_samples_attempted, right.support_samples_attempted);
            EXPECT_EQ(
                    left.support_failure_position_unavailable,
                    right.support_failure_position_unavailable);
            EXPECT_EQ(
                    left.support_failure_depth_octiles,
                    right.support_failure_depth_octiles);
            EXPECT_EQ(left.components_built, right.components_built);
            EXPECT_EQ(left.component_size_buckets, right.component_size_buckets);
            EXPECT_EQ(
                    left.component_primary_rejected_columns,
                    right.component_primary_rejected_columns);
            EXPECT_EQ(
                    left.component_primary_rejected_area,
                    right.component_primary_rejected_area);
            EXPECT_EQ(
                    left.component_primary_rejected_span,
                    right.component_primary_rejected_span);
            EXPECT_EQ(
                    left.component_primary_rejected_direction,
                    right.component_primary_rejected_direction);
            EXPECT_EQ(left.components_accepted, right.components_accepted);
            if(compare_timings) {
                EXPECT_DOUBLE_EQ(
                        left.timings.leaf_scan_seconds,
                        right.timings.leaf_scan_seconds);
                EXPECT_DOUBLE_EQ(
                        left.timings.vertical_seconds,
                        right.timings.vertical_seconds);
                EXPECT_DOUBLE_EQ(
                        left.timings.support_seconds,
                        right.timings.support_seconds);
                EXPECT_DOUBLE_EQ(
                        left.timings.component_seconds,
                        right.timings.component_seconds);
                EXPECT_DOUBLE_EQ(left.timings.total_seconds, right.timings.total_seconds);
            }
        }

    }// namespace

    TEST(GlobalFrontierDetectorTest, RejectsInvalidConfiguration)
    {
        auto config = testConfig();
        config.min_direction_consistency = 1.1F;
        EXPECT_THROW({ GlobalFrontierDetector detector {config}; }, std::invalid_argument);

        config = testConfig();
        config.max_trace_geometry_elements = 0U;
        EXPECT_THROW({ GlobalFrontierDetector detector {config}; }, std::invalid_argument);
    }

    TEST(GlobalFrontierDetectorTest, RejectsSupportRayAboveConfiguredSampleLimit)
    {
        auto config = testConfig();
        config.max_support_samples_per_column = 1U;
        EXPECT_THROW({ GlobalFrontierDetector detector {config}; }, std::invalid_argument);
    }

    TEST(GlobalFrontierDetectorTest, UsesOneCheckedSupportSampleCountAtPrecisionBoundary)
    {
        auto config = testConfig();
        config.resolution = 0.032432430982589723;
        config.support_depth = 0.162162155F;
        config.max_support_samples_per_column = 5U;
        config.min_z_layers = 5U;
        config.min_z_span = static_cast<float>(4.0 * config.resolution);

        octomap::OcTree tree(config.resolution);
        const octomap::OcTreeKey anchor_key = tree.coordToKey(0.0, 0.0, 0.0);
        for(int z_offset = -2; z_offset <= 2; ++z_offset) {
            tree.updateNode(
                    octomap::OcTreeKey {
                            anchor_key[0], anchor_key[1],
                            static_cast<octomap::key_type>(
                                    static_cast<int>(anchor_key[2]) + z_offset)},
                    false, true);
        }
        for(int depth = 1; depth <= 5; ++depth) {
            tree.updateNode(
                    octomap::OcTreeKey {
                            static_cast<octomap::key_type>(
                                    static_cast<int>(anchor_key[0]) - depth),
                            anchor_key[1], anchor_key[2]},
                    false, true);
        }
        tree.updateInnerOccupancy();

        const TracedFrontierDetectionResult traced =
                GlobalFrontierDetector(config).detectWithTrace(tree);
        const FrontierColumnKey expected_key {
                static_cast<std::int64_t>(anchor_key[0]),
                static_cast<std::int64_t>(anchor_key[1])};
        const auto candidate = std::find_if(
                traced.trace.candidates.begin(), traced.trace.candidates.end(),
                [&](const FrontierCandidateTrace & value) {
                    return value.key == expected_key;
                });
        ASSERT_NE(candidate, traced.trace.candidates.end());
        const auto selected = std::find_if(
                candidate->support_attempts.begin(), candidate->support_attempts.end(),
                [](const FrontierSupportAttemptTrace & attempt) {
                    return attempt.selected;
                });
        ASSERT_NE(selected, candidate->support_attempts.end());
        ASSERT_EQ(selected->samples.size(), 5U);
        EXPECT_TRUE(std::all_of(
                selected->samples.begin(), selected->samples.end(),
                [](const FrontierSupportSampleTrace & sample) {
                    return sample.state == FrontierTraceSampleState::Free;
                }));
    }

    TEST(GlobalFrontierDetectorTest, SingleThinObservationDoesNotBecomeRegion)
    {
        octomap::OcTree tree(0.1);
        for(int y = -20; y <= 20; ++y) {
            for(int z = 12; z <= 18; ++z) {
                setFree(tree, 1.0F, 0.1F * static_cast<float>(y), 0.1F * static_cast<float>(z));
            }
        }
        tree.updateInnerOccupancy();
        const auto result = GlobalFrontierDetector(testConfig()).detect(tree);
        EXPECT_TRUE(result.accepted());
        EXPECT_TRUE(result.regions.empty());
        EXPECT_GT(result.raw_columns, 0U);
        expectCompleteConservation(result);
        EXPECT_GT(result.diagnostics.support_rejected_unknown, 0U);
        EXPECT_EQ(result.diagnostics.components_accepted, 0U);
    }

    TEST(GlobalFrontierDetectorTest, EmptyTreeHasNoFrontierEvidence)
    {
        octomap::OcTree tree(0.1);

        const auto result = GlobalFrontierDetector(testConfig()).detect(tree);

        EXPECT_EQ(result.status, FrontierDetectionStatus::Empty);
        EXPECT_TRUE(result.regions.empty());
        expectCompleteConservation(result);
        EXPECT_EQ(result.diagnostics.scanned_free_voxels, 0U);
        EXPECT_EQ(result.diagnostics.unknown_neighbor_candidate_columns, 0U);
    }

    TEST(GlobalFrontierDetectorTest, SupportedCorridorFrontierIsDetected)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -1.8F, 0.8F}, {1.0F, 1.8F, 2.2F}, false);
        fillKeyBox(tree, {-0.1F, -1.9F, 0.7F}, {-0.1F, 1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, -1.9F, 0.7F}, {1.0F, -1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, 1.9F, 0.7F}, {1.0F, 1.9F, 2.3F}, true);

        const auto result = GlobalFrontierDetector(testConfig()).detect(tree);
        ASSERT_GT(result.supported_columns, 0U)
                << "raw=" << result.raw_columns
                << " vertical_rejected=" << result.vertical_rejected_columns
                << " support_rejected=" << result.support_rejected_columns
                << " reason=" << result.reason;
        ASSERT_FALSE(result.regions.empty())
                << "raw=" << result.raw_columns << " supported=" << result.supported_columns
                << " reason=" << result.reason;
        ASSERT_EQ(result.status, FrontierDetectionStatus::Accepted);
        ASSERT_FALSE(result.regions.empty());
        const auto region_it = std::find_if(
                result.regions.begin(), result.regions.end(),
                [](const FrontierRegion & region) {
                    return region.unknown_direction.x > 0.5F;
                });
        ASSERT_NE(region_it, result.regions.end())
                << "detected regions=" << result.regions.size();
        EXPECT_GE(region_it->columns.size(), 2U);
        EXPECT_GT(region_it->area, 0.05F);
        EXPECT_GT(region_it->direction_consistency, 0.25F);
        EXPECT_GT(region_it->unknown_direction.x, 0.5F);
        EXPECT_NEAR(region_it->unknown_direction.y, 0.0F, 0.5F);
        expectCompleteConservation(result);
    }

    TEST(GlobalFrontierDetectorTest, TwoSeparatedCorridorFrontiersRemainDistinct)
    {
        octomap::OcTree tree(0.1);
        for(const float center_y : {-2.0F, 2.0F}) {
            fillKeyBox(
                    tree, {0.0F, center_y - 0.8F, 0.8F},
                    {1.0F, center_y + 0.8F, 2.2F}, false);
            fillKeyBox(
                    tree, {-0.1F, center_y - 0.9F, 0.7F},
                    {-0.1F, center_y + 0.9F, 2.3F}, true);
            fillKeyBox(
                    tree, {0.0F, center_y - 0.9F, 0.7F},
                    {1.0F, center_y - 0.9F, 2.3F}, true);
            fillKeyBox(
                    tree, {0.0F, center_y + 0.9F, 0.7F},
                    {1.0F, center_y + 0.9F, 2.3F}, true);
        }

        const auto result = GlobalFrontierDetector(testConfig()).detect(tree);
        const std::size_t forward_regions = static_cast<std::size_t>(std::count_if(
                result.regions.begin(), result.regions.end(),
                [](const FrontierRegion & region) {
                    return region.unknown_direction.x > 0.5F;
                }));

        EXPECT_EQ(forward_regions, 2U);
        expectCompleteConservation(result);
    }

    TEST(GlobalFrontierDetectorTest, InwardEvidenceIsIndependentFromBodyClearance)
    {
        octomap::OcTree tree(0.1);
        addCorridor(tree, 0, 10, -6, 6, 12, 18);
        for(int y = -6; y <= 6; ++y) {
            for(int z = 12; z <= 18; ++z) {
                setOccupied(tree, -0.1F, 0.1F * static_cast<float>(y),
                            0.1F * static_cast<float>(z));
            }
            setOccupied(tree, 0.9F, 0.1F * static_cast<float>(y), 1.7F);
        }
        for(int x = 0; x <= 10; ++x) {
            for(int z = 12; z <= 18; ++z) {
                setOccupied(tree, 0.1F * static_cast<float>(x), -0.7F,
                            0.1F * static_cast<float>(z));
                setOccupied(tree, 0.1F * static_cast<float>(x), 0.7F,
                            0.1F * static_cast<float>(z));
            }
        }
        tree.updateInnerOccupancy();

        const auto result = GlobalFrontierDetector(testConfig()).detect(tree);
        const auto forward = std::find_if(
                result.regions.begin(), result.regions.end(),
                [](const FrontierRegion & candidate) {
                    return candidate.unknown_direction.x > 0.5F;
                });

        ASSERT_NE(forward, result.regions.end());
        EXPECT_GT(result.supported_columns, 0U);
        expectCompleteConservation(result);

        const KnownFreePathChecker checker;
        EXPECT_FALSE(checker.checkBody(tree, forward->representative).safe());
    }

    TEST(GlobalFrontierDetectorTest, DefaultConfigurationAcceptsSupportedTunnelFrontier)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -2.0F, 1.0F}, {2.0F, 2.0F, 2.0F}, false);
        fillKeyBox(tree, {0.0F, -2.1F, 0.9F}, {2.0F, -2.1F, 2.1F}, true);
        fillKeyBox(tree, {0.0F, 2.1F, 0.9F}, {2.0F, 2.1F, 2.1F}, true);

        const auto result = GlobalFrontierDetector {}.detect(tree);

        ASSERT_TRUE(result.accepted()) << result.reason;
        EXPECT_FALSE(result.regions.empty())
                << "raw=" << result.raw_columns
                << " supported=" << result.supported_columns
                << " vertical_rejected=" << result.vertical_rejected_columns
                << " support_rejected=" << result.support_rejected_columns
                << " reason=" << result.reason;
        expectCompleteConservation(result);
    }

    TEST(GlobalFrontierDetectorTest, SupportFailureClassifiesOccupiedFirstFailure)
    {
        octomap::OcTree tree(0.1);
        constexpr auto CENTER = static_cast<octomap::key_type>(32'768U);
        constexpr auto CANDIDATE_X = static_cast<octomap::key_type>(32'770U);
        addKeyColumn(tree, CANDIDATE_X, CENTER, CENTER);
        addKeySupportRay(tree, CANDIDATE_X, CENTER, CENTER);
        tree.updateNode(
                octomap::OcTreeKey {
                        static_cast<octomap::key_type>(CANDIDATE_X - 1U),
                        CENTER, CENTER},
                true, true);
        tree.updateInnerOccupancy();

        auto config = testConfig();
        config.min_columns = 1U;
        config.min_area = 0.001F;
        config.min_span = 0.001F;
        const auto result = GlobalFrontierDetector(config).detect(tree);

        expectCompleteConservation(result);
        EXPECT_GT(result.diagnostics.support_rejected_occupied, 0U);
        EXPECT_EQ(
                std::accumulate(
                        result.diagnostics.support_failure_depth_octiles.begin(),
                        result.diagnostics.support_failure_depth_octiles.end(),
                        std::uint64_t {0U})
                        + result.diagnostics.support_failure_position_unavailable,
                supportRejected(result.diagnostics));
    }

    TEST(GlobalFrontierDetectorTest, SupportFailureClassifiesUnknownOnRay)
    {
        octomap::OcTree tree(0.1);
        constexpr auto CENTER = static_cast<octomap::key_type>(32'768U);
        constexpr auto CANDIDATE_X = static_cast<octomap::key_type>(32'770U);
        addKeyColumn(tree, CANDIDATE_X, CENTER, CENTER);
        tree.updateNode(
                octomap::OcTreeKey {
                        static_cast<octomap::key_type>(CANDIDATE_X - 1U),
                        CENTER, CENTER},
                false, true);
        tree.updateInnerOccupancy();

        auto config = testConfig();
        config.min_columns = 1U;
        config.min_area = 0.001F;
        config.min_span = 0.001F;
        const auto traced = GlobalFrontierDetector(config).detectWithTrace(tree);

        expectCompleteConservation(traced.result);
        EXPECT_GT(traced.result.diagnostics.support_rejected_unknown, 0U);
        const auto candidate = std::find_if(
                traced.trace.candidates.begin(), traced.trace.candidates.end(),
                [](const FrontierCandidateTrace & value) {
                    return std::any_of(
                            value.support_attempts.begin(), value.support_attempts.end(),
                            [](const FrontierSupportAttemptTrace & attempt) {
                                return attempt.unknown_direction.x > 0.5F
                                       && attempt.failure
                                                  == FrontierSupportFailure::Unknown;
                            });
                });
        ASSERT_NE(candidate, traced.trace.candidates.end());
        const auto attempt = std::find_if(
                candidate->support_attempts.begin(), candidate->support_attempts.end(),
                [](const FrontierSupportAttemptTrace & value) {
                    return value.unknown_direction.x > 0.5F
                           && value.failure == FrontierSupportFailure::Unknown;
                });
        ASSERT_NE(attempt, candidate->support_attempts.end());
        ASSERT_EQ(attempt->samples.size(), 2U);
        EXPECT_EQ(attempt->samples.front().state, FrontierTraceSampleState::Free);
        EXPECT_EQ(attempt->samples.back().state, FrontierTraceSampleState::Unknown);
        EXPECT_TRUE(attempt->has_first_failure_position);
    }

    TEST(GlobalFrontierDetectorTest, SupportFailureClassifiesOutOfBoundsFirstFailure)
    {
        octomap::OcTree tree(0.1);
        constexpr auto MIN_CANDIDATE_X = static_cast<octomap::key_type>(1U);
        constexpr auto CENTER = static_cast<octomap::key_type>(32'768U);
        addKeyColumn(tree, MIN_CANDIDATE_X, CENTER, CENTER);
        for(int lateral = -1; lateral <= 1; ++lateral) {
            for(int vertical = -2; vertical <= 2; ++vertical) {
                tree.updateNode(
                        octomap::OcTreeKey {
                                static_cast<octomap::key_type>(0U),
                                static_cast<octomap::key_type>(
                                        static_cast<int>(CENTER) + lateral),
                                static_cast<octomap::key_type>(
                                        static_cast<int>(CENTER) + vertical)},
                        false, true);
            }
        }
        tree.updateInnerOccupancy();

        auto config = testConfig();
        config.min_columns = 1U;
        config.min_area = 0.001F;
        config.min_span = 0.001F;
        const auto result = GlobalFrontierDetector(config).detect(tree);

        expectCompleteConservation(result);
        EXPECT_GT(result.diagnostics.support_rejected_out_of_bounds, 0U);
    }

    TEST(GlobalFrontierDetectorTest, ComponentPrimaryRejectionUsesFixedPredicateOrder)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -1.8F, 0.8F}, {1.0F, 1.8F, 2.2F}, false);
        fillKeyBox(tree, {-0.1F, -1.9F, 0.7F}, {-0.1F, 1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, -1.9F, 0.7F}, {1.0F, -1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, 1.9F, 0.7F}, {1.0F, 1.9F, 2.3F}, true);

        auto columns_config = testConfig();
        columns_config.min_columns = 10'000U;
        const auto columns = GlobalFrontierDetector(columns_config).detect(tree);
        expectCompleteConservation(columns);
        EXPECT_EQ(
                columns.diagnostics.component_primary_rejected_columns,
                columns.diagnostics.components_built);

        auto area_config = testConfig();
        area_config.min_columns = 1U;
        area_config.min_area = 10'000.0F;
        const auto area = GlobalFrontierDetector(area_config).detect(tree);
        expectCompleteConservation(area);
        EXPECT_EQ(
                area.diagnostics.component_primary_rejected_area,
                area.diagnostics.components_built);

        auto span_config = testConfig();
        span_config.min_columns = 1U;
        span_config.min_area = 0.001F;
        span_config.min_span = 10'000.0F;
        const auto span = GlobalFrontierDetector(span_config).detect(tree);
        expectCompleteConservation(span);
        EXPECT_EQ(
                span.diagnostics.component_primary_rejected_span,
                span.diagnostics.components_built);
    }

    TEST(GlobalFrontierDetectorTest, DirectionIsTheLastComponentRejectionPredicate)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -1.0F, 1.0F}, {1.0F, 1.0F, 2.0F}, false);
        auto config = testConfig();
        config.min_columns = 1U;
        config.min_area = 0.001F;
        config.min_span = 0.001F;
        config.min_direction_consistency = 1.0F;

        const auto result = GlobalFrontierDetector(config).detect(tree);

        expectCompleteConservation(result);
        EXPECT_GT(result.diagnostics.component_primary_rejected_direction, 0U);
        EXPECT_EQ(result.diagnostics.component_primary_rejected_columns, 0U);
        EXPECT_EQ(result.diagnostics.component_primary_rejected_area, 0U);
        EXPECT_EQ(result.diagnostics.component_primary_rejected_span, 0U);
    }

    TEST(GlobalFrontierDetectorTest, StageTimingCollectionDoesNotChangeRegions)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -1.8F, 0.8F}, {1.0F, 1.8F, 2.2F}, false);
        fillKeyBox(tree, {-0.1F, -1.9F, 0.7F}, {-0.1F, 1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, -1.9F, 0.7F}, {1.0F, -1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, 1.9F, 0.7F}, {1.0F, 1.9F, 2.3F}, true);
        auto without_timing = testConfig();
        auto with_timing = without_timing;
        with_timing.collect_stage_timings = true;

        const auto baseline = GlobalFrontierDetector(without_timing).detect(tree);
        const auto measured = GlobalFrontierDetector(with_timing).detect(tree);

        expectSameRegions(baseline, measured, false);
        EXPECT_EQ(baseline.diagnostics.timings.total_seconds, 0.0);
        EXPECT_GT(measured.diagnostics.timings.total_seconds, 0.0);
    }


    TEST(GlobalFrontierDetectorTest, FreeMapWithoutFrontierStillHonorsScanLimit)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {-1.0F, -1.0F, -1.0F}, {1.0F, 1.0F, 1.0F}, false);
        auto config = testConfig();
        config.max_scanned_free_voxels = 10U;
        config.max_frontier_columns = 10'000U;

        const auto result = GlobalFrontierDetector(config).detect(tree);

        EXPECT_EQ(result.status, FrontierDetectionStatus::ResourceLimit);
        EXPECT_TRUE(result.regions.empty());
        EXPECT_FALSE(result.diagnostics.complete);
    }
    TEST(GlobalFrontierDetectorTest, ResourceLimitRejectsWholeFrame)
    {
        octomap::OcTree tree(0.1);
        addCorridor(tree, 0, 2, -2, 2, 12, 18);
        auto config = testConfig();
        config.max_frontier_columns = 1U;
        const auto result = GlobalFrontierDetector(config).detect(tree);
        EXPECT_EQ(result.status, FrontierDetectionStatus::ResourceLimit);
        EXPECT_TRUE(result.regions.empty());
        EXPECT_FALSE(result.diagnostics.complete);
    }

    TEST(GlobalFrontierDetectorTest, InvalidResolutionProducesPartialDiagnostics)
    {
        octomap::OcTree tree(0.2);
        const auto result = GlobalFrontierDetector(testConfig()).detect(tree);

        EXPECT_EQ(result.status, FrontierDetectionStatus::Invalid);
        EXPECT_FALSE(result.diagnostics.complete);
        EXPECT_EQ(result.diagnostics.scanned_free_voxels, 0U);
        EXPECT_EQ(result.reason, "tree resolution does not match detector resolution");
    }

    TEST(GlobalFrontierDetectorTest, TraceUsesTheSameDetectionResultAndExposesGeometry)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -1.8F, 0.8F}, {1.0F, 1.8F, 2.2F}, false);
        fillKeyBox(tree, {-0.1F, -1.9F, 0.7F}, {-0.1F, 1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, -1.9F, 0.7F}, {1.0F, -1.9F, 2.3F}, true);
        fillKeyBox(tree, {0.0F, 1.9F, 0.7F}, {1.0F, 1.9F, 2.3F}, true);
        const GlobalFrontierDetector detector(testConfig());

        const auto baseline = detector.detect(tree);
        const auto traced = detector.detectWithTrace(tree);

        expectSameRegions(baseline, traced.result);
        ASSERT_FALSE(traced.trace.candidates.empty());
        EXPECT_TRUE(std::any_of(
                traced.trace.candidates.begin(), traced.trace.candidates.end(),
                [](const FrontierCandidateTrace & candidate) {
                    return candidate.vertical_passed && candidate.support_passed
                           && std::any_of(
                                   candidate.support_attempts.begin(),
                                   candidate.support_attempts.end(),
                                   [](const FrontierSupportAttemptTrace & attempt) {
                                       return attempt.selected && attempt.passed()
                                              && !attempt.samples.empty();
                                   });
                }));
        ASSERT_FALSE(traced.trace.components.empty());
        EXPECT_TRUE(std::any_of(
                traced.trace.components.begin(), traced.trace.components.end(),
                [](const FrontierComponentTrace & component) {
                    return !component.columns.empty() && !component.edges.empty();
                }));
        EXPECT_FALSE(traced.trace.truncated);
    }

    TEST(GlobalFrontierDetectorTest, TraceLimitsDoNotChangeDetectionResult)
    {
        octomap::OcTree tree(0.1);
        fillKeyBox(tree, {0.0F, -1.8F, 0.8F}, {1.0F, 1.8F, 2.2F}, false);
        auto config = testConfig();
        config.max_trace_candidates = 1U;
        config.max_trace_support_samples = 1U;
        config.max_trace_components = 1U;
        config.max_trace_geometry_elements = 1U;
        const GlobalFrontierDetector detector(config);

        const auto baseline = detector.detect(tree);
        const auto traced = detector.detectWithTrace(tree);

        expectSameRegions(baseline, traced.result);
        EXPECT_TRUE(traced.trace.truncated);
        EXPECT_LE(traced.trace.candidates.size(), 1U);
        EXPECT_LE(traced.trace.components.size(), 1U);
        EXPECT_LE(traceGeometryElements(traced.trace), 1U);
    }

    TEST(GlobalFrontierDetectorTest, TraceSeparatesDirectionalSupportAttempts)
    {
        octomap::OcTree tree(0.1);
        constexpr auto CENTER = static_cast<octomap::key_type>(32'768U);
        constexpr auto CANDIDATE = static_cast<octomap::key_type>(32'770U);
        addKeyColumn(tree, CANDIDATE, CANDIDATE, CENTER);

        for(int depth = 1; depth <= 2; ++depth) {
            for(int lateral = -1; lateral <= 1; ++lateral) {
                for(int vertical = -2; vertical <= 2; ++vertical) {
                    tree.updateNode(
                            octomap::OcTreeKey {
                                    static_cast<octomap::key_type>(CANDIDATE - depth),
                                    static_cast<octomap::key_type>(
                                            static_cast<int>(CANDIDATE) - lateral),
                                    static_cast<octomap::key_type>(
                                            static_cast<int>(CENTER) + vertical)},
                            false, true);
                    tree.updateNode(
                            octomap::OcTreeKey {
                                    static_cast<octomap::key_type>(
                                            static_cast<int>(CANDIDATE) + lateral),
                                    static_cast<octomap::key_type>(CANDIDATE - depth),
                                    static_cast<octomap::key_type>(
                                            static_cast<int>(CENTER) + vertical)},
                            false, true);
                }
            }
        }
        tree.updateNode(
                octomap::OcTreeKey {
                        static_cast<octomap::key_type>(CANDIDATE - 1U),
                        CANDIDATE, CENTER},
                true, true);
        tree.updateInnerOccupancy();

        auto config = testConfig();
        config.min_columns = 1U;
        config.min_area = 0.001F;
        config.min_span = 0.001F;
        const GlobalFrontierDetector detector(config);
        const auto baseline = detector.detect(tree);
        const auto traced = detector.detectWithTrace(tree);

        expectSameRegions(baseline, traced.result);
        const auto candidate = std::find_if(
                traced.trace.candidates.begin(), traced.trace.candidates.end(),
                [](const FrontierCandidateTrace & trace) {
                    return trace.key
                                   == FrontierColumnKey {
                                           static_cast<std::int64_t>(CANDIDATE),
                                           static_cast<std::int64_t>(CANDIDATE)}
                           && trace.support_attempts.size() >= 2U
                           && trace.support_passed;
                });
        ASSERT_NE(candidate, traced.trace.candidates.end());
        ASSERT_GE(candidate->support_attempts.size(), 2U);
        const auto & rejected = candidate->support_attempts[0];
        const auto & selected = candidate->support_attempts[1];
        EXPECT_EQ(rejected.failure, FrontierSupportFailure::Occupied);
        EXPECT_TRUE(rejected.has_first_failure_position);
        EXPECT_FALSE(rejected.selected);
        EXPECT_FALSE(rejected.samples.empty());
        EXPECT_EQ(selected.failure, FrontierSupportFailure::None);
        EXPECT_FALSE(selected.has_first_failure_position);
        EXPECT_TRUE(selected.selected);
        EXPECT_FALSE(selected.samples.empty());
        EXPECT_NE(rejected.unknown_direction.x, selected.unknown_direction.x);
        EXPECT_NE(rejected.unknown_direction.y, selected.unknown_direction.y);
    }

    TEST(GlobalFrontierDetectorTest, SupportAnchorUsesUpperMedianZAndLexicographicXY)
    {
        octomap::OcTree tree(0.1);
        constexpr auto CENTER = static_cast<octomap::key_type>(32'768U);
        constexpr auto LOWER_X = static_cast<octomap::key_type>(32'772U);
        constexpr auto HIGHER_X = static_cast<octomap::key_type>(32'774U);
        constexpr auto LOWEST_Y = CENTER;
        constexpr auto MIDDLE_Y = static_cast<octomap::key_type>(CENTER + 1U);
        constexpr auto HIGHEST_Y = static_cast<octomap::key_type>(CENTER + 2U);
        const std::array<std::pair<octomap::key_type, octomap::key_type>, 3U>
                raw_keys {{{LOWER_X, HIGHEST_Y},
                           {HIGHER_X, LOWEST_Y},
                           {LOWER_X, MIDDLE_Y}}};
        for(const auto & [x, y] : raw_keys) {
            for(int z_offset = -2; z_offset <= 3; ++z_offset) {
                tree.updateNode(
                        octomap::OcTreeKey {
                                x, y,
                                static_cast<octomap::key_type>(
                                        static_cast<int>(CENTER) + z_offset)},
                        false, true);
            }
            addKeySupportRay(
                    tree, x, y,
                    static_cast<octomap::key_type>(CENTER + 1U));
        }
        tree.updateInnerOccupancy();

        auto config = testConfig();
        config.column_stride_voxels = 4U;
        config.min_columns = 1U;
        config.min_area = 0.001F;
        config.min_span = 0.001F;
        const auto traced = GlobalFrontierDetector(config).detectWithTrace(tree);

        const FrontierColumnKey expected_column {
                static_cast<std::int64_t>(LOWER_X / 4U),
                static_cast<std::int64_t>(LOWEST_Y / 4U)};
        const auto candidate = std::find_if(
                traced.trace.candidates.begin(), traced.trace.candidates.end(),
                [&](const FrontierCandidateTrace & trace) {
                    return trace.key == expected_column;
                });
        ASSERT_NE(candidate, traced.trace.candidates.end());
        const auto attempt = std::find_if(
                candidate->support_attempts.begin(), candidate->support_attempts.end(),
                [](const FrontierSupportAttemptTrace & value) {
                    return value.unknown_direction.x > 0.5F;
                });
        ASSERT_NE(attempt, candidate->support_attempts.end());

        const octomap::point3d expected_anchor = tree.keyToCoord(
                octomap::OcTreeKey {
                        LOWER_X, MIDDLE_Y,
                        static_cast<octomap::key_type>(CENTER + 1U)});
        EXPECT_FLOAT_EQ(attempt->anchor.x, expected_anchor.x());
        EXPECT_FLOAT_EQ(attempt->anchor.y, expected_anchor.y());
        EXPECT_FLOAT_EQ(attempt->anchor.z, expected_anchor.z());
        ASSERT_EQ(attempt->samples.size(), 2U);
        for(std::size_t index = 0U; index < attempt->samples.size(); ++index) {
            const octomap::point3d expected_sample = tree.keyToCoord(
                    octomap::OcTreeKey {
                            static_cast<octomap::key_type>(
                                    LOWER_X
                                    - static_cast<octomap::key_type>(index + 1U)),
                            MIDDLE_Y, static_cast<octomap::key_type>(CENTER + 1U)});
            EXPECT_EQ(attempt->samples[index].state, FrontierTraceSampleState::Free);
            EXPECT_FLOAT_EQ(attempt->samples[index].position.x, expected_sample.x());
            EXPECT_FLOAT_EQ(attempt->samples[index].position.y, expected_sample.y());
            EXPECT_FLOAT_EQ(attempt->samples[index].position.z, expected_sample.z());
        }
    }

}// namespace SwarmController
