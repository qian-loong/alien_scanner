#include "swarm_controller/GlobalFrontierDetector.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
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
            config.support_depth = 0.2F;
            config.support_width = 0.1F;
            config.max_frontier_columns = 10'000U;
            return config;
        }

    }// namespace

    TEST(GlobalFrontierDetectorTest, RejectsInvalidConfiguration)
    {
        auto config = testConfig();
        config.min_direction_consistency = 1.1F;
        EXPECT_THROW({ GlobalFrontierDetector detector {config}; }, std::invalid_argument);
    }

    TEST(GlobalFrontierDetectorTest, RejectsSupportEnvelopeAboveConfiguredSampleLimit)
    {
        auto config = testConfig();
        config.max_support_samples_per_column = 29U;
        EXPECT_THROW({ GlobalFrontierDetector detector {config}; }, std::invalid_argument);
    }

    TEST(GlobalFrontierDetectorTest, SingleThinObservationDoesNotBecomeSupportedRegion)
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
        EXPECT_EQ(result.supported_columns, 0U);
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
    }

    TEST(GlobalFrontierDetectorTest, IncompleteVerticalSupportEnvelopeIsRejected)
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

        EXPECT_EQ(forward, result.regions.end());
        EXPECT_GT(result.support_rejected_columns, 0U);
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
    }

}// namespace SwarmController
