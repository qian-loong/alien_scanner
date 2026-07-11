#include "swarm_controller/KnownFreePathChecker.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>

namespace SwarmController {

    namespace {

        void fillBox(
                octomap::OcTree & tree, const Point3f & minimum, const Point3f & maximum,
                bool occupied)
        {
            const octomap::OcTreeKey min_key =
                    tree.coordToKey(minimum.x, minimum.y, minimum.z);
            const octomap::OcTreeKey max_key =
                    tree.coordToKey(maximum.x, maximum.y, maximum.z);
            for(std::uint32_t x = min_key[0]; x <= max_key[0]; ++x) {
                for(std::uint32_t y = min_key[1]; y <= max_key[1]; ++y) {
                    for(std::uint32_t z = min_key[2]; z <= max_key[2]; ++z) {
                        tree.updateNode(
                                octomap::OcTreeKey {
                                        static_cast<octomap::key_type>(x),
                                        static_cast<octomap::key_type>(y),
                                        static_cast<octomap::key_type>(z),
                                },
                                occupied, true);
                    }
                }
            }
            tree.updateInnerOccupancy();
        }

        octomap::OcTree makeFreeVolume(double resolution)
        {
            octomap::OcTree tree(resolution);
            fillBox(
                    tree, Point3f {-1.0F, -1.0F, -1.0F},
                    Point3f {2.0F, 1.0F, 1.0F}, false);
            return tree;
        }

    }// namespace

    TEST(KnownFreePathCheckerTest, AcceptsSweptBodyInsideKnownFreeVolume)
    {
        octomap::OcTree tree = makeFreeVolume(0.1);
        KnownFreePathChecker checker;

        const PathCheckResult result =
                checker.checkSegment(tree, Point3f {0.0F, 0.0F, 0.0F}, Point3f {1.0F, 0.0F, 0.0F});

        EXPECT_EQ(result.status, PathCheckStatus::Safe);
        EXPECT_FALSE(result.first_blocked_position.has_value());
    }

    TEST(KnownFreePathCheckerTest, TreatsUnknownAsBlocked)
    {
        octomap::OcTree tree(0.1);
        fillBox(
                tree, Point3f {-0.1F, -0.1F, -0.1F},
                Point3f {1.1F, 0.1F, 0.1F}, false);
        KnownFreePathChecker checker;

        const PathCheckResult result =
                checker.checkSegment(tree, Point3f {0.0F, 0.0F, 0.0F}, Point3f {1.0F, 0.0F, 0.0F});

        EXPECT_EQ(result.status, PathCheckStatus::UnknownBlocked);
        EXPECT_TRUE(result.first_blocked_position.has_value());
    }

    TEST(KnownFreePathCheckerTest, DetectsOccupiedVoxelAndConservativeWallClearance)
    {
        octomap::OcTree tree = makeFreeVolume(0.1);
        fillBox(
                tree, Point3f {0.4F, 0.5F, -0.1F},
                Point3f {0.6F, 0.6F, 0.1F}, true);
        KnownFreePathChecker checker;

        const PathCheckResult result =
                checker.checkSegment(tree, Point3f {0.0F, 0.0F, 0.0F}, Point3f {1.0F, 0.0F, 0.0F});

        EXPECT_EQ(result.status, PathCheckStatus::OccupiedBlocked);
    }

    TEST(KnownFreePathCheckerTest, PreservesPhysicalEnvelopeAcrossResolutions)
    {
        KnownFreePathChecker checker;
        octomap::OcTree      coarse = makeFreeVolume(0.2);
        octomap::OcTree      fine   = makeFreeVolume(0.05);

        EXPECT_TRUE(
                checker
                        .checkSegment(
                                coarse, Point3f {0.0F, 0.0F, 0.0F},
                                Point3f {1.0F, 0.0F, 0.0F})
                        .safe());
        EXPECT_TRUE(
                checker
                        .checkSegment(
                                fine, Point3f {0.0F, 0.0F, 0.0F},
                                Point3f {1.0F, 0.0F, 0.0F})
                        .safe());
    }

    TEST(KnownFreePathCheckerTest, RejectsInvalidConfigurationAndInput)
    {
        BodyEnvelopeConfig config;
        config.sample_spacing_fraction = 0.0F;
        EXPECT_THROW(KnownFreePathChecker {config}, std::invalid_argument);

        octomap::OcTree tree(0.1);
        KnownFreePathChecker checker;
        const float nan = std::numeric_limits<float>::quiet_NaN();
        EXPECT_EQ(
                checker.checkBody(tree, Point3f {nan, 0.0F, 0.0F}).status,
                PathCheckStatus::InvalidInput);
    }

}// namespace SwarmController
