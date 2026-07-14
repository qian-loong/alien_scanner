#include "cave_world/TruthCollisionAuditor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace CaveWorld {

    namespace {

        class SphericalVoid final : public ICaveField
        {
        public:
            explicit SphericalVoid(float radius) : radius_(radius) {}

            bool isSolid(float x, float y, float z) const override
            {
                return x * x + y * y + z * z >= radius_ * radius_;
            }

            bool raycast(
                    const Point3 & origin, const Point3 & dir, float max_range,
                    float & out_dist) const override
            {
                const float projection = origin.x * dir.x + origin.y * dir.y
                                         + origin.z * dir.z;
                const float c = origin.x * origin.x + origin.y * origin.y
                                + origin.z * origin.z - radius_ * radius_;
                const float discriminant = projection * projection - c;
                if(discriminant < 0.0F) {
                    return false;
                }
                out_dist = -projection + std::sqrt(discriminant);
                return out_dist >= 0.0F && out_dist <= max_range;
            }

            std::vector<Point3> sampleSurface() const override { return {}; }

        private:
            float radius_;
        };

    }// namespace

    TEST(TruthCollisionAuditorTest, ReportsClearBodyAndApproximateClearance)
    {
        SphericalVoid field(2.0F);
        TruthCollisionAuditConfig config;
        config.robot_radius       = 0.25F;
        config.robot_half_height  = 0.15F;
        config.sample_spacing     = 0.05F;
        TruthCollisionAuditor auditor(field, config);

        const auto result = auditor.assess(Point3 {0.0F, 0.0F, 0.0F});

        EXPECT_FALSE(result.collided);
        EXPECT_FALSE(result.first_solid_sample.has_value());
        EXPECT_NEAR(result.minimum_clearance, 1.717F, 0.01F);
    }

    TEST(TruthCollisionAuditorTest, DetectsEnvelopeIntersection)
    {
        SphericalVoid field(2.0F);
        TruthCollisionAuditor auditor(field);

        const auto result = auditor.assess(Point3 {1.85F, 0.0F, 0.0F});

        EXPECT_TRUE(result.collided);
        EXPECT_GT(result.solid_sample_count, 0U);
        EXPECT_TRUE(result.first_solid_sample.has_value());
        EXPECT_LT(result.minimum_clearance, 0.0F);
    }

    TEST(TruthCollisionAuditorTest, RejectsInvalidConfiguration)
    {
        SphericalVoid field(2.0F);
        TruthCollisionAuditConfig config;
        config.sample_spacing = 0.0F;
        EXPECT_THROW(TruthCollisionAuditor(field, config), std::invalid_argument);
    }

}// namespace CaveWorld
