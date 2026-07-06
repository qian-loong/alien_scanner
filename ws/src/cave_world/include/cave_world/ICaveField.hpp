#ifndef CAVE_WORLD_ICAVEFIELD_HPP
#define CAVE_WORLD_ICAVEFIELD_HPP

#include <vector>

namespace CaveWorld {
    struct Point3 {
        float x {};
        float y {};
        float z {};
    };

    class ICaveField
    {
    public:
        virtual ~ICaveField() = default;

        virtual bool isSolid(float x, float y, float z) const = 0;

        virtual bool raycast(const Point3 & origin, const Point3 & dir, float max_range, float & out_dist) const = 0;

        virtual std::vector<Point3> sampleSurface() const = 0;
    };
}// namespace CaveWorld

#endif// CAVE_WORLD_ICAVEFIELD_HPP
