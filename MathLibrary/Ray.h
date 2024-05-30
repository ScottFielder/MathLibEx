#ifndef RAY_H
#define RAY_H
#include <Vector.h>
namespace MATHEX{
    using namespace MATH;
    struct Ray {
    public:
        Vec3 start;
        Vec3 direction;
        Ray(const Vec3 &start_, const Vec3 &direction_): 
            start(start_), direction(direction_){}

        const Vec3 getPos(const float t) const {
            return start + (t * (direction - start));
        }
    };
}
#endif

