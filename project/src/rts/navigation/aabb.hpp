#ifndef AABB_HPP
#define AABB_HPP

#include "rts/math/float2.hpp"

namespace NavigationSystemCode
{
    struct Aabb
    {
        float minX;
        float maxX;
        float minY;
        float maxY;

        bool IsInside(Float2 &point);
        bool IsInsideOrOnTheBoundary(Float2 &point);
        static bool AreBoundsOverlapping(Aabb &boundsA, Aabb &boundsB);
        string GetString();
    };
}

#endif
