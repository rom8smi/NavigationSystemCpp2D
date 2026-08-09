#include "aabb.hpp"

namespace NavigationSystemCode
{
    bool Aabb::IsInside(Float2 &point)
    {
        return point.x > minX && point.x < maxX && point.y > minY && point.y < maxY;
    }

    bool Aabb::IsInsideOrOnTheBoundary(Float2 &point)
    {
        return point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY;
    }

    bool Aabb::AreBoundsOverlapping(Aabb &boundsA, Aabb &boundsB)
    {
        Float2 minXminY = Float2(boundsB.minX, boundsB.minY);
        Float2 maxXminY = Float2(boundsB.maxX, boundsB.minY);
        Float2 minXmaxY = Float2(boundsB.minX, boundsB.maxY);
        Float2 maxXmaxY = Float2(boundsB.maxX, boundsB.maxY);

        if (
            boundsA.IsInsideOrOnTheBoundary(minXminY) ||
            boundsA.IsInsideOrOnTheBoundary(maxXminY) ||
            boundsA.IsInsideOrOnTheBoundary(minXmaxY) ||
            boundsA.IsInsideOrOnTheBoundary(maxXmaxY))
        {
            return true;
        }
        return false;
    }

    string Aabb::GetString()
    {
        return "(x: " + to_string(minX) + " " + to_string(maxX) + ", y: " + to_string(minY) + " " + to_string(maxY) + ")";
    }
}
