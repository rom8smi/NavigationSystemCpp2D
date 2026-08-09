#ifndef LINE_SEGMENT_INTERSECTION_RESULT_HPP
#define LINE_SEGMENT_INTERSECTION_RESULT_HPP

#include "float2.hpp"

namespace NavigationSystemCode
{
    struct LineSegmentsIntersectionResult
    {
        bool intersects;
        Float2 intersection;
    };
}

#endif