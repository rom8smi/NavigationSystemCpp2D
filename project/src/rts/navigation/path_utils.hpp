#ifndef PATH_UTILS_HPP
#define PATH_UTILS_HPP

#include "rts/math/float2.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct PathUtils
    {
        static float CalculateTotalPathLength(vector<Float2> &waypoints);
        static float CalculatePathLength(vector<Float2> &waypoints, int startIndex);
    };
}

#endif
