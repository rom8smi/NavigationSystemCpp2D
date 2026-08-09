#ifndef PATH_HPP
#define PATH_HPP

#include "rts/math/float2.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct Path
    {
        vector<Float2> waypoints;
        bool success;
        int lowestHCostNode;
    };
}

#endif
