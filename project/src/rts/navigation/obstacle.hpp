#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include "rts/math/float2.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct Obstacle
    {
        int pointsIndexStart;
        int pointsCount;
        vector<Float2> obstacleCorners;
        vector<bool> isCornerIntersectingWithWorldBounds;
        vector<int> nSplits;
        Float2 center;
        float largest_corner_distance;
        bool isWalkable;
    };
}

#endif
