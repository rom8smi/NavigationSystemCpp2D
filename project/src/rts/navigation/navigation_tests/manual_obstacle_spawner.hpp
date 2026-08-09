#ifndef MANUAL_OBSTACLE_SPAWNER_HPP
#define MANUAL_OBSTACLE_SPAWNER_HPP

#include "rts/math/float2.hpp"
#include "rts/math/float3.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ManualObstacleSpawner
    {
        Float2 center;
        Float3 size;
        bool is_visible;
        bool is_walkable;
    };
}

#endif
