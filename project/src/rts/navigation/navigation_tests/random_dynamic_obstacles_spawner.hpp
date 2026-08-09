#ifndef RANDOM_DYNAMIC_OBSTACLES_SPAWNER_HPP
#define RANDOM_DYNAMIC_OBSTACLES_SPAWNER_HPP

#include "rts/math/float2.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct RandomDynamicObstaclesSpawner
    {
        Float2 min;
        Float2 max;
        Float2 min_size;
        Float2 max_size;
        int seed;
        bool randomize_rotation;
        float creation_time_interval;
        float remaining_time_min;
        float remaining_time_max;
        float creation_time_passed;
    };
}

#endif
