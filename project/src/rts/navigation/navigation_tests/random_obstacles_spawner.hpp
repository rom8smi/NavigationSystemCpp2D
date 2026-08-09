#ifndef RANDOM_OBSTACLES_SPAWNER_HPP
#define RANDOM_OBSTACLES_SPAWNER_HPP

#include "rts/math/float2.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct RandomObstaclesSpawner
    {
        int number_to_spawn;
        Float2 min;
        Float2 max;
        Float2 min_size;
        Float2 max_size;
        int seed;
        bool randomize_rotation;
    };
}

#endif
