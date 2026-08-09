#ifndef CHAINED_OBSTACLE_SPAWNER_HPP
#define CHAINED_OBSTACLE_SPAWNER_HPP

#include "rts/math/float2.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ChainedObstacleSpawner
    {
        Float2 start;
        int number_to_spawn;
        float obstacle_size;
        Float2 chain_direction;
        bool is_visible;
    };
}

#endif
