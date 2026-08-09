#ifndef MANUAL_AGENT_CIRCULAR_SPAWNER_HPP
#define MANUAL_AGENT_CIRCULAR_SPAWNER_HPP

#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ManualAgentCircularSpawner
    {
        Rgba color;
        Float2 center;
        float radius;
        int number_to_spawn;
        bool set_destination_opposite_in_circle;
        int agent_type;
        int seed;
    };
}

#endif
