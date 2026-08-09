#ifndef MANUAL_AGENT_RECTANGULAR_SPAWNER_HPP
#define MANUAL_AGENT_RECTANGULAR_SPAWNER_HPP

#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ManualAgentRectangularSpawner
    {
        Rgba color;
        Float2 center;
        Float2 size;
        int number_to_spawn;
        Float2 destination;
        int agent_type;
        int seed;
    };
}

#endif
