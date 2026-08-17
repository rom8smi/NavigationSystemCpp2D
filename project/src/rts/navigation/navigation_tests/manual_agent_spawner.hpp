#ifndef MANUAL_AGENT_SPAWNER_HPP
#define MANUAL_AGENT_SPAWNER_HPP

#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ManualAgentSpawner
    {
        Rgba color;
        Float2 position;
        Float2 destination;
        int agent_type;
    };
}

#endif
