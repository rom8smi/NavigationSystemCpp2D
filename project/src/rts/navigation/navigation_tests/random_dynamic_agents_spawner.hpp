#ifndef RANDOM_DYNAMIC_AGENTS_SPAWNER_HPP
#define RANDOM_DYNAMIC_AGENTS_SPAWNER_HPP

#include "manual_agent_rectangular_spawner.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct RandomDynamicAgentsSpawner
    {
        vector<ManualAgentRectangularSpawner> manual_agent_rectangular_spawners;
    };
}

#endif
