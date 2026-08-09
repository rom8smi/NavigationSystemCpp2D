#ifndef AGENT_HPP
#define AGENT_HPP

#include "rts/math/float2.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct Agent
    {
        int unitIndex;
        int agentTypeIndex;
        float speed;
        Float2 destination;
        vector<Float2> waypoints;
        int currentWaypointIndex;
        bool followingPath;
        bool searchPathLater;
        bool destinationUpdateNeeded;
        Float2 pathVelocity;
        Float2 localAvoidanceVelocity;
        float powerFactorSum;
        Float2 finalVelocity;
        float remainingPathDistance;
        int pathMovementFailuresCount;
        float density;
    };
}

#endif
