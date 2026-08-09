#ifndef PATHFINDING_NODE_HPP
#define PATHFINDING_NODE_HPP

namespace NavigationSystemCode
{
    struct PathfindingNode
    {
        float gCost;
        float hCost;
        int parent;
        int heapIndex;
        bool isInClosedSet;
    };
}

#endif
