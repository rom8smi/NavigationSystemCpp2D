#ifndef PATHFINDING_HPP
#define PATHFINDING_HPP

#include "rts/math/float2.hpp"
#include <vector>
#include "heap.hpp"
#include "nav_mesh.hpp"
#include "path.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct Pathfinding
    {
        Heap openSet;
        vector<int> closedSet;
        vector<PathfindingNode> nodes;
        vector<Float2> nodePositions;
        vector<vector<int>> nodeNeighbours;
        vector<float> additionalCosts;
        vector<bool> additionalCostsModified;
        vector<int> addedAdditionalCosts;
        int nodesCount;
        bool useIterations;
        float costIncrement;

        void CreateNodes(NavMesh &navMesh);
        Path FindPath(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathWithOrWithoutIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathWithIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathWithoutIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathToExactTarget(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        void ClearPathSearch();
        void ClearAdditionalCosts();
        void RetracePath(vector<Float2> &waypoints, int startNode, int endNode, Float2 &startPos, Float2 &targetPos);
        void SimplifyPath(
            vector<Float2> &waypoints,
            NavMesh &navMesh);
        bool CanWaypointsBeMerged(
            vector<Float2> &waypoints,
            int i,
            NavMesh &navMesh);
        vector<Float2> ReversePath(vector<Float2> &waypoints);
        int UpdatePositionNode(Float2 &position, NavMesh &navMesh, int nodeIndex);
        float GetDistance(int nodeA, int nodeB);
    };
}

#endif
