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
        vector<int> nodeEdgeRefs;
        vector<int> nodeEdgeRefsInverted;
        vector<float> additionalCosts;
        vector<bool> additionalCostsModified;
        vector<int> addedAdditionalCosts;
        int nodesCount;
        bool useIterations;
        float costIncrement;
        bool triangleEdgesMode;

        void CreateNodes(NavMesh &navMesh);
        void CreateNodesCorners(NavMesh &navMesh);
        void CreateNodesEdges(NavMesh &navMesh);
        Path FindPath(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathWithOrWithoutIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathWithIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathWithoutIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        Path FindPathToExactTarget(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh);
        void ClearPathSearch();
        void ClearAdditionalCosts();
        void RetracePath(
            vector<Float2> &waypoints,
            vector<int> &waypointIndices,
            int startNode,
            int endNode);
        void SimplifyPathEdges(
            vector<Float2> &waypoints,
            vector<int> &waypointIndices,
            NavMesh &navMesh);
        void SimplifyPathEdgesInner(
            vector<Float2> &waypoints,
            vector<int> &leftPortalsEdgeIndices,
            vector<int> &rightPortalsEdgeIndices,
            NavMesh &navMesh,
            vector<Float2> &simplifiedWaypoints);
        void GetPortalPosition(
            int index,
            int cornerIndex,
            vector<Float2> &waypoints,
            NavMesh &navMesh,
            Float2 &position);
        float Orient2D(Float2 &a, Float2 &b, Float2 &c);
        void SimplifyPathCorners(
            vector<Float2> &waypoints,
            NavMesh &navMesh);
        bool CanWaypointsBeMerged(
            vector<Float2> &waypoints,
            int i,
            NavMesh &navMesh);
        vector<Float2> ReversePath(vector<Float2> &waypoints);
        int UpdatePositionNodeEdges(Float2 &position, NavMesh &navMesh, int nodeIndex);
        int UpdatePositionNodeCorners(Float2 &position, NavMesh &navMesh, int nodeIndex);
        float GetDistance(int nodeA, int nodeB);
    };
}

#endif
