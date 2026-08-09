#include "pathfinding.hpp"
#include "path_utils.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

namespace NavigationSystemCode
{
    void Pathfinding::CreateNodes(NavMesh &navMesh)
    {
        useIterations = true;
        costIncrement = 20.0f;
        openSet.Clear(nodes);
        nodes.clear();
        nodePositions.clear();
        nodeNeighbours.clear();
        additionalCosts.clear();
        additionalCostsModified.clear();

        for (int i = 0; i < navMesh.allPoints.size(); i++)
        {
            nodes.push_back(PathfindingNode{
                .gCost = 0,
                .hCost = 0,
                .parent = -1,
                .heapIndex = -1,
                .isInClosedSet = false});

            nodePositions.push_back(navMesh.allPoints[i]);
            nodeNeighbours.push_back(vector<int>());
            additionalCosts.push_back(0.0f);
            additionalCostsModified.push_back(false);
        }

        vector<DelaunatorEdge> edges = navMesh.allEdges;

        for (int i = 0; i < edges.size(); i++)
        {
            int p = edges[i].p;
            int q = edges[i].q;

            if (navMesh.edgesWalkability[edges[i].index])
            {
                nodeNeighbours[p].push_back(q);
                nodeNeighbours[q].push_back(p);
            }
        }

        nodesCount = nodes.size();

        for (int i = 0; i < 2; i++)
        {
            nodes.push_back(PathfindingNode());
            nodePositions.push_back(Float2(0.0f, 0.0f));
            nodeNeighbours.push_back(vector<int>());
            additionalCosts.push_back(0.0f);
            additionalCostsModified.push_back(false);
        }
    }

    Path Pathfinding::FindPath(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh)
    {
        Path path = FindPathWithOrWithoutIterations(startPos, targetPos, navMesh);

        if (!path.success && path.lowestHCostNode < nodesCount)
        {
            Float2 newTargetPos = nodePositions[path.lowestHCostNode];
            newTargetPos = navMesh.FindNearestObstacleHullEdgePointToTarget(path.lowestHCostNode, newTargetPos, targetPos);
            path = FindPathWithOrWithoutIterations(startPos, newTargetPos, navMesh);
        }

        return path;
    }

    Path Pathfinding::FindPathWithOrWithoutIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh)
    {
        if (useIterations)
        {
            return FindPathWithIterations(startPos, targetPos, navMesh);
        }
        return FindPathWithoutIterations(startPos, targetPos, navMesh);
    }

    Path Pathfinding::FindPathWithIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh)
    {
        for (int i = 0; i < 2; i++)
        {
            targetPos = navMesh.TryMoveToWalkableArea(targetPos).position;
        }

        if (nodesCount != navMesh.allPoints.size())
        {
            GodotUtils::print("Pathfinding nodes and triangulation points count does not match: " + to_string(nodes.size()) + " " + to_string(navMesh.allPoints.size()));
        }

        vector<Path> paths;
        for (int i = 0; i < 2; i++)
        {
            Path path = FindPathToExactTarget(startPos, targetPos, navMesh);

            if (!path.success || path.waypoints.size() < 2)
            {
                ClearPathSearch();
                ClearAdditionalCosts();

                return path;
            }

            for (int j = 0; j < closedSet.size(); j++)
            {
                int nodeIndex = closedSet[j];
                additionalCosts[nodeIndex] += costIncrement;

                if (!additionalCostsModified[nodeIndex])
                {
                    addedAdditionalCosts.push_back(nodeIndex);
                    additionalCostsModified[nodeIndex] = true;
                }
            }

            ClearPathSearch();
            paths.push_back(path);
        }

        ClearAdditionalCosts();

        float largestLength = MathUtils::FLOAT_MAX_VALUE;
        Path shortestPath;

        for (int i = 0; i < paths.size(); i++)
        {
            float currentLenth = PathUtils::CalculateTotalPathLength(paths[i].waypoints);
            if (currentLenth < largestLength)
            {
                largestLength = currentLenth;
                shortestPath = paths[i];
            }
        }

        return shortestPath;
    }

    Path Pathfinding::FindPathWithoutIterations(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh)
    {
        for (int i = 0; i < 2; i++)
        {
            targetPos = navMesh.TryMoveToWalkableArea(targetPos).position;
        }

        if (nodesCount != navMesh.allPoints.size())
        {
            GodotUtils::print("Pathfinding nodes and triangulation points count does not match: " + to_string(nodes.size()) + " " + to_string(navMesh.allPoints.size()));
        }
        Path path = FindPathToExactTarget(startPos, targetPos, navMesh);
        ClearPathSearch();
        return path;
    }

    Path Pathfinding::FindPathToExactTarget(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh)
    {
        int startNode = nodesCount;
        int targetNode = nodesCount + 1;

        int startTriangle = UpdatePositionNode(startPos, navMesh, startNode);
        int targetTriangle = UpdatePositionNode(targetPos, navMesh, targetNode);

        bool pathSuccess = false;
        float lowestHCost = MathUtils::FLOAT_MAX_VALUE;
        PathfindingNode node;

        int lowestHCostNode = startNode;

        if (startTriangle != -1 && targetTriangle != -1)
        {
            if (startTriangle == targetTriangle)
            {
                openSet.Clear(nodes);
                closedSet.clear();

                return Path{
                    .waypoints = vector<Float2>{targetPos},
                    .success = true,
                    .lowestHCostNode = lowestHCostNode};
            }

            openSet.Add(startNode, nodes);

            while (openSet.count > 0)
            {
                int currentNode = openSet.RemoveFirst(nodes);

                node = nodes[currentNode];
                node.isInClosedSet = true;
                nodes[currentNode] = node;

                closedSet.push_back(currentNode);

                if (currentNode == targetNode)
                {
                    pathSuccess = true;
                    break;
                }

                vector<int> neighbours = nodeNeighbours[currentNode];
                int neighboursCount = neighbours.size();

                for (int i = 0; i < neighboursCount; i++)
                {
                    int neighbour = neighbours[i];

                    if (nodes[neighbour].isInClosedSet)
                    {
                        continue;
                    }

                    float newMovementCostToNeighbour = nodes[currentNode].gCost + GetDistance(currentNode, neighbour);
                    if (newMovementCostToNeighbour + additionalCosts[currentNode] < nodes[neighbour].gCost || nodes[neighbour].heapIndex == -1)
                    {
                        node = nodes[neighbour];
                        node.gCost = newMovementCostToNeighbour;
                        nodes[neighbour] = node;

                        float hCost = GetDistance(neighbour, targetNode);
                        if (hCost < lowestHCost)
                        {
                            lowestHCostNode = neighbour;
                            lowestHCost = hCost;
                        }

                        node = nodes[neighbour];
                        node.hCost = hCost;
                        node.parent = currentNode;
                        nodes[neighbour] = node;

                        if (nodes[neighbour].heapIndex == -1)
                        {
                            openSet.Add(neighbour, nodes);
                        }
                        else
                        {
                            openSet.UpdateItem(neighbour, nodes);
                        }
                    }
                }
            }
        }

        vector<Float2> waypoints;

        if (pathSuccess)
        {
            RetracePath(waypoints, startNode, targetNode, startPos, targetPos);
            SimplifyPath(waypoints, navMesh);

            waypoints.erase(waypoints.begin() + waypoints.size() - 1);
            waypoints = ReversePath(waypoints);
        }

        return Path{
            .waypoints = waypoints,
            .success = pathSuccess,
            .lowestHCostNode = lowestHCostNode};
    }

    void Pathfinding::ClearPathSearch()
    {
        PathfindingNode node;
        for (int i = 0; i < closedSet.size(); i++)
        {
            int nodeIndex = closedSet[i];

            node = nodes[nodeIndex];
            node.gCost = 0;
            node.hCost = 0;
            node.parent = -1;
            node.heapIndex = -1;
            node.isInClosedSet = false;
            nodes[nodeIndex] = node;
        }

        openSet.Clear(nodes);
        closedSet.clear();

        for (int i = nodesCount; i < nodesCount + 2; i++)
        {
            vector<int> currentNodeNeighbours = nodeNeighbours[i];

            for (int j = 0; j < currentNodeNeighbours.size(); j++)
            {
                int p = currentNodeNeighbours[j];
                int lastNeighbourNode = nodeNeighbours[p].size() - 1;
                nodeNeighbours[p].erase(nodeNeighbours[p].begin() + lastNeighbourNode);
            }
        }
    }

    void Pathfinding::ClearAdditionalCosts()
    {
        for (int i = 0; i < addedAdditionalCosts.size(); i++)
        {
            int nodeIndex = addedAdditionalCosts[i];
            additionalCosts[nodeIndex] = 0.0f;
            additionalCostsModified[nodeIndex] = false;
        }

        addedAdditionalCosts.clear();
    }

    void Pathfinding::RetracePath(vector<Float2> &waypoints, int startNode, int endNode, Float2 &startPos, Float2 &targetPos)
    {
        int currentNode = endNode;
        Float2 waypointPosition;

        while (currentNode != startNode)
        {
            waypointPosition = nodePositions[currentNode];
            waypoints.push_back(waypointPosition);
            currentNode = nodes[currentNode].parent;
        }

        waypointPosition = nodePositions[startNode];
        waypoints.push_back(waypointPosition);
    }

    void Pathfinding::SimplifyPath(
        vector<Float2> &waypoints,
        NavMesh &navMesh)
    {
        vector<bool> mergeConsidered;
        vector<float> straightLineDistancesSqr;

        mergeConsidered.resize(waypoints.size() - 2);
        straightLineDistancesSqr.resize(waypoints.size() - 2);

        for (int i = 0; i < mergeConsidered.size(); i++)
        {
            mergeConsidered[i] = false;
            straightLineDistancesSqr[i] = (waypoints[i] - waypoints[i + 2]).length_squared();
        }

        bool mergeFound = true;

        while (mergeFound)
        {
            mergeFound = false;
            float largestDistanceSqr = 0.0f;
            int largestDistanceSqrIndex = -1;

            for (int i = 0; i < straightLineDistancesSqr.size(); i++)
            {
                if (!mergeConsidered[i])
                {
                    if (straightLineDistancesSqr[i] > largestDistanceSqr)
                    {
                        largestDistanceSqr = straightLineDistancesSqr[i];
                        largestDistanceSqrIndex = i;
                        mergeFound = true;
                    }
                }
            }

            if (mergeFound)
            {
                mergeConsidered[largestDistanceSqrIndex] = true;
                if (CanWaypointsBeMerged(waypoints, largestDistanceSqrIndex, navMesh))
                {
                    int removalIndex = largestDistanceSqrIndex + 1;

                    waypoints.erase(waypoints.begin() + removalIndex);

                    mergeConsidered.erase(mergeConsidered.begin() + largestDistanceSqrIndex);
                    straightLineDistancesSqr.erase(straightLineDistancesSqr.begin() + largestDistanceSqrIndex);

                    int waypointsSize = waypoints.size();
                    if (removalIndex - 1 >= 0 && removalIndex + 1 < waypointsSize)
                    {
                        straightLineDistancesSqr[largestDistanceSqrIndex] = (waypoints[removalIndex - 1] - waypoints[removalIndex + 1]).length_squared();
                        mergeConsidered[largestDistanceSqrIndex] = false;
                    }
                    if (removalIndex - 2 >= 0 && removalIndex < waypointsSize)
                    {
                        straightLineDistancesSqr[largestDistanceSqrIndex - 1] = (waypoints[removalIndex - 2] - waypoints[removalIndex]).length_squared();
                        mergeConsidered[largestDistanceSqrIndex - 1] = false;
                    }
                }
            }
        }
    }

    bool Pathfinding::CanWaypointsBeMerged(
        vector<Float2> &waypoints,
        int i,
        NavMesh &navMesh)
    {
        if (i + 2 >= waypoints.size())
        {
            return false;
        }

        Float2 p1 = waypoints[i];
        Float2 p3 = waypoints[i + 2];

        if (navMesh.CanPointsBeReachedInStraightLine(p1, p3))
        {
            return true;
        }

        return false;
    }

    vector<Float2> Pathfinding::ReversePath(vector<Float2> &waypoints)
    {
        vector<Float2> reversedWaypoints;
        int waypointsCount = waypoints.size();

        for (int i = waypointsCount - 1; i >= 0; i--)
        {
            reversedWaypoints.push_back(waypoints[i]);
        }

        return reversedWaypoints;
    }

    int Pathfinding::UpdatePositionNode(Float2 &position, NavMesh &navMesh, int nodeIndex)
    {
        int triangle = navMesh.FindWalkableTriangleForPoint(position);

        if (triangle != -1 && navMesh.trianglesWalkability[triangle] != -1)
        {
            triangle = -1;
        }

        PathfindingNode node = PathfindingNode{
            .gCost = 0,
            .hCost = 0,
            .parent = -1,
            .heapIndex = -1,
            .isInClosedSet = false};
        vector<int> neighbours;

        if (triangle != -1)
        {
            for (int i = 0; i < navMesh.allTriangles[triangle].points.size(); i++)
            {
                int p = navMesh.allTriangles[triangle].points[i];

                neighbours.push_back(p);
                nodeNeighbours[p].push_back(nodeIndex);
            }
        }

        nodes[nodeIndex] = node;
        nodePositions[nodeIndex] = position;
        nodeNeighbours[nodeIndex] = neighbours;
        return triangle;
    }

    float Pathfinding::GetDistance(int nodeA, int nodeB)
    {
        Float2 centerA = nodePositions[nodeA];
        Float2 centerB = nodePositions[nodeB];

        return (centerA - centerB).length();
    }
}
