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
        triangleEdgesMode = true;

        if (triangleEdgesMode)
        {
            CreateNodesEdges(navMesh);
        }
        else
        {
            CreateNodesCorners(navMesh);
        }
    }

    void Pathfinding::CreateNodesCorners(NavMesh &navMesh)
    {
        costIncrement = 20.0f;
        openSet.Clear(nodes);
        nodes.clear();
        nodePositions.clear();
        nodeNeighbours.clear();
        additionalCosts.clear();
        additionalCostsModified.clear();
        nodeEdgeRefsInverted.clear();
        nodeEdgeRefs.clear();

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

    void Pathfinding::CreateNodesEdges(NavMesh &navMesh)
    {
        useIterations = false;
        costIncrement = 1.0f;
        openSet.Clear(nodes);
        nodes.clear();
        nodePositions.clear();
        nodeNeighbours.clear();
        additionalCosts.clear();
        additionalCostsModified.clear();
        nodeEdgeRefsInverted.clear();

        nodeEdgeRefs.resize(navMesh.delaunator.trianglesLen);

        for (int i = 0; i < navMesh.delaunator.trianglesLen; i++)
        {
            nodeEdgeRefs[i] = -1;
        }

        vector<int> walkableIndices;
        vector<bool> repetitive;
        vector<int> nodeIndicesInTriangle;

        int trianglesCount = navMesh.delaunator.trianglesLen / 3;

        for (int t = 0; t < trianglesCount; t++)
        {
            walkableIndices.clear();
            repetitive.clear();

            for (int i = 0; i < 3; i++)
            {
                int e = t * 3 + i;
                int opposite = navMesh.delaunator.halfedges[e];

                if (opposite != -1)
                {
                    if (navMesh.trianglesWalkability[Delaunator::TriangleOfEdge(e)] == -1 &&
                        navMesh.trianglesWalkability[Delaunator::TriangleOfEdge(opposite)] == -1)
                    {
                        walkableIndices.push_back(i);
                        if (e > opposite)
                        {
                            repetitive.push_back(true);
                        }
                        else
                        {
                            repetitive.push_back(false);
                        }
                    }
                }
            }

            int walkableIndicesCount = walkableIndices.size();
            nodeIndicesInTriangle.resize(walkableIndicesCount);

            for (int i = 0; i < walkableIndicesCount; i++)
            {
                if (repetitive[i])
                {
                    int e = t * 3 + walkableIndices[i];
                    int opposite = navMesh.delaunator.halfedges[e];

                    nodeIndicesInTriangle[i] = nodeEdgeRefs[opposite];
                }
                else
                {
                    int e = t * 3 + walkableIndices[i];
                    int opposite = navMesh.delaunator.halfedges[e];

                    int p = navMesh.delaunator.triangles[e];
                    int q = navMesh.delaunator.triangles[Delaunator::NextHalfedge(e)];

                    Float2 center = (navMesh.allPoints[p] + navMesh.allPoints[q]) * 0.5f;

                    int currentNodesCount = nodes.size();
                    nodeIndicesInTriangle[i] = currentNodesCount;

                    nodes.push_back(PathfindingNode{
                        .gCost = 0,
                        .hCost = 0,
                        .parent = -1,
                        .heapIndex = -1,
                        .isInClosedSet = false});
                    nodePositions.push_back(center);
                    nodeNeighbours.push_back(vector<int>());
                    additionalCosts.push_back(0.0f);
                    additionalCostsModified.push_back(false);
                    nodeEdgeRefsInverted.push_back(e);
                    nodeEdgeRefs[e] = currentNodesCount;
                    nodeEdgeRefs[opposite] = currentNodesCount;
                }
            }

            for (int i = 0; i < walkableIndicesCount; i++)
            {
                for (int j = i + 1; j < walkableIndicesCount; j++)
                {
                    int nodeA = nodeIndicesInTriangle[i];
                    int nodeB = nodeIndicesInTriangle[j];
                    nodeNeighbours[nodeA].push_back(nodeB);
                    nodeNeighbours[nodeB].push_back(nodeA);
                }
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
            nodeEdgeRefsInverted.push_back(-1);
        }
    }

    Path Pathfinding::FindPath(Float2 &startPos, Float2 &targetPos, NavMesh &navMesh)
    {
        Path path = FindPathWithOrWithoutIterations(startPos, targetPos, navMesh);

        if (!path.success && path.lowestHCostNode < nodesCount)
        {
            Float2 newTargetPos = nodePositions[path.lowestHCostNode];

            if (triangleEdgesMode)
            {
                int e = nodeEdgeRefsInverted[path.lowestHCostNode];
                if (e != -1)
                {
                    int p = navMesh.delaunator.triangles[e];
                    int q = navMesh.delaunator.triangles[Delaunator::NextHalfedge(e)];

                    Float2 newTargetPosP = navMesh.FindNearestObstacleHullEdgePointToTarget(p, newTargetPos, targetPos);
                    Float2 newTargetPosQ = navMesh.FindNearestObstacleHullEdgePointToTarget(q, newTargetPos, targetPos);

                    float distanceSqrP = (newTargetPosP - targetPos).length_squared();
                    float distanceSqrQ = (newTargetPosQ - targetPos).length_squared();

                    if (distanceSqrP < distanceSqrQ)
                    {
                        newTargetPos = newTargetPosP;
                    }
                    else
                    {
                        newTargetPos = newTargetPosQ;
                    }
                }
            }

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

        if (!triangleEdgesMode && nodesCount != navMesh.allPoints.size())
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

        if (!triangleEdgesMode && nodesCount != navMesh.allPoints.size())
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

        int startTriangle;
        int targetTriangle;

        if (triangleEdgesMode)
        {
            startTriangle = UpdatePositionNodeEdges(startPos, navMesh, startNode);
            targetTriangle = UpdatePositionNodeEdges(targetPos, navMesh, targetNode);
        }
        else
        {
            startTriangle = UpdatePositionNodeCorners(startPos, navMesh, startNode);
            targetTriangle = UpdatePositionNodeCorners(targetPos, navMesh, targetNode);
        }

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
            vector<int> waypointIndices;
            RetracePath(waypoints, waypointIndices, startNode, targetNode);

            if (triangleEdgesMode)
            {
                SimplifyPathEdges(waypoints, waypointIndices, navMesh);
            }
            else
            {
                SimplifyPathCorners(waypoints, navMesh);
            }

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

    void Pathfinding::RetracePath(
        vector<Float2> &waypoints,
        vector<int> &waypointIndices,
        int startNode,
        int endNode)
    {
        int currentNode = endNode;
        Float2 waypointPosition;

        while (currentNode != startNode)
        {
            waypointPosition = nodePositions[currentNode];
            waypoints.push_back(waypointPosition);
            waypointIndices.push_back(currentNode);
            currentNode = nodes[currentNode].parent;
        }

        waypointPosition = nodePositions[startNode];
        waypoints.push_back(waypointPosition);
        waypointIndices.push_back(startNode);
    }

    void Pathfinding::SimplifyPathEdges(
        vector<Float2> &waypoints,
        vector<int> &waypointIndices,
        NavMesh &navMesh)
    {
        int waypointIndicesCount = waypointIndices.size();
        if (waypointIndicesCount < 3)
        {
            return;
        }

        vector<int> leftPortalsEdgeIndices;
        vector<int> rightPortalsEdgeIndices;

        leftPortalsEdgeIndices.resize(waypointIndicesCount);
        rightPortalsEdgeIndices.resize(waypointIndicesCount);

        leftPortalsEdgeIndices[0] = -1;
        rightPortalsEdgeIndices[0] = -1;

        leftPortalsEdgeIndices[waypointIndicesCount - 1] = -1;
        rightPortalsEdgeIndices[waypointIndicesCount - 1] = -1;

        for (int i = 0; i < waypointIndices.size() - 2; i++)
        {
            int previousWaypointIndex = waypointIndices[i];
            int currentWaypointIndex = waypointIndices[i + 1];
            int nextWaypointIndex = waypointIndices[i + 2];

            int edgeIndex = nodeEdgeRefsInverted[currentWaypointIndex];

            Float2 pathDirectionA = (nodePositions[currentWaypointIndex] - nodePositions[previousWaypointIndex]).normalized();
            Float2 pathDirectionB = (nodePositions[nextWaypointIndex] - nodePositions[currentWaypointIndex]).normalized();

            Float2 pathDirection = (pathDirectionA + pathDirectionB) * 0.5f;

            int p = navMesh.delaunator.triangles[edgeIndex];
            int q = navMesh.delaunator.triangles[Delaunator::NextHalfedge(edgeIndex)];

            Float2 perpendicularDirection = navMesh.allPoints[p] - nodePositions[currentWaypointIndex];

            if (pathDirection.cross(perpendicularDirection) < 0.0f)
            {
                leftPortalsEdgeIndices[i + 1] = p;
                rightPortalsEdgeIndices[i + 1] = q;
            }
            else
            {
                leftPortalsEdgeIndices[i + 1] = q;
                rightPortalsEdgeIndices[i + 1] = p;
            }
        }

        vector<Float2> simplifiedWaypoints;

        SimplifyPathEdgesInner(
            waypoints,
            leftPortalsEdgeIndices,
            rightPortalsEdgeIndices,
            navMesh,
            simplifiedWaypoints);

        waypoints.clear();
        for (int i = 0; i < simplifiedWaypoints.size(); i++)
        {
            waypoints.push_back(simplifiedWaypoints[i]);
        }
    }

    void Pathfinding::SimplifyPathEdgesInner(
        vector<Float2> &waypoints,
        vector<int> &leftPortalsEdgeIndices,
        vector<int> &rightPortalsEdgeIndices,
        NavMesh &navMesh,
        vector<Float2> &simplifiedWaypoints)
    {
        int totalPoints = waypoints.size();
        Float2 apexPosition = waypoints[0];

        simplifiedWaypoints.push_back(apexPosition);

        int apexIndex = 0;
        int leftIndex = 0;
        int rightIndex = 0;

        int apexCornerIndex = -1;
        int lastAddedCornerIndex = -1;

        for (int i = 1; i < totalPoints; i++)
        {
            int currentLeftCornerIndex = leftPortalsEdgeIndices[i];
            int currentRightCornerIndex = rightPortalsEdgeIndices[i];

            int activeLeftCornerIndex = leftPortalsEdgeIndices[leftIndex];
            int activeRightCornerIndex = rightPortalsEdgeIndices[rightIndex];

            Float2 currentRightPosition;
            GetPortalPosition(i, currentRightCornerIndex, waypoints, navMesh, currentRightPosition);

            Float2 activeRightPosition;
            GetPortalPosition(rightIndex, activeRightCornerIndex, waypoints, navMesh, activeRightPosition);

            bool isRightTightening = (rightIndex == apexIndex) ||
                                     (Orient2D(apexPosition, activeRightPosition, currentRightPosition) <= 0.0f);

            if (isRightTightening)
            {
                bool sameAsLeft = (currentRightCornerIndex != -1 && currentRightCornerIndex == activeLeftCornerIndex);
                bool sameAsApex = (currentRightCornerIndex != -1 && apexCornerIndex != -1 && currentRightCornerIndex == apexCornerIndex);

                Float2 activeLeftPosition;
                GetPortalPosition(leftIndex, activeLeftCornerIndex, waypoints, navMesh, activeLeftPosition);

                if (rightIndex == apexIndex || sameAsLeft || sameAsApex ||
                    Orient2D(apexPosition, activeLeftPosition, currentRightPosition) >= 0.0f)
                {
                    rightIndex = i;
                }
                else
                {
                    int leftCornerIndex = activeLeftCornerIndex;

                    if (leftCornerIndex < 0 || leftCornerIndex != lastAddedCornerIndex)
                    {
                        simplifiedWaypoints.push_back(activeLeftPosition);
                        lastAddedCornerIndex = leftCornerIndex;
                    }

                    apexPosition = activeLeftPosition;
                    apexIndex = leftIndex;
                    apexCornerIndex = leftCornerIndex;

                    leftIndex = apexIndex;
                    rightIndex = apexIndex;

                    i = apexIndex;
                    continue;
                }
            }

            Float2 currentLeftPosition;
            GetPortalPosition(i, currentLeftCornerIndex, waypoints, navMesh, currentLeftPosition);

            Float2 activeLeftPositionCurrent;
            GetPortalPosition(leftIndex, activeLeftCornerIndex, waypoints, navMesh, activeLeftPositionCurrent);

            bool isLeftTightening = (leftIndex == apexIndex) ||
                                    (Orient2D(apexPosition, activeLeftPositionCurrent, currentLeftPosition) >= 0.0f);

            if (isLeftTightening)
            {
                bool sameAsRight = (currentLeftCornerIndex != -1 && currentLeftCornerIndex == activeRightCornerIndex);
                bool sameAsApex = (currentLeftCornerIndex != -1 && apexCornerIndex != -1 && currentLeftCornerIndex == apexCornerIndex);

                Float2 activeRightPositionCurrent;
                GetPortalPosition(rightIndex, activeRightCornerIndex, waypoints, navMesh, activeRightPositionCurrent);

                if (leftIndex == apexIndex || sameAsRight || sameAsApex ||
                    Orient2D(apexPosition, activeRightPositionCurrent, currentLeftPosition) <= 0.0f)
                {
                    leftIndex = i;
                }
                else
                {
                    int rightCornerIndex = activeRightCornerIndex;

                    if (rightCornerIndex < 0 || rightCornerIndex != lastAddedCornerIndex)
                    {
                        simplifiedWaypoints.push_back(activeRightPositionCurrent);
                        lastAddedCornerIndex = rightCornerIndex;
                    }

                    apexPosition = activeRightPositionCurrent;
                    apexIndex = rightIndex;
                    apexCornerIndex = rightCornerIndex;

                    leftIndex = apexIndex;
                    rightIndex = apexIndex;

                    i = apexIndex;
                    continue;
                }
            }
        }

        if (apexIndex < totalPoints - 1)
        {
            simplifiedWaypoints.push_back(waypoints[totalPoints - 1]);
        }
    }

    void Pathfinding::GetPortalPosition(
        int index,
        int cornerIndex,
        vector<Float2> &waypoints,
        NavMesh &navMesh,
        Float2 &position)
    {
        position = cornerIndex >= 0 ? navMesh.allPoints[cornerIndex] : waypoints[index];
    }

    float Pathfinding::Orient2D(Float2 &a, Float2 &b, Float2 &c)
    {
        Float2 ba = b - a;
        Float2 ca = c - a;
        return ba.cross(ca);
    }

    void Pathfinding::SimplifyPathCorners(
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

    int Pathfinding::UpdatePositionNodeEdges(Float2 &position, NavMesh &navMesh, int nodeIndex)
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
            for (int i = 0; i < 3; i++)
            {
                int e = 3 * triangle + i;
                int opposite = navMesh.delaunator.halfedges[e];

                if (opposite != -1)
                {
                    if (navMesh.trianglesWalkability[Delaunator::TriangleOfEdge(e)] == -1 &&
                        navMesh.trianglesWalkability[Delaunator::TriangleOfEdge(opposite)] == -1)
                    {
                        neighbours.push_back(nodeEdgeRefs[e]);
                        nodeNeighbours[nodeEdgeRefs[e]].push_back(nodeIndex);
                    }
                }
            }
        }

        nodes[nodeIndex] = node;
        nodePositions[nodeIndex] = position;
        nodeNeighbours[nodeIndex] = neighbours;
        return triangle;
    }

    int Pathfinding::UpdatePositionNodeCorners(Float2 &position, NavMesh &navMesh, int nodeIndex)
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
