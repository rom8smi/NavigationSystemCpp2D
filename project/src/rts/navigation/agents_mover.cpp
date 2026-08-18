#include "agents_mover.hpp"
#include "path_utils.hpp"
#include "rts/math/math_utils.hpp"

namespace NavigationSystemCode
{
    NavigationSystemCode::AgentsMover::AgentsMover()
    {
        epsilon = 0.001f;
        findPathsIndex = 0;
        maxRepathsCount = 40;
        velocitySmoothingMin = 0.8f;
        velocitySmoothingMax = 0.97f;
        localAvoidancePowerFactor = 5;
        maxLocalAvoidanceNeighbours = 10;
    }

    void AgentsMover::MoveAgents(
        vector<Agent> &agents,
        vector<Float2> &agentPositions,
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts,
        vector<AgentType> &agentTypes,
        float largestAgentRadius,
        NavMesh &navMesh,
        Pathfinding &pathfinding,
        float deltaTime)
    {
        Repath(agents, agentPositions, pathfinding, navMesh);
        CalculatePathVelocities(agents, agentPositions, navMesh);

        if (agentPositions.size() < 3)
        {
            FindNeighboursDirect(agents, agentPositions, agentNeighbours, agentNeighbourCounts, largestAgentRadius);
        }
        else
        {
            triangulation.Create(agentPositions);
            FindNeighboursWithTriangulation(agentNeighbours, agentNeighbourCounts);
        }

        CalculateVelocitiesFromNeighbours(agents, agentPositions, agentNeighbours, agentNeighbourCounts, agentTypes, navMesh, deltaTime);
    }

    void AgentsMover::CalculatePathVelocities(vector<Agent> &agents, vector<Float2> &agentPositions, NavMesh &navMesh)
    {
        for (int i = 0; i < agents.size(); i++)
        {
            agents[i].pathVelocity = FollowPath(agentPositions[i], agents[i], navMesh);
        }
    }

    void AgentsMover::FindNeighboursDirect(
        vector<Agent> &agents,
        vector<Float2> &agentPositions,
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts,
        float largestAgentRadius)
    {
        for (int i = 0; i < agentNeighbourCounts.size(); i++)
        {
            agentNeighbourCounts[i] = 0;
        }

        float maxNeighbourDistanceSquare = 4.0f * largestAgentRadius * largestAgentRadius;

        for (int i = 0; i < agents.size(); i++)
        {
            Float2 agentPosition = agentPositions[i];

            for (int j = i + 1; j < agents.size(); j++)
            {
                Float2 relative = agentPosition - agentPositions[j];
                float distanceSquare = relative.length_squared();

                if (distanceSquare <= maxNeighbourDistanceSquare)
                {
                    if (agentNeighbourCounts[i] < agentNeighbours[i].size())
                    {
                        agentNeighbours[i][agentNeighbourCounts[i]] = j;
                    }
                    else
                    {
                        agentNeighbours[i].push_back(j);
                    }

                    if (agentNeighbourCounts[j] < agentNeighbours[j].size())
                    {
                        agentNeighbours[j][agentNeighbourCounts[j]] = i;
                    }
                    else
                    {
                        agentNeighbours[j].push_back(i);
                    }

                    agentNeighbourCounts[i]++;
                    agentNeighbourCounts[j]++;
                }
            }
        }

        for (int i = 0; i < agentNeighbourCounts.size(); i++)
        {
            if (agentNeighbours[i].size() > agentNeighbourCounts[i] + 2)
            {
                agentNeighbours[i].resize(agentNeighbourCounts[i]);
            }
        }
    }

    void AgentsMover::FindNeighboursWithTriangulation(
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts)
    {
        for (int i = 0; i < agentNeighbourCounts.size(); i++)
        {
            agentNeighbourCounts[i] = 0;
        }

        for (int e = 0; e < triangulation.trianglesLen; e++)
        {
            if (e > triangulation.halfedges[e])
            {
                int p = triangulation.triangles[e];
                int q = triangulation.triangles[Delaunator::NextHalfedge(e)];

                if (agentNeighbourCounts[p] < agentNeighbours[p].size())
                {
                    agentNeighbours[p][agentNeighbourCounts[p]] = q;
                }
                else
                {
                    agentNeighbours[p].push_back(q);
                }

                if (agentNeighbourCounts[q] < agentNeighbours[q].size())
                {
                    agentNeighbours[q][agentNeighbourCounts[q]] = p;
                }
                else
                {
                    agentNeighbours[q].push_back(p);
                }

                agentNeighbourCounts[p]++;
                agentNeighbourCounts[q]++;
            }
        }

        for (int i = 0; i < agentNeighbourCounts.size(); i++)
        {
            if (agentNeighbours[i].size() > agentNeighbourCounts[i] + 2)
            {
                agentNeighbours[i].resize(agentNeighbourCounts[i]);
            }
        }
    }

    void AgentsMover::CalculateVelocitiesFromNeighbours(
        vector<Agent> &agents,
        vector<Float2> &agentPositions,
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts,
        vector<AgentType> &agentTypes,
        NavMesh &navMesh,
        float deltaTime)
    {
        for (int i = 0; i < agents.size(); i++)
        {
            Float2 agentPosition = agentPositions[i];
            Float2 avoidanceVelocity = Float2(0.0f, 0.0f);
            Float2 softAvoidanceVelocity = Float2(0.0f, 0.0f);
            bool hasSoftAvoidance = false;
            float powerFactorSum = 0.0f;
            int agentTypeIndex = agents[i].agentTypeIndex;
            float radius = agentTypes[agentTypeIndex].radius;

            vector<int> neighbours = agentNeighbours[i];
            int neighboursCount = agentNeighbourCounts[i];

            for (int k = 0; k < neighboursCount; k++)
            {
                int neighbour = neighbours[k];
                int neighbourAgentTypeIndex = agents[neighbour].agentTypeIndex;
                float neighbourRadius = agentTypes[neighbourAgentTypeIndex].radius;

                float radiiSum = radius + neighbourRadius;
                float radiiSumSquare = radiiSum * radiiSum;

                Float2 relative = agentPosition - agentPositions[neighbour];
                float distanceSquare = relative.length_squared();

                if (distanceSquare < epsilon)
                {
                    distanceSquare = epsilon;
                }

                float normalizedDistanceSquare = distanceSquare / radiiSumSquare;

                if (normalizedDistanceSquare < 1.0f)
                {
                    if (agentTypes[neighbourAgentTypeIndex].softLocalAvoidance)
                    {
                        softAvoidanceVelocity += relative.normalized();
                        hasSoftAvoidance = true;
                    }
                    else
                    {
                        float powerFactor = 1.0f;
                        for (int l = 0; l < localAvoidancePowerFactor; l++)
                        {
                            powerFactor *= normalizedDistanceSquare;
                        }
                        avoidanceVelocity += relative.normalized() / powerFactor;
                        powerFactorSum += 1.0f / powerFactor;
                    }
                }
            }

            if (hasSoftAvoidance)
            {
                softAvoidanceVelocity.normalize();
                softAvoidanceVelocity = softAvoidanceVelocity * (agents[i].pathVelocity.length() * 0.8f);
                avoidanceVelocity += softAvoidanceVelocity;
            }

            agents[i].localAvoidanceVelocity = avoidanceVelocity;
            agents[i].powerFactorSum = powerFactorSum;

            if (!agentTypes[agentTypeIndex].isStatic)
            {
                CalculateFinalVelocities(agents, agentPositions, navMesh, deltaTime, i);
            }
        }
    }

    void AgentsMover::CalculateFinalVelocities(vector<Agent> &agents, vector<Float2> &agentPositions, NavMesh &navMesh, float deltaTime, int i)
    {
        float previousDensity = agents[i].density;
        float density = 1.0f - MathUtils::clamp01(1.0f / agents[i].powerFactorSum);
        density = 0.9f * previousDensity + 0.1f * density;
        agents[i].density = density;

        Float2 finalVelocity = agents[i].finalVelocity;
        float currentVelocitySmoothingFactor = MathUtils::interpolate_clamped(density, 0.0f, 1.0f, velocitySmoothingMin, velocitySmoothingMax);
        float speedDeltaTime = agents[i].speed * deltaTime;

        finalVelocity = finalVelocity * currentVelocitySmoothingFactor +
                        (agents[i].pathVelocity + agents[i].localAvoidanceVelocity).normalized() * (1.0f - currentVelocitySmoothingFactor) * speedDeltaTime;

        Float2 currentPosition = agentPositions[i];
        Float2 newPosition = currentPosition + finalVelocity;
        GetNearestWalkablePositionResult getNearestWalkablePositionResult = navMesh.TryMoveToWalkableArea(newPosition);

        if (getNearestWalkablePositionResult.wasMoved)
        {
            newPosition = getNearestWalkablePositionResult.position;
            finalVelocity = newPosition - currentPosition;

            if (finalVelocity.length_squared() > speedDeltaTime * speedDeltaTime)
            {
                finalVelocity = finalVelocity.normalized() * speedDeltaTime;
            }
        }

        agents[i].finalVelocity = finalVelocity;
        agentPositions[i] += finalVelocity;
    }

    void AgentsMover::Repath(vector<Agent> &agents, vector<Float2> &agentPositions, Pathfinding &pathfinding, NavMesh &navMesh)
    {
        int repathsCount = 0;

        for (int i = 0; i < agents.size(); i++)
        {
            findPathsIndex++;
            if (findPathsIndex >= agents.size())
            {
                findPathsIndex = 0;
            }

            if (agents[findPathsIndex].destinationUpdateNeeded)
            {
                Float2 targetPosition = agents[findPathsIndex].destination;
                Path path = pathfinding.FindPath(agentPositions[findPathsIndex], targetPosition, navMesh);

                if (path.success)
                {
                    agents[findPathsIndex].waypoints = path.waypoints;
                    agents[findPathsIndex].currentWaypointIndex = 0;
                    agents[findPathsIndex].followingPath = true;
                    agents[findPathsIndex].searchPathLater = false;
                }
                else
                {
                    agents[findPathsIndex].searchPathLater = true;
                }

                agents[findPathsIndex].destinationUpdateNeeded = false;
                repathsCount++;

                if (repathsCount >= maxRepathsCount)
                {
                    return;
                }
            }
            else
            {
                if (agents[findPathsIndex].followingPath)
                {
                    int currentWaypointIndex = agents[findPathsIndex].currentWaypointIndex;
                    float remainingPathDistance = (agents[findPathsIndex].waypoints[currentWaypointIndex] - agentPositions[findPathsIndex]).length() +
                                                  PathUtils::CalculatePathLength(agents[findPathsIndex].waypoints, currentWaypointIndex);

                    if (remainingPathDistance >= agents[findPathsIndex].remainingPathDistance)
                    {
                        agents[findPathsIndex].pathMovementFailuresCount++;

                        if (agents[findPathsIndex].pathMovementFailuresCount > 50)
                        {
                            Float2 targetPosition = agents[findPathsIndex].destination;
                            Path path = pathfinding.FindPath(agentPositions[findPathsIndex], targetPosition, navMesh);

                            if (path.success)
                            {
                                agents[findPathsIndex].waypoints = path.waypoints;
                                agents[findPathsIndex].currentWaypointIndex = 0;
                            }

                            agents[findPathsIndex].pathMovementFailuresCount = 0;
                            repathsCount++;

                            if (repathsCount >= maxRepathsCount)
                            {
                                return;
                            }
                        }
                    }

                    agents[findPathsIndex].remainingPathDistance = remainingPathDistance;
                }
                else if (agents[findPathsIndex].searchPathLater)
                {
                    agents[findPathsIndex].pathMovementFailuresCount++;

                    if (agents[findPathsIndex].pathMovementFailuresCount > 100)
                    {
                        Float2 targetPosition = agents[findPathsIndex].destination;
                        Path path = pathfinding.FindPath(agentPositions[findPathsIndex], targetPosition, navMesh);

                        if (path.success)
                        {
                            agents[findPathsIndex].waypoints = path.waypoints;
                            agents[findPathsIndex].currentWaypointIndex = 0;
                            agents[findPathsIndex].followingPath = true;
                            agents[findPathsIndex].searchPathLater = false;
                        }

                        agents[findPathsIndex].pathMovementFailuresCount = 0;
                        repathsCount++;

                        if (repathsCount >= maxRepathsCount)
                        {
                            return;
                        }
                    }
                }
            }
        }
    }

    Float2 AgentsMover::FollowPath(Float2 &position, Agent &agent, NavMesh &navMesh)
    {
        if (!agent.followingPath)
        {
            return Float2(0.0f, 0.0f);
        }

        Float2 relative = agent.waypoints[agent.currentWaypointIndex] - position;
        float stopDistanceSquare = MathUtils::interpolate_clamped(agent.density, 0.0f, 1.0f, 0.01f, 1.5f);
        float currentDistanceSqr = relative.length_squared();

        if (currentDistanceSqr < stopDistanceSquare)
        {
            bool canNextWaypointBeReached = true;
            int nextWaypoint = agent.currentWaypointIndex + 1;

            if (nextWaypoint < agent.waypoints.size() &&
                currentDistanceSqr > 0.2f * stopDistanceSquare &&
                !navMesh.CanPointsBeReachedInStraightLine(position, agent.waypoints[nextWaypoint]))
            {
                canNextWaypointBeReached = false;
            }

            if (canNextWaypointBeReached)
            {
                agent.currentWaypointIndex++;
                if (agent.currentWaypointIndex >= agent.waypoints.size())
                {
                    agent.followingPath = false;
                    return Float2(0.0f, 0.0f);
                }
            }
        }

        return relative.normalized();
    }
}
