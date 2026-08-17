#include "navigation_system.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/math/angle_utils.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/math/obstacle_utils.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

namespace NavigationSystemCode
{
    void NavigationSystem::ready(GodotWorld &godot_world)
    {
        agentTypes = vector<AgentType>{
            // 0 Test1
            AgentType{
                .radius = 1.0f,
                .isStatic = false,
                .speed = 2.5f},
            // 1 Test2
            AgentType{
                .radius = 2.5f,
                .isStatic = false,
                .speed = 1.5f},
            // 2 Test3
            AgentType{
                .radius = 2.5f,
                .isStatic = false,
                .speed = 4.0f},
            // 3 Test4
            AgentType{
                .radius = 1.0f,
                .isStatic = false,
                .speed = 2.0f},
            // 4 Test5
            AgentType{
                .radius = 2.5f,
                .isStatic = false,
                .speed = 3.0f},
            // 5 Test agent small
            AgentType{
                .radius = 0.5f,
                .isStatic = false,
                .speed = 5.0f},
            // 6 Test agent large
            AgentType{
                .radius = 0.7f,
                .isStatic = false,
                .speed = 5.0f},
            // 7 Test agent medium
            AgentType{
                .radius = 0.6f,
                .isStatic = false,
                .speed = 5.0f},
            // 8 Test circular obstacle
            AgentType{
                .radius = 2.0f,
                .isStatic = true,
                .speed = 0.0f},
            // 9 Test6
            AgentType{
                .radius = 1.0f,
                .isStatic = true,
                .speed = 0.0f}};

        largestAgentRadius = 0.0f;
        for (int i = 0; i < agentTypes.size(); i++)
        {
            largestAgentRadius = MathUtils::max(largestAgentRadius, agentTypes[i].radius);
        }

        if (use_debug_agents_mover)
        {
            debug_agents_mover.ready();
        }
    }

    void NavigationSystem::process(
        float delta)
    {
        if (use_debug_agents_mover)
        {
            debug_agents_mover.process(
                agentsMover,
                agents,
                agentPositions,
                agentNeighbours,
                agentNeighbourCounts,
                agentTypes,
                largestAgentRadius,
                navMesh,
                pathfinding,
                delta);
        }
        else
        {
            agentsMover.MoveAgents(
                agents,
                agentPositions,
                agentNeighbours,
                agentNeighbourCounts,
                agentTypes,
                largestAgentRadius,
                navMesh,
                pathfinding,
                delta);
        }
    }

    void NavigationSystem::AddAgent(Float2 &position, int agentTypeIndex, int unitIndex, float speed)
    {
        Agent agent{
            .unitIndex = unitIndex,
            .agentTypeIndex = agentTypeIndex,
            .speed = speed,
            .destination = Float2(0.0f, 0.0f),
            .currentWaypointIndex = 0,
            .followingPath = false,
            .searchPathLater = false,
            .destinationUpdateNeeded = false,
            .pathVelocity = Float2(0.0f, 0.0f),
            .localAvoidanceVelocity = Float2(0.0f, 0.0f),
            .powerFactorSum = 0.0f,
            .finalVelocity = Float2(0.0f, 0.0f),
            .remainingPathDistance = 0.0f,
            .pathMovementFailuresCount = 0,
            .density = 0.0f};

        agents.push_back(agent);
        agentPositions.push_back(position);
        agentNeighbours.push_back(vector<int>());
        agentNeighbourCounts.push_back(0);
    }

    bool NavigationSystem::HasPath(int agentIndex)
    {
        return agents[agentIndex].followingPath ||
               agents[agentIndex].destinationUpdateNeeded ||
               agents[agentIndex].searchPathLater;
    }

    void NavigationSystem::SetDestination(int agentIndex, Float2 &destination)
    {
        agents[agentIndex].destinationUpdateNeeded = true;
        agents[agentIndex].destination = destination;
    }

    void NavigationSystem::SetDestinationInstant(int agentIndex, Float2 &destination)
    {
        agents[agentIndex].currentWaypointIndex = 0;
        agents[agentIndex].destination = destination;

        Path path = pathfinding.FindPath(agentPositions[agentIndex], destination, navMesh);

        if (path.success)
        {
            agents[agentIndex].waypoints = path.waypoints;
            agents[agentIndex].currentWaypointIndex = 0;
            agents[agentIndex].followingPath = true;
            agents[agentIndex].searchPathLater = false;
        }

        agents[agentIndex].pathMovementFailuresCount = 0;
    }

    void NavigationSystem::stop(int agent_index)
    {
        agents[agent_index].followingPath = false;
        agents[agent_index].searchPathLater = false;
        agents[agent_index].destinationUpdateNeeded = false;
        agents[agent_index].currentWaypointIndex = 0;
        agents[agent_index].destination = agentPositions[agent_index];
        agents[agent_index].pathMovementFailuresCount = 0;
        agents[agent_index].remainingPathDistance = 0.0f;
        agents[agent_index].waypoints.clear();
    }

    bool NavigationSystem::has_path_or_destination_update_needed(int agent_index)
    {
        return agents[agent_index].followingPath || agents[agent_index].destinationUpdateNeeded;
    }

    bool NavigationSystem::is_obstacle_part_in_unwalkable_navmesh(Obstacle &obstacle)
    {
        int corners_count = obstacle.obstacleCorners.size();

        for (int i = 0; i < corners_count; i++)
        {
            if (navMesh.FindUnwalkableTriangleForPoint(obstacle.obstacleCorners[i]) != -1)
            {
                return true;
            }
        }

        for (int i = 0; i < corners_count; i++)
        {
            int i_next = i + 1;
            if (i_next == corners_count)
            {
                i_next = 0;
            }

            if (!navMesh.CanPointsBeReachedInStraightLine(obstacle.obstacleCorners[i], obstacle.obstacleCorners[i_next]))
            {
                return true;
            }
        }

        return false;
    }

    void NavigationSystem::reset()
    {
        agents.clear();
        agentPositions.clear();
        agentNeighbours.clear();
        agentNeighbourCounts.clear();
        obstacles.clear();
        unextended_obstacles.clear();
    }
}
