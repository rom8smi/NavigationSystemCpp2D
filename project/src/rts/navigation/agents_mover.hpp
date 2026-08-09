#ifndef AGENTS_MOVER_HPP
#define AGENTS_MOVER_HPP

#include "rts/math/float2.hpp"
#include "rts/navigation/delaunator/delaunator.hpp"
#include <vector>
#include "agent.hpp"
#include "agent_type.hpp"
#include "nav_mesh.hpp"
#include "pathfinding.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct AgentsMover
    {
        float epsilon;
        int findPathsIndex;
        int maxRepathsCount;
        float velocitySmoothingMin;
        float velocitySmoothingMax;
        int localAvoidancePowerFactor;
        int maxLocalAvoidanceNeighbours;
        Delaunator triangulation;

        AgentsMover();
        void MoveAgents(
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts,
            vector<AgentType> &agentTypes,
            float largestAgentRadius,
            NavMesh &navMesh,
            Pathfinding &pathfinding,
            float deltaTime);
        void CalculatePathVelocities(
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            NavMesh &navMesh);
        void FindNeighboursDirect(
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts,
            float largestAgentRadius);
        void FindNeighboursWithTriangulation(
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts);
        void CalculateVelocitiesFromNeighbours(
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts,
            vector<AgentType> &agentTypes,
            NavMesh &navMesh,
            float deltaTime);
        void CalculateFinalVelocities(
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            NavMesh &navMesh,
            float deltaTime,
            int i);
        void Repath(vector<Agent> &agents, vector<Float2> &agentPositions, Pathfinding &pathfinding, NavMesh &navMesh);
        Float2 FollowPath(Float2 &position, Agent &agent, NavMesh &navMesh);
    };
}

#endif
