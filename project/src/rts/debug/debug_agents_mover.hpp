#ifndef DEBUG_AGENTS_MOVER_HPP
#define DEBUG_AGENTS_MOVER_HPP

#include "rts/navigation/agents_mover.hpp"
#include "rts/file/file.hpp"

namespace NavigationSystemCode
{
    struct DebugAgentsMover
    {
        int debug_count;
        int debug_count_passed;
        float total_time_passed;
        int number_of_debug_steps;
        int number_of_debug_steps_to_finish;
        File file;

        void ready();
        void process(
            AgentsMover &system,
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts,
            vector<AgentType> &agentTypes,
            float largestAgentRadius,
            NavMesh &navMesh,
            Pathfinding &pathfinding,
            float delta);
        void process_debug(
            AgentsMover &system,
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts,
            vector<AgentType> &agentTypes,
            float largestAgentRadius,
            NavMesh &navMesh,
            Pathfinding &pathfinding,
            float delta);
        void process_regular(
            AgentsMover &system,
            vector<Agent> &agents,
            vector<Float2> &agentPositions,
            vector<vector<int>> &agentNeighbours,
            vector<int> &agentNeighbourCounts,
            vector<AgentType> &agentTypes,
            float largestAgentRadius,
            NavMesh &navMesh,
            Pathfinding &pathfinding,
            float delta);
    };
}

#endif
