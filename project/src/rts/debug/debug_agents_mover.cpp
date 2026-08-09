#include "debug_agents_mover.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"
#include <chrono>

namespace NavigationSystemCode
{
    void DebugAgentsMover::ready()
    {
        debug_count = 3;
        debug_count_passed = 0;
        total_time_passed = 0.0f;
        number_of_debug_steps = 0;
        number_of_debug_steps_to_finish = 15000;

        file.open("DebugAgentsMover.dat");
        file.write("# time repath find_neighbours calculate_velocities_from_neighbours adjust_agent_positions_for_obstacles \n");
    }

    void DebugAgentsMover::process(
        AgentsMover &system,
        vector<Agent> &agents,
        vector<Float2> &agentPositions,
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts,
        vector<AgentType> &agentTypes,
        float largestAgentRadius,
        NavMesh &navMesh,
        Pathfinding &pathfinding,
        float delta)
    {
        debug_count_passed++;
        total_time_passed += delta;

        if (number_of_debug_steps < number_of_debug_steps_to_finish && debug_count_passed > debug_count)
        {
            debug_count_passed -= debug_count;
            number_of_debug_steps++;

            process_debug(
                system,
                agents,
                agentPositions,
                agentNeighbours,
                agentNeighbourCounts,
                agentTypes,
                largestAgentRadius,
                navMesh,
                pathfinding,
                delta);

            if (number_of_debug_steps >= number_of_debug_steps_to_finish)
            {
                file.close();
            }
        }
        else
        {
            process_regular(
                system,
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

    void DebugAgentsMover::process_debug(
        AgentsMover &system,
        vector<Agent> &agents,
        vector<Float2> &agentPositions,
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts,
        vector<AgentType> &agentTypes,
        float largestAgentRadius,
        NavMesh &navMesh,
        Pathfinding &pathfinding,
        float delta)
    {
        auto stopwatch = std::chrono::high_resolution_clock::now();

        system.Repath(agents, agentPositions, pathfinding, navMesh);

        float t1 = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - stopwatch).count();

        system.CalculatePathVelocities(agents, agentPositions, navMesh);

        float t2 = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - stopwatch).count();

        if (agentPositions.size() < 3)
        {
            system.FindNeighboursDirect(agents, agentPositions, agentNeighbours, agentNeighbourCounts, largestAgentRadius);
        }
        else
        {
            system.triangulation.Create(agentPositions);
            system.FindNeighboursWithTriangulation(agentNeighbours, agentNeighbourCounts);
        }

        float t3 = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - stopwatch).count();

        system.CalculateVelocitiesFromNeighbours(agents, agentPositions, agentNeighbours, agentNeighbourCounts, agentTypes, navMesh, delta);

        float t4 = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - stopwatch).count();

        float t_repath = t1;
        float t_calculate_path_velocities = t2 - t1;
        float t_find_neighbours = t3 - t2;
        float t_calculate_velocities_from_neighbours = t4 - t3;

        file.write(
            to_string(total_time_passed) + " " +
            to_string(t_repath) + " " +
            to_string(t_find_neighbours) + " " +
            to_string(t_calculate_velocities_from_neighbours) + "\n");

        float debug_steps_fraction = (100.0f * number_of_debug_steps) / number_of_debug_steps_to_finish;
        GodotUtils::print(debug_steps_fraction);
    }

    void DebugAgentsMover::process_regular(
        AgentsMover &system,
        vector<Agent> &agents,
        vector<Float2> &agentPositions,
        vector<vector<int>> &agentNeighbours,
        vector<int> &agentNeighbourCounts,
        vector<AgentType> &agentTypes,
        float largestAgentRadius,
        NavMesh &navMesh,
        Pathfinding &pathfinding,
        float delta)
    {
        system.MoveAgents(
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
