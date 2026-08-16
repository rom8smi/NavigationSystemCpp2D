#ifndef NAVIGATION_SYSTEM_HPP
#define NAVIGATION_SYSTEM_HPP

#include "pathfinding.hpp"
#include "agents_mover.hpp"
#include "agent_type.hpp"
#include "rts/debug/debug_agents_mover.hpp"
#include "rts/godot_wrapper/godot_world.hpp"

#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct NavigationSystem
    {
        NavMesh navMesh;
        Pathfinding pathfinding;
        AgentsMover agentsMover;
        DebugAgentsMover debug_agents_mover;
        bool use_debug_agents_mover = false;

        vector<Agent> agents;
        vector<Float2> agentPositions;
        vector<vector<int>> agentNeighbours;
        vector<int> agentNeighbourCounts;
        vector<AgentType> agentTypes;

        vector<Obstacle> obstacles;
        vector<Obstacle> unextended_obstacles;

        Aabb worldBounds;
        Aabb paddedWorldBounds;

        float largestAgentRadius;

        void ready(GodotWorld &godot_world);
        void process(
            float delta);
        void AddAgent(Float2 &position, int agentTypeIndex, int unitIndex, float speed);
        bool HasPath(int agentIndex);
        void SetDestination(int agentIndex, Float2 &destination);
        void SetDestinationInstant(int agentIndex, Float2 &destination);
        void stop(int agent_index);
        bool has_path_or_destination_update_needed(int agent_index);
        bool is_obstacle_part_in_unwalkable_navmesh(Obstacle &obstacle);
        void reset();
    };
}

#endif
