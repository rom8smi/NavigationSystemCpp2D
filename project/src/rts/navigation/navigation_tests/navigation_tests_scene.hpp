#ifndef NAVIGATION_TESTS_SCENE_HPP
#define NAVIGATION_TESTS_SCENE_HPP

#include "manual_obstacle_spawner.hpp"
#include "chained_obstacle_spawner.hpp"
#include "random_dynamic_obstacles_spawner.hpp"
#include "random_obstacles_spawner.hpp"
#include "manual_agent_rectangular_spawner.hpp"
#include "manual_agent_circular_spawner.hpp"
#include "random_dynamic_agents_spawner.hpp"
#include "random_dynamic_obstacle.hpp"
#include "random_dynamic_agent.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct NavigationTestsScene
    {
        vector<ManualObstacleSpawner> manual_obstacle_spawners;
        vector<ChainedObstacleSpawner> chained_obstacle_spawners;
        vector<RandomDynamicObstaclesSpawner> random_dynamic_obstacle_spawners;
        vector<RandomObstaclesSpawner> random_obstacle_spawners;
        vector<ManualAgentRectangularSpawner> manual_agent_rectangular_spawners;
        vector<ManualAgentCircularSpawner> manual_agent_circular_spawners;
        vector<RandomDynamicAgentsSpawner> random_dynamic_agent_spawners;
        vector<RandomDynamicObstacle> random_dynamic_obstacles;
        vector<RandomDynamicAgent> random_dynamic_agents;
        bool random_dynamic_obstacles_visible;

        void create_scene_properties(int current_scene_index);
        void create_blocked_destination_scene();
        void create_blocked_destination_with_bridge_scene();
        void create_central_passage_scene();
        void create_chained_obstacles_scene();
        void create_circles_scene();
        void create_rectangle_spawners_scene();
        void create_rectangle_spawners_with_obstacles_scene();
        void create_random_dynamic_obstacles_scene();
        void create_random_obstacles_scene();
        void create_random_rotated_obstacles_scene();
        void create_spiral_scene();
        void reset();
    };
}

#endif
