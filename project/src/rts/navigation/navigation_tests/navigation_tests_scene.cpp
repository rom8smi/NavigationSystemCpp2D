#include "navigation_tests_scene.hpp"
#include "navigation_tests_scene_indices.hpp"

namespace NavigationSystemCode
{
    void NavigationTestsScene::create_scene_properties(int current_scene_index)
    {
        if (current_scene_index == NavigationTestsSceneIndices::BLOCKED_DESTINATION)
        {
            create_blocked_destination_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::BLOCKED_DESTINATION_WITH_BRIDGE)
        {
            create_blocked_destination_with_bridge_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::CENTRAL_PASSAGE)
        {
            create_central_passage_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::CHAINED_OBSTACLES)
        {
            create_chained_obstacles_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::CIRCLES)
        {
            create_circles_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::RECTANGLE_SPAWNERS)
        {
            create_rectangle_spawners_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::RECTANGLE_SPAWNERS_WITH_OBSTACLES)
        {
            create_rectangle_spawners_with_obstacles_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::RANDOM_DYNAMIC_OBSTACLES)
        {
            create_random_dynamic_obstacles_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::RANDOM_OBSTACLES)
        {
            create_random_obstacles_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::RANDOM_ROTATED_OBSTACLES)
        {
            create_random_rotated_obstacles_scene();
        }
        else if (current_scene_index == NavigationTestsSceneIndices::SPIRAL)
        {
            create_spiral_scene();
        }
        else
        {
            create_blocked_destination_scene();
        }
    }

    void NavigationTestsScene::create_blocked_destination_scene()
    {
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-4.0f, -12.0f),
            .size = Float3(10.0f, 10.0f, 150.0f),
            .is_visible = true,
            .is_walkable = false});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-40.0f, 0.0f),
                .size = Float2(40.0f, 40.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(20.0f, 0.0f),
                .size = Float2(20.0f, 40.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::create_blocked_destination_with_bridge_scene()
    {
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-4.0f, -12.0f),
            .size = Float3(10.0f, 10.0f, 150.0f),
            .is_visible = false,
            .is_walkable = false});

        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-4.0f, 0.0f),
            .size = Float3(15.0f, 10.0f, 20.0f),
            .is_visible = false,
            .is_walkable = true});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-40.0f, 0.0f),
                .size = Float2(40.0f, 40.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(20.0f, 0.0f),
                .size = Float2(20.0f, 40.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::create_central_passage_scene()
    {
        manual_agent_circular_spawners.push_back(
            ManualAgentCircularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(0.0f, 0.0f),
                .radius = 40.0f,
                .number_to_spawn = 400,
                .set_destination_opposite_in_circle = true,
                .agent_type = 5,
                .seed = 0});

        manual_agent_circular_spawners.push_back(
            ManualAgentCircularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(0.0f, 0.0f),
                .radius = 40.0f,
                .number_to_spawn = 400,
                .set_destination_opposite_in_circle = true,
                .agent_type = 5,
                .seed = 1});
    }

    void NavigationTestsScene::create_chained_obstacles_scene()
    {
        chained_obstacle_spawners.push_back(ChainedObstacleSpawner{
            .start = Float2(0.0f, 50.0f),
            .number_to_spawn = 300,
            .obstacle_size = 0.3f,
            .chain_direction = Float2(0.0f, -1.0f),
            .is_visible = false});
    }

    void NavigationTestsScene::create_circles_scene()
    {
        // Obstacle 1
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(0.1f, -28.3f),
            .size = Float3(65.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 2
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(32.8f, 1.2f),
            .size = Float3(5.0f, 10.0f, 65.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 3
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(0.1f, -7.9f),
            .size = Float3(25.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 4
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(9.4f, 1.5f),
            .size = Float3(5.0f, 10.0f, 20.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 5
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-30.0f, 1.2f),
            .size = Float3(5.0f, 10.0f, 65.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 6
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(0.1f, 30.9f),
            .size = Float3(65.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 7
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(0.1f, 11.8f),
            .size = Float3(25.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 8
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-10.0f, 1.5f),
            .size = Float3(5.0f, 10.0f, 20.0f),
            .is_visible = true,
            .is_walkable = false});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(0.0f, 0.0f),
                .size = Float2(10.0f, 10.0f),
                .number_to_spawn = 100,
                .destination = Float2(-42.0f, 42.0f),
                .agent_type = 6,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-40.0f, 40.0f),
                .size = Float2(10.0f, 10.0f),
                .number_to_spawn = 100,
                .destination = Float2(40.0f, -40.0f),
                .agent_type = 5,
                .seed = 1});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 1.0f, 0.0f, 1.0f),
                .center = Float2(20.0f, -20.0f),
                .size = Float2(10.0f, 10.0f),
                .number_to_spawn = 100,
                .destination = Float2(-40.0f, 40.0f),
                .agent_type = 7,
                .seed = 1});
    }

    void NavigationTestsScene::create_rectangle_spawners_scene()
    {
        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::create_rectangle_spawners_with_obstacles_scene()
    {
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-15.4f, 12.0f),
            .size = Float3(10.0f, 10.0f, 86.0f),
            .is_visible = true,
            .is_walkable = false});

        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(16.3f, -11.2f),
            .size = Float3(10.0f, 10.0f, 84.5f),
            .is_visible = true,
            .is_walkable = false});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-30.0f, 0.0f),
                .size = Float2(15.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(30.0f, 0.0f),
                .size = Float2(15.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::create_random_dynamic_obstacles_scene()
    {
        random_dynamic_obstacles_visible = true;

        random_dynamic_obstacle_spawners.push_back(
            RandomDynamicObstaclesSpawner{
                .min = Float2(-50.0f, -50.0f),
                .max = Float2(50.0f, 50.0f),
                .min_size = Float2(0.3f, 0.3f),
                .max_size = Float2(9.3f, 9.3f),
                .seed = 2,
                .randomize_rotation = true,
                .creation_time_interval = 0.1f,
                .remaining_time_min = 15.0f,
                .remaining_time_max = 20.0f,
                .creation_time_passed = 0.0f});

        vector<ManualAgentRectangularSpawner> inner_spawners;
        inner_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        inner_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});

        random_dynamic_agent_spawners.push_back(RandomDynamicAgentsSpawner{
            .manual_agent_rectangular_spawners = inner_spawners});
    }

    void NavigationTestsScene::create_random_obstacles_scene()
    {
        random_dynamic_obstacles_visible = true;

        random_obstacle_spawners.push_back(
            RandomObstaclesSpawner{
                .number_to_spawn = 40,
                .min = Float2(-50.0f, -50.0f),
                .max = Float2(50.0f, 50.0f),
                .min_size = Float2(0.3f, 0.3f),
                .max_size = Float2(9.3f, 9.3f),
                .seed = 2,
                .randomize_rotation = false});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::create_random_rotated_obstacles_scene()
    {
        random_dynamic_obstacles_visible = true;

        random_obstacle_spawners.push_back(
            RandomObstaclesSpawner{
                .number_to_spawn = 40,
                .min = Float2(-50.0f, -50.0f),
                .max = Float2(50.0f, 50.0f),
                .min_size = Float2(0.3f, 0.3f),
                .max_size = Float2(9.3f, 9.3f),
                .seed = 2,
                .randomize_rotation = true});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(35.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(20.0f, 0.0f),
                .size = Float2(30.0f, 30.0f),
                .number_to_spawn = 400,
                .destination = Float2(-34.0f, 0.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::create_spiral_scene()
    {
        // Obstacle 1
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-31.8f, 12.0f),
            .size = Float3(5.0f, 10.0f, 86.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 2
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(0.1f, -28.3f),
            .size = Float3(65.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 3
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(32.8f, 1.2f),
            .size = Float3(5.0f, 10.0f, 65.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 4
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(8.8f, 30.8f),
            .size = Float3(48.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 5
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(-12.3f, 10.8f),
            .size = Float3(5.0f, 10.0f, 43.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 6
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(0.1f, -7.9f),
            .size = Float3(25.0f, 10.0f, 5.0f),
            .is_visible = true,
            .is_walkable = false});
        // Obstacle 7
        manual_obstacle_spawners.push_back(ManualObstacleSpawner{
            .center = Float2(9.4f, 1.5f),
            .size = Float3(5.0f, 10.0f, 20.0f),
            .is_visible = true,
            .is_walkable = false});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(1.0f, 0.0f, 0.0f, 1.0f),
                .center = Float2(-40.0f, 40.0f),
                .size = Float2(10.0f, 10.0f),
                .number_to_spawn = 400,
                .destination = Float2(0.0f, 0.0f),
                .agent_type = 5,
                .seed = 0});

        manual_agent_rectangular_spawners.push_back(
            ManualAgentRectangularSpawner{
                .color = Rgba(0.0f, 0.0f, 1.0f, 1.0f),
                .center = Float2(0.0f, 0.0f),
                .size = Float2(10.0f, 10.0f),
                .number_to_spawn = 400,
                .destination = Float2(-42.0f, 42.0f),
                .agent_type = 6,
                .seed = 1});
    }

    void NavigationTestsScene::reset()
    {
        manual_obstacle_spawners.clear();
        chained_obstacle_spawners.clear();
        random_dynamic_obstacle_spawners.clear();
        random_obstacle_spawners.clear();
        manual_agent_rectangular_spawners.clear();
        manual_agent_circular_spawners.clear();
        random_dynamic_agent_spawners.clear();
        random_dynamic_obstacles.clear();
        random_dynamic_agents.clear();
    }
}
