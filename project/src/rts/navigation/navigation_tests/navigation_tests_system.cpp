#include "navigation_tests_system.hpp"
#include "rts/godot_wrapper/godot_mesh_utils.hpp"
#include "rts/godot_wrapper/godot_standard_material3d_wrapper.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/math/obstacle_utils.hpp"
#include "rts/navigation/obstacles_kd_tree.hpp"
#include "rts/math/angle_utils.hpp"
#include <godot_cpp/classes/capsule_mesh.hpp>
#include "rts/godot_wrapper/godot_utils.hpp"

namespace NavigationSystemCode
{
    void NavigationTestsSystem::ready(GodotNode3DWrapper &node3D)
    {
        run_navigation_tests = false;
        capsule_mesh.create();
        box_mesh.create();

        create_standard_multimesh(agents_multimesh, node3D, capsule_mesh.mesh, "NavigationTestsAgents");
        create_standard_multimesh(obstacles_multimesh, node3D, box_mesh.mesh, "NavigationTestsObstacles");

        current_scene_index = -1;
        number_of_agents = 0;
        obstacle_color = Rgba(0.53f, 0.42f, 0.235f, 1.0f);
    }

    void NavigationTestsSystem::create_standard_multimesh(
        GodotMultimesh &multimesh,
        GodotNode3DWrapper &node3D,
        godot::Mesh *mesh,
        const string &multimesh_name)
    {
        multimesh.multi_mesh.create();

        multimesh.multi_mesh.set_name(multimesh_name + "Multimesh");
        multimesh.multi_mesh.set_transform_format_3d();
        multimesh.multi_mesh.set_mesh(mesh);
        multimesh.set_use_colors(true);
        multimesh.set_use_custom_data(false);

        multimesh.multi_mesh_instance.create(node3D.node3D);
        multimesh.multi_mesh_instance.set_name(multimesh_name + "MultimeshInstance3D");
        multimesh.multi_mesh_instance.set_multimesh(multimesh.multi_mesh.multi_mesh);
        multimesh.multi_mesh_instance.set_lod_bias(0.0f);
        multimesh.multi_mesh_instance.set_ignore_occlusion_culling(true);

        GodotStandardMaterial3DWrapper material;
        material.create();
        material.set_flag_albedo_from_vertex_color(true);

        multimesh.multi_mesh_instance.set_material_override(material.material);

        multimesh.instance_count = 0;
        multimesh.update_buffer_chunk_size();
    }

    void NavigationTestsSystem::process(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world,
        float delta)
    {
        if (!run_navigation_tests)
        {
            return;
        }

        for (int i = 0; i < number_of_agents; i++)
        {
            Float3 position = VectorUtils::float2_x0z(navigation_system.agentPositions[i]);
            agents_multimesh.update_instance_position(i, position);
        }

        if (number_of_agents > 0)
        {
            agents_multimesh.set_buffer();
        }

        update_random_dynamic_obstacles(navigation_system, godot_world, delta);
        update_random_dynamic_agents(navigation_system, delta);
    }

    void NavigationTestsSystem::update_random_dynamic_obstacles(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world,
        float delta)
    {
        int random_dynamic_obstacle_spawners_count = navigation_tests_scene.random_dynamic_obstacle_spawners.size();

        if (random_dynamic_obstacle_spawners_count > 0)
        {
            for (int i = 0; i < random_dynamic_obstacle_spawners_count; i++)
            {
                navigation_tests_scene.random_dynamic_obstacle_spawners[i].creation_time_passed += delta;
                float creation_time_interval = navigation_tests_scene.random_dynamic_obstacle_spawners[i].creation_time_interval;

                if (navigation_tests_scene.random_dynamic_obstacle_spawners[i].creation_time_passed >
                    creation_time_interval)
                {
                    if (try_create_random_obstacle(
                            navigation_tests_scene.random_dynamic_obstacle_spawners[i].min,
                            navigation_tests_scene.random_dynamic_obstacle_spawners[i].max,
                            navigation_tests_scene.random_dynamic_obstacle_spawners[i].min_size,
                            navigation_tests_scene.random_dynamic_obstacle_spawners[i].max_size,
                            navigation_tests_scene.random_dynamic_obstacle_spawners[i].randomize_rotation,
                            true,
                            navigation_system,
                            godot_world))
                    {
                        navigation_tests_scene.random_dynamic_obstacle_spawners[i].creation_time_passed = 0.0f;

                        navigation_tests_scene.random_dynamic_obstacles.push_back(RandomDynamicObstacle{
                            .remaining_time = random.next_float(
                                navigation_tests_scene.random_dynamic_obstacle_spawners[i].remaining_time_min,
                                navigation_tests_scene.random_dynamic_obstacle_spawners[i].remaining_time_max)});
                    }
                }
            }

            int i_new = 0;
            bool removals_needed = false;
            int random_dynamic_obstacles_count = navigation_tests_scene.random_dynamic_obstacles.size();

            vector<int> new_indices;
            new_indices.resize(random_dynamic_obstacles_count);

            for (int i = 0; i < random_dynamic_obstacles_count; i++)
            {
                navigation_tests_scene.random_dynamic_obstacles[i].remaining_time -= delta;

                if (navigation_tests_scene.random_dynamic_obstacles[i].remaining_time < 0.0f)
                {
                    new_indices[i] = -1;
                    removals_needed = true;
                }
                else
                {
                    new_indices[i] = i_new;
                    i_new++;
                }
            }

            if (removals_needed)
            {
                if (navigation_tests_scene.random_dynamic_obstacles_visible)
                {
                    obstacles_multimesh.remove_by_mask(new_indices, true);
                    obstacles_multimesh.set_buffer();
                }

                for (int i = 0; i < navigation_tests_scene.random_dynamic_obstacles.size(); i++)
                {
                    int new_index = new_indices[i];
                    if (new_index != -1)
                    {
                        navigation_tests_scene.random_dynamic_obstacles[new_index] = navigation_tests_scene.random_dynamic_obstacles[i];
                        navigation_system.obstacles[new_index] = navigation_system.obstacles[i];
                        navigation_system.unextended_obstacles[new_index] = navigation_system.unextended_obstacles[i];
                    }
                }

                navigation_tests_scene.random_dynamic_obstacles.erase(
                    navigation_tests_scene.random_dynamic_obstacles.begin() + i_new,
                    navigation_tests_scene.random_dynamic_obstacles.end());

                navigation_system.obstacles.erase(
                    navigation_system.obstacles.begin() + i_new,
                    navigation_system.obstacles.end());

                navigation_system.unextended_obstacles.erase(
                    navigation_system.unextended_obstacles.begin() + i_new,
                    navigation_system.unextended_obstacles.end());

                refresh_navmesh(navigation_system, godot_world);
            }
        }
    }

    bool NavigationTestsSystem::try_create_random_obstacle(
        Float2 &min,
        Float2 &max,
        Float2 &min_size,
        Float2 &max_size,
        bool randomize_rotation,
        bool consider_indersection,
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        float x = random.next_float(min.x, max.x);
        float y = random.next_float(min.y, max.y);

        float dx = random.next_float(min_size.x, max_size.x);
        float dy = random.next_float(min_size.y, max_size.y);

        float rotation = 0.0f;
        if (randomize_rotation)
        {
            rotation = AngleUtils::degrees_to_radians(random.next_float(-180.0f, 180.0f));
        }

        Float2 center = Float2(x, y);
        Float3 scale = Float3(dx, 1.0f, dy);

        if (try_add_obstacle(
                center,
                scale,
                0.5f,
                -rotation,
                false,
                consider_indersection,
                navigation_system))
        {
            if (navigation_tests_scene.random_dynamic_obstacles_visible)
            {
                add_visible_obstacle(center,
                                     rotation,
                                     scale);
                obstacles_multimesh.set_buffer();
            }

            refresh_navmesh(navigation_system, godot_world);

            return true;
        }

        return false;
    }

    void NavigationTestsSystem::update_random_dynamic_agents(
        NavigationSystem &navigation_system,
        float delta)
    {
        int random_dynamic_agent_spawners_count = navigation_tests_scene.random_dynamic_agent_spawners.size();

        if (random_dynamic_agent_spawners_count > 0)
        {
            int number_of_spawners = 0;
            for (int i = 0; i < random_dynamic_agent_spawners_count; i++)
            {
                number_of_spawners += navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners.size();
            }

            vector<int> agent_counts_by_spawner_type;
            agent_counts_by_spawner_type.resize(number_of_spawners);

            for (int i = 0; i < number_of_spawners; i++)
            {
                agent_counts_by_spawner_type[i] = 0;
            }

            int random_dynamic_agents_count = navigation_tests_scene.random_dynamic_agents.size();

            for (int i = 0; i < random_dynamic_agents_count; i++)
            {
                int agent_type = navigation_tests_scene.random_dynamic_agents[i].agent_type;
                agent_counts_by_spawner_type[agent_type]++;
            }

            int spawner_index = 0;
            for (int i = 0; i < random_dynamic_agent_spawners_count; i++)
            {
                for (int j = 0; j < navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners.size(); j++)
                {
                    int number_to_spawn = navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners[j].number_to_spawn -
                                          agent_counts_by_spawner_type[spawner_index];

                    create_rectangular_spawner_agents(
                        navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners[j],
                        navigation_system,
                        number_to_spawn,
                        spawner_index,
                        true);
                    spawner_index++;
                }
            }

            int i_new = 0;
            bool removals_needed = false;
            random_dynamic_agents_count = navigation_tests_scene.random_dynamic_agents.size();

            vector<int> new_indices;
            new_indices.resize(random_dynamic_agents_count);

            float remove_probability = 0.001f;

            for (int i = 0; i < random_dynamic_agents_count; i++)
            {
                if (!navigation_system.HasPath(i) &&
                    random.next_float() < remove_probability)
                {
                    new_indices[i] = -1;
                    removals_needed = true;
                }
                else
                {
                    new_indices[i] = i_new;
                    i_new++;
                }
            }

            if (removals_needed)
            {
                agents_multimesh.remove_by_mask(new_indices, true);
                agents_multimesh.set_buffer();

                for (int i = 0; i < navigation_tests_scene.random_dynamic_agents.size(); i++)
                {
                    int new_index = new_indices[i];
                    if (new_index != -1)
                    {
                        navigation_tests_scene.random_dynamic_agents[new_index] = navigation_tests_scene.random_dynamic_agents[i];
                        navigation_system.agents[new_index] = navigation_system.agents[i];
                        navigation_system.agentPositions[new_index] = navigation_system.agentPositions[i];
                        navigation_system.agentNeighbours[new_index] = navigation_system.agentNeighbours[i];
                        navigation_system.agentNeighbourCounts[new_index] = navigation_system.agentNeighbourCounts[i];
                    }
                }

                navigation_tests_scene.random_dynamic_agents.erase(
                    navigation_tests_scene.random_dynamic_agents.begin() + i_new,
                    navigation_tests_scene.random_dynamic_agents.end());

                navigation_system.agents.erase(
                    navigation_system.agents.begin() + i_new,
                    navigation_system.agents.end());

                navigation_system.agentPositions.erase(
                    navigation_system.agentPositions.begin() + i_new,
                    navigation_system.agentPositions.end());

                navigation_system.agentNeighbours.erase(
                    navigation_system.agentNeighbours.begin() + i_new,
                    navigation_system.agentNeighbours.end());

                navigation_system.agentNeighbourCounts.erase(
                    navigation_system.agentNeighbourCounts.begin() + i_new,
                    navigation_system.agentNeighbourCounts.end());

                number_of_agents = i_new;
            }
        }
    }

    void NavigationTestsSystem::apply_scene(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world,
        const Scene &scene)
    {
        run_navigation_tests = scene.run_navigation_tests;

        if (run_navigation_tests)
        {
            current_scene_index = 0;
            create_all(navigation_system, godot_world);
        }
    }

    void NavigationTestsSystem::create_all(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        create_plane_mesh(godot_world);
        navigation_tests_scene.create_scene_properties(current_scene_index);
        create_obstacle_meshes();
        create_navmesh_obstacles(navigation_system, godot_world);
        create_random_obstacles(navigation_system, godot_world);
        refresh_navmesh(navigation_system, godot_world);
        create_all_agents(navigation_system);
    }

    void NavigationTestsSystem::create_plane_mesh(GodotWorld &godot_world)
    {
        string name = "NavigationTestsPlane";
        Rgba color = Rgba(0.5f, 0.6f, 0.2f, 1.0f);

        GodotStandardMaterial3DWrapper material;
        material.create();
        material.set_albedo(color);

        plane_mesh_instance.create(godot_world.node3D.node3D);

        plane_mesh_instance.set_mesh(GodotMeshUtils::get_quad_mesh_xz(100.0f, 100.0f, true));
        plane_mesh_instance.set_material_override(material.material);
        plane_mesh_instance.set_name(name);
    }

    void NavigationTestsSystem::create_obstacle_meshes()
    {
        int manual_obstacle_spawners_count = navigation_tests_scene.manual_obstacle_spawners.size();

        for (int i = 0; i < manual_obstacle_spawners_count; i++)
        {
            if (navigation_tests_scene.manual_obstacle_spawners[i].is_visible)
            {
                add_visible_obstacle(navigation_tests_scene.manual_obstacle_spawners[i].center,
                                     0.0f,
                                     navigation_tests_scene.manual_obstacle_spawners[i].size);
            }
        }

        int chained_obstacle_spawners_count = navigation_tests_scene.chained_obstacle_spawners.size();

        for (int i = 0; i < chained_obstacle_spawners_count; i++)
        {
            if (navigation_tests_scene.chained_obstacle_spawners[i].is_visible)
            {
                float obstacle_size = navigation_tests_scene.chained_obstacle_spawners[i].obstacle_size;
                float margin = 0.1f * obstacle_size;
                float obstacle_separation = obstacle_size - margin;
                Float3 scale = Float3(obstacle_size, obstacle_size, obstacle_size);

                for (int j = 0; j < navigation_tests_scene.chained_obstacle_spawners[i].number_to_spawn; j++)
                {
                    Float2 center = navigation_tests_scene.chained_obstacle_spawners[i].start +
                                    navigation_tests_scene.chained_obstacle_spawners[i].chain_direction * j * obstacle_separation;

                    add_visible_obstacle(center,
                                         0.0f,
                                         scale);
                }
            }
        }

        obstacles_multimesh.set_buffer();
    }

    void NavigationTestsSystem::add_visible_obstacle(Float2 &position_xz, float rotation, Float3 &scale)
    {
        int index_on_multimesh = obstacles_multimesh.get_used_instance_count();
        int multi_mesh_count = index_on_multimesh + 1;
        obstacles_multimesh.set_instance_count(multi_mesh_count);
        obstacles_multimesh.add_instance_to_buffer();

        Float3 position = Float3(position_xz.x, 1.0f, position_xz.y);
        Float3x3 basis;

        Float3 full_rotation = Float3(0.0f, rotation, 0.0f);

        basis.scale(scale);
        basis = Float3x3::rotated(full_rotation, basis);

        obstacles_multimesh.update_instance_position_basis(index_on_multimesh, position, basis);
        obstacles_multimesh.update_instance_color(index_on_multimesh, obstacle_color);
    }

    void NavigationTestsSystem::create_navmesh_obstacles(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        float epsilon = 0.001f;

        navigation_system.worldBounds = Aabb{
            .minX = -50.0f,
            .maxX = 50.0f,
            .minY = -50.0f,
            .maxY = 50.0f};

        navigation_system.paddedWorldBounds = Aabb{
            .minX = -50.0f + epsilon,
            .maxX = 50.0f - epsilon,
            .minY = -50.0f + epsilon,
            .maxY = 50.0f - epsilon};

        navigation_system.obstacles.clear();
        navigation_system.unextended_obstacles.clear();

        int manual_obstacle_spawners_count = navigation_tests_scene.manual_obstacle_spawners.size();

        for (int i = 0; i < manual_obstacle_spawners_count; i++)
        {
            try_add_obstacle(
                navigation_tests_scene.manual_obstacle_spawners[i].center,
                navigation_tests_scene.manual_obstacle_spawners[i].size,
                1.0f,
                0.0f,
                navigation_tests_scene.manual_obstacle_spawners[i].is_walkable,
                false,
                navigation_system);
        }

        int chained_obstacle_spawners_count = navigation_tests_scene.chained_obstacle_spawners.size();

        for (int i = 0; i < chained_obstacle_spawners_count; i++)
        {
            float obstacle_size = navigation_tests_scene.chained_obstacle_spawners[i].obstacle_size;
            float margin = 0.1f * obstacle_size;
            float obstacle_separation = obstacle_size - margin;
            Float3 scale = Float3(obstacle_size, obstacle_size, obstacle_size);

            for (int j = 0; j < navigation_tests_scene.chained_obstacle_spawners[i].number_to_spawn; j++)
            {
                Float2 center = navigation_tests_scene.chained_obstacle_spawners[i].start +
                                navigation_tests_scene.chained_obstacle_spawners[i].chain_direction * j * obstacle_separation;

                try_add_obstacle(
                    center,
                    scale,
                    margin,
                    0.0f,
                    false,
                    false,
                    navigation_system);
            }
        }
    }

    void NavigationTestsSystem::create_random_obstacles(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        for (int i = 0; i < navigation_tests_scene.random_obstacle_spawners.size(); i++)
        {
            random.set_seed(navigation_tests_scene.random_obstacle_spawners[i].seed);

            for (int j = 0; j < navigation_tests_scene.random_obstacle_spawners[i].number_to_spawn; j++)
            {
                try_create_random_obstacle(
                    navigation_tests_scene.random_obstacle_spawners[i].min,
                    navigation_tests_scene.random_obstacle_spawners[i].max,
                    navigation_tests_scene.random_obstacle_spawners[i].min_size,
                    navigation_tests_scene.random_obstacle_spawners[i].max_size,
                    navigation_tests_scene.random_obstacle_spawners[i].randomize_rotation,
                    false,
                    navigation_system,
                    godot_world);
            }
        }
    }

    bool NavigationTestsSystem::try_add_obstacle(
        Float2 &center,
        Float3 &size,
        float margin,
        float rotation,
        bool is_walkable,
        bool consider_intersection,
        NavigationSystem &navigation_system)
    {
        vector<Float2> corners;

        Float2 offset_before_rotation = Float2(0.0f, 0.0f);
        Float2 offset_after_rotation = center;

        ObstacleUtils::rectangle_with_offset_and_rotation(
            size.x * 0.5f + margin,
            size.z * 0.5f + margin,
            offset_before_rotation,
            offset_after_rotation,
            rotation,
            corners);

        Obstacle obstacle = ObstacleUtils::Create(
            corners,
            navigation_system.worldBounds,
            2.0f * 100.0f,
            is_walkable);

        if (consider_intersection && navigation_system.obstacles.size() > 0)
        {
            ObstaclesKdTree obstacles_kd_tree;
            obstacles_kd_tree.build(navigation_system.obstacles);

            if (obstacles_kd_tree.intersects(obstacle, navigation_system.obstacles))
            {
                return false;
            }
        }

        navigation_system.obstacles.push_back(obstacle);

        Obstacle unextended_obstacle = ObstacleUtils::Create(
            corners,
            navigation_system.worldBounds,
            2.0f * 100.0f,
            is_walkable);
        navigation_system.unextended_obstacles.push_back(unextended_obstacle);

        return true;
    }

    void NavigationTestsSystem::refresh_navmesh(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        navigation_system.navMesh.Create(navigation_system.obstacles, navigation_system.paddedWorldBounds);
        navigation_system.pathfinding.CreateNodes(navigation_system.navMesh);

        create_all_nav_mesh_drawers(navigation_system, godot_world);
    }

    void NavigationTestsSystem::create_all_nav_mesh_drawers(
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        float terrain_size = 100.0f;
        int resolution = 20;

        plane_nav_mesh_drawer.create_nav_mesh_drawers = true;
        plane_nav_mesh_drawer.create_obstacle_drawers = false;

        plane_nav_mesh_drawer.walkable_mesh_color = Rgba(0.0f, 0.4f, 0.8f, 1.0f);
        plane_nav_mesh_drawer.unwalkable_mesh_color = Rgba(0.8f, 0.4f, 0.0f, 1.0f);

        plane_nav_mesh_drawer.line_width = 0.2f;

        plane_nav_mesh_drawer.create_walkable_mesh_drawer(terrain_size, resolution, navigation_system.navMesh, godot_world);
        if (navigation_system.obstacles.size() > 0)
        {
            plane_nav_mesh_drawer.create_unwalkable_mesh_drawer(terrain_size, resolution, navigation_system.navMesh, godot_world);
            plane_nav_mesh_drawer.create_obstacles_mesh_drawer(terrain_size, resolution, navigation_system.obstacles, godot_world);
        }
        else
        {
            plane_nav_mesh_drawer.reset_unwalkable_mesh_drawer();
            plane_nav_mesh_drawer.reset_obstacles_mesh_drawer();
        }
    }

    void NavigationTestsSystem::create_all_agents(NavigationSystem &navigation_system)
    {
        number_of_agents = 0;

        for (int i = 0; i < navigation_tests_scene.manual_agent_rectangular_spawners.size(); i++)
        {
            random.set_seed(navigation_tests_scene.manual_agent_rectangular_spawners[i].seed);
            create_rectangular_spawner_agents(
                navigation_tests_scene.manual_agent_rectangular_spawners[i],
                navigation_system,
                navigation_tests_scene.manual_agent_rectangular_spawners[i].number_to_spawn,
                i,
                false);
        }

        for (int i = 0; i < navigation_tests_scene.manual_agent_circular_spawners.size(); i++)
        {
            random.set_seed(navigation_tests_scene.manual_agent_circular_spawners[i].seed);
            create_circular_spawner_agents(
                navigation_tests_scene.manual_agent_circular_spawners[i],
                navigation_system);
        }

        int agent_type_index = 0;
        for (int i = 0; i < navigation_tests_scene.random_dynamic_agent_spawners.size(); i++)
        {
            for (int j = 0; j < navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners.size(); j++)
            {
                random.set_seed(navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners[j].seed);
                create_rectangular_spawner_agents(
                    navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners[j],
                    navigation_system,
                    navigation_tests_scene.random_dynamic_agent_spawners[i].manual_agent_rectangular_spawners[j].number_to_spawn,
                    agent_type_index,
                    true);
                agent_type_index++;
            }
        }
    }

    void NavigationTestsSystem::create_rectangular_spawner_agents(
        ManualAgentRectangularSpawner &manual_agent_rectangular_spawner,
        NavigationSystem &navigation_system,
        int number_to_spawn,
        int agent_type,
        bool add_random_dynamic_agent)
    {
        for (int i = 0; i < number_to_spawn; i++)
        {
            int index_on_multimesh = agents_multimesh.get_used_instance_count();
            int multi_mesh_count = index_on_multimesh + 1;
            agents_multimesh.set_instance_count(multi_mesh_count);
            agents_multimesh.add_instance_to_buffer();

            float x = random.next_float(
                manual_agent_rectangular_spawner.center.x - 0.5f * manual_agent_rectangular_spawner.size.x,
                manual_agent_rectangular_spawner.center.x + 0.5f * manual_agent_rectangular_spawner.size.x);
            float y = random.next_float(
                manual_agent_rectangular_spawner.center.y - 0.5f * manual_agent_rectangular_spawner.size.y,
                manual_agent_rectangular_spawner.center.y + 0.5f * manual_agent_rectangular_spawner.size.y);

            Float2 position_xz = Float2(x, y);
            Float3 position = Float3(x, 1.0f, y);

            Float3 scale = Float3(1.0f, 1.0f, 1.0f);
            Float3x3 basis = Float3x3::from_scale(scale);

            agents_multimesh.update_instance_position_basis(index_on_multimesh, position, basis);
            agents_multimesh.update_instance_color(index_on_multimesh, manual_agent_rectangular_spawner.color);

            navigation_system.AddAgent(position_xz, manual_agent_rectangular_spawner.agent_type, -1, 5.0f);
            navigation_system.SetDestination(number_of_agents, manual_agent_rectangular_spawner.destination);

            if (add_random_dynamic_agent)
            {
                navigation_tests_scene.random_dynamic_agents.push_back(RandomDynamicAgent{
                    .agent_type = agent_type});
            }

            number_of_agents++;
        }

        agents_multimesh.set_buffer();
    }

    void NavigationTestsSystem::create_circular_spawner_agents(
        ManualAgentCircularSpawner &manual_agent_circular_spawner,
        NavigationSystem &navigation_system)
    {
        for (int i = 0; i < manual_agent_circular_spawner.number_to_spawn; i++)
        {
            int index_on_multimesh = agents_multimesh.get_used_instance_count();
            int multi_mesh_count = index_on_multimesh + 1;
            agents_multimesh.set_instance_count(multi_mesh_count);
            agents_multimesh.add_instance_to_buffer();

            Float2 position_xz = VectorUtils::random_inside_unit_circle(random) *
                                     manual_agent_circular_spawner.radius +
                                 manual_agent_circular_spawner.center;

            Float3 position = Float3(position_xz.x, 1.0f, position_xz.y);

            Float3 scale = Float3(1.0f, 1.0f, 1.0f);
            Float3x3 basis = Float3x3::from_scale(scale);

            agents_multimesh.update_instance_position_basis(index_on_multimesh, position, basis);
            agents_multimesh.update_instance_color(index_on_multimesh, manual_agent_circular_spawner.color);

            navigation_system.AddAgent(position_xz, manual_agent_circular_spawner.agent_type, -1, 5.0f);

            if (manual_agent_circular_spawner.set_destination_opposite_in_circle)
            {
                Float2 destination = -position_xz;
                navigation_system.SetDestination(number_of_agents, destination);
            }

            number_of_agents++;
        }

        agents_multimesh.set_buffer();
    }

    void NavigationTestsSystem::set_scene_to_load(
        int scene_index,
        NavigationSystem &navigation_system,
        GodotWorld &godot_world)
    {
        reset(navigation_system, false);

        current_scene_index = scene_index;
        create_all(navigation_system, godot_world);
    }

    void NavigationTestsSystem::reset(
        NavigationSystem &navigation_system,
        bool reset_plane_nav_mesh_drawer)
    {
        if (current_scene_index != -1)
        {
            navigation_system.reset();
            agents_multimesh.reset();
            obstacles_multimesh.reset();
            plane_mesh_instance.destroy();

            if (reset_plane_nav_mesh_drawer)
            {
                plane_nav_mesh_drawer.reset();
            }

            navigation_tests_scene.reset();

            current_scene_index = -1;
            number_of_agents = 0;
        }
    }
}
