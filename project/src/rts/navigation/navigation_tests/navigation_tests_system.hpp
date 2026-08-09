#ifndef NAVIGATION_TESTS_SYSTEM_HPP
#define NAVIGATION_TESTS_SYSTEM_HPP

#include "rts/navigation/navigation_system.hpp"
#include "manual_obstacle_spawner.hpp"
#include "manual_agent_rectangular_spawner.hpp"
#include "navigation_tests_scene.hpp"
#include "rts/navigation/drawers/plane_nav_mesh_drawer.hpp"
#include "rts/scene/scene.hpp"
#include "rts/godot_wrapper/godot_capsule_mesh_wrapper.hpp"
#include "rts/godot_wrapper/godot_box_mesh_wrapper.hpp"
#include "rts/godot_wrapper/godot_multimesh.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct NavigationTestsSystem
    {
        bool run_navigation_tests;
        GodotCapsuleMeshWrapper capsule_mesh;
        GodotBoxMeshWrapper box_mesh;
        GodotMultimesh agents_multimesh;
        GodotMultimesh obstacles_multimesh;
        Random random;
        PlaneNavMeshDrawer plane_nav_mesh_drawer;
        Rgba obstacle_color;
        NavigationTestsScene navigation_tests_scene;
        GodotMeshInstance3DWrapper plane_mesh_instance;

        int current_scene_index;
        int number_of_agents;

        void ready(GodotNode3DWrapper &node3D);
        void create_standard_multimesh(
            GodotMultimesh &multimesh,
            GodotNode3DWrapper &node3D,
            godot::Mesh *mesh,
            const string &multimesh_name);
        void process(
            NavigationSystem &navigation_system,
            GodotWorld &godot_world,
            float delta);
        void update_random_dynamic_obstacles(
            NavigationSystem &navigation_system,
            GodotWorld &godot_world,
            float delta);
        bool try_create_random_obstacle(
            Float2 &min,
            Float2 &max,
            Float2 &min_size,
            Float2 &max_size,
            bool randomize_rotation,
            bool consider_indersection,
            NavigationSystem &navigation_system,
            GodotWorld &godot_world);
        void update_random_dynamic_agents(
            NavigationSystem &navigation_system,
            float delta);
        void apply_scene(
            NavigationSystem &navigation_system,
            GodotWorld &godot_world,
            const Scene &scene);
        void create_all(
            NavigationSystem &navigation_system,
            GodotWorld &godot_world);
        void create_plane_mesh(GodotWorld &godot_world);
        void create_obstacle_meshes();
        void add_visible_obstacle(Float2 &position_xz, float rotation, Float3 &scale);
        void create_navmesh_obstacles(
            NavigationSystem &navigation_system,
            GodotWorld &godot_world);
        void create_random_obstacles(
            NavigationSystem &navigation_system,
            GodotWorld &godot_world);
        bool try_add_obstacle(
            Float2 &center,
            Float3 &size,
            float margin,
            float rotation,
            bool is_walkable,
            bool consider_intersection,
            NavigationSystem &navigation_system);
        void refresh_navmesh(NavigationSystem &navigation_system, GodotWorld &godot_world);
        void create_all_nav_mesh_drawers(NavigationSystem &navigation_system, GodotWorld &godot_world);
        void create_all_agents(NavigationSystem &navigation_system);
        void create_rectangular_spawner_agents(
            ManualAgentRectangularSpawner &manual_agent_rectangular_spawner,
            NavigationSystem &navigation_system,
            int number_to_spawn,
            int agent_type,
            bool add_random_dynamic_agent);
        void create_circular_spawner_agents(
            ManualAgentCircularSpawner &manual_agent_circular_spawner,
            NavigationSystem &navigation_system);
        void set_scene_to_load(
            int scene_index,
            NavigationSystem &navigation_system,
            GodotWorld &godot_world);
        void reset(
            NavigationSystem &navigation_system,
            bool reset_plane_nav_mesh_drawer);
    };
}

#endif
