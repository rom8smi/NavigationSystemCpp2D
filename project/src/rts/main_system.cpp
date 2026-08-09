#include "main_system.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/debug/debug.hpp"

namespace NavigationSystemCode
{
    MainSystem::MainSystem() {}

    MainSystem::MainSystem(GodotWorld p_godot_world)
    {
        godot_world = p_godot_world;
        Debug::clear();
    }

    void MainSystem::ready()
    {
        scene_system.load_navigation_tests_scene();

        godot_screen_system.ready(godot_world);
        godot_world.create_world();

        navigation_system.ready(godot_world);

        ui_system.create(
            godot_world.main_component,
            godot_screen_system,
            godot_world);
        navigation_tests_system.ready(godot_world.node3D);

        set_from_loaded_scene();
    }

    void MainSystem::process(float delta)
    {
        godot_screen_system.process(godot_world);

        navigation_system.process(
            delta);

        navigation_tests_system.process(navigation_system, godot_world, delta);

        ui_system.process(
            godot_screen_system,
            navigation_system,
            godot_world,
            navigation_tests_system,
            delta);
    }

    void MainSystem::set_from_loaded_scene()
    {
        godot_world.set_camera_position(scene_system.scene.camera_position);

        godot_world.set_camera_rotation(scene_system.scene.camera_rotation);
        godot_world.apply_scene(scene_system.scene);

        ui_system.apply_scene(
            scene_system.scene,
            godot_screen_system);

        navigation_tests_system.apply_scene(navigation_system, godot_world, scene_system.scene);
    }

    void MainSystem::reset()
    {
        navigation_system.reset();
    }
}
