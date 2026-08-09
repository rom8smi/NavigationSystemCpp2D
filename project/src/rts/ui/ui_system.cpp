#include "ui_system.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"
#include <string>
#include "rts/debug/debug.hpp"

using namespace std;

namespace NavigationSystemCode
{
    void UISystem::create(
        GodotNodeWrapper &p_main_component,
        GodotScreenSystem &godot_screen_system,
        GodotWorld &godot_world)
    {
        control.create(p_main_component.node);
        fps_counter_ui_system.create(control);
        navigation_tests_ui_system.create(control, godot_screen_system);
    }

    void UISystem::process(
        GodotScreenSystem &godot_screen_system,
        NavigationSystem &navigation_system,
        GodotWorld &godot_world,
        NavigationTestsSystem &navigation_tests_system,
        float delta)
    {
        fps_counter_ui_system.process(delta);

        navigation_tests_ui_system.process(
            navigation_tests_system,
            godot_screen_system,
            navigation_system,
            godot_world);
    }

    void UISystem::apply_scene(
        Scene &scene,
        GodotScreenSystem &godot_screen_system)
    {
        navigation_tests_ui_system.set_visible(scene.run_navigation_tests, godot_screen_system);
    }
}
