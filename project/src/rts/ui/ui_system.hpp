#ifndef UI_SYSTEM_HPP
#define UI_SYSTEM_HPP

#include <godot_cpp/classes/node.hpp>
#include "rts/godot_wrapper/godot_control_wrapper.hpp"
#include "fps_counter_ui_system.hpp"
#include "navigation_tests_ui_system.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    struct UISystem
    {
        GodotControlWrapper control;
        FPSCounterUISystem fps_counter_ui_system;
        NavigationTestsUISystem navigation_tests_ui_system;

        void create(
            GodotNodeWrapper &p_main_component,
            GodotScreenSystem &godot_screen_system,
            GodotWorld &godot_world);
        void process(
            GodotScreenSystem &godot_screen_system,
            NavigationSystem &navigation_system,
            GodotWorld &godot_world,
            NavigationTestsSystem &navigation_tests_system,
            float delta);
        void apply_scene(
            Scene &scene,
            GodotScreenSystem &godot_screen_system);
    };
}

#endif
