#ifndef NAVIGATION_TESTS_UI_SYSTEM_HPP
#define NAVIGATION_TESTS_UI_SYSTEM_HPP

#include "rts/navigation/navigation_tests/navigation_tests_system.hpp"
#include "rts/godot_wrapper/godot_control_wrapper.hpp"
#include "rts/godot_wrapper/godot_screen_system.hpp"
#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"
#include "color_rect_ui_directional_group.hpp"
#include "text_ui_directional_group.hpp"
#include "button_ui_directional_group.hpp"

namespace NavigationSystemCode
{
    struct NavigationTestsUISystem
    {
        ColorRectUIDirectionalGroup button_colors;
        TextUIDirectionalGroup button_texts;
        ButtonUIDirectionalGroup buttons;

        int number_of_buttons;
        vector<string> button_names;
        vector<int> scene_indices;
        float last_window_width;
        float last_window_height;

        void create(
            GodotControlWrapper &godot_control,
            GodotScreenSystem &godot_screen_system);
        void set_button_and_scene_references();
        void create_buttons(
            GodotControlWrapper &godot_control,
            GodotScreenSystem &godot_screen_system);
        void process(
            NavigationTestsSystem &navigation_tests_system,
            GodotScreenSystem &godot_screen_system,
            NavigationSystem &navigation_system,
            GodotWorld &godot_world);
        void update_buttons_size(GodotScreenSystem &godot_screen_system);
        void set_visible(bool visible, GodotScreenSystem &godot_screen_system);
    };
}

#endif
