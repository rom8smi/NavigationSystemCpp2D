#ifndef GODOT_SCREEN_SYSTEM_HPP
#define GODOT_SCREEN_SYSTEM_HPP

#include "rts/godot_wrapper/godot_world.hpp"

namespace NavigationSystemCode
{
    struct GodotScreenSystem
    {
        int width;
        int height;
        float mean_size;
        float mean_size_reciprocal;
        float window_width;
        float window_height;

        void ready(GodotWorld &godot_world);
        void process(GodotWorld &godot_world);
        void update_screen_size(GodotWorld &godot_world);
        float to_absolute_width(float relative_width);
        float to_absolute_height(float relative_height);
        bool is_on_screen(
            Float3 &position,
            Float3 &camera_position,
            Float3 &camera_look_direction,
            GodotWorld &godot_world);
        bool is_behind_camera(
            Float3 &position,
            Float3 &camera_position,
            Float3 &camera_look_direction);
    };
}

#endif
