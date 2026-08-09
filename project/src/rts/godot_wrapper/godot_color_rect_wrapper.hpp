#ifndef GODOT_COLOR_RECT_WRAPPER_HPP
#define GODOT_COLOR_RECT_WRAPPER_HPP

#include "godot_control_wrapper.hpp"
#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"
#include <godot_cpp/classes/color_rect.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotColorRectWrapper
    {
        ColorRect *color_rect;
        bool is_visible;

        void create(GodotControlWrapper &godot_control);
        void set_begin(const Float2 &begin);
        void set_end(const Float2 &end);
        void set_color(const Rgba &rgba);
        void set_visible(bool visible);
    };
}

#endif
