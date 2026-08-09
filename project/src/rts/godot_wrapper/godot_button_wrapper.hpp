#ifndef GODOT_BUTTON_WRAPPER_HPP
#define GODOT_BUTTON_WRAPPER_HPP

#include "godot_control_wrapper.hpp"
#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"
#include <godot_cpp/classes/base_button.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotButtonWrapper
    {
        BaseButton *button;
        bool previous_pressed;
        Float2 begin;
        Float2 end;
        bool is_visible;

        void create(GodotControlWrapper &godot_control);
        void set_begin(Float2 &p_begin);
        void set_end(Float2 &p_end);
        void set_visible(bool visible);
        void process(int &press_mode);
        bool is_hovered(const Float2 &mouse_position);
    };
}

#endif
