#ifndef COLOR_RECT_UI_DIRECTIONAL_GROUP_HPP
#define COLOR_RECT_UI_DIRECTIONAL_GROUP_HPP

#include <vector>
#include "rts/godot_wrapper/godot_color_rect_wrapper.hpp"

namespace NavigationSystemCode
{
    struct ColorRectUIDirectionalGroup
    {
        vector<GodotColorRectWrapper> ui_elements;
        int ui_elements_count;

        void create(GodotControlWrapper &p_godot_control, int p_ui_elements_count);
        void set_positions(Float2 &begin, Float2 &size, Float2 &gap);
        void set_color(Rgba &rgba);
        void set_visible(bool visible);
        void set_element_visible(int index, bool visible);
        bool is_element_visible(int index);
    };
}

#endif
