#ifndef BUTTON_UI_DIRECTIONAL_GROUP_HPP
#define BUTTON_UI_DIRECTIONAL_GROUP_HPP

#include <vector>
#include "rts/godot_wrapper/godot_button_wrapper.hpp"

namespace NavigationSystemCode
{
    struct ButtonUIDirectionalGroup
    {
        vector<GodotButtonWrapper> ui_elements;
        int ui_elements_count;

        void create(GodotControlWrapper &p_godot_control, int p_ui_elements_count);
        void set_positions(Float2 &begin, Float2 &size, Float2 &gap);
        bool is_any_button_hovered(const Float2 &mouse_position);
        int get_hovered_button_index(const Float2 &mouse_position);
        void set_visible(bool visible);
        void set_element_visible(int index, bool visible);
        bool is_element_visible(int index);
        int get_visible_elements_count();
        int get_visible_element_index(int index);
        void process(int &press_mode, int &index);
    };
}

#endif
