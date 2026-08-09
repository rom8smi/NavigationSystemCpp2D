#ifndef TEXT_UI_DIRECTIONAL_GROUP_HPP
#define TEXT_UI_DIRECTIONAL_GROUP_HPP

#include <vector>
#include "rts/godot_wrapper/godot_rich_text_label_wrapper.hpp"

namespace NavigationSystemCode
{
    struct TextUIDirectionalGroup
    {
        vector<GodotRichTextLabelWrapper> ui_elements;
        int ui_elements_count;

        void create(GodotControlWrapper &p_godot_control, int p_ui_elements_count);
        void set_positions(Float2 &begin, Float2 &size, Float2 &gap);
        void set_text(const string &text, int index);
        void set_font_size(int font_size);
        void set_text_color(Rgba &rgba);
        void set_element_text_color(int index, Rgba &rgba);
        void set_use_text_bbcode(bool value);
        void set_vertical_alignment_center();
        void set_visible(bool visible);
        void set_element_visible(int index, bool visible);
        bool is_element_visible(int index);
    };
}

#endif
