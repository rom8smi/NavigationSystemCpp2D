#ifndef GODOT_RICH_TEXT_LABEL_WRAPPER_HPP
#define GODOT_RICH_TEXT_LABEL_WRAPPER_HPP

#include "godot_control_wrapper.hpp"
#include "rts/math/float2.hpp"
#include "rts/math/rgba.hpp"
#include <godot_cpp/classes/rich_text_label.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotRichTextLabelWrapper
    {
        RichTextLabel *rich_text_label;
        bool is_visible;

        void create(GodotControlWrapper &godot_control);
        void set_begin(const Float2 &begin);
        void set_end(const Float2 &end);
        void set_text(const string &text);
        void set_font_size(int font_size);
        void set_text_color(const Rgba &rgba);
        void set_use_text_bbcode(bool value);
        void set_vertical_alignment_center();
        void set_visible(bool visible);
    };
}

#endif
