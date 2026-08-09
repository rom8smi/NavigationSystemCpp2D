#include "godot_rich_text_label_wrapper.hpp"
#include "godot_utils.hpp"

namespace NavigationSystemCode
{
    void GodotRichTextLabelWrapper::create(GodotControlWrapper &godot_control)
    {
        rich_text_label = memnew(RichTextLabel);
        godot_control.control->add_child(rich_text_label);
        is_visible = true;
    }

    void GodotRichTextLabelWrapper::set_begin(const Float2 &begin)
    {
        rich_text_label->set_begin(GodotUtils::to_vector2(begin));
    }

    void GodotRichTextLabelWrapper::set_end(const Float2 &end)
    {
        rich_text_label->set_end(GodotUtils::to_vector2(end));
    }

    void GodotRichTextLabelWrapper::set_text(const string &text)
    {
        rich_text_label->set_text(text.c_str());
    }

    void GodotRichTextLabelWrapper::set_font_size(int font_size)
    {
        rich_text_label->add_theme_font_size_override("normal_font_size", font_size);
    }

    void GodotRichTextLabelWrapper::set_text_color(const Rgba &rgba)
    {
        rich_text_label->add_theme_color_override("default_color", GodotUtils::to_color(rgba));
    }

    void GodotRichTextLabelWrapper::set_use_text_bbcode(bool value)
    {
        rich_text_label->set_use_bbcode(value);
    }

    void GodotRichTextLabelWrapper::set_vertical_alignment_center()
    {
        rich_text_label->set_vertical_alignment(VerticalAlignment::VERTICAL_ALIGNMENT_CENTER);
    }

    void GodotRichTextLabelWrapper::set_visible(bool visible)
    {
        is_visible = visible;
        rich_text_label->set_visible(visible);
    }
}
