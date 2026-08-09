#include "text_ui_directional_group.hpp"

namespace NavigationSystemCode
{
    void TextUIDirectionalGroup::create(GodotControlWrapper &p_godot_control, int p_ui_elements_count)
    {
        ui_elements_count = p_ui_elements_count;
        ui_elements.resize(ui_elements_count);

        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].create(p_godot_control);
        }
    }

    void TextUIDirectionalGroup::set_positions(Float2 &begin, Float2 &size, Float2 &gap)
    {
        int element_index = 0;

        for (int i = 0; i < ui_elements_count; i++)
        {
            if (is_element_visible(i))
            {
                Float2 ui_element_begin = begin + gap * element_index;
                Float2 ui_element_end = ui_element_begin + size;

                ui_elements[i].set_begin(ui_element_begin);
                ui_elements[i].set_end(ui_element_end);

                element_index++;
            }
        }
    }

    void TextUIDirectionalGroup::set_text(const string &text, int index)
    {
        ui_elements[index].set_text(text);
    }

    void TextUIDirectionalGroup::set_font_size(int font_size)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].set_font_size(font_size);
        }
    }

    void TextUIDirectionalGroup::set_text_color(Rgba &rgba)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            set_element_text_color(i, rgba);
        }
    }

    void TextUIDirectionalGroup::set_element_text_color(int index, Rgba &rgba)
    {
        ui_elements[index].set_text_color(rgba);
    }

    void TextUIDirectionalGroup::set_use_text_bbcode(bool value)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].set_use_text_bbcode(value);
        }
    }

    void TextUIDirectionalGroup::set_vertical_alignment_center()
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].set_vertical_alignment_center();
        }
    }

    void TextUIDirectionalGroup::set_visible(bool visible)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            set_element_visible(i, visible);
        }
    }

    void TextUIDirectionalGroup::set_element_visible(int index, bool visible)
    {
        ui_elements[index].set_visible(visible);
    }

    bool TextUIDirectionalGroup::is_element_visible(int index)
    {
        return ui_elements[index].is_visible;
    }
}
