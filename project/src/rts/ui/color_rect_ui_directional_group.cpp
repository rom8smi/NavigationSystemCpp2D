#include "color_rect_ui_directional_group.hpp"

namespace NavigationSystemCode
{
    void ColorRectUIDirectionalGroup::create(GodotControlWrapper &p_godot_control, int p_ui_elements_count)
    {
        ui_elements_count = p_ui_elements_count;
        ui_elements.resize(ui_elements_count);

        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].create(p_godot_control);
        }
    }

    void ColorRectUIDirectionalGroup::set_positions(Float2 &begin, Float2 &size, Float2 &gap)
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

    void ColorRectUIDirectionalGroup::set_color(Rgba &rgba)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].set_color(rgba);
        }
    }

    void ColorRectUIDirectionalGroup::set_visible(bool visible)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            set_element_visible(i, visible);
        }
    }

    void ColorRectUIDirectionalGroup::set_element_visible(int index, bool visible)
    {
        ui_elements[index].set_visible(visible);
    }

    bool ColorRectUIDirectionalGroup::is_element_visible(int index)
    {
        return ui_elements[index].is_visible;
    }
}
