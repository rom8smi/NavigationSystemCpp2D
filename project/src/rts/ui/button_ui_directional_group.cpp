#include "button_ui_directional_group.hpp"
#include "ui_button_press_mode.hpp"

namespace NavigationSystemCode
{
    void ButtonUIDirectionalGroup::create(GodotControlWrapper &p_godot_control, int p_ui_elements_count)
    {
        ui_elements_count = p_ui_elements_count;
        ui_elements.resize(ui_elements_count);

        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].create(p_godot_control);
        }
    }

    void ButtonUIDirectionalGroup::set_positions(Float2 &begin, Float2 &size, Float2 &gap)
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

    bool ButtonUIDirectionalGroup::is_any_button_hovered(const Float2 &mouse_position)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            if (ui_elements[i].is_hovered(mouse_position))
            {
                return true;
            }
        }
        return false;
    }

    int ButtonUIDirectionalGroup::get_hovered_button_index(const Float2 &mouse_position)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            if (ui_elements[i].is_hovered(mouse_position))
            {
                return i;
            }
        }
        return -1;
    }

    void ButtonUIDirectionalGroup::set_visible(bool visible)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            ui_elements[i].set_visible(visible);
        }
    }

    void ButtonUIDirectionalGroup::set_element_visible(int index, bool visible)
    {
        ui_elements[index].set_visible(visible);
    }

    bool ButtonUIDirectionalGroup::is_element_visible(int index)
    {
        return ui_elements[index].is_visible;
    }

    int ButtonUIDirectionalGroup::get_visible_elements_count()
    {
        int count = 0;
        for (int i = 0; i < ui_elements_count; i++)
        {
            if (is_element_visible(i))
            {
                count++;
            }
        }
        return count;
    }

    int ButtonUIDirectionalGroup::get_visible_element_index(int index)
    {
        int count = -1;

        for (int i = 0; i < ui_elements_count; i++)
        {
            if (is_element_visible(i))
            {
                count++;
                if (i == index)
                {
                    return count;
                }
            }
        }

        return -1;
    }

    void ButtonUIDirectionalGroup::process(int &press_mode, int &index)
    {
        for (int i = 0; i < ui_elements_count; i++)
        {
            int button_press_mode;
            ui_elements[i].process(button_press_mode);

            if (button_press_mode != UIButtonPressMode::UNPRESSED)
            {
                press_mode = button_press_mode;
                index = i;
                return;
            }
        }

        press_mode = UIButtonPressMode::UNPRESSED;
        index = -1;
    }
}
