#include "godot_button_wrapper.hpp"
#include "godot_utils.hpp"
#include "rts/ui/ui_button_press_mode.hpp"

namespace NavigationSystemCode
{
    void GodotButtonWrapper::create(GodotControlWrapper &godot_control)
    {
        previous_pressed = false;
        button = memnew(BaseButton);
        godot_control.control->add_child(button);
        begin = Float2(0.0f, 0.0f);
        end = Float2(0.0f, 0.0f);
        is_visible = true;
    }

    void GodotButtonWrapper::set_begin(Float2 &p_begin)
    {
        begin = p_begin;
        button->set_begin(GodotUtils::to_vector2(begin));
    }

    void GodotButtonWrapper::set_end(Float2 &p_end)
    {
        end = p_end;
        button->set_end(GodotUtils::to_vector2(end));
    }

    void GodotButtonWrapper::set_visible(bool visible)
    {
        is_visible = visible;
        button->set_visible(visible);
    }

    void GodotButtonWrapper::process(int &press_mode)
    {
        bool pressed = button->is_pressed();
        if (!previous_pressed && pressed)
        {
            press_mode = UIButtonPressMode::PRESSED_DOWN;
        }
        else if (previous_pressed && !pressed)
        {
            press_mode = UIButtonPressMode::PRESSED_UP;
        }
        else if (pressed)
        {
            press_mode = UIButtonPressMode::PRESSED;
        }
        else
        {
            press_mode = UIButtonPressMode::UNPRESSED;
        }

        previous_pressed = pressed;
    }

    bool GodotButtonWrapper::is_hovered(const Float2 &mouse_position)
    {
        if (is_visible)
        {
            float mouse_x = mouse_position.x;
            float mouse_y = mouse_position.y;

            if (mouse_x > begin.x && mouse_x < end.x && mouse_y > begin.y && mouse_y < end.y)
            {
                return true;
            }
        }
        return false;
    }
}
