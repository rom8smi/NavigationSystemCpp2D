#include "godot_color_rect_wrapper.hpp"
#include "godot_utils.hpp"

namespace NavigationSystemCode
{
    void GodotColorRectWrapper::create(GodotControlWrapper &godot_control)
    {
        color_rect = memnew(ColorRect);
        godot_control.control->add_child(color_rect);
        is_visible = true;
    }

    void GodotColorRectWrapper::set_begin(const Float2 &begin)
    {
        color_rect->set_begin(GodotUtils::to_vector2(begin));
    }

    void GodotColorRectWrapper::set_end(const Float2 &end)
    {
        color_rect->set_end(GodotUtils::to_vector2(end));
    }

    void GodotColorRectWrapper::set_color(const Rgba &rgba)
    {
        color_rect->set_color(GodotUtils::to_color(rgba));
    }

    void GodotColorRectWrapper::set_visible(bool visible)
    {
        is_visible = true;
        color_rect->set_visible(visible);
    }
}
