#include <godot_cpp/classes/display_server.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include "godot_screen_system.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    void GodotScreenSystem::ready(GodotWorld &godot_world)
    {
        update_screen_size(godot_world);
    }

    void GodotScreenSystem::process(GodotWorld &godot_world)
    {
        update_screen_size(godot_world);
    }

    void GodotScreenSystem::update_screen_size(GodotWorld &godot_world)
    {
        auto visible_rect_size = godot_world.main_component.node->get_viewport()->get_visible_rect().size;
        window_width = visible_rect_size.x;
        window_height = visible_rect_size.y;

        auto screen_size = DisplayServer::get_singleton()->screen_get_size();

        width = screen_size.x;
        if (width < window_width)
        {
            width = window_width;
        }

        height = screen_size.y;
        if (height < window_height)
        {
            height = window_height;
        }

        mean_size = 0.5 * (width + height);
        mean_size_reciprocal = 1.0 / mean_size;
    }

    float GodotScreenSystem::to_absolute_width(float relative_width)
    {
        return relative_width * window_width;
    }

    float GodotScreenSystem::to_absolute_height(float relative_height)
    {
        return relative_height * window_height;
    }

    bool GodotScreenSystem::is_on_screen(
        Float3 &position,
        Float3 &camera_position,
        Float3 &camera_look_direction,
        GodotWorld &godot_world)
    {
        Float2 screen_position = godot_world.world_to_screen_position(position);

        if (screen_position.x >= 0.0f &&
            screen_position.x <= width &&
            screen_position.y >= 0.0f &&
            screen_position.y <= height &&
            !is_behind_camera(position, camera_position, camera_look_direction))
        {
            return true;
        }
        return false;
    }

    bool GodotScreenSystem::is_behind_camera(
        Float3 &position,
        Float3 &camera_position,
        Float3 &camera_look_direction)
    {
        Float3 direction = (position - camera_position).normalized();
        if (camera_look_direction.dot(direction) < 0.0f)
        {
            return true;
        }

        return false;
    }
}
