#ifndef GODOT_CONTROL_WRAPPER_HPP
#define GODOT_CONTROL_WRAPPER_HPP

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/control.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotControlWrapper
    {
        Control *control;

        void create(Node *p_main_component);
    };
}

#endif
