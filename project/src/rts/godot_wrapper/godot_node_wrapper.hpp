#ifndef GODOT_NODE_WRAPPER_HPP
#define GODOT_NODE_WRAPPER_HPP

#include <godot_cpp/classes/node.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotNodeWrapper
    {
        Node *node;
    };
}

#endif
