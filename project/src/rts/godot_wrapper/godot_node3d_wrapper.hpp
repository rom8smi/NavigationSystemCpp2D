#ifndef GODOT_NODE3D_WRAPPER_HPP
#define GODOT_NODE3D_WRAPPER_HPP

#include <godot_cpp/classes/node3d.hpp>
#include <string>

using namespace godot;
using namespace std;

namespace NavigationSystemCode
{
    struct GodotNode3DWrapper
    {
        Node3D *node3D;

        void create(Node *parent);
        void set_name(const string &name);
    };
}

#endif
