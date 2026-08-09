#ifndef GODOT_BOX_MESH_WRAPPER_HPP
#define GODOT_BOX_MESH_WRAPPER_HPP

#include <godot_cpp/classes/box_mesh.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotBoxMeshWrapper
    {
        godot::Mesh *mesh;

        void create();
    };
}

#endif
