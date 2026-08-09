#ifndef GODOT_CAPSULE_MESH_WRAPPER_HPP
#define GODOT_CAPSULE_MESH_WRAPPER_HPP

#include <godot_cpp/classes/capsule_mesh.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotCapsuleMeshWrapper
    {
        godot::Mesh *mesh;

        void create();
    };
}

#endif
