#ifndef GODOT_DIRECTIONAL_LIGHT3D_WRAPPER_HPP
#define GODOT_DIRECTIONAL_LIGHT3D_WRAPPER_HPP

#include <godot_cpp/classes/directional_light3d.hpp>
#include "rts/math/float3.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotDirectionalLight3DWrapper
    {
        DirectionalLight3D *directional_light3d;

        void create(Node *parent);
        void set_rotation(const Float3 &rotation);
        void set_name(const string &name);
    };
}

#endif
