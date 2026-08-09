#ifndef GODOT_MESH_INSTANCE3D_WRAPPER_HPP
#define GODOT_MESH_INSTANCE3D_WRAPPER_HPP

#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/material.hpp>
#include <string>
#include "rts/math/float3.hpp"

using namespace godot;
using namespace std;

namespace NavigationSystemCode
{
    struct GodotMeshInstance3DWrapper
    {
        MeshInstance3D *mesh_instance3d;

        void create(Node *parent);
        void set_mesh(Ref<godot::Mesh> mesh);
        void set_mesh(godot::Mesh *mesh);
        Ref<godot::Material> get_active_material(int p_surface);
        void set_material_override(Ref<Material> material);
        void set_material_override(Material *material);
        void set_surface_override_material(int p_surface, const Ref<Material> &p_material);
        void set_position(const Float3 &position);
        void set_rotation(const Float3 &radians);
        void set_name(const string &name);
        void set_visible(const bool visible);
        void destroy();
    };
}

#endif
