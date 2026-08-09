#ifndef GODOT_MULTI_MESH_INSTANCE3D_WRAPPER_HPP
#define GODOT_MULTI_MESH_INSTANCE3D_WRAPPER_HPP

#include <godot_cpp/classes/multi_mesh.hpp>
#include <godot_cpp/classes/multi_mesh_instance3d.hpp>
#include <godot_cpp/classes/material.hpp>
#include <string>

using namespace godot;
using namespace std;

namespace NavigationSystemCode
{
    struct GodotMultiMeshInstance3DWrapper
    {
        MultiMeshInstance3D *multi_mesh_instance3d;

        void create(Node *parent);
        void set_multimesh(MultiMesh *multi_mesh);
        void set_material_override(Material *material);
        void set_material_override(Ref<Material> material);
        void set_lod_bias(float value);
        void set_ignore_occlusion_culling(bool value);
        void set_name(const string &name);
        void set_visible(bool visible);
    };
}

#endif
