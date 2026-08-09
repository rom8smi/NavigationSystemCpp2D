#ifndef GODOT_MULTI_MESH_WRAPPER_HPP
#define GODOT_MULTI_MESH_WRAPPER_HPP

#include <godot_cpp/classes/multi_mesh.hpp>
#include <string>

using namespace godot;
using namespace std;

namespace NavigationSystemCode
{
    struct GodotMultiMeshWrapper
    {
        MultiMesh *multi_mesh;

        void create();
        void set_use_colors(bool value);
        void set_use_custom_data(bool value);
        void set_instance_count(int value);
        void set_visible_instance_count(int value);
        void set_buffer(const PackedFloat32Array &buffer);
        PackedFloat32Array get_buffer();
        void set_transform_format_3d();
        void set_mesh(const Ref<godot::Mesh> &mesh);
        void set_name(const string &name);
    };
}

#endif
