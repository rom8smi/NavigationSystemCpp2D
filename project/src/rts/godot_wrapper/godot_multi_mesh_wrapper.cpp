#include "godot_multi_mesh_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotMultiMeshWrapper::create()
    {
        multi_mesh = memnew(MultiMesh);
    }

    void GodotMultiMeshWrapper::set_use_colors(bool value)
    {
        multi_mesh->set_use_colors(value);
    }

    void GodotMultiMeshWrapper::set_use_custom_data(bool value)
    {
        multi_mesh->set_use_custom_data(value);
    }

    void GodotMultiMeshWrapper::set_instance_count(int value)
    {
        multi_mesh->set_instance_count(value);
    }

    void GodotMultiMeshWrapper::set_visible_instance_count(int value)
    {
        multi_mesh->set_visible_instance_count(value);
    }

    void GodotMultiMeshWrapper::set_buffer(const PackedFloat32Array &buffer)
    {
        multi_mesh->set_buffer(buffer);
    }

    PackedFloat32Array GodotMultiMeshWrapper::get_buffer()
    {
        return multi_mesh->get_buffer();
    }

    void GodotMultiMeshWrapper::set_transform_format_3d()
    {
        multi_mesh->set_transform_format(MultiMesh::TRANSFORM_3D);
    }

    void GodotMultiMeshWrapper::set_mesh(const Ref<godot::Mesh> &mesh)
    {
        multi_mesh->set_mesh(mesh);
    }

    void GodotMultiMeshWrapper::set_name(const string &name)
    {
        multi_mesh->set_name(name.c_str());
    }
}
