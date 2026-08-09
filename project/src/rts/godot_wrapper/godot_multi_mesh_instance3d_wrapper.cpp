#include "godot_multi_mesh_instance3d_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotMultiMeshInstance3DWrapper::create(Node *parent)
    {
        multi_mesh_instance3d = memnew(MultiMeshInstance3D);
        parent->add_child(multi_mesh_instance3d);
    }

    void GodotMultiMeshInstance3DWrapper::set_multimesh(MultiMesh *multi_mesh)
    {
        multi_mesh_instance3d->set_multimesh(multi_mesh);
    }

    void GodotMultiMeshInstance3DWrapper::set_material_override(Material *material)
    {
        multi_mesh_instance3d->set_material_override(material);
    }

    void GodotMultiMeshInstance3DWrapper::set_material_override(Ref<Material> material)
    {
        multi_mesh_instance3d->set_material_override(material);
    }

    void GodotMultiMeshInstance3DWrapper::set_lod_bias(float value)
    {
        multi_mesh_instance3d->set_lod_bias(value);
    }

    void GodotMultiMeshInstance3DWrapper::set_ignore_occlusion_culling(bool value)
    {
        multi_mesh_instance3d->set_ignore_occlusion_culling(value);
    }

    void GodotMultiMeshInstance3DWrapper::set_name(const string &name)
    {
        multi_mesh_instance3d->set_name(name.c_str());
    }

    void GodotMultiMeshInstance3DWrapper::set_visible(bool visible)
    {
        multi_mesh_instance3d->set_visible(visible);
    }
}
