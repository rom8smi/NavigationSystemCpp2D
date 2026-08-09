#include "godot_mesh_instance3d_wrapper.hpp"
#include "godot_utils.hpp"

namespace NavigationSystemCode
{
    void GodotMeshInstance3DWrapper::create(Node *parent)
    {
        mesh_instance3d = memnew(MeshInstance3D);
        parent->add_child(mesh_instance3d);
    }

    void GodotMeshInstance3DWrapper::set_mesh(Ref<godot::Mesh> mesh)
    {
        mesh_instance3d->set_mesh(mesh);
    }

    void GodotMeshInstance3DWrapper::set_mesh(godot::Mesh *mesh)
    {
        mesh_instance3d->set_mesh(mesh);
    }

    Ref<godot::Material> GodotMeshInstance3DWrapper::get_active_material(int p_surface)
    {
        return mesh_instance3d->get_active_material(p_surface);
    }

    void GodotMeshInstance3DWrapper::set_material_override(Ref<Material> material)
    {
        mesh_instance3d->set_material_override(material);
    }

    void GodotMeshInstance3DWrapper::set_material_override(Material *material)
    {
        mesh_instance3d->set_material_override(material);
    }

    void GodotMeshInstance3DWrapper::set_surface_override_material(int p_surface, const Ref<Material> &p_material)
    {
        mesh_instance3d->set_surface_override_material(p_surface, p_material);
    }

    void GodotMeshInstance3DWrapper::set_position(const Float3 &position)
    {
        mesh_instance3d->set_position(GodotUtils::to_vector3(position));
    }

    void GodotMeshInstance3DWrapper::set_rotation(const Float3 &radians)
    {
        mesh_instance3d->set_rotation(GodotUtils::to_vector3(radians));
    }

    void GodotMeshInstance3DWrapper::set_name(const string &name)
    {
        mesh_instance3d->set_name(name.c_str());
    }

    void GodotMeshInstance3DWrapper::set_visible(const bool visible)
    {
        mesh_instance3d->set_visible(visible);
    }

    void GodotMeshInstance3DWrapper::destroy()
    {
        memdelete(mesh_instance3d);
    }
}
