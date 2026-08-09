#include "godot_directional_light3d_wrapper.hpp"
#include "godot_utils.hpp"

namespace NavigationSystemCode
{
    void GodotDirectionalLight3DWrapper::create(Node *parent)
    {
        directional_light3d = memnew(DirectionalLight3D);
        parent->add_child(directional_light3d);
    }

    void GodotDirectionalLight3DWrapper::set_rotation(const Float3 &rotation)
    {
        directional_light3d->set_rotation(GodotUtils::to_vector3(rotation));
    }

    void GodotDirectionalLight3DWrapper::set_name(const string &name)
    {
        directional_light3d->set_name(name.c_str());
    }
}
