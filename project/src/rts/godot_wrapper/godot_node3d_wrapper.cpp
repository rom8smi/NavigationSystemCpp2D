#include "godot_node3d_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotNode3DWrapper::create(Node *parent)
    {
        node3D = memnew(Node3D);
        parent->add_child(node3D);
    }

    void GodotNode3DWrapper::set_name(const string &name)
    {
        node3D->set_name(name.c_str());
    }
}
