#include "godot_box_mesh_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotBoxMeshWrapper::create()
    {
        mesh = memnew(BoxMesh);
    }
}
