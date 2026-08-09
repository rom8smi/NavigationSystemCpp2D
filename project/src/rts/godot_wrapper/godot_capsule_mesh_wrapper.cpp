#include "godot_capsule_mesh_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotCapsuleMeshWrapper::create()
    {
        mesh = memnew(CapsuleMesh);
    }
}
