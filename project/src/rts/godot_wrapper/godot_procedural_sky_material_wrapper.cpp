#include "godot_procedural_sky_material_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotProceduralSkyMaterialWrapper::create()
    {
        material = memnew(ProceduralSkyMaterial);
    }
}
