#ifndef GODOT_PROCEDURAL_SKY_MATERIAL_WRAPPER_HPP
#define GODOT_PROCEDURAL_SKY_MATERIAL_WRAPPER_HPP

#include <godot_cpp/classes/procedural_sky_material.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotProceduralSkyMaterialWrapper
    {
        ProceduralSkyMaterial *material;

        void create();
    };
}

#endif
