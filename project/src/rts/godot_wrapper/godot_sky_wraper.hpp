#ifndef GODOT_SKY_WRAPPER_HPP
#define GODOT_SKY_WRAPPER_HPP

#include <godot_cpp/classes/sky.hpp>
#include <godot_cpp/classes/material.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotSkyWrapper
    {
        Sky *sky;

        void create();
        void set_material(Material *material);
    };
}

#endif
