#ifndef GODOT_STANDARD_MATERIAL3D_WRAPPER_HPP
#define GODOT_STANDARD_MATERIAL3D_WRAPPER_HPP

#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include "rts/math/rgba.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotStandardMaterial3DWrapper
    {
        StandardMaterial3D *material;

        void create();
        void set_material(Ref<godot::Material> &p_material);
        Rgba get_albedo();
        void set_albedo(const Rgba &color);
        void set_texture(Ref<ViewportTexture> &texture);
        void set_texture(Ref<Texture2D> &texture);
        void set_transparency_alpha();
        void set_shading_mode_unshaded();
        void set_billboard_mode_enabled();
        void set_flag_albedo_from_vertex_color(bool value);
        void set_flag_billboard_keep_scale(bool value);
        void destroy();
    };
}

#endif
