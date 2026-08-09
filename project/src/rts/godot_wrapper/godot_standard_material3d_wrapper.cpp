#include "godot_standard_material3d_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotStandardMaterial3DWrapper::create()
    {
        material = memnew(StandardMaterial3D);
    }

    void GodotStandardMaterial3DWrapper::set_material(Ref<godot::Material> &p_material)
    {
        material = (StandardMaterial3D *)p_material.ptr();
    }

    Rgba GodotStandardMaterial3DWrapper::get_albedo()
    {
        Color color = material->get_albedo();
        return Rgba(color.r, color.g, color.b, color.a);
    }

    void GodotStandardMaterial3DWrapper::set_albedo(const Rgba &color)
    {
        material->set_albedo(Color(color.r, color.g, color.b, color.a));
    }

    void GodotStandardMaterial3DWrapper::set_texture(Ref<ViewportTexture> &texture)
    {
        material->set_texture(BaseMaterial3D::TextureParam::TEXTURE_ALBEDO, texture);
    }

    void GodotStandardMaterial3DWrapper::set_texture(Ref<Texture2D> &texture)
    {
        material->set_texture(BaseMaterial3D::TextureParam::TEXTURE_ALBEDO, texture);
    }

    void GodotStandardMaterial3DWrapper::set_transparency_alpha()
    {
        material->set_transparency(BaseMaterial3D::Transparency::TRANSPARENCY_ALPHA);
    }

    void GodotStandardMaterial3DWrapper::set_shading_mode_unshaded()
    {
        material->set_shading_mode(BaseMaterial3D::ShadingMode::SHADING_MODE_UNSHADED);
    }

    void GodotStandardMaterial3DWrapper::set_billboard_mode_enabled()
    {
        material->set_billboard_mode(BaseMaterial3D::BillboardMode::BILLBOARD_ENABLED);
    }

    void GodotStandardMaterial3DWrapper::set_flag_albedo_from_vertex_color(bool value)
    {
        material->set_flag(BaseMaterial3D::Flags::FLAG_ALBEDO_FROM_VERTEX_COLOR, value);
    }

    void GodotStandardMaterial3DWrapper::set_flag_billboard_keep_scale(bool value)
    {
        material->set_flag(BaseMaterial3D::Flags::FLAG_BILLBOARD_KEEP_SCALE, value);
    }

    void GodotStandardMaterial3DWrapper::destroy()
    {
        memdelete(material);
    }
}
