#include "godot_sky_wraper.hpp"

namespace NavigationSystemCode
{
    void GodotSkyWrapper::create()
    {
        sky = memnew(Sky);
    }

    void GodotSkyWrapper::set_material(Material *material)
    {
        sky->set_material(material);
    }
}
