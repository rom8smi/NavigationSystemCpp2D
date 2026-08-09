#include "godot_environment_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotEnvironmentWrapper::create()
    {
        environment = memnew(Environment);
    }

    void GodotEnvironmentWrapper::set_background_clear_color()
    {
        environment->set_background(Environment::BGMode::BG_CLEAR_COLOR);
    }

    void GodotEnvironmentWrapper::set_background_sky()
    {
        environment->set_background(Environment::BGMode::BG_SKY);
    }

    void GodotEnvironmentWrapper::set_ambient_source_sky()
    {
        environment->set_ambient_source(Environment::AmbientSource::AMBIENT_SOURCE_SKY);
    }

    void GodotEnvironmentWrapper::set_sky(Sky *sky)
    {
        environment->set_sky(sky);
    }
}
