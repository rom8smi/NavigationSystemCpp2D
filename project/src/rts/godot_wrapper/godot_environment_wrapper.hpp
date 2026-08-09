#ifndef GODOT_ENVIRONMENT_WRAPPER_HPP
#define GODOT_ENVIRONMENT_WRAPPER_HPP

#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/sky.hpp>

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotEnvironmentWrapper
    {
        Environment *environment;

        void create();
        void set_background_clear_color();
        void set_background_sky();
        void set_ambient_source_sky();
        void set_sky(Sky *sky);
    };
}

#endif
