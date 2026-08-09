#ifndef MAIN_SYSTEM_HPP
#define MAIN_SYSTEM_HPP

#include "rts/scene/scene_system.hpp"
#include "rts/godot_wrapper/godot_world.hpp"
#include "rts/navigation/navigation_system.hpp"
#include "rts/ui/ui_system.hpp"

namespace NavigationSystemCode
{
	struct MainSystem
	{
        SceneSystem scene_system;
		GodotWorld godot_world;
		UISystem ui_system;
		GodotScreenSystem godot_screen_system;
		NavigationSystem navigation_system;
		NavigationTestsSystem navigation_tests_system;

		MainSystem();
		MainSystem(GodotWorld p_godot_world);

		void ready();
		void process(float delta);

        void set_from_loaded_scene();
        void reset();
    };
}

#endif
