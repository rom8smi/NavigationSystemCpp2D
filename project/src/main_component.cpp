#include "main_component.hpp"
#include <godot_cpp/classes/engine.hpp>

void MainComponent::_bind_methods()
{
}

void MainComponent::_ready()
{
	if (Engine::get_singleton()->is_editor_hint() == false)
	{
		main_system = MainSystem(GodotWorld(this));
		f_updates = 0.0f;
		updates_per_frame = 1.0f;

		main_system.ready();
	}
}

void MainComponent::_process(double delta)
{
	if (Engine::get_singleton()->is_editor_hint() == false)
	{
		f_updates += updates_per_frame;

		if (f_updates > 1.0f)
		{
			int i_updates = (int)f_updates;

			for (int i = 0; i < i_updates; i++)
			{
				process_inner(delta);
			}

			f_updates -= i_updates;
		}
	}
}

void MainComponent::process_inner(double delta)
{
	main_system.process(delta);
}

void MainComponent::_input(const Ref<InputEvent> &event)
{
	if (Engine::get_singleton()->is_editor_hint() == false)
	{
	}
}
