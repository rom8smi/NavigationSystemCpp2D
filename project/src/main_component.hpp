#ifndef MAIN_COMPONENT_HPP
#define MAIN_COMPONENT_HPP

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/classes/input_event.hpp>

#include "rts/main_system.hpp"

using namespace godot;
using namespace NavigationSystemCode;

class MainComponent : public Node
{
	GDCLASS(MainComponent, Node);

	MainSystem main_system;

	float f_updates;
    float updates_per_frame;

	bool use_debug_main_system = false;

public:
	static void _bind_methods();
	virtual void _ready() override;
	virtual void _process(double delta) override;
    void process_inner(double delta);
    virtual void _input(const Ref<InputEvent> &event) override;
};

#endif
