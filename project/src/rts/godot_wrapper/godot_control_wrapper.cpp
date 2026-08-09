#include "godot_control_wrapper.hpp"

namespace NavigationSystemCode
{
    void GodotControlWrapper::create(Node *p_main_component)
    {
        control = memnew(Control);
        p_main_component->add_child(control);
    }
}
