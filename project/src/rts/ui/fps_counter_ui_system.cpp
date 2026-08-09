#include "fps_counter_ui_system.hpp"

namespace NavigationSystemCode
{
    void FPSCounterUISystem::create(GodotControlWrapper &control)
    {
        fps_counter_label.create(control);

        Float2 begin = Float2(0.0f, 0.0f);
        Float2 end = Float2(150.0f, 50.0f);
        string text = "";

        fps_counter_label.set_begin(begin);
        fps_counter_label.set_end(end);
        fps_counter_label.set_text(text);
        fps_counter_label.set_font_size(32);
    }

    void FPSCounterUISystem::process(float delta)
    {
        float fps = 1.0f / delta;
        int fps_int = fps;
        
        string s = to_string(fps_int) + " FPS";
        fps_counter_label.set_text(s);
    }
}
