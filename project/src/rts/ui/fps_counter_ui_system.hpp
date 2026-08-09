#ifndef FPS_COUNTER_UI_SYSTEM_HPP
#define FPS_COUNTER_UI_SYSTEM_HPP

#include "rts/godot_wrapper/godot_control_wrapper.hpp"
#include "rts/godot_wrapper/godot_rich_text_label_wrapper.hpp"
#include "rts/math/float2.hpp"

namespace NavigationSystemCode
{
    struct FPSCounterUISystem
    {
        GodotRichTextLabelWrapper fps_counter_label;
        
        void create(GodotControlWrapper &control);
        void process(float delta);
    };
}

#endif
