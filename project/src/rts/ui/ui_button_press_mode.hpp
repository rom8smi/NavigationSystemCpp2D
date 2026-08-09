#ifndef UI_BUTTON_PRESS_MODE_HPP
#define UI_BUTTON_PRESS_MODE_HPP

namespace NavigationSystemCode
{
    struct UIButtonPressMode
    {
        static const int UNPRESSED = 0;
        static const int PRESSED = 1;
        static const int PRESSED_DOWN = 2;
        static const int PRESSED_UP = 3;
    };
}

#endif
