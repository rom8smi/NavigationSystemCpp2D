#ifndef ANGLE_UTILS_HPP
#define ANGLE_UTILS_HPP

#include <limits>

namespace NavigationSystemCode
{
    struct AngleUtils
    {
        static float degrees_to_radians(float degrees);
        static float radians_to_degrees(float radians);
        static float smallest_angle_difference_unsigned(float a, float b);
        static float smallest_angle_difference(float a, float b);
        static float lerp_angle(float from, float to, float weight, float max);
        static float lerp_angle(float from, float to, float weight);
        static float angle_difference(float from, float to);
        static float normalized_radians(float radians);
    };
}

#endif
