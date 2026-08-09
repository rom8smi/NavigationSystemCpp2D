#include "angle_utils.hpp"
#include "math_utils.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

namespace NavigationSystemCode
{
    float AngleUtils::degrees_to_radians(float degrees)
    {
        return degrees * 0.01745329238f;
    }

    float AngleUtils::radians_to_degrees(float radians)
    {
        return radians * 57.29578018f;
    }

    // Returns radians between 0 and pi
    float AngleUtils::smallest_angle_difference_unsigned(float a, float b)
    {
        return MathUtils::abs(smallest_angle_difference(a, b));
    }

    // Returns radians between -pi and pi
    float AngleUtils::smallest_angle_difference(float a, float b)
    {
        float normalized_a = normalized_radians(a);
        float normalized_b = normalized_radians(b);

        float difference = normalized_b - normalized_a;

        if (difference > MathUtils::PI)
        {
            difference -= 2.0f * MathUtils::PI;
        }
        else if (difference <= -MathUtils::PI)
        {
            difference += 2.0f * MathUtils::PI;
        }

        return difference;
    }

    float AngleUtils::lerp_angle(float from, float to, float weight, float max)
    {
        float angle_to_rotate = angle_difference(from, to) * weight;
        float abs_angle_difference = MathUtils::abs(angle_to_rotate);
        if (abs_angle_difference > max - MathUtils::FLOAT_EPSILON)
        {
            float sign_angle_difference = angle_to_rotate / abs_angle_difference;
            angle_to_rotate = sign_angle_difference * MathUtils::clamp(abs_angle_difference, -MathUtils::FLOAT_EPSILON, max);
        }
        return from + angle_to_rotate;
    }

    float AngleUtils::lerp_angle(float from, float to, float weight)
    {
        return from + angle_difference(from, to) * weight;
    }

    float AngleUtils::angle_difference(float from, float to)
    {
        float difference = MathUtils::fmod(to - from, MathUtils::TAU);
        return MathUtils::fmod(2.0f * difference, MathUtils::TAU) - difference;
    }

    // Normalizes radians between -pi and pi
    float AngleUtils::normalized_radians(float radians)
    {
        float result = radians;
        float two_pi = 2.0f * MathUtils::PI;

        if (result > MathUtils::PI)
        {
            result -= two_pi;
        }
        if (result <= -MathUtils::PI)
        {
            result += two_pi;
        }

        return result;
    }
}
