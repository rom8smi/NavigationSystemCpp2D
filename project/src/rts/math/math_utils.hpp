#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <limits>

namespace NavigationSystemCode
{
	struct MathUtils
	{
		static constexpr const float PI = 3.14159265f;
		static constexpr const float TAU = 6.28318531f;
		static constexpr const int INT_MAX_VALUE = (std::numeric_limits<int>::max)();
		static constexpr const float FLOAT_MIN_VALUE = std::numeric_limits<float>::lowest();
		static constexpr const float FLOAT_MAX_VALUE = (std::numeric_limits<float>::max)();
		static constexpr const float FLOAT_EPSILON = (std::numeric_limits<float>::epsilon)();

		static float interpolate(float x, float x0, float x1, float y0, float y1);
		static float interpolate_clamped(float x, float x0, float x1, float y0, float y1);
		static int clamp(int value, int min, int max);
		static float clamp(float value, float min, float max);
		static float clamp01(float value);
		static int round_to_int(float f);
		static int abs(int i);
		static float abs(float f);
		static float min(float a, float b);
		static float max(float a, float b);
		static int min(int a, int b);
		static int max(int a, int b);
		static int floor(float f);
		static float sqrt(float f);
        static float pow(float f, float p);
        static float sin(float f);
        static float cos(float f);
		static float atan2(float y, float x);
		static float fmod(float y, float x);
		static bool filter_condition(bool fail_condition, bool filter);
	};
}

#endif
