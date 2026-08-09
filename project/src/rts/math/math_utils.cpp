#include "math_utils.hpp"
#include <cmath>

namespace NavigationSystemCode
{
	float MathUtils::interpolate(float x, float x0, float x1, float y0, float y1)
	{
		return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
	}

	float MathUtils::interpolate_clamped(float x, float x0, float x1, float y0, float y1)
	{
		float y = y0 + (y1 - y0) * (x - x0) / (x1 - x0);

		if (y0 < y1)
		{
			if (y < y0)
			{
				y = y0;
			}
			if (y > y1)
			{
				y = y1;
			}
		}
		else if (y0 > y1)
		{
			if (y > y0)
			{
				y = y0;
			}
			if (y < y1)
			{
				y = y1;
			}
		}

		return y;
	}

	int MathUtils::clamp(int value, int min, int max)
	{
		if (value < min)
		{
			return min;
		}
		if (value > max)
		{
			return max;
		}
		return value;
	}

	float MathUtils::clamp(float value, float min, float max)
	{
		if (value < min)
		{
			return min;
		}
		if (value > max)
		{
			return max;
		}
		return value;
	}

	float MathUtils::clamp01(float value)
	{
		return clamp(value, 0.0f, 1.0f);
	}

	int MathUtils::round_to_int(float f)
	{
		return (int)(f + 0.5f);
	}

	int MathUtils::abs(int i)
	{
		if (i < 0)
		{
			return -i;
		}
		return i;
	}

	float MathUtils::abs(float f)
	{
		if (f < 0.0f)
		{
			return -f;
		}
		return f;
	}

	float MathUtils::min(float a, float b)
	{
		if (a < b)
		{
			return a;
		}
		return b;
	}

	float MathUtils::max(float a, float b)
	{
		if (a > b)
		{
			return a;
		}
		return b;
	}

	int MathUtils::min(int a, int b)
	{
		if (a < b)
		{
			return a;
		}
		return b;
	}

	int MathUtils::max(int a, int b)
	{
		if (a > b)
		{
			return a;
		}
		return b;
	}

	int MathUtils::floor(float f)
	{
		if (f < 0.0f)
		{
			return (int)f - 1;
		}

		return (int)f;
	}

	float MathUtils::sqrt(float f)
	{
		return std::sqrt(f);
	}

	float MathUtils::pow(float f, float p)
	{
		return std::pow(f, p);
	}

	float MathUtils::sin(float f)
	{
		return std::sin(f);
	}

	float MathUtils::cos(float f)
	{
		return std::cos(f);
	}

	float MathUtils::atan2(float y, float x)
	{
		return std::atan2(y, x);
	}

	float MathUtils::fmod(float x, float y)
	{
		return std::fmod(x, y);
	}

	bool MathUtils::filter_condition(bool fail_condition, bool filter)
	{
		if (!filter)
		{
			return true;
		}
		if (fail_condition)
		{
			return false;
		}
		return true;
	}
}
