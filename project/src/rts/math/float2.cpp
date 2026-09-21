#include "float2.hpp"
#include <string>
#include <cmath>

using namespace std;

namespace NavigationSystemCode
{
	Float2::Float2()
	{
		x = 0.0f;
		y = 0.0f;
	}

	Float2::Float2(float p_x, float p_y)
	{
		x = p_x;
		y = p_y;
	}

	float Float2::length() const
	{
		return sqrt(x * x + y * y);
	}

	float Float2::length_squared() const
	{
		return x * x + y * y;
	}

	float Float2::dot(const Float2 &p_v) const
	{
		return x * p_v.x + y * p_v.y;
	}

	float Float2::cross(const Float2 &p_v) const
	{
		return x * p_v.y - y * p_v.x;
	}

	// Returns angle between [-180, 180] degrees. See Tests::angle_to_test() for confirmation
	float Float2::angle_to(const Float2 &p_v) const
	{
		return atan2(cross(p_v), dot(p_v));
	}

	Float2 Float2::rotated(float angle) const
	{
		float sine = sin(angle);
		float cosi = cos(angle);
		return Float2(x * cosi - y * sine, x * sine + y * cosi);
	}

	void Float2::normalize()
	{
		float l = x * x + y * y;
		if (l != 0)
		{
			l = sqrtf(l);
			x /= l;
			y /= l;
		}
	}

	Float2 Float2::normalized() const
	{
		float p_x = x;
		float p_y = y;
		float l = x * x + y * y;
		if (l != 0)
		{
			l = sqrtf(l);
			p_x /= l;
			p_y /= l;
		}

		return Float2(p_x, p_y);
	}

	float Float2::get_axis(int axis) const
	{
		if (axis == 0)
		{
			return x;
		}
		return y;
	}

	Float2 Float2::project(const Float2 &p_to) const
	{
		return p_to * (dot(p_to) / p_to.length_squared());
	}

	Float2 Float2::operator+(const Float2 &p_v) const
	{
		return Float2(x + p_v.x, y + p_v.y);
	}

	Float2 &Float2::operator+=(const Float2 &p_v)
	{
		x += p_v.x;
		y += p_v.y;
		return *this;
	}

	Float2 Float2::operator-(const Float2 &p_v) const
	{
		return Float2(x - p_v.x, y - p_v.y);
	}

	Float2 Float2::operator*(float r) const
	{
		return Float2(x * r, y * r);
	}

	Float2 Float2::operator/(float r) const
	{
		return Float2(x / r, y / r);
	}

	Float2 Float2::operator-() const
	{
		return Float2(-x, -y);
	}

	string Float2::get_string() const
	{
		return "(" + to_string(x) + ", " + to_string(y) + ")";
	}
}
