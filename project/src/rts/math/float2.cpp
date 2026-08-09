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

	float Float2::length()
	{
		return sqrt(x * x + y * y);
	}

	float Float2::length_squared()
	{
		return x * x + y * y;
	}

	float Float2::dot(Float2 &p_v)
	{
		return x * p_v.x + y * p_v.y;
	}

	float Float2::cross(Float2 &p_v)
	{
		return x * p_v.y - y * p_v.x;
	}

	float Float2::angle_to(Float2 &p_v)
	{
		return atan2(cross(p_v), dot(p_v));
	}

	Float2 Float2::rotated(float angle)
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

	Float2 Float2::normalized()
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

	float Float2::get_axis(int axis)
	{
		if (axis == 0)
		{
			return x;
		}
		return y;
	}

	Float2 Float2::project(Float2 &p_to)
	{
		return p_to * (dot(p_to) / p_to.length_squared());
	}

	Float2 Float2::operator+(Float2 p_v)
	{
		return Float2(x + p_v.x, y + p_v.y);
	}

	void Float2::operator+=(Float2 p_v)
	{
		x += p_v.x;
		y += p_v.y;
	}

	Float2 Float2::operator-(Float2 p_v)
	{
		return Float2(x - p_v.x, y - p_v.y);
	}

	Float2 Float2::operator*(float r)
	{
		return Float2(x * r, y * r);
	}

	Float2 Float2::operator/(float r)
	{
		return Float2(x / r, y / r);
	}

	Float2 Float2::operator-()
	{
		return Float2(-x, -y);
	}

	string Float2::get_string()
	{
		return "(" + to_string(x) + ", " + to_string(y) + ")";
	}
}
