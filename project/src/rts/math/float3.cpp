#include "float3.hpp"
#include <cmath>
#include <string>

using namespace std;

namespace NavigationSystemCode
{
	Float3::Float3()
	{
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}

	Float3::Float3(float p_x, float p_y, float p_z)
	{
		x = p_x;
		y = p_y;
		z = p_z;
	}

	float Float3::length()
	{
		return sqrt(x * x + y * y + z * z);
	}

	float Float3::length_squared()
	{
		return x * x + y * y + z * z;
	}

	float Float3::dot(Float3 b)
	{
		return x * b.x + y * b.y + z * b.z;
	}

	Float3 Float3::cross(Float3 &b)
	{
		float bx = b.x;
		float by = b.y;
		float bz = b.z;

		return Float3(
			(y * bz) - (z * by),
			(z * bx) - (x * bz),
			(x * by) - (y * bx));
	}

	void Float3::normalize()
	{
		float l = x * x + y * y + z * z;
		if (l != 0)
		{
			l = sqrtf(l);
			x /= l;
			y /= l;
			z /= l;
		}
	}

	Float3 Float3::normalized()
	{
		float p_x = x;
		float p_y = y;
		float p_z = z;
		float l = x * x + y * y + z * z;
		if (l != 0)
		{
			l = sqrtf(l);
			p_x /= l;
			p_y /= l;
			p_z /= l;
		}

		return Float3(p_x, p_y, p_z);
	}

	// Based on answer https://stackoverflow.com/questions/42421611/3d-vector-rotation-in-c
	Float3 Float3::rotate(Float3 v, Float3 k, float phi)
	{
		float cos = std::cos(phi);
		float sin = std::sin(phi);

		return (v * cos) + (k.cross(v) * sin) + (k * k.dot(v)) * (1.0 - cos);
	}

	float Float3::angle_to(Float3 p_to)
	{
		return atan2(cross(p_to).length(), dot(p_to));
	}

	float Float3::signed_angle_to(Float3 p_to, Float3 p_axis)
	{
		Float3 cross_to = cross(p_to);
		float unsigned_angle = atan2(cross_to.length(), dot(p_to));
		float sign = cross_to.dot(p_axis);
		return (sign < 0) ? -unsigned_angle : unsigned_angle;
	}

	float Float3::get_axis(int axis)
	{
		if (axis == 0)
		{
			return x;
		}
		if (axis == 1)
		{
			return y;
		}
		return z;
	}

	Float3 Float3::project(Float3 &p_to)
	{
		return p_to * (dot(p_to) / p_to.length_squared());
	}

	Float3 Float3::operator+(Float3 p_v)
	{
		return Float3(x + p_v.x, y + p_v.y, z + p_v.z);
	}

	Float3 Float3::operator-(Float3 p_v)
	{
		return Float3(x - p_v.x, y - p_v.y, z - p_v.z);
	}

	Float3 Float3::operator-()
	{
		return Float3(-x, -y, -z);
	}

	Float3 Float3::operator*(float r)
	{
		return Float3(x * r, y * r, z * r);
	}

    Float3 Float3::operator/(float r)
    {
        return Float3(x / r, y / r, z / r);
    }

    void Float3::operator+=(Float3 p_v)
	{
		x += p_v.x;
		y += p_v.y;
		z += p_v.z;
	}

	string Float3::get_string()
	{
		return "(" + to_string(x) + ", " + to_string(y) + ", " + to_string(z) + ")";
	}
}
