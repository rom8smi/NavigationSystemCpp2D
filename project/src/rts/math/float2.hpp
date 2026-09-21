#ifndef FLOAT2_HPP
#define FLOAT2_HPP

#include <string>

using namespace std;

namespace NavigationSystemCode
{
	struct Float2
	{
		float x;
		float y;

		Float2();
		Float2(float p_x, float p_y);
		float length() const;
		float length_squared() const;
		float dot(const Float2 &p_v) const;
		float cross(const Float2 &p_v) const;
		float angle_to(const Float2 &p_v) const;
		Float2 rotated(float angle) const;
		void normalize();
		Float2 normalized() const;
		float get_axis(int axis) const;
		Float2 project(const Float2 &p_to) const;
		Float2 operator+(const Float2 &p_v) const;
		Float2 &operator+=(const Float2 &p_v);
		Float2 operator-(const Float2 &p_v) const;
		Float2 operator*(float r) const;
		Float2 operator/(float r) const;
		Float2 operator-() const;
		string get_string() const;
	};
}

#endif
