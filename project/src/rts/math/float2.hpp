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
		float length();
		float length_squared();
		float dot(Float2 &p_v);
		float cross(Float2 &p_v);
		float angle_to(Float2 &p_v);
		Float2 rotated(float angle);
		void normalize();
		Float2 normalized();
		float get_axis(int axis);
		Float2 project(Float2 &p_to);
		Float2 operator+(Float2 p_v);
		void operator+=(Float2 p_v);
		Float2 operator-(Float2 p_v);
		Float2 operator*(float r);
		Float2 operator/(float r);
		Float2 operator-();
		string get_string();
	};
}

#endif
