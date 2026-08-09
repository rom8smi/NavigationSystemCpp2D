#ifndef FLOAT3_HPP
#define FLOAT3_HPP

#include <string>

using namespace std;

namespace NavigationSystemCode
{
	struct Float3
	{
		float x;
		float y;
		float z;

		Float3();
		Float3(float p_x, float p_y, float p_z);
		float length();
		float length_squared();
		float dot(Float3 b);
		Float3 cross(Float3 &b);
		void normalize();
		Float3 normalized();
		static Float3 rotate(Float3 v, Float3 k, float phi);
		float angle_to(Float3 p_to);
		float signed_angle_to(Float3 p_to, Float3 p_axis);
		float get_axis(int axis);
		Float3 project(Float3 &p_to);
		Float3 operator+(Float3 p_v);
		Float3 operator-(Float3 p_v);
		Float3 operator-();
		Float3 operator*(float r);
		Float3 operator/(float r);
		void operator+=(Float3 p_v);
		string get_string();
	};
}

#endif
