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
		float length() const;
		float length_squared() const;
		float dot(const Float3 &b) const;
		Float3 cross(const Float3 &b) const;
		void normalize();
		Float3 normalized() const;
		static Float3 rotate(const Float3 &v, const Float3 &k, float phi);
		float angle_to(const Float3 &p_to) const;
		float signed_angle_to(const Float3 &p_to, const Float3 &p_axis) const;
		float get_axis(int axis) const;
		Float3 project(const Float3 &p_to) const;
		Float3 operator+(const Float3 &p_v) const;
		Float3 operator-(const Float3 &p_v) const;
		Float3 operator-() const;
		Float3 operator*(float r) const;
		Float3 operator/(float r) const;
		Float3 &operator+=(const Float3 &p_v);
		string get_string() const;
	};
}

#endif
