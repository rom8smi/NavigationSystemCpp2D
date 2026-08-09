#ifndef GODOT_UTILS_HPP
#define GODOT_UTILS_HPP

#include <string>
#include <vector>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include "rts/math/float2.hpp"
#include "rts/math/float3.hpp"
#include "rts/math/rgba.hpp"

using namespace std;
using namespace godot;

namespace NavigationSystemCode
{
	struct GodotUtils
	{
		static void print(string message);
		static void print(int message);
		static void print(float message);
		static Float2 to_float2(const Vector2 &v);
		static Vector2 to_vector2(const Float2 &v);
		static Float3 to_float3(const Vector3 &v);
		static Vector3 to_vector3(const Float3 &v);
		static Color to_color(const Rgba &rgba);
		static PackedInt32Array to_packed_int_array(vector<int> &v);
		static PackedVector2Array to_packed_vector2_array(vector<Float2> &v);
		static PackedVector3Array to_packed_vector3_array(vector<Float3> &v);
		static void quit();
	};
}

#endif
