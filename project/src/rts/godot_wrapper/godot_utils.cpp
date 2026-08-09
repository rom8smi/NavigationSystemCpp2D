#include <string>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include "godot_utils.hpp"

using namespace std;

namespace NavigationSystemCode
{
    void GodotUtils::print(string message)
    {
        UtilityFunctions::print(message.c_str());
    }

    void GodotUtils::print(int message)
    {
        UtilityFunctions::print(message);
    }

    void GodotUtils::print(float message)
    {
        UtilityFunctions::print(message);
    }

    Float2 GodotUtils::to_float2(const Vector2 &v)
    {
        return Float2(v.x, v.y);
    }

    Vector2 GodotUtils::to_vector2(const Float2 &v)
    {
        return Vector2(v.x, v.y);
    }

    Float3 GodotUtils::to_float3(const Vector3 &v)
    {
        return Float3(v.x, v.y, v.z);
    }

    Vector3 GodotUtils::to_vector3(const Float3 &v)
    {
        return Vector3(v.x, v.y, v.z);
    }

    Color GodotUtils::to_color(const Rgba &rgba)
    {
        return Color(rgba.r, rgba.g, rgba.b, rgba.a);
    }

    PackedInt32Array GodotUtils::to_packed_int_array(vector<int> &v)
    {
        PackedInt32Array result;

        for (int i = 0; i < v.size(); i++)
        {
            result.push_back(v[i]);
        }

        return result;
    }

    PackedVector2Array GodotUtils::to_packed_vector2_array(vector<Float2> &v)
    {
        PackedVector2Array result;

        for (int i = 0; i < v.size(); i++)
        {
            result.push_back(to_vector2(v[i]));
        }

        return result;
    }

    PackedVector3Array GodotUtils::to_packed_vector3_array(vector<Float3> &v)
    {
        PackedVector3Array result;

        for (int i = 0; i < v.size(); i++)
        {
            result.push_back(to_vector3(v[i]));
        }

        return result;
    }

    void GodotUtils::quit()
    {
        ((SceneTree *)(Engine::get_singleton()->get_main_loop()))->quit();
    }
}
