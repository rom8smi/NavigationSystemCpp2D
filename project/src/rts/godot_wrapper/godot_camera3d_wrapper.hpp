#ifndef GODOT_CAMERA3D_WRAPPER_HPP
#define GODOT_CAMERA3D_WRAPPER_HPP

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/environment.hpp>
#include "rts/math/float2.hpp"
#include "rts/math/float3.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotCamera3DWrapper
    {
        Camera3D *camera;

        void create(Node *parent);
        void set_projection_orthogonal();
        void set_projection_perspective();
        void set_environment(Environment *environment);
        void set_position(const Float3 &position);
        void set_rotation(const Float3 &rotation);
        void set_name(const string &name);
        void set_far(float value);
        void set_size(float value);
        void set_fov(float value);
        Float3 project_ray_normal(const Float2 &screen_position);
        Float2 unproject_position(const Float3 &world_position);
    };
}

#endif
