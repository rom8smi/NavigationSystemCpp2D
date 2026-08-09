#include "godot_camera3d_wrapper.hpp"
#include "godot_utils.hpp"

namespace NavigationSystemCode
{
    void GodotCamera3DWrapper::create(Node *parent)
    {
        camera = memnew(Camera3D);
        parent->add_child(camera);
    }

    void GodotCamera3DWrapper::set_projection_orthogonal()
    {
        camera->set_projection(Camera3D::ProjectionType::PROJECTION_ORTHOGONAL);
    }

    void GodotCamera3DWrapper::set_projection_perspective()
    {
        camera->set_projection(Camera3D::ProjectionType::PROJECTION_PERSPECTIVE);
    }

    void GodotCamera3DWrapper::set_environment(Environment *environment)
    {
        camera->set_environment(environment);
    }

    void GodotCamera3DWrapper::set_position(const Float3 &position)
    {
        camera->set_position(GodotUtils::to_vector3(position));
    }

    void GodotCamera3DWrapper::set_rotation(const Float3 &rotation)
    {
        camera->set_rotation(GodotUtils::to_vector3(rotation));
    }

    void GodotCamera3DWrapper::set_name(const string &name)
    {
        camera->set_name(name.c_str());
    }

    void GodotCamera3DWrapper::set_far(float value)
    {
        camera->set_far(value);
    }

    void GodotCamera3DWrapper::set_size(float value)
    {
        camera->set_size(value);
    }

    void GodotCamera3DWrapper::set_fov(float value)
    {
        camera->set_fov(value);
    }

    Float3 GodotCamera3DWrapper::project_ray_normal(const Float2 &screen_position)
    {
        const Vector3 world_position = camera->project_ray_normal(GodotUtils::to_vector2(screen_position));
        return GodotUtils::to_float3(world_position);
    }

    Float2 GodotCamera3DWrapper::unproject_position(const Float3 &world_position)
    {
        const Vector2 screen_position = camera->unproject_position(GodotUtils::to_vector3(world_position));
        return GodotUtils::to_float2(screen_position);
    }
}
