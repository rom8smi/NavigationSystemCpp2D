#include <string>
#include "godot_world.hpp"
#include "rts/math/angle_utils.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/math/heap_sort.hpp"
#include "godot_utils.hpp"
#include "godot_mesh_utils.hpp"
#include "godot_standard_material3d_wrapper.hpp"
#include "godot_box_mesh_wrapper.hpp"

namespace NavigationSystemCode
{
    GodotWorld::GodotWorld() {}

    GodotWorld::GodotWorld(Node *p_main_component)
    {
        main_component.node = p_main_component;
    }

    void GodotWorld::create_world()
    {
        node3D.create(main_component.node);
        node3D.set_name("Node3D");

        camera3D.create(node3D.node3D);
        camera3D.set_name("Camera3D");
        camera3D.set_far(10000.0f);
        camera3D.set_fov(60.0f);

        procedural_sky_material.create();

        sky.create();
        sky.set_material(procedural_sky_material.material);

        environment.create();
        environment.set_background_sky();
        environment.set_background_sky();
        environment.set_sky(sky.sky);
        camera3D.set_environment(environment.environment);

        directional_light3d.create(node3D.node3D);
        directional_light3d.set_name("DirectionalLight3D");
        directional_light3d.set_rotation(Float3(AngleUtils::degrees_to_radians(-45.0f), AngleUtils::degrees_to_radians(180.0f), 0.0f));
    }

    void GodotWorld::create_terrain(Mesh &terrain_mesh)
    {
        string name = "Terrain";
        Rgba color = Rgba(0.25f, 0.4f, 0.15f, 1.0f);
        create_mesh_instance(terrain_mesh, terrain_mesh_instance, name, color);
        terrain_mesh.clear();
    }

    void GodotWorld::create_mesh_instance(Mesh &rts_mesh, GodotMeshInstance3DWrapper &mesh_instance, string &name, Rgba &color)
    {
        GodotStandardMaterial3DWrapper material;
        material.create();
        material.set_albedo(color);

        mesh_instance.create(node3D.node3D);

        mesh_instance.set_mesh(GodotMeshUtils::rts_mesh_to_godot_mesh(rts_mesh));
        mesh_instance.set_material_override(material.material);
        mesh_instance.set_name(name);
    }

    void GodotWorld::create_debug_gizmo_cubes(vector<Float3> &positions)
    {
        for (int i = 0; i < positions.size(); i++)
        {
            GodotStandardMaterial3DWrapper material;
            material.create();
            material.set_albedo(Rgba(1.0f, 1.0f, 1.0f, 1.0f));

            GodotBoxMeshWrapper box_mesh;
            box_mesh.create();

            GodotMeshInstance3DWrapper mesh_instance;
            mesh_instance.create(node3D.node3D);

            mesh_instance.set_mesh(box_mesh.mesh);
            mesh_instance.set_material_override(material.material);

            mesh_instance.set_position(positions[i]);
        }
    }

    void GodotWorld::set_camera_position(const Float3 &position)
    {
        camera3D.set_position(position);
    }

    void GodotWorld::set_camera_rotation(const Float3 &rotation)
    {
        camera3D.set_rotation(rotation);
    }

    Float3 GodotWorld::screen_to_world_position(const Float2 &screen_position)
    {
        return camera3D.project_ray_normal(screen_position);
    }

    Float2 GodotWorld::world_to_screen_position(const Float3 &world_position)
    {
        return camera3D.unproject_position(world_position);
    }

    void GodotWorld::apply_scene(const Scene &scene)
    {
        if (scene.perspective_camera)
        {
            camera3D.set_projection_perspective();
            camera3D.set_fov(scene.camera_fov);
        }
        else
        {
            camera3D.set_projection_orthogonal();
            camera3D.set_size(scene.camera_size);
        }
    }
}
