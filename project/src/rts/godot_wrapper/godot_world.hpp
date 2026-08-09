#ifndef GODOT_WORLD_HPP
#define GODOT_WORLD_HPP

#include <godot_cpp/classes/node.hpp>
#include "godot_node_wrapper.hpp"
#include "godot_node3d_wrapper.hpp"
#include "godot_camera3d_wrapper.hpp"
#include "godot_procedural_sky_material_wrapper.hpp"
#include "godot_sky_wraper.hpp"
#include "godot_environment_wrapper.hpp"
#include "godot_directional_light3d_wrapper.hpp"
#include "godot_mesh_instance3d_wrapper.hpp"
#include "rts/math/float3.hpp"
#include "rts/math/rgba.hpp"
#include "rts/mesh/mesh.hpp"
#include "rts/scene/scene.hpp"

using namespace godot;

namespace NavigationSystemCode
{
	struct GodotWorld
	{
		GodotNodeWrapper main_component;
		GodotNode3DWrapper node3D;
		GodotCamera3DWrapper camera3D;
		GodotProceduralSkyMaterialWrapper procedural_sky_material;
		GodotSkyWrapper sky;
		GodotEnvironmentWrapper environment;
		GodotDirectionalLight3DWrapper directional_light3d;
		GodotMeshInstance3DWrapper terrain_mesh_instance;

		GodotWorld();
		GodotWorld(Node *p_main_component);
		void create_world();
		void create_terrain(Mesh &terrain_mesh);
		void create_mesh_instance(Mesh &rts_mesh, GodotMeshInstance3DWrapper &mesh_instance, string &name, Rgba &color);
		void create_debug_gizmo_cubes(vector<Float3> &positions);
		void set_camera_position(const Float3 &position);
		void set_camera_rotation(const Float3 &rotation);
		Float3 screen_to_world_position(const Float2 &screen_position);
		Float2 world_to_screen_position(const Float3 &world_position);
        void apply_scene(const Scene &scene);
    };
}

#endif
