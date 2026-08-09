#ifndef GODOT_MESH_UTILS_HPP
#define GODOT_MESH_UTILS_HPP

#include <godot_cpp/classes/array_mesh.hpp>
#include "rts/mesh/mesh.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotMeshUtils
    {
       static Ref<ArrayMesh> get_quad_mesh_xy(float size, bool invert);
       static Ref<ArrayMesh> get_quad_mesh_xy(float size_x, float size_y, bool invert);
       static Ref<ArrayMesh> get_quad_mesh_xz(float size_x, float size_z, bool invert);
       static Ref<ArrayMesh> get_quad_mesh(PackedVector3Array &vertices, bool invert);
       static NavigationSystemCode::Mesh godot_mesh_to_combined_rts_mesh(Ref<godot::Mesh> &godot_mesh);
       static Ref<godot::Mesh> rts_mesh_to_godot_mesh(NavigationSystemCode::Mesh &rts_mesh);
       static void copy_rts_mesh(NavigationSystemCode::Mesh &original_mesh, NavigationSystemCode::Mesh &new_mesh);
       static void offset_rts_mesh(NavigationSystemCode::Mesh &rts_mesh, const Float3 &offset);
       static void rotate_y_rts_mesh(NavigationSystemCode::Mesh &rts_mesh, const float &y_rotation);
       static void calculate_triangle_centroids(NavigationSystemCode::Mesh &rts_mesh, vector<Float3> &triangle_centroids);
       static void calculate_triangle_radii(NavigationSystemCode::Mesh &rts_mesh, vector<Float3> &triangle_centroids, vector<float> &triangle_radii);
       static float calculate_mesh_radii(NavigationSystemCode::Mesh &rts_mesh);
       static void combined_mesh_indices_test(NavigationSystemCode::Mesh &rts_mesh, const string &key);
    };
}

#endif
