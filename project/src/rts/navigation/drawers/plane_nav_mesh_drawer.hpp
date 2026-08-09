#ifndef PLANE_NAV_MESH_DRAWER_HPP
#define PLANE_NAV_MESH_DRAWER_HPP

#include "rts/math/float2.hpp"
#include "rts/mesh/mesh.hpp"
#include "rts/godot_wrapper/godot_world.hpp"
#include "rts/navigation/nav_mesh.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct PlaneNavMeshDrawer
    {
        vector<GodotMeshInstance3DWrapper> walkable_mesh;
        vector<GodotMeshInstance3DWrapper> unwalkable_mesh;
        vector<GodotMeshInstance3DWrapper> obstacles_mesh;
        bool create_nav_mesh_drawers;
        bool create_obstacle_drawers;
        Rgba walkable_mesh_color;
        Rgba unwalkable_mesh_color;
        Rgba obstacles_mesh_color;
        float line_width;

        PlaneNavMeshDrawer();
        void create_walkable_mesh_drawer(float terrain_size, int resolution, NavMesh &navMesh, GodotWorld &godot_world);
        void create_unwalkable_mesh_drawer(float terrain_size, int resolution, NavMesh &navMesh, GodotWorld &godot_world);
        void create_obstacles_mesh_drawer(float terrain_size, int resolution, vector<Obstacle> &obstacles, GodotWorld &godot_world);
        Mesh merge_line_meshes(vector<Mesh> &line_meshes);
        Mesh create_drawer_line(float terrain_size, int resolution, vector<Float2> &points);
        void reset();
        void reset_walkable_mesh_drawer();
        void reset_unwalkable_mesh_drawer();
        void reset_obstacles_mesh_drawer();
    };
}

#endif
