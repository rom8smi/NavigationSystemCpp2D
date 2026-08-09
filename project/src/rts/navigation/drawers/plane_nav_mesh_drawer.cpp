#include "plane_nav_mesh_drawer.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/math/heap_sort.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

namespace NavigationSystemCode
{
    PlaneNavMeshDrawer::PlaneNavMeshDrawer()
    {
        create_nav_mesh_drawers = false;
        create_obstacle_drawers = false;

        walkable_mesh_color = Rgba(0.0f, 0.2f, 0.4f, 1.0f);
        unwalkable_mesh_color = Rgba(0.4f, 0.2f, 0.0f, 1.0f);
        obstacles_mesh_color = Rgba(0.6f, 0.6f, 0.0f, 1.0f);

        line_width = 0.5f;
    }

    void PlaneNavMeshDrawer::create_walkable_mesh_drawer(float terrain_size, int resolution, NavMesh &navMesh, GodotWorld &godot_world)
    {
        if (!create_nav_mesh_drawers)
        {
            return;
        }

        reset_walkable_mesh_drawer();

        vector<Mesh> line_meshes;

        vector<DelaunatorEdge> edges = navMesh.delaunator.GetEdges();
        for (int i = 0; i < edges.size(); i++)
        {
            int edgeIndex = edges[i].index;

            if (navMesh.edgesWalkability[edgeIndex])
            {
                int p = edges[i].p;
                int q = edges[i].q;

                vector<Float2> line{
                    navMesh.allPoints[p],
                    navMesh.allPoints[q]};

                line_meshes.push_back(create_drawer_line(terrain_size, resolution, line));
            }
        }

        Mesh final_mesh = merge_line_meshes(line_meshes);

        for (int i = 0; i < line_meshes.size(); i++)
        {
            line_meshes[i].clear();
        }

        string name = "WalkableMesh";

        GodotMeshInstance3DWrapper mesh_instance_wrapper;
        godot_world.create_mesh_instance(final_mesh, mesh_instance_wrapper, name, walkable_mesh_color);
        walkable_mesh.push_back(mesh_instance_wrapper);
        final_mesh.clear();
    }

    void PlaneNavMeshDrawer::create_unwalkable_mesh_drawer(float terrain_size, int resolution, NavMesh &navMesh, GodotWorld &godot_world)
    {
        if (!create_nav_mesh_drawers)
        {
            return;
        }

        reset_unwalkable_mesh_drawer();

        vector<Mesh> line_meshes;

        vector<DelaunatorEdge> edges = navMesh.delaunator.GetEdges();
        for (int i = 0; i < edges.size(); i++)
        {
            int edgeIndex = edges[i].index;

            if (!navMesh.edgesWalkability[edgeIndex])
            {
                int p = edges[i].p;
                int q = edges[i].q;

                vector<Float2> line{
                    navMesh.allPoints[p],
                    navMesh.allPoints[q]};

                line_meshes.push_back(create_drawer_line(terrain_size, resolution, line));
            }
        }

        Mesh final_mesh = merge_line_meshes(line_meshes);

        for (int i = 0; i < line_meshes.size(); i++)
        {
            line_meshes[i].clear();
        }

        string name = "UnwalkableMesh";

        GodotMeshInstance3DWrapper mesh_instance_wrapper;
        godot_world.create_mesh_instance(final_mesh, mesh_instance_wrapper, name, unwalkable_mesh_color);
        unwalkable_mesh.push_back(mesh_instance_wrapper);
        final_mesh.clear();
    }

    void PlaneNavMeshDrawer::create_obstacles_mesh_drawer(float terrain_size, int resolution, vector<Obstacle> &obstacles, GodotWorld &godot_world)
    {
        if (!create_obstacle_drawers)
        {
            return;
        }

        reset_obstacles_mesh_drawer();

        vector<Mesh> line_meshes;

        for (int i = 0; i < obstacles.size(); i++)
        {
            int obstacle_corners_size = obstacles[i].obstacleCorners.size();

            for (int j = 0; j < obstacle_corners_size; j++)
            {
                int j_next = j + 1;
                if (j == obstacle_corners_size - 1)
                {
                    j_next = 0;
                }

                vector<Float2> line{
                    obstacles[i].obstacleCorners[j],
                    obstacles[i].obstacleCorners[j_next]};

                line_meshes.push_back(create_drawer_line(terrain_size, resolution, line));
            }
        }

        Mesh final_mesh = merge_line_meshes(line_meshes);

        for (int i = 0; i < line_meshes.size(); i++)
        {
            line_meshes[i].clear();
        }

        string name = "ObstaclesMesh";

        GodotMeshInstance3DWrapper mesh_instance_wrapper;
        godot_world.create_mesh_instance(final_mesh, mesh_instance_wrapper, name, obstacles_mesh_color);
        obstacles_mesh.push_back(mesh_instance_wrapper);
        final_mesh.clear();
    }

    Mesh PlaneNavMeshDrawer::merge_line_meshes(vector<Mesh> &line_meshes)
    {
        Mesh final_mesh;
        for (int i = 0; i < line_meshes.size(); i++)
        {
            int triangles_start = final_mesh.vertices.size();

            for (int j = 0; j < line_meshes[i].vertices.size(); j++)
            {
                final_mesh.vertices.push_back(line_meshes[i].vertices[j]);
            }
            for (int j = 0; j < line_meshes[i].uvs.size(); j++)
            {
                final_mesh.uvs.push_back(line_meshes[i].uvs[j]);
            }
            for (int j = 0; j < line_meshes[i].triangles.size(); j++)
            {
                final_mesh.triangles.push_back(line_meshes[i].triangles[j] + triangles_start);
            }
        }

        return final_mesh;
    }

    Mesh NavigationSystemCode::PlaneNavMeshDrawer::create_drawer_line(float terrain_size, int resolution, vector<Float2> &points)
    {
        float cell_size = terrain_size / resolution;
        int terrain_resolution = resolution;

        for (int i = 0; i <= terrain_resolution; i++)
        {
            Float2 p1 = Float2(i * cell_size, 0.0f);
            Float2 p2 = Float2(i * cell_size, terrain_size);

            LineSegmentsIntersectionResult result = VectorUtils::LineSegmentsIntersection(points[0], points[1], p1, p2);
            if (result.intersects)
            {
                points.push_back(result.intersection);
            }
        }

        for (int i = 0; i <= terrain_resolution; i++)
        {
            Float2 p1 = Float2(0.0f, i * cell_size);
            Float2 p2 = Float2(terrain_size, i * cell_size);

            LineSegmentsIntersectionResult result = VectorUtils::LineSegmentsIntersection(points[0], points[1], p1, p2);
            if (result.intersects)
            {
                points.push_back(result.intersection);
            }
        }

        vector<float> distances_from_start;
        vector<int> distances_from_start_sorting_indices;

        distances_from_start.resize(points.size());
        distances_from_start_sorting_indices.resize(points.size());

        for (int i = 0; i < points.size(); i++)
        {
            distances_from_start[i] = (points[i] - points[0]).length();
            distances_from_start_sorting_indices[i] = i;
        }

        HeapSort::Sort(distances_from_start_sorting_indices, distances_from_start);

        vector<Float2> sorted_points;
        sorted_points.resize(points.size());

        for (int i = 0; i < points.size(); i++)
        {
            sorted_points[i] = points[distances_from_start_sorting_indices[i]];
        }

        for (int i = 0; i < sorted_points.size() - 1; i++)
        {
            float distance_sqr = (sorted_points[i] - sorted_points[i + 1]).length_squared();

            if (distance_sqr < 0.0001f)
            {
                sorted_points.erase(sorted_points.begin() + i);
                i--;
            }
        }

        vector<Float3> mesh_points;
        vector<Float2> mesh_uvs;
        vector<int> mesh_triangles;

        float offset_above_terrain = 1.0f;
        float epsilon = 0.001f;

        for (int i = 0; i < sorted_points.size() - 1; i++)
        {
            Float2 p1 = sorted_points[i];
            Float2 p2 = sorted_points[i + 1];

            Float2 dir = (p2 - p1).normalized();
            Float2 perp = VectorUtils::PerpendicularCounterClockwise(dir) * 0.5f * line_width;

            Float2 left = p1 - perp;
            Float2 right = p1 + perp;

            Float2 left_clamped = left;
            Float2 right_clamped = right;

            float right_y = 0.0f;
            float left_y = 0.0f;

            mesh_points.push_back(Float3(right.x, right_y, right.y));
            mesh_points.push_back(Float3(left.x, left_y, left.y));

            mesh_uvs.push_back(Float2(0.0f, 0.0f));
            mesh_uvs.push_back(Float2(0.0f, 0.0f));

            mesh_triangles.push_back(2 * i);
            mesh_triangles.push_back(2 * i + 1);
            mesh_triangles.push_back(2 * i + 3);

            mesh_triangles.push_back(2 * i);
            mesh_triangles.push_back(2 * i + 3);
            mesh_triangles.push_back(2 * i + 2);

            if (i + 1 == sorted_points.size() - 1)
            {
                left = p2 - perp;
                right = p2 + perp;

                left_clamped = left;
                right_clamped = right;

                right_y = 0.0f;
                left_y = 0.0f;

                mesh_points.push_back(Float3(right.x, right_y, right.y));
                mesh_points.push_back(Float3(left.x, left_y, left.y));

                mesh_uvs.push_back(Float2(0.0f, 0.0f));
                mesh_uvs.push_back(Float2(0.0f, 0.0f));
            }
        }

        return Mesh{
            .vertices = mesh_points,
            .uvs = mesh_uvs,
            .triangles = mesh_triangles};
    }

    void PlaneNavMeshDrawer::reset()
    {
        reset_walkable_mesh_drawer();
        reset_unwalkable_mesh_drawer();
        reset_obstacles_mesh_drawer();
    }

    void PlaneNavMeshDrawer::reset_walkable_mesh_drawer()
    {
        for (int i = 0; i < walkable_mesh.size(); i++)
        {
            walkable_mesh[i].destroy();
        }
        walkable_mesh.clear();
    }

    void PlaneNavMeshDrawer::reset_unwalkable_mesh_drawer()
    {
        for (int i = 0; i < unwalkable_mesh.size(); i++)
        {
            unwalkable_mesh[i].destroy();
        }
        unwalkable_mesh.clear();
    }

    void PlaneNavMeshDrawer::reset_obstacles_mesh_drawer()
    {
        for (int i = 0; i < obstacles_mesh.size(); i++)
        {
            obstacles_mesh[i].destroy();
        }
        obstacles_mesh.clear();
    }
}
