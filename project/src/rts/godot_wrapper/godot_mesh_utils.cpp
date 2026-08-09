#include "godot_mesh_utils.hpp"
#include <godot_cpp/classes/surface_tool.hpp>
#include "godot_utils.hpp"
#include "rts/math/math_utils.hpp"

namespace NavigationSystemCode
{
    Ref<ArrayMesh> GodotMeshUtils::get_quad_mesh_xy(float size, bool invert)
    {
        PackedVector3Array vertices;

        float vertex_pos = 0.5f * size;

        vertices.push_back(Vector3(-vertex_pos, vertex_pos, 0.0f));
        vertices.push_back(Vector3(vertex_pos, vertex_pos, 0.0f));
        vertices.push_back(Vector3(-vertex_pos, -vertex_pos, 0.0f));
        vertices.push_back(Vector3(vertex_pos, -vertex_pos, 0.0f));

        return get_quad_mesh(vertices, invert);
    }

    Ref<ArrayMesh> GodotMeshUtils::get_quad_mesh_xy(float size_x, float size_y, bool invert)
    {
        PackedVector3Array vertices;

        float vertex_pos_x = 0.5f * size_x;
        float vertex_pos_y = 0.5f * size_y;

        vertices.push_back(Vector3(-vertex_pos_x, vertex_pos_y, 0.0f));
        vertices.push_back(Vector3(vertex_pos_x, vertex_pos_y, 0.0f));
        vertices.push_back(Vector3(-vertex_pos_x, -vertex_pos_y, 0.0f));
        vertices.push_back(Vector3(vertex_pos_x, -vertex_pos_y, 0.0f));

        return get_quad_mesh(vertices, invert);
    }

    Ref<ArrayMesh> GodotMeshUtils::get_quad_mesh_xz(float size_x, float size_z, bool invert)
    {
        PackedVector3Array vertices;

        float vertex_pos_x = 0.5f * size_x;
        float vertex_pos_z = 0.5f * size_z;

        vertices.push_back(Vector3(-vertex_pos_x, 0.0f, vertex_pos_z));
        vertices.push_back(Vector3(vertex_pos_x, 0.0f, vertex_pos_z));
        vertices.push_back(Vector3(-vertex_pos_x, 0.0f, -vertex_pos_z));
        vertices.push_back(Vector3(vertex_pos_x, 0.0f, -vertex_pos_z));

        return get_quad_mesh(vertices, invert);
    }

    Ref<ArrayMesh> GodotMeshUtils::get_quad_mesh(PackedVector3Array &vertices, bool invert)
    {
        PackedVector2Array uvs;
        PackedInt32Array indices;

        uvs.push_back(Vector2(0.0f, 0.0f));
        uvs.push_back(Vector2(1.0f, 0.0f));
        uvs.push_back(Vector2(0.0f, 1.0f));
        uvs.push_back(Vector2(1.0f, 1.0f));

        if (invert)
        {
            indices.push_back(0);
            indices.push_back(3);
            indices.push_back(1);
            indices.push_back(0);
            indices.push_back(2);
            indices.push_back(3);
        }
        else
        {
            indices.push_back(0);
            indices.push_back(1);
            indices.push_back(3);
            indices.push_back(0);
            indices.push_back(3);
            indices.push_back(2);
        }

        ArrayMesh *array_mesh = memnew(ArrayMesh);
        Array arrays;
        arrays.resize(ArrayMesh::ARRAY_MAX);

        arrays[ArrayMesh::ARRAY_VERTEX] = vertices;
        arrays[ArrayMesh::ARRAY_TEX_UV] = uvs;
        arrays[ArrayMesh::ARRAY_INDEX] = indices;

        array_mesh->add_surface_from_arrays(godot::Mesh::PRIMITIVE_TRIANGLES, arrays);

        SurfaceTool *surface_tool = memnew(SurfaceTool);
        surface_tool->create_from(array_mesh, 0);
        surface_tool->generate_normals();

        return surface_tool->commit();
    }

    NavigationSystemCode::Mesh GodotMeshUtils::godot_mesh_to_combined_rts_mesh(Ref<godot::Mesh> &godot_mesh)
    {
        vector<Float3> vertices;
        vector<int> triangles;

        int surface_count = godot_mesh->get_surface_count();

        for (int i = 0; i < surface_count; i++)
        {
            Array array = godot_mesh->surface_get_arrays(i);
            PackedVector3Array vertex_array = array[ArrayMesh::ARRAY_VERTEX];
            PackedInt32Array index_array = array[ArrayMesh::ARRAY_INDEX];

            int triangles_offset = vertices.size();

            for (int j = 0; j < vertex_array.size(); j++)
            {
                vertices.push_back(GodotUtils::to_float3(vertex_array[j]));
            }

            for (int j = 0; j < index_array.size(); j++)
            {
                triangles.push_back(index_array[j] + triangles_offset);
            }
        }

        return NavigationSystemCode::Mesh{
            .vertices = vertices,
            .triangles = triangles};
    }

    Ref<godot::Mesh> GodotMeshUtils::rts_mesh_to_godot_mesh(NavigationSystemCode::Mesh &rts_mesh)
    {
        ArrayMesh *array_mesh = memnew(ArrayMesh);
        Array arrays;
        arrays.resize(ArrayMesh::ARRAY_MAX);

        arrays[ArrayMesh::ARRAY_VERTEX] = GodotUtils::to_packed_vector3_array(rts_mesh.vertices);
        arrays[ArrayMesh::ARRAY_TEX_UV] = GodotUtils::to_packed_vector2_array(rts_mesh.uvs);
        arrays[ArrayMesh::ARRAY_INDEX] = GodotUtils::to_packed_int_array(rts_mesh.triangles);

        array_mesh->add_surface_from_arrays(godot::Mesh::PRIMITIVE_TRIANGLES, arrays);

        SurfaceTool *surface_tool = memnew(SurfaceTool);
        surface_tool->create_from(array_mesh, 0);
        surface_tool->generate_normals();

        return surface_tool->commit();
    }

    void GodotMeshUtils::copy_rts_mesh(NavigationSystemCode::Mesh &original_mesh, NavigationSystemCode::Mesh &new_mesh)
    {
        new_mesh.vertices.resize(original_mesh.vertices.size());
        for (int i = 0; i < original_mesh.vertices.size(); i++)
        {
            new_mesh.vertices[i] = original_mesh.vertices[i];
        }

        new_mesh.uvs.resize(original_mesh.uvs.size());
        for (int i = 0; i < original_mesh.uvs.size(); i++)
        {
            new_mesh.uvs[i] = original_mesh.uvs[i];
        }

        new_mesh.triangles.resize(original_mesh.triangles.size());
        for (int i = 0; i < original_mesh.triangles.size(); i++)
        {
            new_mesh.triangles[i] = original_mesh.triangles[i];
        }
    }

    void GodotMeshUtils::offset_rts_mesh(NavigationSystemCode::Mesh &rts_mesh, const Float3 &offset)
    {
        for (int i = 0; i < rts_mesh.vertices.size(); i++)
        {
            rts_mesh.vertices[i] += offset;
        }
    }

    void GodotMeshUtils::rotate_y_rts_mesh(NavigationSystemCode::Mesh &rts_mesh, const float &y_rotation)
    {
        Float3 up = Float3(0.0f, 1.0f, 0.0f);

        for (int i = 0; i < rts_mesh.vertices.size(); i++)
        {
            rts_mesh.vertices[i] = Float3::rotate(rts_mesh.vertices[i], up, y_rotation);
        }
    }

    void GodotMeshUtils::calculate_triangle_centroids(NavigationSystemCode::Mesh &rts_mesh, vector<Float3> &triangle_centroids)
    {
        int triangles_count = rts_mesh.triangles.size() / 3;
        triangle_centroids.resize(triangles_count);

        for (int i = 0; i < triangles_count; i++)
        {
            int p_a = rts_mesh.triangles[3 * i];
            int p_b = rts_mesh.triangles[3 * i + 1];
            int p_c = rts_mesh.triangles[3 * i + 2];
            triangle_centroids[i] = (rts_mesh.vertices[p_a] + rts_mesh.vertices[p_b] + rts_mesh.vertices[p_c]) / 3.0f;
        }
    }

    void GodotMeshUtils::calculate_triangle_radii(NavigationSystemCode::Mesh &rts_mesh, vector<Float3> &triangle_centroids, vector<float> &triangle_radii)
    {
        int triangles_count = rts_mesh.triangles.size() / 3;
        triangle_radii.resize(triangles_count);

        for (int i = 0; i < triangles_count; i++)
        {
            int p_a = rts_mesh.triangles[3 * i];
            int p_b = rts_mesh.triangles[3 * i + 1];
            int p_c = rts_mesh.triangles[3 * i + 2];

            Float3 centroid = triangle_centroids[i];

            float radius = (rts_mesh.vertices[p_a] - centroid).length();
            radius = MathUtils::max(radius, (rts_mesh.vertices[p_b] - centroid).length());
            radius = MathUtils::max(radius, (rts_mesh.vertices[p_c] - centroid).length());

            triangle_radii[i] = radius;
        }
    }

    float GodotMeshUtils::calculate_mesh_radii(NavigationSystemCode::Mesh &rts_mesh)
    {
        float radius = 0.0f;

        for (int i = 0; i < rts_mesh.vertices.size(); i++)
        {
            radius = MathUtils::max(radius, rts_mesh.vertices[i].length());
        }

        return radius;
    }

    void GodotMeshUtils::combined_mesh_indices_test(NavigationSystemCode::Mesh &rts_mesh, const string &key)
    {
        int i1 = 0;
        int i2 = 0;
        vector<int> triangle_referred_vertices_count;

        for (int i = 0; i < rts_mesh.vertices.size(); i++)
        {
            triangle_referred_vertices_count.push_back(0);
        }

        for (int i = 0; i < rts_mesh.triangles.size(); i++)
        {
            if (rts_mesh.triangles[i] >= rts_mesh.vertices.size())
            {
                i1++;
            }
            else if (rts_mesh.triangles[i] < 0)
            {
                i2++;
            }
            else
            {
                int triangle = rts_mesh.triangles[i];
                triangle_referred_vertices_count[triangle]++;
            }
        }

        int i3 = 0;
        for (int i = 0; i < triangle_referred_vertices_count.size(); i++)
        {
            if (triangle_referred_vertices_count[i] == 0)
            {
                i3++;
            }
        }

        GodotUtils::print(key + " " + to_string(rts_mesh.vertices.size()) + " " + to_string(rts_mesh.triangles.size()) + " | " + to_string(i1) + " " + to_string(i2) + " " + to_string(i3));
    }
}
