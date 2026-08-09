#ifndef GODOT_MULTIMESH_HPP
#define GODOT_MULTIMESH_HPP

#include <vector>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include "rts/math/float3.hpp"
#include "rts/math/float_3x3.hpp"
#include "rts/math/rgba.hpp"
#include "godot_multi_mesh_wrapper.hpp"
#include "godot_multi_mesh_instance3d_wrapper.hpp"

using namespace godot;

namespace NavigationSystemCode
{
    struct GodotMultimesh
	{
        int start_index;
        int update_count;
        bool use_colors;
        bool use_custom_data;
        int total_instance_count;
        int instance_count;
        int buffer_count;
        int total_buffer_count;
        GodotMultiMeshWrapper multi_mesh;
        GodotMultiMeshInstance3DWrapper multi_mesh_instance;
        PackedFloat32Array buffer;
        int buffer_chunk_size;
        bool buffer_dirty;
        int instance_count_max_offset;

        GodotMultimesh();
        void set_use_colors(bool value);
        void set_use_custom_data(bool value);
        int get_used_instance_count();
        void set_instance_count(int count);
        void update_instance_position(int index, const Float3 &position);
        void add_to_instance_position_y(int index, float dy);
        void update_instance_basis(int index, const Float3x3 &basis);
        void update_instance_position_basis(int index, const Float3 &position, const Float3x3 &basis);
        void update_instance_color(int index, const Rgba &color);
        void update_instance_color_alpha(int index, float alpha);
        void update_instance_custom_data(int index, const Rgba &custom_data);
        void update_instance_custom_data_alpha(int index, float custom_data_alpha);
        void add_instance_to_buffer();
        void clear_buffer();
        void resize_buffer(int size);
        void set_buffer();
        void update_buffer_chunk_size();
        void remove_at(int index);
        void remove_by_mask(const vector<int> &mask, bool copy_old_values);
        int get_instance_count_max_offset(int new_count);
        void create_shader_compile_instances(bool use_visible_position);
        void remove_shader_compile_instances();
        void set_visible(bool visible);
        void reset();
    };
}

#endif
