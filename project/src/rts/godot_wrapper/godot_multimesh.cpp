#include "godot_multimesh.hpp"

namespace NavigationSystemCode
{
    GodotMultimesh::GodotMultimesh()
    {
        start_index = 0;
        update_count = 0;
        instance_count = 0;
        total_instance_count = 0;
        buffer_count = 0;
        total_buffer_count = 0;
        buffer_dirty = false;
        instance_count_max_offset = 10;
    }

    void GodotMultimesh::set_use_colors(bool value)
    {
        use_colors = value;
        multi_mesh.set_use_colors(value);
    }

    void GodotMultimesh::set_use_custom_data(bool value)
    {
        use_custom_data = value;
        multi_mesh.set_use_custom_data(value);
    }

    int GodotMultimesh::get_used_instance_count()
    {
        return instance_count - start_index;
    }

    void GodotMultimesh::set_instance_count(int count)
    {
        instance_count = start_index + count;
        int old_total_instance_count = total_instance_count;
        total_instance_count = get_instance_count_max_offset(instance_count);

        if (old_total_instance_count != total_instance_count)
        {
            multi_mesh.set_instance_count(total_instance_count);
        }
        multi_mesh.set_visible_instance_count(instance_count);
    }

    void GodotMultimesh::update_instance_position(int index, const Float3 &position)
    {
        int i_start = (start_index + index) * buffer_chunk_size;

        buffer[i_start + 3] = position.x;
        buffer[i_start + 7] = position.y;
        buffer[i_start + 11] = position.z;
        buffer_dirty = true;
    }

    void GodotMultimesh::add_to_instance_position_y(int index, float dy)
    {
        int i_start = (start_index + index) * buffer_chunk_size;
        buffer[i_start + 7] += dy;
        buffer_dirty = true;
    }

    void GodotMultimesh::update_instance_basis(int index, const Float3x3 &basis)
    {
        int i_start = (start_index + index) * buffer_chunk_size;

        buffer[i_start] = basis.xx;
        buffer[i_start + 1] = basis.xy;
        buffer[i_start + 2] = basis.xz;

        buffer[i_start + 4] = basis.yx;
        buffer[i_start + 5] = basis.yy;
        buffer[i_start + 6] = basis.yz;

        buffer[i_start + 8] = basis.zx;
        buffer[i_start + 9] = basis.zy;
        buffer[i_start + 10] = basis.zz;

        buffer_dirty = true;
    }

    void GodotMultimesh::update_instance_position_basis(int index, const Float3 &position, const Float3x3 &basis)
    {
        int i_start = (start_index + index) * buffer_chunk_size;

        buffer[i_start] = basis.xx;
        buffer[i_start + 1] = basis.xy;
        buffer[i_start + 2] = basis.xz;
        buffer[i_start + 3] = position.x;

        buffer[i_start + 4] = basis.yx;
        buffer[i_start + 5] = basis.yy;
        buffer[i_start + 6] = basis.yz;
        buffer[i_start + 7] = position.y;

        buffer[i_start + 8] = basis.zx;
        buffer[i_start + 9] = basis.zy;
        buffer[i_start + 10] = basis.zz;
        buffer[i_start + 11] = position.z;

        buffer_dirty = true;
    }

    void GodotMultimesh::update_instance_color(int index, const Rgba &color)
    {
        int i_start = (start_index + index) * buffer_chunk_size;

        buffer[i_start + 12] = color.r;
        buffer[i_start + 13] = color.g;
        buffer[i_start + 14] = color.b;
        buffer[i_start + 15] = color.a;

        buffer_dirty = true;
    }

    void GodotMultimesh::update_instance_color_alpha(int index, float alpha)
    {
        int i_start = (start_index + index) * buffer_chunk_size;
        buffer[i_start + 15] = alpha;
        buffer_dirty = true;
    }

    void GodotMultimesh::update_instance_custom_data(int index, const Rgba &custom_data)
    {
        int i_start = (start_index + index) * buffer_chunk_size;

        buffer[i_start + 16] = custom_data.r;
        buffer[i_start + 17] = custom_data.g;
        buffer[i_start + 18] = custom_data.b;
        buffer[i_start + 19] = custom_data.a;

        buffer_dirty = true;
    }

    void GodotMultimesh::update_instance_custom_data_alpha(int index, float custom_data_alpha)
    {
        int i_start = (start_index + index) * buffer_chunk_size;
        buffer[i_start + 19] = custom_data_alpha;
        buffer_dirty = true;
    }

    void GodotMultimesh::add_instance_to_buffer()
    {
        int old_buffer_count = buffer_count;
        resize_buffer(old_buffer_count + buffer_chunk_size);

        for (int i = 0; i < buffer_chunk_size; i++)
        {
            buffer[old_buffer_count + i] = 0.0f;
        }
        buffer_dirty = true;
    }

    void GodotMultimesh::clear_buffer()
    {
        resize_buffer(start_index * buffer_chunk_size);
        buffer_dirty = true;
    }

    void GodotMultimesh::resize_buffer(int size)
    {
        buffer_count = size;
        int old_total_buffer_count = total_buffer_count;
        total_buffer_count = get_instance_count_max_offset(buffer_count / buffer_chunk_size) * buffer_chunk_size;

        if (old_total_buffer_count != total_buffer_count)
        {
            buffer.resize(total_buffer_count);
        }
    }

    void GodotMultimesh::set_buffer()
    {
        if (instance_count > 0)
        {
            multi_mesh.set_buffer(buffer);
        }
        buffer_dirty = false;
    }

    void GodotMultimesh::update_buffer_chunk_size()
    {
        multi_mesh.set_instance_count(1);
        multi_mesh.set_visible_instance_count(1);

        PackedFloat32Array b = multi_mesh.get_buffer();
        buffer_chunk_size = b.size();

        multi_mesh.set_instance_count(0);
        multi_mesh.set_visible_instance_count(0);
    }

    void GodotMultimesh::remove_at(int index)
    {
        int i_new = 0;
        int used_instance_count = get_used_instance_count();

        for (int i = 0; i < used_instance_count; i++)
        {
            int i_start = i + start_index;
            int i_start_new = i_new + start_index;

            if (i != index)
            {
                int buffer_start = i_start * buffer_chunk_size;
                int buffer_start_new = i_start_new * buffer_chunk_size;

                for (int j = 0; j < buffer_chunk_size; j++)
                {
                    int j_old = buffer_start + j;
                    int j_new = buffer_start_new + j;
                    buffer[j_new] = buffer[j_old];
                }

                i_new++;
            }
        }

        resize_buffer((i_new + start_index) * buffer_chunk_size);
        set_instance_count(i_new);
    }

    void GodotMultimesh::remove_by_mask(const vector<int> &mask, bool copy_old_values)
    {
        int i_new = 0;
        int used_instance_count = get_used_instance_count();

        for (int i = 0; i < used_instance_count; i++)
        {
            int i_start = i + start_index;
            int i_start_new = i_new + start_index;

            if (mask[i] != -1)
            {
                if (copy_old_values)
                {
                    int buffer_start = i_start * buffer_chunk_size;
                    int buffer_start_new = i_start_new * buffer_chunk_size;

                    for (int j = 0; j < buffer_chunk_size; j++)
                    {
                        int j_old = buffer_start + j;
                        int j_new = buffer_start_new + j;
                        buffer[j_new] = buffer[j_old];
                    }
                }

                i_new++;
            }
        }

        resize_buffer((i_new + start_index) * buffer_chunk_size);
        set_instance_count(i_new);
    }

    int GodotMultimesh::get_instance_count_max_offset(int new_count)
    {
        int result = ((int)((1.0f * (new_count - 1)) / instance_count_max_offset) + 1) * instance_count_max_offset;
        return result;
    }

    void GodotMultimesh::create_shader_compile_instances(bool use_visible_position)
    {
        Float3 position;
        if (use_visible_position)
        {
            position = Float3(1431.679443f, 62.830219f, 1494.000000f);
        }
        else
        {
            position = Float3(1132.597900, -76.633812, 1494.000000);
        }

        set_instance_count(1);
        clear_buffer();
        add_instance_to_buffer();

        Float3x3 basis;
        Float3 euler_rotation = Float3(0.0f, 0.0f, 0.0f);
        basis.set_euler(euler_rotation);

        update_instance_position_basis(0, position, basis);

        if (use_colors)
        {
            Rgba color = Rgba(1.0f, 1.0f, 1.0f, 1.0f);
            update_instance_color(0, color);
        }

        if (use_custom_data)
        {
            Rgba custom_data = Rgba(1.0f, 1.0f, 1.0f, 0.0f);
            update_instance_custom_data(0, custom_data);
        }

        set_buffer();
        start_index = 1;
        update_count = 0;
    }

    void GodotMultimesh::remove_shader_compile_instances()
    {
        if (update_count < 2)
        {
            update_count++;
        }
        else if (start_index > 0)
        {
            int i_start = start_index * buffer_chunk_size;
            int new_buffer_size = instance_count * buffer_chunk_size - i_start;

            for (int i = 0; i < new_buffer_size; i++)
            {
                buffer[i] = buffer[i + i_start];
            }

            resize_buffer(new_buffer_size);
            int old_start_index = start_index;
            start_index = 0;
            set_instance_count(instance_count - old_start_index);

            set_buffer();
        }
    }

    void GodotMultimesh::set_visible(bool visible)
    {
        multi_mesh_instance.set_visible(visible);
    }

    void GodotMultimesh::reset()
    {
        resize_buffer(start_index * buffer_chunk_size);
        set_instance_count(0);
    }
}
