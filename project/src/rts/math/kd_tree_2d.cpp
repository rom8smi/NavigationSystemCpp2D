#include "kd_tree_2d.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"
#include "math_utils.hpp"
#include "heap_sort.hpp"

namespace NavigationSystemCode
{
    KdTree2D::KdTree2D()
    {
        size = 0;
    }

    void KdTree2D::make_from_points(vector<Float2> &p_points)
    {
        points = p_points;
        make_from_points();
    }

    void KdTree2D::make_from_points()
    {
        size = points.size();

        initial_pivot_index = -1;
        left_childs.resize(size);
        right_childs.resize(size);
        axies.resize(size);
        indices_global.resize(size);

        for (int i = 0; i < size; i++)
        {
            left_childs[i] = -1;
            right_childs[i] = -1;
            axies[i] = -1;
            indices_global[i] = i;
        }

        if (size > 0)
        {
            make_from_points_inner(0, 0, size - 1, -1, -1, true);
        }
    }

    void KdTree2D::make_from_points_inner(int depth, int start_index, int end_index, int parent_pivot_index, int direction, bool is_first_time)
    {
        int axis = depth % 2;
        int split_point = find_pivot_index(start_index, end_index, axis);

        if (is_first_time)
        {
            initial_pivot_index = indices_global[split_point];
        }

        int pivot_index = indices_global[split_point];
        axies[pivot_index] = axis;

        if (parent_pivot_index > -1)
        {
            if (direction == 0)
            {
                left_childs[parent_pivot_index] = pivot_index;
            }
            else if (direction == 1)
            {
                right_childs[parent_pivot_index] = pivot_index;
            }
        }

        int left_end_index = split_point - 1;

        if (left_end_index >= start_index)
        {
            make_from_points_inner(depth + 1, start_index, left_end_index, pivot_index, 0, false);
        }

        int right_start_index = split_point + 1;

        if (right_start_index <= end_index)
        {
            make_from_points_inner(depth + 1, right_start_index, end_index, pivot_index, 1, false);
        }
    }

    void KdTree2D::swap_elements(int a, int b)
    {
        int temp = indices_global[a];
        indices_global[a] = indices_global[b];
        indices_global[b] = temp;
    }

    int KdTree2D::find_split_point(int start_index, int end_index, int axis)
    {
        int start_index_global = indices_global[start_index];
        int end_index_global = indices_global[end_index];

        float a = points[start_index_global].get_axis(axis);
        float b = points[end_index_global].get_axis(axis);
        int mid_index = (start_index + end_index) / 2;
        int mid_index_global = indices_global[mid_index];

        float m = points[mid_index_global].get_axis(axis);

        if (a > b)
        {
            if (m > a)
            {
                return start_index;
            }

            if (b > m)
            {
                return end_index;
            }

            return mid_index;
        }
        else
        {
            if (a > m)
            {
                return start_index;
            }

            if (m > b)
            {
                return end_index;
            }

            return mid_index;
        }
    }

    int KdTree2D::find_pivot_index(int start_index, int end_index, int axis)
    {
        int split_point = find_split_point(start_index, end_index, axis);

        int initial_index = indices_global[split_point];
        Float2 pivot = points[initial_index];
        swap_elements(start_index, split_point);

        int current_pt = start_index + 1;
        int end_pt = end_index;

        while (current_pt <= end_pt)
        {
            int curr_pt_index = indices_global[current_pt];
            Float2 curr = points[curr_pt_index];

            if (curr.get_axis(axis) > pivot.get_axis(axis))
            {
                swap_elements(current_pt, end_pt);
                end_pt--;
            }
            else
            {
                swap_elements(current_pt - 1, current_pt);
                current_pt++;
            }
        }

        return current_pt - 1;
    }

    KdTreeNeighbourResult KdTree2D::find_nearest(Float2 &query)
    {
        float best_sq_distance = 1000000.0f;
        int best_index = -1;

        if (size > 0)
        {
            search(query, best_sq_distance, best_index, initial_pivot_index);
        }

        return KdTreeNeighbourResult{
            .index = best_index,
            .sq_distance = best_sq_distance};
    }

    void KdTree2D::find_nearests_ball(Float2 &query, float r, vector<int> &neighbours)
    {
        if (size == 0)
        {
            return;
        }

        float bestSqDist = MathUtils::FLOAT_MAX_VALUE;
        int bestIndex = -1;
        float rSq = r * r;
        search_ball(query, bestSqDist, rSq, bestIndex, initial_pivot_index, neighbours);
    }

    void KdTree2D::find_nearests_ball_sort(Float2 &query, float r, vector<int> &neighbours)
    {
        if (size == 0)
        {
            return;
        }

        float bestSqDist = MathUtils::FLOAT_MAX_VALUE;
        int bestIndex = -1;
        float rSq = r * r;
        vector<float> sq_distances;
        search_ball(query, bestSqDist, rSq, bestIndex, initial_pivot_index, sq_distances, neighbours);
        HeapSort::Sort(neighbours, sq_distances);
    }

    void KdTree2D::search(Float2 &query, float &best_sq_distance, int &best_index, int pind)
    {
        Float2 current_point = points[pind];
        int left_child = left_childs[pind];
        int right_child = right_childs[pind];
        int axis = axies[pind];

        Float2 relative = current_point - query;
        float sq_distance = relative.length_squared();

        if (sq_distance < best_sq_distance)
        {
            best_sq_distance = sq_distance;
            best_index = pind;
        }

        float plane_distance = query.get_axis(axis) - current_point.get_axis(axis);

        int selector = plane_distance <= 0 ? 0 : 1;
        int child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        if (child_index > -1)
        {
            search(query, best_sq_distance, best_index, child_index);
        }

        selector = (selector + 1) % 2;
        child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        float sq_plane_distance = plane_distance * plane_distance;

        if (child_index > -1 && best_sq_distance > sq_plane_distance)
        {
            search(query, best_sq_distance, best_index, child_index);
        }
    }

    void KdTree2D::search_k(Float2 &query, float &best_sq_distance, float &min_sq_distance, int &best_index, int pind)
    {
        Float2 current_point = points[pind];
        int left_child = left_childs[pind];
        int right_child = right_childs[pind];
        int axis = axies[pind];

        Float2 relative = current_point - query;
        float sq_distance = relative.length_squared();

        if (sq_distance < best_sq_distance)
        {
            if (sq_distance > min_sq_distance)
            {
                best_sq_distance = sq_distance;
                best_index = pind;
            }
        }

        float plane_distance = query.get_axis(axis) - current_point.get_axis(axis);

        int selector = plane_distance <= 0 ? 0 : 1;
        int child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        if (child_index > -1)
        {
            search_k(query, best_sq_distance, min_sq_distance, best_index, child_index);
        }

        selector = (selector + 1) % 2;
        child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        float sq_plane_distance = plane_distance * plane_distance;

        if (child_index > -1 && best_sq_distance > sq_plane_distance)
        {
            search_k(query, best_sq_distance, min_sq_distance, best_index, child_index);
        }
    }

    void KdTree2D::search_ball(
        Float2 &query,
        float &best_sq_distance,
        float &min_sq_distance,
        int &best_index,
        int pind,
        vector<int> &neighbours)
    {
        Float2 current_point = points[pind];
        int left_child = left_childs[pind];
        int right_child = right_childs[pind];
        int axis = axies[pind];

        Float2 relative = current_point - query;
        float sq_distance = relative.length_squared();

        if (sq_distance < best_sq_distance)
        {
            best_sq_distance = sq_distance;
            best_index = pind;

            if (sq_distance <= min_sq_distance)
            {
                neighbours.push_back(best_index);
                best_sq_distance = min_sq_distance;
            }
        }

        float plane_distance = query.get_axis(axis) - current_point.get_axis(axis);

        int selector = plane_distance <= 0 ? 0 : 1;
        int child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        if (child_index > -1)
        {
            search_ball(query, best_sq_distance, min_sq_distance, best_index, child_index, neighbours);
        }

        selector = (selector + 1) % 2;
        child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        float sq_plane_distance = plane_distance * plane_distance;

        if (child_index > -1 && best_sq_distance > sq_plane_distance)
        {
            search_ball(query, best_sq_distance, min_sq_distance, best_index, child_index, neighbours);
        }
    }

    void KdTree2D::search_ball(
        Float2 &query,
        float &best_sq_distance,
        float &min_sq_distance,
        int &best_index,
        int pind,
        vector<float> &sq_distances,
        vector<int> &neighbours)
    {
        Float2 current_point = points[pind];
        int left_child = left_childs[pind];
        int right_child = right_childs[pind];
        int axis = axies[pind];

        Float2 relative = current_point - query;
        float sq_distance = relative.length_squared();

        if (sq_distance < best_sq_distance)
        {
            best_sq_distance = sq_distance;
            best_index = pind;

            if (sq_distance <= min_sq_distance)
            {
                neighbours.push_back(best_index);
                sq_distances.push_back(sq_distance);
                best_sq_distance = min_sq_distance;
            }
        }

        float plane_distance = query.get_axis(axis) - current_point.get_axis(axis);

        int selector = plane_distance <= 0 ? 0 : 1;
        int child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        if (child_index > -1)
        {
            search_ball(query, best_sq_distance, min_sq_distance, best_index, child_index, sq_distances, neighbours);
        }

        selector = (selector + 1) % 2;
        child_index = -1;

        if (selector == 0)
        {
            child_index = left_child;
        }
        else if (selector == 1)
        {
            child_index = right_child;
        }

        float sq_plane_distance = plane_distance * plane_distance;

        if (child_index > -1 && best_sq_distance > sq_plane_distance)
        {
            search_ball(query, best_sq_distance, min_sq_distance, best_index, child_index, sq_distances, neighbours);
        }
    }

    void KdTree2D::clear()
    {
        size = 0;
        initial_pivot_index = -1;
        points.clear();
        left_childs.clear();
        right_childs.clear();
        axies.clear();
        indices_global.clear();
    }
}
