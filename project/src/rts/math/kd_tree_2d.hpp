#ifndef KD_TREE_2D_HPP
#define KD_TREE_2D_HPP

#include <vector>
#include "float2.hpp"
#include "kd_tree_neighbour_result.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct KdTree2D
    {
        int initial_pivot_index;
        vector<Float2> points;
        vector<int> left_childs;
        vector<int> right_childs;
        vector<int> axies;
        vector<int> indices_global;
        int size;

        KdTree2D();
        void make_from_points(vector<Float2> &p_points);
        void make_from_points();
        void make_from_points_inner(int depth, int start_index, int end_index, int parent_pivot_index, int direction, bool is_first_time);
        void swap_elements(int a, int b);
        int find_split_point(int start_index, int end_index, int axis);
        int find_pivot_index(int start_index, int end_index, int axis);
        KdTreeNeighbourResult find_nearest(Float2 &query);
        void find_nearests_ball(Float2 &query, float r, vector<int> &neighbours);
        void find_nearests_ball_sort(Float2 &query, float r, vector<int> &neighbours);
        void search(Float2 &query, float &best_sq_distance, int &best_index, int pind);
        void search_k(Float2 &query, float &best_sq_distance, float &min_sq_distance, int &best_index, int pind);
        void search_ball(
            Float2 &query,
            float &best_sq_distance,
            float &min_sq_distance,
            int &best_index,
            int pind,
            vector<int> &neighbours);
        void search_ball(
            Float2 &query,
            float &best_sq_distance,
            float &min_sq_distance,
            int &best_index,
            int pind,
            vector<float> &sq_distances,
            vector<int> &neighbours);
        void clear();
    };
}

#endif
