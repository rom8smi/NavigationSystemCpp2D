#ifndef OBSTACLE_UTILS_HPP
#define OBSTACLE_UTILS_HPP

#include "float2.hpp"
#include "random.hpp"
#include <vector>
#include "rts/navigation/aabb.hpp"
#include "rts/navigation/obstacle.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ObstacleUtils
	{
        static Obstacle Create(
            vector<Float2> &p_obstacleCorners,
            bool p_isWalkable);
        static Obstacle Create(
            vector<Float2> &p_obstacleCorners,
            Aabb &p_worldBounds,
            float clip_line_length,
            bool p_isWalkable);
        static void Clip(
            vector<Float2> &p_obstacleCorners,
            vector<bool> &p_isCornerIntersectingWithWorldBounds,
            float minX,
            float rotation,
            float clip_line_length);
        static void rectangle_with_offset_and_rotation(
            float width,
            float height,
            Float2 &offset_before_rotation,
            Float2 &offset_after_rotation,
            float rotation,
            vector<Float2> &rectangle);
        static vector<Float2> rectangle_with_cut_corners(float width, float height, float gap, Float2 &offset);
        static bool are_all_corners_inside_bounds(vector<Float2> &corners, Aabb &bounds);
        static bool are_obstacles_intersecting(vector<Float2> &corners_a, vector<Float2> &corners_b);
        static bool try_get_random_point_inside_obstacle(Obstacle &obstacle, Random &random, Float2 &point_inside_obstacle);
    };
}

#endif
