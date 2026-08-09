#ifndef OBSTACLES_KD_TREE_HPP
#define OBSTACLES_KD_TREE_HPP

#include "rts/math/kd_tree_2d.hpp"
#include <vector>
#include "rts/math/float2.hpp"
#include "obstacle.hpp"

namespace NavigationSystemCode
{
	struct ObstaclesKdTree
	{
		KdTree2D kd_tree;
        vector<Float2> centers;
        float largest_corners_distance;

        void build(vector<Obstacle> &obstacles);
        bool intersects(Obstacle &obstacle, vector<Obstacle> &obstacles);
        bool intersects_linear(Obstacle &obstacle, vector<Obstacle> &obstacles);
    };
}

#endif
