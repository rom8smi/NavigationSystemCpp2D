#include "obstacles_kd_tree.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/math/obstacle_utils.hpp"

namespace NavigationSystemCode
{
    void ObstaclesKdTree::build(vector<Obstacle> &obstacles)
    {
        int obstacles_count = obstacles.size();
        centers.resize(obstacles_count);

        largest_corners_distance = 0.0f;

        for (int i = 0; i < obstacles_count; i++)
        {
            Float2 center = obstacles[i].center;            
            centers[i] = center;
            largest_corners_distance = MathUtils::max(largest_corners_distance, obstacles[i].largest_corner_distance);
        }

        kd_tree.make_from_points(centers);
    }

    bool ObstaclesKdTree::intersects(Obstacle &obstacle, vector<Obstacle> &obstacles)
    {
        int corners_count = obstacle.obstacleCorners.size();
        int obstacles_count = obstacles.size();

        if (obstacles_count == 0 || corners_count == 0)
        {
            return false;
        }

        Float2 center = obstacle.center;
        float obstacle_largest_corner_distance = obstacle.largest_corner_distance;
        float distance_to_search = largest_corners_distance + obstacle_largest_corner_distance + 0.1f;

        vector<int> center_neighbours;
        kd_tree.find_nearests_ball(center, distance_to_search, center_neighbours);

        for (int i = 0; i < center_neighbours.size(); i++)
        {
            int neighbourIndex = center_neighbours[i];

            if (ObstacleUtils::are_obstacles_intersecting(obstacle.obstacleCorners, obstacles[neighbourIndex].obstacleCorners))
            {
                return true;
            }
        }

        return false;
    }

    bool ObstaclesKdTree::intersects_linear(Obstacle &obstacle, vector<Obstacle> &obstacles)
    {
        Float2 center_a = obstacle.center;
        float r_a = obstacle.largest_corner_distance;

        for (int i = 0; i < obstacles.size(); i++)
        {
            Float2 center_b = obstacles[i].center;
            float r_b = obstacles[i].largest_corner_distance;

            float centers_distance = (center_a - center_b).length();
            float largest_center_distance = r_a + r_b + 0.1f;

            if (centers_distance < largest_center_distance)
            {
                if (ObstacleUtils::are_obstacles_intersecting(obstacle.obstacleCorners, obstacles[i].obstacleCorners))
                {
                    return true;
                }
            }
        }
        return false;
    }
}