#include "obstacle_utils.hpp"
#include "math_utils.hpp"
#include "vector_utils.hpp"

namespace NavigationSystemCode
{
    Obstacle ObstacleUtils::Create(
        vector<Float2> &p_obstacleCorners,
        bool p_isWalkable)
    {
        Obstacle obstacle;
        obstacle.obstacleCorners = p_obstacleCorners;
        obstacle.isWalkable = p_isWalkable;
        int minSplits = 0;

        for (int i = 0; i < obstacle.obstacleCorners.size(); i++)
        {
            obstacle.isCornerIntersectingWithWorldBounds.push_back(false);
        }

        int obstacle_corners_size = obstacle.obstacleCorners.size();

        for (int i = 0; i < obstacle_corners_size; i++)
        {
            obstacle.nSplits.push_back(minSplits);
        }

        obstacle.center = Float2(0.0f, 0.0f);
        for (int i = 0; i < obstacle_corners_size; i++)
        {
            obstacle.center += obstacle.obstacleCorners[i];
        }
        obstacle.center = obstacle.center / obstacle_corners_size;

        float largest_corner_distance_sqr = 0.0f;
        for (int i = 0; i < obstacle_corners_size; i++)
        {
            float corner_distance_sqr = (obstacle.obstacleCorners[i] - obstacle.center).length_squared();
            if (corner_distance_sqr > largest_corner_distance_sqr)
            {
                largest_corner_distance_sqr = corner_distance_sqr;
            }
        }
        obstacle.largest_corner_distance = MathUtils::sqrt(largest_corner_distance_sqr);

        return obstacle;
    }

    Obstacle ObstacleUtils::Create(
        vector<Float2> &p_obstacleCorners,
        Aabb &p_worldBounds,
        float clip_line_length,
        bool p_isWalkable)
    {
        Obstacle obstacle;
        obstacle.obstacleCorners = p_obstacleCorners;
        obstacle.isWalkable = p_isWalkable;
        int minSplits = 0;

        for (int i = 0; i < obstacle.obstacleCorners.size(); i++)
        {
            obstacle.isCornerIntersectingWithWorldBounds.push_back(false);
        }

        Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, p_worldBounds.minY, 0.0f, clip_line_length);
        Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, -p_worldBounds.maxY, MathUtils::PI, clip_line_length);
        Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, p_worldBounds.minX, -0.5f * MathUtils::PI, clip_line_length);
        Clip(obstacle.obstacleCorners, obstacle.isCornerIntersectingWithWorldBounds, -p_worldBounds.maxX, 0.5f * MathUtils::PI, clip_line_length);

        int obstacle_corners_size = obstacle.obstacleCorners.size();

        for (int i = 0; i < obstacle_corners_size; i++)
        {
            obstacle.nSplits.push_back(minSplits);
        }

        obstacle.center = Float2(0.0f, 0.0f);
        for (int i = 0; i < obstacle_corners_size; i++)
        {
            obstacle.center += obstacle.obstacleCorners[i];
        }
        obstacle.center = obstacle.center / obstacle_corners_size;

        float largest_corner_distance_sqr = 0.0f;
        for (int i = 0; i < obstacle_corners_size; i++)
        {
            float corner_distance_sqr = (obstacle.obstacleCorners[i] - obstacle.center).length_squared();
            if (corner_distance_sqr > largest_corner_distance_sqr)
            {
                largest_corner_distance_sqr = corner_distance_sqr;
            }
        }
        obstacle.largest_corner_distance = MathUtils::sqrt(largest_corner_distance_sqr);

        return obstacle;
    }

    void ObstacleUtils::Clip(
        vector<Float2> &p_obstacleCorners,
        vector<bool> &p_isCornerIntersectingWithWorldBounds,
        float minX,
        float rotation,
        float clip_line_length)
    {
        vector<Float2> potentialCorners;
        vector<bool> potentialIntersectionPoints;
        vector<bool> potentialOldIntersectionPoints;

        int obstacleCornersCount = p_obstacleCorners.size();

        for (int i = 0; i < obstacleCornersCount; i++)
        {
            int i1 = i;
            int i2 = i + 1;
            if (i2 >= obstacleCornersCount)
            {
                i2 -= obstacleCornersCount;
            }

            Float2 p1 = p_obstacleCorners[i1];
            Float2 p2 = p_obstacleCorners[i2];

            Float2 p1rotated = p1.rotated(rotation);
            Float2 p2rotated = p2.rotated(rotation);

            bool isIntersecting = p_isCornerIntersectingWithWorldBounds[i1];

            Float2 left = Float2(minX, -clip_line_length);
            Float2 right = Float2(minX, clip_line_length);

            LineSegmentsIntersectionResult lineSegmentsIntersectionResult = VectorUtils::LineSegmentsIntersection(
                p1rotated,
                p2rotated,
                left,
                right);

            if (lineSegmentsIntersectionResult.intersects)
            {
                if (p1rotated.x > minX)
                {
                    potentialCorners.push_back(p1rotated);
                    potentialIntersectionPoints.push_back(isIntersecting);
                    potentialOldIntersectionPoints.push_back(true);

                    potentialCorners.push_back(lineSegmentsIntersectionResult.intersection);
                    potentialIntersectionPoints.push_back(true);
                    potentialOldIntersectionPoints.push_back(false);
                }
                else if (p1rotated.x < minX)
                {
                    potentialCorners.push_back(lineSegmentsIntersectionResult.intersection);
                    potentialIntersectionPoints.push_back(true);
                    potentialOldIntersectionPoints.push_back(false);

                    potentialCorners.push_back(p1rotated);
                    potentialIntersectionPoints.push_back(isIntersecting);
                    potentialOldIntersectionPoints.push_back(true);
                }
                else
                {
                    potentialCorners.push_back(p1rotated);
                    potentialIntersectionPoints.push_back(isIntersecting);
                    potentialOldIntersectionPoints.push_back(true);
                }
            }
            else
            {
                potentialCorners.push_back(p1rotated);
                potentialIntersectionPoints.push_back(isIntersecting);
                potentialOldIntersectionPoints.push_back(true);
            }
        }

        p_obstacleCorners.clear();
        p_isCornerIntersectingWithWorldBounds.clear();

        for (int i = 0; i < potentialCorners.size(); i++)
        {
            if (potentialCorners[i].x > minX || (potentialIntersectionPoints[i] && !potentialOldIntersectionPoints[i]))
            {
                Float2 corner = potentialCorners[i].rotated(-rotation);
                p_obstacleCorners.push_back(corner);
                p_isCornerIntersectingWithWorldBounds.push_back(potentialIntersectionPoints[i]);
            }
        }
    }

    void ObstacleUtils::rectangle_with_offset_and_rotation(
        float width,
        float height,
        Float2 &offset_before_rotation,
        Float2 &offset_after_rotation,
        float rotation,
        vector<Float2> &rectangle)
    {
        rectangle.resize(4);

        rectangle[0] = offset_after_rotation + (Float2(-width, -height) + offset_before_rotation).rotated(rotation);
        rectangle[1] = offset_after_rotation + (Float2(width, -height) + offset_before_rotation).rotated(rotation);
        rectangle[2] = offset_after_rotation + (Float2(width, height) + offset_before_rotation).rotated(rotation);
        rectangle[3] = offset_after_rotation + (Float2(-width, height) + offset_before_rotation).rotated(rotation);
    }

    vector<Float2> ObstacleUtils::rectangle_with_cut_corners(float width, float height, float gap, Float2 &offset)
    {
        return {
            Float2(-width + gap, -height) + offset,
            Float2(width - gap, -height) + offset,
            Float2(width, -height + gap) + offset,
            Float2(width, height - gap) + offset,
            Float2(width - gap, height) + offset,
            Float2(-width + gap, height) + offset,
            Float2(-width, height - gap) + offset,
            Float2(-width, -height + gap) + offset};
    }

    bool ObstacleUtils::are_all_corners_inside_bounds(vector<Float2> &corners, Aabb &bounds)
    {
        for (int i = 0; i < corners.size(); i++)
        {
            if (!bounds.IsInside(corners[i]))
            {
                return false;
            }
        }
        return true;
    }

    bool ObstacleUtils::are_obstacles_intersecting(vector<Float2> &corners_a, vector<Float2> &corners_b)
    {
        int corners_a_size = corners_a.size();

        for (int i = 0; i < corners_a_size; i++)
        {
            if (VectorUtils::IsPointInPolygon(corners_a[i], corners_b))
            {
                return true;
            }
        }

        int corners_b_size = corners_b.size();

        for (int i = 0; i < corners_b_size; i++)
        {
            if (VectorUtils::IsPointInPolygon(corners_b[i], corners_a))
            {
                return true;
            }
        }

        for (int i = 0; i < corners_a_size; i++)
        {
            int i_next = i + 1;
            if (i_next == corners_a_size)
            {
                i_next = 0;
            }
            Float2 p_a = corners_a[i];
            Float2 q_a = corners_a[i_next];

            for (int j = 0; j < corners_b_size; j++)
            {
                int j_next = j + 1;
                if (j_next == corners_b_size)
                {
                    j_next = 0;
                }
                Float2 p_b = corners_b[j];
                Float2 q_b = corners_b[j_next];

                if (VectorUtils::AreLineSegmentsIntersecting(p_a, q_a, p_b, q_b))
                {
                    return true;
                }
            }
        }

        return false;
    }

    bool ObstacleUtils::try_get_random_point_inside_obstacle(Obstacle &obstacle, Random &random, Float2 &point_inside_obstacle)
    {
        Float2 point = VectorUtils::random_inside_unit_circle(random) * obstacle.largest_corner_distance + obstacle.center;
        if (VectorUtils::IsPointInPolygon(point, obstacle.obstacleCorners))
        {
            point_inside_obstacle = point;
            return true;
        }

        return false;
    }
}
