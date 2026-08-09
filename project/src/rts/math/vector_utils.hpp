#ifndef VECTOR_UTILS_HPP
#define VECTOR_UTILS_HPP

#include "float2.hpp"
#include "float3.hpp"
#include <vector>
#include "line_segment_intersection_result.hpp"
#include "random.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct VectorUtils
    {
        static LineSegmentsIntersectionResult LineSegmentsIntersection(Float2 &a1, Float2 &a2, Float2 &b1, Float2 &b2);
        static LineSegmentsIntersectionResult LineSegmentsIntersection(Float2 &a1, Float2 &a2, Float2 &b1, Float2 &b2, float epsilon);
        static bool AreLineSegmentsIntersecting(Float2 &a1, Float2 &a2, Float2 &b1, Float2 &b2);
        static Float2 find_nearest_point_on_line(Float2 &origin, Float2 &direction, Float2 &point);
        static Float2 FindNearestPointOnLineSegment(Float2 &start, Float2 &end, Float2 &point);
        static float find_nearest_point_distance_sqr_for_lines(Float2 &position, vector<Float2> &line_points);
        static Float2 PerpendicularCounterClockwise(Float2 &vector2);
        static bool IsPointInPolygon(Float2 &p, vector<Float2> &polygon);
        static bool PointInTriangle(Float2 &p, Float2 &p0, Float2 &p1, Float2 &p2);
        static Float2 AdjustForBoundaries(Float2 &p_position, float minX, float maxX, float minY, float maxY, float epsilon);
        static Float2 AdjustForBoundaries(Float2 &p_position, float minX, float maxX, float minY, float maxY, float epsilon, bool &wasAdjusted);
        static bool AreLineSegmentsCollinearAndOverlapping(Float2 &ap, Float2 &aq, Float2 &bp, Float2 &bq, float epsilon);
        static bool PointOnLine2D(Float2 &p, Float2 &a, Float2 &b, float epsilon);
        static bool IsPointCollinearToLineSegment(Float2 &p, Float2 &a, Float2 &b, float epsilon);
        static bool IsPointWithinLineSegment(Float2 &p, Float2 &a, Float2 &b, float epsilon);
        static bool try_get_line_segment_triangle_intersection(
            Float3 &a,
            Float3 &b,
            Float3 &c,
            Float3 &line_segment_start,
            Float3 &line_segment_end,
            Float3 &intersection);
        static bool try_get_ray_triangle_intersection(
            Float3 &a,
            Float3 &b,
            Float3 &c,
            Float3 &ray_origin,
            Float3 &ray_vector,
            Float3 &intersection);
        static Float2 random_on_unit_circle(Random &random);
        static Float2 random_inside_unit_circle(Random &random);
        static Float3 random_on_unit_sphere(Random &random);
        static Float3 random_inside_unit_sphere(Random &random);
        static Float3 random_inside_box_x0z(Random &random);
        static Float3 random_inside_box_x0z(Random &random, Float3 &center, float scale);
        static Float2 middle_point(vector<Float2> &points);
        static Float2 float3_xz(const Float3 &v);
        static Float3 float2_x0z(Float2 &v);
        static float float3_xz_look_at_to_angle(Float2 &xz);
        static Float2 float3_xz_angle_to_look_at(float angle);
        static float radius_on_elipse(float angle, float a, float b);
        static Float3 closest_point_on_triangle(Float3 &p, Float3 &a, Float3 &b, Float3 &c);
        static Float3 random_point_inside_triangle(Float3 &a, Float3 &b, Float3 &c, Random &random);
        static float surface_area_of_triangle(Float3 &a, Float3 &b, Float3 &c);
        static void find_rotation_axis_and_angle(Float3 &direction_a, Float3 &direction_b, bool &axis_found, Float3 &axis, float &axis_rotation);
    };
}

#endif
