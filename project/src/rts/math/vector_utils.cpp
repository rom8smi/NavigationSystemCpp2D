#include "vector_utils.hpp"
#include "math_utils.hpp"
#include <cmath>

namespace NavigationSystemCode
{
    // Based on https://github.com/setchi/Unity-LineSegmentsIntersection
    LineSegmentsIntersectionResult VectorUtils::LineSegmentsIntersection(Float2 &a1, Float2 &a2, Float2 &b1, Float2 &b2)
    {
        Float2 intersection = Float2(0.0f, 0.0f);

        float d = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);

        if (d == 0.0f)
        {
            return LineSegmentsIntersectionResult{
                .intersects = false,
                .intersection = intersection};
        }

        float u = ((b1.x - a1.x) * (b2.y - b1.y) - (b1.y - a1.y) * (b2.x - b1.x)) / d;
        float v = ((b1.x - a1.x) * (a2.y - a1.y) - (b1.y - a1.y) * (a2.x - a1.x)) / d;

        if (u < 0.0f || u > 1.0f || v < 0.0f || v > 1.0f)
        {
            return LineSegmentsIntersectionResult{
                .intersects = false,
                .intersection = intersection};
        }

        intersection.x = a1.x + u * (a2.x - a1.x);
        intersection.y = a1.y + u * (a2.y - a1.y);

        return LineSegmentsIntersectionResult{
            .intersects = true,
            .intersection = intersection};
    }

    LineSegmentsIntersectionResult VectorUtils::LineSegmentsIntersection(Float2 &a1, Float2 &a2, Float2 &b1, Float2 &b2, float epsilon)
    {
        Float2 intersection = Float2(0.0f, 0.0f);

        float d = (a2.x - a1.x) * (b2.y - b1.y) - (a2.y - a1.y) * (b2.x - b1.x);

        if (d < epsilon && d > -epsilon)
        {
            return LineSegmentsIntersectionResult{
                .intersects = false,
                .intersection = intersection};
        }

        float u = ((b1.x - a1.x) * (b2.y - b1.y) - (b1.y - a1.y) * (b2.x - b1.x)) / d;
        float v = ((b1.x - a1.x) * (a2.y - a1.y) - (b1.y - a1.y) * (a2.x - a1.x)) / d;

        if (u < epsilon || u > 1.0f - epsilon || v < epsilon || v > 1.0f - epsilon)
        {
            return LineSegmentsIntersectionResult{
                .intersects = false,
                .intersection = intersection};
        }

        intersection.x = a1.x + u * (a2.x - a1.x);
        intersection.y = a1.y + u * (a2.y - a1.y);

        return LineSegmentsIntersectionResult{
            .intersects = true,
            .intersection = intersection};
    }

    // Based on https://github.com/setchi/Unity-LineSegmentsIntersection
    bool VectorUtils::AreLineSegmentsIntersecting(Float2 &a1, Float2 &a2, Float2 &b1, Float2 &b2)
    {
        float a1x = a1.x;
        float a1y = a1.y;
        float a2x = a2.x;
        float a2y = a2.y;
        float b1x = b1.x;
        float b1y = b1.y;
        float b2x = b2.x;
        float b2y = b2.y;

        float a2x_a1x = a2x - a1x;
        float b2y_b1y = b2y - b1y;
        float a2y_a1y = a2y - a1y;
        float b2x_b1x = b2x - b1x;

        float d = a2x_a1x * b2y_b1y - a2y_a1y * b2x_b1x;

        if (d == 0.0f)
        {
            return false;
        }

        float b1x_a1x = b1x - a1x;
        float b1y_a1y = b1y - a1y;

        float u = (b1x_a1x * b2y_b1y - b1y_a1y * b2x_b1x) / d;

        if (u < 0.0f || u > 1.0f)
        {
            return false;
        }

        float v = (b1x_a1x * a2y_a1y - b1y_a1y * a2x_a1x) / d;

        if (v < 0.0f || v > 1.0f)
        {
            return false;
        }

        return true;
    }

    // Based on https://stackoverflow.com/questions/51905268/how-to-find-closest-point-on-line
    Float2 VectorUtils::find_nearest_point_on_line(Float2 &origin, Float2 &direction, Float2 &point)
    {
        direction.normalize();
        Float2 lhs = point - origin;

        float dot_product = lhs.dot(direction);
        return origin + (direction * dot_product);
    }

    // Based on https://gamedev.stackexchange.com/questions/172001/shortest-distance-to-chain-of-line-segments
    Float2 VectorUtils::FindNearestPointOnLineSegment(Float2 &start, Float2 &end, Float2 &point)
    {
        // Shift the problem to the origin to simplify the math.
        Float2 startToPoint = point - start;
        Float2 startToEnd = end - start;

        // Compute how far along the line is the closest approach to our point.
        float projectedDistance = startToPoint.dot(startToEnd) / startToEnd.length_squared();

        // Restrict this point to within the line segment from start to end.
        projectedDistance = MathUtils::clamp01(projectedDistance);

        // Return this point.
        return start + (startToEnd * projectedDistance);
    }

    float VectorUtils::find_nearest_point_distance_sqr_for_lines(Float2 &position, vector<Float2> &line_points)
    {
        float min_distance_sqr = MathUtils::FLOAT_MAX_VALUE;
        int line_points_size = line_points.size();

        for (int i = 0; i < line_points_size; i++)
        {
            int i_next = i + 1;
            if (i_next >= line_points_size)
            {
                i_next = 0;
            }

            Float2 line_point = VectorUtils::FindNearestPointOnLineSegment(line_points[i], line_points[i_next], position);
            float distance_sqr = (line_point - position).length_squared();

            if (distance_sqr < min_distance_sqr)
            {
                min_distance_sqr = distance_sqr;
            }
        }

        return min_distance_sqr;
    }

    // Based on https://gamedev.stackexchange.com/questions/70075/how-can-i-find-the-perpendicular-to-a-2d-vector
    Float2 VectorUtils::PerpendicularCounterClockwise(Float2 &vector2)
    {
        return Float2(-vector2.y, vector2.x);
    }

    // Based on https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
    bool VectorUtils::IsPointInPolygon(Float2 &p, vector<Float2> &polygon)
    {
        float minX = polygon[0].x;
        float maxX = polygon[0].x;
        float minY = polygon[0].y;
        float maxY = polygon[0].y;

        for (int i = 1; i < polygon.size(); i++)
        {
            Float2 q = polygon[i];
            minX = MathUtils::min(q.x, minX);
            maxX = MathUtils::max(q.x, maxX);
            minY = MathUtils::min(q.y, minY);
            maxY = MathUtils::max(q.y, maxY);
        }

        if (p.x < minX || p.x > maxX || p.y < minY || p.y > maxY)
        {
            return false;
        }

        // https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
        bool inside = false;
        for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
            if ((polygon[i].y > p.y) != (polygon[j].y > p.y) &&
                p.x < (polygon[j].x - polygon[i].x) * (p.y - polygon[i].y) / (polygon[j].y - polygon[i].y) + polygon[i].x)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    // Based on https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
    bool VectorUtils::PointInTriangle(Float2 &p, Float2 &p0, Float2 &p1, Float2 &p2)
    {
        float s = (p0.x - p2.x) * (p.y - p2.y) - (p0.y - p2.y) * (p.x - p2.x);
        float t = (p1.x - p0.x) * (p.y - p0.y) - (p1.y - p0.y) * (p.x - p0.x);

        if ((s < 0) != (t < 0) && s != 0 && t != 0)
            return false;

        float d = (p2.x - p1.x) * (p.y - p1.y) - (p2.y - p1.y) * (p.x - p1.x);
        return d == 0 || (d < 0) == (s + t <= 0);
    }

    Float2 VectorUtils::AdjustForBoundaries(Float2 &p_position, float minX, float maxX, float minY, float maxY, float epsilon)
    {
        Float2 position = p_position;

        if (position.x < minX + epsilon)
        {
            position.x = minX + epsilon;
        }
        if (position.x > maxX - epsilon)
        {
            position.x = maxX - epsilon;
        }

        if (position.y < minY + epsilon)
        {
            position.y = minY + epsilon;
        }
        if (position.y > maxY - epsilon)
        {
            position.y = maxY - epsilon;
        }

        return position;
    }

    Float2 VectorUtils::AdjustForBoundaries(Float2 &p_position, float minX, float maxX, float minY, float maxY, float epsilon, bool &wasAdjusted)
    {
        Float2 position = p_position;
        wasAdjusted = false;

        if (position.x < minX + epsilon)
        {
            position.x = minX + epsilon;
            wasAdjusted = true;
        }
        if (position.x > maxX - epsilon)
        {
            position.x = maxX - epsilon;
            wasAdjusted = true;
        }

        if (position.y < minY + epsilon)
        {
            position.y = minY + epsilon;
            wasAdjusted = true;
        }
        if (position.y > maxY - epsilon)
        {
            position.y = maxY - epsilon;
            wasAdjusted = true;
        }

        return position;
    }

    bool VectorUtils::AreLineSegmentsCollinearAndOverlapping(Float2 &ap, Float2 &aq, Float2 &bp, Float2 &bq, float epsilon)
    {
        return IsPointCollinearToLineSegment(bp, ap, aq, epsilon) &&
               IsPointCollinearToLineSegment(bq, ap, aq, epsilon) &&
               (IsPointWithinLineSegment(bp, ap, aq, epsilon) ||
                IsPointWithinLineSegment(bq, ap, aq, epsilon));
    }

    // Based on https://stackoverflow.com/questions/7050186/find-if-point-lies-on-line-segment
    bool VectorUtils::PointOnLine2D(Float2 &p, Float2 &a, Float2 &b, float epsilon)
    {
        return IsPointCollinearToLineSegment(p, a, b, epsilon) && IsPointWithinLineSegment(p, a, b, epsilon);
    }

    bool VectorUtils::IsPointCollinearToLineSegment(Float2 &p, Float2 &a, Float2 &b, float epsilon)
    {
        // ensure points are collinear
        float zero = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
        if (zero > epsilon || zero < -epsilon)
        {
            return false;
        }
        return true;
    }

    bool VectorUtils::IsPointWithinLineSegment(Float2 &p, Float2 &a, Float2 &b, float epsilon)
    {
        // check if x-coordinates are not equal
        if (a.x - b.x > epsilon || b.x - a.x > epsilon)
        {
            // ensure x is between a.x & b.x (use tolerance)
            return a.x > b.x
                       ? p.x + epsilon > b.x && p.x - epsilon < a.x
                       : p.x + epsilon > a.x && p.x - epsilon < b.x;
        }

        // ensure y is between a.y & b.y (use tolerance)
        return a.y > b.y
                   ? p.y + epsilon > b.y && p.y - epsilon < a.y
                   : p.y + epsilon > a.y && p.y - epsilon < b.y;
    }

    bool VectorUtils::try_get_line_segment_triangle_intersection(
        Float3 &a,
        Float3 &b,
        Float3 &c,
        Float3 &line_segment_start,
        Float3 &line_segment_end,
        Float3 &intersection)
    {
        Float3 line_segment_relative = line_segment_end - line_segment_start;
        Float3 ray_direction = line_segment_relative.normalized();
        Float3 ray_intersection;
        if (try_get_ray_triangle_intersection(a, b, c, line_segment_start, ray_direction, ray_intersection))
        {
            float line_segment_length_sqr = line_segment_relative.length_squared();
            float line_segment_start_to_ray_intersection_distance_sqr = (ray_intersection - line_segment_start).length_squared();

            if (line_segment_start_to_ray_intersection_distance_sqr < line_segment_length_sqr)
            {
                intersection = ray_intersection;
                return true;
            }
        }

        return false;
    }

    // Based on https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
    bool VectorUtils::try_get_ray_triangle_intersection(
        Float3 &a,
        Float3 &b,
        Float3 &c,
        Float3 &ray_origin,
        Float3 &ray_vector,
        Float3 &intersection)
    {
        float epsilon = MathUtils::FLOAT_EPSILON;

        Float3 edge1 = b - a;
        Float3 edge2 = c - a;
        Float3 ray_cross_e2 = ray_vector.cross(edge2);
        float det = edge1.dot(ray_cross_e2);

        if (det > -epsilon && det < epsilon)
        {
            return false; // This ray is parallel to this triangle.
        }

        float inv_det = 1.0 / det;
        Float3 s = ray_origin - a;
        float u = inv_det * s.dot(ray_cross_e2);

        if ((u < 0 && abs(u) > epsilon) || (u > 1 && abs(u - 1) > epsilon))
        {
            return false;
        }

        Float3 s_cross_e1 = s.cross(edge1);
        float v = inv_det * ray_vector.dot(s_cross_e1);

        if ((v < 0 && abs(v) > epsilon) || (u + v > 1 && abs(u + v - 1) > epsilon))
            return false;

        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = inv_det * edge2.dot(s_cross_e1);

        if (t > epsilon) // ray intersection
        {
            intersection = ray_origin + ray_vector * t;
            return true;
        }
        else // This means that there is a line intersection but not a ray intersection.
        {
            return false;
        }
    }

    Float2 VectorUtils::random_on_unit_circle(Random &random)
    {
        float theta = random.next_float() * 2.0f * MathUtils::PI;

        float x = cos(theta);
        float y = sin(theta);

        return Float2(x, y);
    }

    Float2 VectorUtils::random_inside_unit_circle(Random &random)
    {
        float r = sqrt(random.next_float());
        return random_on_unit_circle(random) * r;
    }

    // Based on https://math.stackexchange.com/questions/1585975/how-to-generate-random-points-on-a-sphere
    Float3 VectorUtils::random_on_unit_sphere(Random &random)
    {
        float u = random.next_float();
        float v = random.next_float();

        float phi = acos(2.0f * u - 1.0f) - 0.5f * MathUtils::PI;
        float theta = 2.0f * MathUtils::PI * v;

        float cos_phi = cos(phi);
        float sin_phi = sin(phi);
        float cos_theta = cos(theta);
        float sin_theta = sin(theta);

        float x = cos_phi * cos_theta;
        float y = cos_phi * sin_theta;
        float z = sin_phi;

        return Float3(x, y, z);
    }

    Float3 VectorUtils::random_inside_unit_sphere(Random &random)
    {
        float r = cbrt(random.next_float());
        return random_on_unit_sphere(random) * r;
    }

    Float3 VectorUtils::random_inside_box_x0z(Random &random)
    {
        return Float3(random.next_float(), 0.0f, random.next_float());
    }

    Float3 VectorUtils::random_inside_box_x0z(Random &random, Float3 &center, float scale)
    {
        return (random_inside_box_x0z(random) - Float3(0.5f, 0.0f, 0.5f)) * scale + center;
    }

    Float2 VectorUtils::middle_point(vector<Float2> &points)
    {
        float minx = MathUtils::FLOAT_MAX_VALUE;
        float maxx = MathUtils::FLOAT_MIN_VALUE;
        float miny = MathUtils::FLOAT_MAX_VALUE;
        float maxy = MathUtils::FLOAT_MIN_VALUE;

        for (int i = 0; i < points.size(); i++)
        {
            minx = MathUtils::min(minx, points[i].x);
            maxx = MathUtils::max(maxx, points[i].x);
            miny = MathUtils::min(miny, points[i].y);
            maxy = MathUtils::max(maxy, points[i].y);
        }

        return Float2((minx + maxx) * 0.5f, (miny + maxy) * 0.5f);
    }

    Float2 VectorUtils::float3_xz(const Float3 &v)
    {
        return Float2(v.x, v.z);
    }

    Float3 VectorUtils::float2_x0z(Float2 &v)
    {
        return Float3(v.x, 0.0f, v.y);
    }

    float VectorUtils::float3_xz_look_at_to_angle(Float2 &xz)
    {
        Float2 f0 = Float2(0.0f, -1.0f);
        return xz.angle_to(f0);
    }

    Float2 VectorUtils::float3_xz_angle_to_look_at(float angle)
    {
        Float2 f0 = Float2(0.0f, -1.0f);
        return f0.rotated(-angle);
    }

    float VectorUtils::radius_on_elipse(float angle, float a, float b)
    {
        float x = a * cos(angle);
        float y = b * sin(angle);
        return sqrt(x * x + y * y);
    }

    // Based on https://stackoverflow.com/questions/2924795/fastest-way-to-compute-point-to-triangle-distance-in-3d
    Float3 VectorUtils::closest_point_on_triangle(Float3 &p, Float3 &a, Float3 &b, Float3 &c)
    {
        Float3 ab = b - a;
        Float3 ac = c - a;
        Float3 ap = p - a;

        float d1 = ab.dot(ap);
        float d2 = ac.dot(ap);
        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            return a; // #1
        }

        Float3 bp = p - b;
        float d3 = ab.dot(bp);
        float d4 = ac.dot(bp);
        if (d3 >= 0.0f && d4 <= d3)
        {
            return b; // #2
        }

        Float3 cp = p - c;
        float d5 = ab.dot(cp);
        float d6 = ac.dot(cp);
        if (d6 >= 0.0f && d5 <= d6)
        {
            return c; // #3
        }

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.f)
        {
            float v = d1 / (d1 - d3);
            return a + ab * v; // #4
        }

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            float v = d2 / (d2 - d6);
            return a + ac * v; // #5
        }

        float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * v; // #6
        }

        float denom = 1.0f / (va + vb + vc);
        float v = vb * denom;
        float w = vc * denom;
        return a + ab * v + ac * w; // #0
    }

    // Based on https://stackoverflow.com/questions/19654251/random-point-inside-triangle-inside-java
    Float3 VectorUtils::random_point_inside_triangle(Float3 &a, Float3 &b, Float3 &c, Random &random)
    {
        float r1 = random.next_float();
        float r2 = random.next_float();

        float sqrt_r1 = MathUtils::sqrt(r1);

        float wa = 1.0f - sqrt_r1;
        float wb = sqrt_r1 * (1.0f - r2);
        float wc = sqrt_r1 * r2;

        float x = wa * a.x + wb * b.x + wc * c.x;
        float y = wa * a.y + wb * b.y + wc * c.y;
        float z = wa * a.z + wb * b.z + wc * c.z;

        return Float3(x, y, z);
    }

    // Based on https://gamedev.stackexchange.com/questions/165643/how-to-calculate-the-surface-area-of-a-mesh
    float VectorUtils::surface_area_of_triangle(Float3 &a, Float3 &b, Float3 &c)
    {
        Float3 ba = b - a;
        Float3 ca = c - a;
        return ba.cross(ca).length() * 0.5f;
    }

    void VectorUtils::find_rotation_axis_and_angle(Float3 &direction_a, Float3 &direction_b, bool &axis_found, Float3 &axis, float &axis_rotation)
    {
        if ((direction_a - direction_b).length_squared() > 0.00001f)
        {
            axis_found = true;
            axis = direction_a.cross(direction_b).normalized();
            axis_rotation = direction_a.angle_to(direction_b);
        }
        else
        {
            axis_found = false;
        }
    }
}
