#include "delaunator.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/math/heap_sort.hpp"

// Ported and based on https://github.com/nol1fe/delaunator-sharp

namespace NavigationSystemCode
{
    Delaunator::Delaunator()
    {
        EPSILON = pow(2.0f, -52.0f);
        EDGE_STACK.resize(512);

        for (int i = 0; i < EDGE_STACK.size(); i++)
        {
            EDGE_STACK[i] = 0;
        }
    }

    void Delaunator::Create(vector<Float2> &p_points)
    {
        int n = p_points.size();

        if (n < 3)
        {
            GodotUtils::print("Need at least 3 points");
            return;
        }

        coords.resize(n * 2);

        for (int i = 0; i < n; i++)
        {
            Float2 p = p_points[i];
            coords[2 * i] = p.x;
            coords[2 * i + 1] = p.y;
        }

        int maxTriangles = 2 * n - 5;

        int trianglesHalfedgesCount = maxTriangles * 3;

        triangles.resize(trianglesHalfedgesCount);
        halfedges.resize(trianglesHalfedgesCount);

        for (int i = 0; i < trianglesHalfedgesCount; i++)
        {
            halfedges[i] = -1;
        }

        hashSize = (int)ceil(sqrt(n));

        hullPrev.resize(n);
        hullNext.resize(n);
        hullTri.resize(n);
        hullHash.resize(hashSize);

        for (int i = 0; i < n; i++)
        {
            hullPrev[i] = 0;
            hullNext[i] = 0;
            hullTri[i] = 0;
        }

        for (int i = 0; i < hashSize; i++)
        {
            hullHash[i] = 0;
        }

        ids.resize(n);

        float minX = MathUtils::FLOAT_MAX_VALUE;
        float minY = MathUtils::FLOAT_MAX_VALUE;
        float maxX = -MathUtils::FLOAT_MAX_VALUE;
        float maxY = -MathUtils::FLOAT_MAX_VALUE;

        for (int i = 0; i < n; i++)
        {
            float x = coords[2 * i];
            float y = coords[2 * i + 1];
            if (x < minX)
                minX = x;
            if (y < minY)
                minY = y;
            if (x > maxX)
                maxX = x;
            if (y > maxY)
                maxY = y;
            ids[i] = i;
        }

        float cx = (minX + maxX) / 2;
        float cy = (minY + maxY) / 2;

        float minDist = MathUtils::FLOAT_MAX_VALUE;
        int i0 = 0;
        int i1 = 0;
        int i2 = 0;

        // pick a seed point close to the center
        for (int i = 0; i < n; i++)
        {
            float d = Dist(cx, cy, coords[2 * i], coords[2 * i + 1]);
            if (d < minDist)
            {
                i0 = i;
                minDist = d;
            }
        }
        float i0x = coords[2 * i0];
        float i0y = coords[2 * i0 + 1];

        minDist = MathUtils::FLOAT_MAX_VALUE;

        // find the point closest to the seed
        for (int i = 0; i < n; i++)
        {
            if (i == i0)
                continue;
            float d = Dist(i0x, i0y, coords[2 * i], coords[2 * i + 1]);
            if (d < minDist && d > 0)
            {
                i1 = i;
                minDist = d;
            }
        }

        float i1x = coords[2 * i1];
        float i1y = coords[2 * i1 + 1];

        float minRadius = MathUtils::FLOAT_MAX_VALUE;

        // find the third point which forms the smallest circumcircle with the first two
        for (int i = 0; i < n; i++)
        {
            if (i == i0 || i == i1)
                continue;
            float r = Circumradius(i0x, i0y, i1x, i1y, coords[2 * i], coords[2 * i + 1]);
            if (r < minRadius)
            {
                i2 = i;
                minRadius = r;
            }
        }
        float i2x = coords[2 * i2];
        float i2y = coords[2 * i2 + 1];

        if (minRadius == MathUtils::FLOAT_MAX_VALUE)
        {
            GodotUtils::print("No Delaunay triangulation exists for this input.");
            return;
        }

        if (Orient(i0x, i0y, i1x, i1y, i2x, i2y))
        {
            int i = i1;
            float x = i1x;
            float y = i1y;
            i1 = i2;
            i1x = i2x;
            i1y = i2y;
            i2 = i;
            i2x = x;
            i2y = y;
        }

        Float2 center = Circumcenter(i0x, i0y, i1x, i1y, i2x, i2y);
        cx_final = center.x;
        cy_final = center.y;

        dists.resize(n);

        for (int i = 0; i < n; i++)
        {
            dists[i] = Dist(coords[2 * i], coords[2 * i + 1], center.x, center.y);
        }

        // sort the points by distance from the seed triangle circumcenter
        HeapSort::Sort(ids, dists);

        // set up the seed triangle as the starting hull
        hullStart = i0;
        hullSize = 3;

        hullNext[i0] = hullPrev[i2] = i1;
        hullNext[i1] = hullPrev[i0] = i2;
        hullNext[i2] = hullPrev[i1] = i0;

        hullTri[i0] = 0;
        hullTri[i1] = 1;
        hullTri[i2] = 2;

        hullHash[HashKey(i0x, i0y)] = i0;
        hullHash[HashKey(i1x, i1y)] = i1;
        hullHash[HashKey(i2x, i2y)] = i2;

        trianglesLen = 0;
        AddTriangle(i0, i1, i2, -1, -1, -1);

        float xp = 0;
        float yp = 0;

        for (int k = 0; k < n; k++)
        {
            int i = ids[k];
            float x = coords[2 * i];
            float y = coords[2 * i + 1];

            // skip near-duplicate points
            if (k > 0 && abs(x - xp) <= EPSILON && abs(y - yp) <= EPSILON)
                continue;
            xp = x;
            yp = y;

            // skip seed triangle points
            if (i == i0 || i == i1 || i == i2)
            {
                continue;
            }

            // find a visible edge on the convex hull using edge hash
            int start = 0;
            for (int j = 0; j < hashSize; j++)
            {
                int key = HashKey(x, y);
                start = hullHash[(key + j) % hashSize];
                if (start != -1 && start != hullNext[start])
                    break;
            }

            start = hullPrev[start];
            int e = start;
            int q = hullNext[e];

            while (!Orient(x, y, coords[2 * e], coords[2 * e + 1], coords[2 * q], coords[2 * q + 1]))
            {
                e = q;
                if (e == start)
                {
                    e = (std::numeric_limits<int>::max)();
                    break;
                }

                q = hullNext[e];
            }

            if (e == (std::numeric_limits<int>::max)())
                continue; // likely a near-duplicate point; skip it

            // add the first triangle from the point
            int t = AddTriangle(e, i, hullNext[e], -1, -1, hullTri[e]);

            // recursively flip triangles from the point until they satisfy the Delaunay condition
            hullTri[i] = Legalize(t + 2);
            hullTri[e] = t; // keep track of boundary triangles on the hull
            hullSize++;

            // walk forward through the hull, adding more triangles and flipping recursively
            int next = hullNext[e];
            q = hullNext[next];

            while (Orient(x, y, coords[2 * next], coords[2 * next + 1], coords[2 * q], coords[2 * q + 1]))
            {
                t = AddTriangle(next, i, q, hullTri[i], -1, hullTri[next]);
                hullTri[i] = Legalize(t + 2);
                hullNext[next] = next; // mark as removed
                hullSize--;
                next = q;

                q = hullNext[next];
            }

            // walk backward from the other side, adding more triangles and flipping
            if (e == start)
            {
                q = hullPrev[e];

                while (Orient(x, y, coords[2 * q], coords[2 * q + 1], coords[2 * e], coords[2 * e + 1]))
                {
                    t = AddTriangle(q, i, e, -1, hullTri[e], hullTri[q]);
                    Legalize(t + 2);
                    hullTri[q] = t;
                    hullNext[e] = e; // mark as removed
                    hullSize--;
                    e = q;

                    q = hullPrev[e];
                }
            }

            // update the hull indices
            hullStart = hullPrev[i] = e;
            hullNext[e] = hullPrev[next] = i;
            hullNext[i] = next;

            // save the two new edges in the hash table
            hullHash[HashKey(x, y)] = i;
            hullHash[HashKey(coords[2 * e], coords[2 * e + 1])] = e;
        }
    }

    void Delaunator::ClearTemporaryLists()
    {
        triangles.resize(trianglesLen);
        halfedges.resize(trianglesLen);

        hullPrev.clear();
        hullNext.clear();
        hullTri.clear();

        ids.clear();
        dists.clear();
    }

    int Delaunator::Legalize(int a)
    {
        int i = 0;
        int ar;

        // recursion eliminated with a fixed-size stack
        while (true)
        {
            int b = halfedges[a];

            /* if the pair of triangles doesn't satisfy the Delaunay condition
             * (p1 is inside the circumcircle of [p0, pl, pr]), flip them,
             * then do the same check/flip recursively for the new pair of triangles
             *
             *           pl                    pl
             *          /||\                  /  \
             *       al/ || \bl            al/    \a
             *        /  ||  \              /      \
             *       /  a||b  \    flip    /___ar___\
             *     p0\   ||   /p1   =>   p0\---bl---/p1
             *        \  ||  /              \      /
             *       ar\ || /br             b\    /br
             *          \||/                  \  /
             *           pr                    pr
             */
            int a0 = a - a % 3;
            ar = a0 + (a + 2) % 3;

            if (b == -1)
            { // convex hull edge
                if (i == 0)
                    break;
                a = EDGE_STACK[--i];
                continue;
            }

            int b0 = b - b % 3;
            int al = a0 + (a + 1) % 3;
            int bl = b0 + (b + 2) % 3;

            int p0 = triangles[ar];
            int pr = triangles[a];
            int pl = triangles[al];
            int p1 = triangles[bl];

            bool illegal = InCircle(
                coords[2 * p0], coords[2 * p0 + 1],
                coords[2 * pr], coords[2 * pr + 1],
                coords[2 * pl], coords[2 * pl + 1],
                coords[2 * p1], coords[2 * p1 + 1]);

            if (illegal)
            {
                triangles[a] = p1;
                triangles[b] = p0;

                int hbl = halfedges[bl];

                // edge swapped on the other side of the hull (rare); fix the halfedge reference
                if (hbl == -1)
                {
                    int e = hullStart;
                    do
                    {
                        if (hullTri[e] == bl)
                        {
                            hullTri[e] = a;
                            break;
                        }
                        e = hullPrev[e];
                    } while (e != hullStart);
                }
                Link(a, hbl);
                Link(b, halfedges[ar]);
                Link(ar, bl);

                int br = b0 + (b + 1) % 3;

                // don't worry about hitting the cap: it can only happen on extremely degenerate input
                if (i < EDGE_STACK.size())
                {
                    EDGE_STACK[i++] = br;
                }
            }
            else
            {
                if (i == 0)
                    break;
                a = EDGE_STACK[--i];
            }
        }

        return ar;
    }

    bool Delaunator::InCircle(float ax, float ay, float bx, float by, float cx, float cy, float px, float py)
    {
        float dx = ax - px;
        float dy = ay - py;
        float ex = bx - px;
        float ey = by - py;
        float fx = cx - px;
        float fy = cy - py;

        float ap = dx * dx + dy * dy;
        float bp = ex * ex + ey * ey;
        float cp = fx * fx + fy * fy;

        return dx * (ey * cp - bp * fy) -
                   dy * (ex * cp - bp * fx) +
                   ap * (ex * fy - ey * fx) <
               0;
    }

    int Delaunator::AddTriangle(int i0, int i1, int i2, int a, int b, int c)
    {
        int t = trianglesLen;

        triangles[t] = i0;
        triangles[t + 1] = i1;
        triangles[t + 2] = i2;

        Link(t, a);
        Link(t + 1, b);
        Link(t + 2, c);

        trianglesLen += 3;
        return t;
    }

    void Delaunator::Link(int a, int b)
    {
        halfedges[a] = b;
        if (b != -1)
            halfedges[b] = a;
    }

    int Delaunator::HashKey(float x, float y)
    {
        return (int)(MathUtils::floor(PseudoAngle(x - cx_final, y - cy_final) * hashSize) % hashSize);
    }

    float Delaunator::PseudoAngle(float dx, float dy)
    {
        float p = dx / (abs(dx) + abs(dy));
        return (dy > 0 ? 3 - p : 1 + p) / 4; // [0..1]
    }

    bool Delaunator::Orient(float px, float py, float qx, float qy, float rx, float ry)
    {
        return (qy - py) * (rx - qx) - (qx - px) * (ry - qy) < 0;
    }

    float Delaunator::Circumradius(float ax, float ay, float bx, float by, float cx, float cy)
    {
        float dx = bx - ax;
        float dy = by - ay;
        float ex = cx - ax;
        float ey = cy - ay;
        float bl = dx * dx + dy * dy;
        float cl = ex * ex + ey * ey;
        float d = 0.5f / (dx * ey - dy * ex);
        float x = (ey * bl - dy * cl) * d;
        float y = (dx * cl - ex * bl) * d;
        return x * x + y * y;
    }

    Float2 Delaunator::Circumcenter(float ax, float ay, float bx, float by, float cx, float cy)
    {
        float dx = bx - ax;
        float dy = by - ay;
        float ex = cx - ax;
        float ey = cy - ay;
        float bl = dx * dx + dy * dy;
        float cl = ex * ex + ey * ey;
        float d = 0.5f / (dx * ey - dy * ex);
        float x = ax + (ey * bl - dy * cl) * d;
        float y = ay + (dx * cl - ex * bl) * d;

        return Float2(x, y);
    }

    float Delaunator::Dist(float ax, float ay, float bx, float by)
    {
        float dx = ax - bx;
        float dy = ay - by;
        return dx * dx + dy * dy;
    }

    vector<DelaunatorTriangle> Delaunator::GetTriangles()
    {
        vector<DelaunatorTriangle> triangles;
        for (int t = 0; t < trianglesLen / 3; t++)
        {
            triangles.push_back(DelaunatorTriangle{
                .index = t,
                .points = PointsOfTriangle(t)});
        }
        return triangles;
    }

    vector<DelaunatorEdge> Delaunator::GetEdges()
    {
        vector<DelaunatorEdge> edges;
        for (int e = 0; e < trianglesLen; e++)
        {
            if (e > halfedges[e])
            {
                int p = triangles[e];
                int q = triangles[NextHalfedge(e)];
                edges.push_back(DelaunatorEdge{
                    .index = e,
                    .p = p,
                    .q = q});
            }
        }
        return edges;
    }

    /// <summary>
    /// Returns the three point indices of a given triangle id.
    /// </summary>
    vector<int> Delaunator::PointsOfTriangle(int t)
    {
        vector<int> pointsOfTriangle;
        pointsOfTriangle.resize(3);

        for (int i = 0; i < 3; i++)
        {
            int edge = 3 * t + i;
            pointsOfTriangle[i] = triangles[edge];
        }

        return pointsOfTriangle;
    }

    /// <summary>
    /// Returns the triangle ids adjacent to the given triangle id.
    /// Will return up to three values.
    /// </summary>
    vector<int> Delaunator::TrianglesAdjacentToTriangle(int t)
    {
        vector<int> adjacentTriangles;

        for (int i = 0; i < 3; i++)
        {
            int e = 3 * t + i;
            int opposite = halfedges[e];
            if (opposite >= 0)
            {
                adjacentTriangles.push_back(TriangleOfEdge(opposite));
            }
        }
        return adjacentTriangles;
    }

    int Delaunator::NextHalfedge(int e)
    {
        return (e % 3 == 2) ? e - 2 : e + 1;
    }

    /// <summary>
    /// Returns the three half-edges of a given triangle id.
    /// </summary>
    vector<int> Delaunator::EdgesOfTriangle(int t)
    {
        return vector<int>{3 * t, 3 * t + 1, 3 * t + 2};
    }

    /// <summary>
    /// Returns the triangle id of a given half-edge.
    /// </summary>
    int Delaunator::TriangleOfEdge(int e)
    {
        return e / 3;
    }
}
