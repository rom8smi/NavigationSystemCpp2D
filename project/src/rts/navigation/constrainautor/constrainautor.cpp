#include "constrainautor.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/debug/debug.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

// Ported and based on https://github.com/kninnug/Constrainautor

namespace NavigationSystemCode
{
    void Constrainautor::Create(Delaunator &del, vector<ConstraintEdge> &edges)
    {
        int coordsCount = del.coords.size();
        int numPoints = coordsCount / 2;
        int numEdges = del.triangles.size();
        loopMax = coordsCount * 3;

        // Map every vertex id to the right-most edge that points to that vertex
        vertMap.resize(numPoints);
        for (int i = 0; i < numPoints; i++)
        {
            vertMap[i] = -1;
        }

        // Keep track of edges flipped while constraining
        flips.Create(numEdges);
        // Keep track of constrained edges
        consd.Create(numEdges);

        for (int e = 0; e < numEdges; e++)
        {
            int v = del.triangles[e];
            if (vertMap[v] == -1)
            {
                UpdateVert(e, del);
            }
        }

        ConstrainAll(edges, del);
    }

    void Constrainautor::ClearTemporaryLists()
    {
        vertMap.clear();
        flips.Clear();
        consd.Clear();
    }

    void Constrainautor::ConstrainOne(int segP1, int segP2, Delaunator &del)
    {
        Debug::index_assert(segP1, vertMap.size(), "bbb2");
        int start = vertMap[segP1];
        int edg = start;

        int iLoop = 0;
        // Loop over edges touching segP1
        do
        {
            iLoop++;
            if (iLoop > loopMax)
            {
                // Debug::log("aaa1");
                return;
            }
            if (edg == -1)
            {
                // This is sometimes randomly happening
                // Debug::log("aaa11 " + to_string(iLoop) + "" + to_string(loopMax));
                return;
            }

            Debug::index_assert(edg, del.triangles.size(), "bbb3");
            int p4 = del.triangles[edg];
            int nxt = NextEdge(edg);

            // Already constrained in reverse order
            if (p4 == segP2)
            {
                Protect(edg, del);
                return;
            }

            int opp = PrevEdge(edg);
            Debug::index_assert(opp, del.triangles.size(), "bbb4");
            int p3 = del.triangles[opp];

            // Already constrained
            if (p3 == segP2)
            {
                Protect(nxt, del);
                return;
            }

            // Edge opposite segP1 intersects constraint
            if (IntersectSegments(segP1, segP2, p3, p4, del))
            {
                edg = opp;
                break;
            }

            Debug::index_assert(nxt, del.halfedges.size(), "bbb5");
            int adj = del.halfedges[nxt];
            edg = adj;
        } while (edg != -1 && edg != start);

        int conEdge = edg;
        int rescan = -1;
        iLoop = 0;

        // int edgAssignmentReason = -1;

        while (edg != -1)
        {
            iLoop++;
            // if(iLoop > loopMax - 10)
            // {
            //     int p = del.triangles[edg];
            //     int q = del.triangles[NextEdge(edg)];

            //     // Float2 ff1 = Float2(del.coords[2 * p], del.coords[2 * p + 1]);
            //     // Float2 ff2 = Float2(del.coords[2 * q], del.coords[2 * q + 1]);

            //     Debug::log("aaa2a "+ to_string(edg) +" "+to_string(edgAssignmentReason));
            // }
            if (iLoop > loopMax)
            {
                // Debug::log("aaa2");
                return;
            }

            Debug::index_assert(edg, del.halfedges.size(), "bbb6");
            int adj = del.halfedges[edg];
            int bot = PrevEdge(edg);
            int top = PrevEdge(adj);
            int rgt = NextEdge(adj);

            Debug::index_assert(edg, del.triangles.size(), "bbb7");
            Debug::index_assert(adj, del.triangles.size(), "bbb8");
            Debug::index_assert(bot, del.triangles.size(), "bbb9");
            Debug::index_assert(top, del.triangles.size(), "bbb10");
            bool convex = IntersectSegments(
                del.triangles[edg],
                del.triangles[adj],
                del.triangles[bot],
                del.triangles[top],
                del);

            if (!convex)
            {
                // if(iLoop > loopMax - 10)
                // {
                //     Debug::log("aaa2b "+ to_string(edg) +" "+to_string(adj) +" "+to_string(bot) +" "+to_string(top) +" "+to_string(convex));
                // }

                if (rescan == -1)
                {
                    rescan = edg;
                }

                Debug::index_assert(top, del.triangles.size(), "bbb11");
                if (del.triangles[top] == segP2)
                {
                    if (edg == rescan)
                    {
                        return;
                    }
                    edg = rescan;
                    // edgAssignmentReason = 0;
                    rescan = -1;
                    continue;
                }

                Debug::index_assert(top, del.triangles.size(), "bbb12");
                Debug::index_assert(adj, del.triangles.size(), "bbb13");
                Debug::index_assert(rgt, del.triangles.size(), "bbb14");
                if (IntersectSegments(segP1, segP2, del.triangles[top], del.triangles[adj], del))
                {
                    edg = top;
                    // edgAssignmentReason = 1;
                }
                else if (IntersectSegments(segP1, segP2, del.triangles[rgt], del.triangles[top], del))
                {
                    edg = rgt;
                    // edgAssignmentReason = 2;
                }
                else if (rescan == edg)
                {
                    return;
                }

                continue;
            }

            FlipDiagonal(edg, del);

            Debug::index_assert(bot, del.triangles.size(), "bbb15");
            Debug::index_assert(top, del.triangles.size(), "bbb16");
            if (IntersectSegments(segP1, segP2, del.triangles[bot], del.triangles[top], del))
            {
                if (rescan == -1)
                {
                    rescan = bot;
                }
                if (rescan == bot)
                {
                    return;
                }
            }

            Debug::index_assert(top, del.triangles.size(), "bbb17");
            Debug::index_assert(rgt, del.triangles.size(), "bbb18");
            if (del.triangles[top] == segP2)
            {
                conEdge = top;
                edg = rescan;
                rescan = -1;
                // edgAssignmentReason = 3;
            }
            else if (IntersectSegments(segP1, segP2, del.triangles[rgt], del.triangles[top], del))
            {
                edg = rgt;
                // edgAssignmentReason = 4;
            }
        }

        int halfedgesCount = del.halfedges.size();
        if (conEdge <= -1 || conEdge >= halfedgesCount)
        {
            return;
        }

        Protect(conEdge, del);
        Delaunify(false, del);
    }

    void Constrainautor::ConstrainAll(vector<ConstraintEdge> &edges, Delaunator &del)
    {
        for (int i = 0; i < edges.size(); i++)
        {
            Debug::index_assert(i, edges.size(), "bbb1");
            ConstrainOne(edges[i].p, edges[i].q, del);
        }
    }

    void Constrainautor::Delaunify(bool deep, Delaunator &del)
    {
        int len = del.halfedges.size();
        int flipped;
        int iLoop = 0;

        do
        {
            iLoop++;
            if (iLoop > loopMax)
            {
                // Debug::log("aaa3");
                return;
            }

            flipped = 0;
            for (int edg = 0; edg < len; edg++)
            {
                if (consd.Has(edg))
                {
                    continue;
                }

                flips.Remove(edg);
                Debug::index_assert(edg, del.halfedges.size(), "bbb19");
                int adj = del.halfedges[edg];
                if (adj == -1)
                {
                    continue;
                }

                flips.Remove(adj);
                if (!IsDelaunay(edg, del))
                {
                    FlipDiagonal(edg, del);
                    flipped++;
                }
            }
        } while (deep && flipped > 0);
    }

    int Constrainautor::Protect(int edg, Delaunator &del)
    {
        Debug::index_assert(edg, del.halfedges.size(), "bbb20");
        int adj = del.halfedges[edg];
        flips.Remove(edg);
        consd.Add(edg);

        if (adj != -1)
        {
            flips.Remove(adj);
            consd.Add(adj);
            return adj;
        }

        return -edg;
    }

    bool Constrainautor::MarkFlip(int edg, Delaunator &del)
    {
        if (consd.Has(edg))
        {
            return false;
        }

        Debug::index_assert(edg, del.halfedges.size(), "bbb21");
        int adj = del.halfedges[edg];
        if (adj != -1)
        {
            flips.Add(edg);
            flips.Add(adj);
        }
        return true;
    }

    void Constrainautor::FlipDiagonal(int edg, Delaunator &del)
    {
        int adj = del.halfedges[edg];
        int bot = PrevEdge(edg);
        int lft = NextEdge(edg);
        int top = PrevEdge(adj);
        int rgt = NextEdge(adj);
        Debug::index_assert(bot, del.halfedges.size(), "bbb22");
        Debug::index_assert(top, del.halfedges.size(), "bbb23");
        int adjBot = del.halfedges[bot];
        int adjTop = del.halfedges[top];

        if (consd.Has(edg))
        {
            return;
        }

        // Move edg to top
        Debug::index_assert(edg, del.triangles.size(), "bbb24");
        Debug::index_assert(top, del.triangles.size(), "bbb25");
        del.triangles[edg] = del.triangles[top];

        Debug::index_assert(edg, del.halfedges.size(), "bbb26");
        del.halfedges[edg] = adjTop;
        if (!flips.Set(edg, flips.Has(top)))
        {
            consd.Set(edg, consd.Has(top));
        }
        if (adjTop != -1)
        {
            Debug::index_assert(adjTop, del.halfedges.size(), "bbb27");
            del.halfedges[adjTop] = edg;
        }
        Debug::index_assert(bot, del.halfedges.size(), "bbb28");
        del.halfedges[bot] = top;

        // Move adj to bot
        Debug::index_assert(adj, del.triangles.size(), "bbb29");
        Debug::index_assert(bot, del.triangles.size(), "bbb30");
        del.triangles[adj] = del.triangles[bot];

        Debug::index_assert(adj, del.halfedges.size(), "bbb31");
        del.halfedges[adj] = adjBot;
        if (!flips.Set(adj, flips.Has(bot)))
        {
            consd.Set(adj, consd.Has(bot));
        }
        if (adjBot != -1)
        {
            Debug::index_assert(adjBot, del.halfedges.size(), "bbb32");
            del.halfedges[adjBot] = adj;
        }
        Debug::index_assert(top, del.halfedges.size(), "bbb33");
        del.halfedges[top] = bot;

        MarkFlip(edg, del);
        MarkFlip(lft, del);
        MarkFlip(adj, del);
        MarkFlip(rgt, del);

        flips.Add(bot);
        consd.Remove(bot);
        flips.Add(top);
        consd.Remove(top);

        UpdateVert(edg, del);
        UpdateVert(lft, del);
        UpdateVert(adj, del);
        UpdateVert(rgt, del);
    }

    bool Constrainautor::IsDelaunay(int edg, Delaunator &del)
    {
        Debug::index_assert(edg, del.halfedges.size(), "bbb34");
        int adj = del.halfedges[edg];

        if (adj == -1)
        {
            return true;
        }

        Debug::index_assert(PrevEdge(edg), del.triangles.size(), "bbb35");
        int p1 = del.triangles[PrevEdge(edg)];
        Debug::index_assert(edg, del.triangles.size(), "bbb36");
        int p2 = del.triangles[edg];
        Debug::index_assert(NextEdge(edg), del.triangles.size(), "bbb37");
        int p3 = del.triangles[NextEdge(edg)];
        Debug::index_assert(PrevEdge(adj), del.triangles.size(), "bbb38");
        int px = del.triangles[PrevEdge(adj)];

        return !InCircle(p1, p2, p3, px, del);
    }

    int Constrainautor::UpdateVert(int start, Delaunator &del)
    {
        Debug::index_assert(start, del.triangles.size(), "bbb39");
        int v = del.triangles[start];
        int inc = PrevEdge(start);

        Debug::index_assert(inc, del.halfedges.size(), "bbb40");
        int adj = del.halfedges[inc];

        while (adj != -1 && adj != start)
        {
            inc = PrevEdge(adj);
            Debug::index_assert(inc, del.halfedges.size(), "bbb41");
            adj = del.halfedges[inc];
        }

        Debug::index_assert(v, vertMap.size(), "bbb42");
        vertMap[v] = inc;
        return inc;
    }

    bool Constrainautor::IntersectSegments(int p1, int p2, int p3, int p4, Delaunator &del)
    {
        if (p1 == p3 || p1 == p4 || p2 == p3 || p2 == p4)
        {
            return false;
        }

        Debug::index_assert(p1 * 2, del.coords.size(), "bbb43");
        Debug::index_assert(p1 * 2 + 1, del.coords.size(), "bbb44");
        Debug::index_assert(p2 * 2, del.coords.size(), "bbb45");
        Debug::index_assert(p2 * 2 + 1, del.coords.size(), "bbb46");
        Debug::index_assert(p3 * 2, del.coords.size(), "bbb47");
        Debug::index_assert(p3 * 2 + 1, del.coords.size(), "bbb48");
        Debug::index_assert(p4 * 2, del.coords.size(), "bbb49");
        Debug::index_assert(p4 * 2 + 1, del.coords.size(), "bbb50");

        // Float2 vp1 = Float2(del.coords[p1 * 2], del.coords[p1 * 2 + 1]);
        // Float2 vp2 = Float2(del.coords[p2 * 2], del.coords[p2 * 2 + 1]);
        // Float2 vp3 = Float2(del.coords[p3 * 2], del.coords[p3 * 2 + 1]);
        // Float2 vp4 = Float2(del.coords[p4 * 2], del.coords[p4 * 2 + 1]);

        // LineSegmentsIntersectionResult result = VectorUtils::LineSegmentsIntersection(vp1, vp2, vp3, vp4, 0.001f);

        return IntersectSegments(
            del.coords[p1 * 2], del.coords[p1 * 2 + 1],
            del.coords[p2 * 2], del.coords[p2 * 2 + 1],
            del.coords[p3 * 2], del.coords[p3 * 2 + 1],
            del.coords[p4 * 2], del.coords[p4 * 2 + 1]);

        // if(result.intersects != result2)
        // {
        //     // Debug::log("uuu1 ");
        // }

        // return result.intersects;
    }

    bool Constrainautor::InCircle(int p1, int p2, int p3, int px, Delaunator &del)
    {
        Debug::index_assert(p1 * 2, del.coords.size(), "bbb51");
        Debug::index_assert(p1 * 2 + 1, del.coords.size(), "bbb52");
        Debug::index_assert(p2 * 2, del.coords.size(), "bbb53");
        Debug::index_assert(p2 * 2 + 1, del.coords.size(), "bbb54");
        Debug::index_assert(p3 * 2, del.coords.size(), "bbb55");
        Debug::index_assert(p3 * 2 + 1, del.coords.size(), "bbb56");
        Debug::index_assert(px * 2, del.coords.size(), "bbb57");
        Debug::index_assert(px * 2 + 1, del.coords.size(), "bbb58");

        return InCircle(
                   del.coords[p1 * 2], del.coords[p1 * 2 + 1],
                   del.coords[p2 * 2], del.coords[p2 * 2 + 1],
                   del.coords[p3 * 2], del.coords[p3 * 2 + 1],
                   del.coords[px * 2], del.coords[px * 2 + 1]) < 0.0f;
    }

    int Constrainautor::NextEdge(int e)
    {
        return (e % 3 == 2) ? e - 2 : e + 1;
    }

    int Constrainautor::PrevEdge(int e)
    {
        return (e % 3 == 0) ? e + 2 : e - 1;
    }

    float Constrainautor::Orient2D(float ax, float ay, float bx, float by, float cx, float cy)
    {
        float acx = ax - cx;
        float bcx = bx - cx;
        float acy = ay - cy;
        float bcy = by - cy;
        return acx * bcy - acy * bcx;
    }

    float Constrainautor::InCircle(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy)
    {
        float adx = ax - dx;
        float ady = ay - dy;
        float bdx = bx - dx;
        float bdy = by - dy;
        float cdx = cx - dx;
        float cdy = cy - dy;

        float abdet = adx * bdy - bdx * ady;
        float bcdet = bdx * cdy - cdx * bdy;
        float cadet = cdx * ady - adx * cdy;
        float alift = adx * adx + ady * ady;
        float blift = bdx * bdx + bdy * bdy;
        float clift = cdx * cdx + cdy * cdy;

        return alift * bcdet + blift * cadet + clift * abdet;
    }

    bool Constrainautor::IntersectSegments(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y, float p4x, float p4y)
    {
        float x0 = Orient2D(p1x, p1y, p3x, p3y, p4x, p4y);
        float y0 = Orient2D(p2x, p2y, p3x, p3y, p4x, p4y);

        if ((x0 > 0 && y0 > 0) || (x0 < 0 && y0 < 0))
        {
            return false;
        }

        float x1 = Orient2D(p3x, p3y, p1x, p1y, p2x, p2y);
        float y1 = Orient2D(p4x, p4y, p1x, p1y, p2x, p2y);

        if ((x1 > 0 && y1 > 0) || (x1 < 0 && y1 < 0))
        {
            return false;
        }

        float epsilon = 0.0001f;
        // Check for degenerate collinear case
        // if (x0 == 0 && y0 == 0 && x1 == 0 && y1 == 0)
        if (x0 < epsilon && x0 > -epsilon &&
            y0 < epsilon && y0 > -epsilon &&
            x1 < epsilon && x1 > -epsilon &&
            y1 < epsilon && y1 > -epsilon)
        // if (x0 == 0 && y0 == 0 && x1 == 0 && y1 == 0)
        {
            return !(MathUtils::max(p3x, p4x) < MathUtils::min(p1x, p2x) ||
                     MathUtils::max(p1x, p2x) < MathUtils::min(p3x, p4x) ||
                     MathUtils::max(p3y, p4y) < MathUtils::min(p1y, p2y) ||
                     MathUtils::max(p1y, p2y) < MathUtils::min(p3y, p4y));
        }

        return true;
    }
}
