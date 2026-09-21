#include "constrainautor.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/debug/debug.hpp"
#include "rts/math/vector_utils.hpp"
#include "rts/godot_wrapper/godot_utils.hpp"

// Ported and based on https://github.com/kninnug/Constrainautor

namespace NavigationSystemCode
{
    void Constrainautor::Create(Delaunator &del, vector<ConstraintEdge> &edges, const string &reason)
    {
        int coordsCount = del.coords.size();
        int numPoints = coordsCount / 2;
        int numEdges = del.triangles.size();
        loopMax = coordsCount * 3;
        debugInCircleToleranceErrors = false;

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
        dirtyEdges.clear();
        dirtyEdgeQueued.resize(numEdges);
        inCircleComparisonMismatchCount = 0;
        inCircleComparisonLoggedCount = 0;
        for (int i = 0; i < numEdges; i++)
        {
            dirtyEdgeQueued[i] = false;
        }

        for (int e = 0; e < numEdges; e++)
        {
            int v = del.triangles[e];
            if (vertMap[v] == -1)
            {
                UpdateVert(e, del);
            }
        }

        ConstrainAll(edges, del, reason);
    }

    void Constrainautor::ClearTemporaryLists()
    {
        if (inCircleComparisonMismatchCount > 0)
        {
            Debug::log("Constrainautor InCircle comparison mismatches " + to_string(inCircleComparisonMismatchCount));
        }

        vertMap.clear();
        dirtyEdges.clear();
        dirtyEdgeQueued.clear();
        flips.Clear();
        consd.Clear();
    }

    void Constrainautor::ConstrainOne(int segP1, int segP2, Delaunator &del, const string &reason)
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
                Debug::log("Constrainautor 1 loopMax " + to_string(loopMax));
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
                Debug::log("Constrainautor 2 loopMax " + to_string(loopMax));
                return;
            }

            Debug::index_assert(edg, del.halfedges.size(), "bbb6");
            if (edg < 0)
            {
                return;
            }

            int adj = del.halfedges[edg];
            Debug::index_assert(adj, del.triangles.size(), "bbb7 " + reason);

            if (adj < 0)
            {
                return;
            }

            int bot = PrevEdge(edg);
            int top = PrevEdge(adj);
            int rgt = NextEdge(adj);

            Debug::index_assert(edg, del.triangles.size(), "bbb8");
            Debug::index_assert(bot, del.triangles.size(), "bbb9");

            if (bot < 0)
            {
                return;
            }

            Debug::index_assert(top, del.triangles.size(), "bbb10");

            if (top < 0)
            {
                return;
            }

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

    void Constrainautor::ConstrainAll(vector<ConstraintEdge> &edges, Delaunator &del, const string &reason)
    {
        for (int i = 0; i < edges.size(); i++)
        {
            Debug::index_assert(i, edges.size(), "bbb1");
            ConstrainOne(edges[i].p, edges[i].q, del, reason);
        }
    }

    void Constrainautor::Delaunify(bool deep, Delaunator &del)
    {
        int flipped;
        int iLoop = 0;

        do
        {
            iLoop++;
            if (iLoop > loopMax)
            {
                Debug::log("Constrainautor 3 loopMax " + to_string(loopMax));
                return;
            }

            flipped = 0;
            int dirtyEdgeLoopMax = del.halfedges.size() * 4;
            int edgeChecks = 0;
            while (dirtyEdges.size() > 0)
            {
                edgeChecks++;
                if (edgeChecks > dirtyEdgeLoopMax)
                {
                    Debug::log("Constrainautor dirty edge loopMax " + to_string(dirtyEdgeLoopMax));
                    dirtyEdges.clear();
                    for (int i = 0; i < dirtyEdgeQueued.size(); i++)
                    {
                        dirtyEdgeQueued[i] = false;
                    }
                    return;
                }

                int dirtyEdgeIndex = dirtyEdges.size() - 1;
                int edg = dirtyEdges[dirtyEdgeIndex];
                dirtyEdges.erase(dirtyEdges.begin() + dirtyEdgeIndex);
                dirtyEdgeQueued[edg] = false;

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
        } while (deep && flipped > 0 && dirtyEdges.size() > 0);
    }

    void Constrainautor::EnqueueDirtyEdge(int edg, Delaunator &del)
    {
        if (edg < 0 || edg >= del.halfedges.size() || dirtyEdgeQueued[edg])
        {
            return;
        }

        dirtyEdgeQueued[edg] = true;
        dirtyEdges.push_back(edg);
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

        EnqueueDirtyEdge(bot, del);
        EnqueueDirtyEdge(top, del);
        EnqueueDirtyEdge(lft, del);
        EnqueueDirtyEdge(rgt, del);
        EnqueueDirtyEdge(adjTop, del);
        EnqueueDirtyEdge(adjBot, del);

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

        double ax = del.coords[p1 * 2];
        double ay = del.coords[p1 * 2 + 1];
        double bx = del.coords[p2 * 2];
        double by = del.coords[p2 * 2 + 1];
        double cx = del.coords[p3 * 2];
        double cy = del.coords[p3 * 2 + 1];
        double dx = del.coords[px * 2];
        double dy = del.coords[px * 2 + 1];

        double scale = MathUtils::abs(ax - dx);
        scale = MathUtils::max(scale, MathUtils::abs(ay - dy));
        scale = MathUtils::max(scale, MathUtils::abs(bx - dx));
        scale = MathUtils::max(scale, MathUtils::abs(by - dy));
        scale = MathUtils::max(scale, MathUtils::abs(cx - dx));
        scale = MathUtils::max(scale, MathUtils::abs(cy - dy));

        double determinant = InCircle(ax, ay, bx, by, cx, cy, dx, dy);
        double tolerance = 64.0 * MathUtils::DOUBLE_EPSILON * scale * scale * scale * scale;
        bool result = determinant < -tolerance;

        if (debugInCircleToleranceErrors)
        {
            bool resultWithoutTolerance = determinant < 0.0f;
            if (resultWithoutTolerance != result)
            {
                inCircleComparisonMismatchCount++;
                if (inCircleComparisonLoggedCount < 32)
                {
                    Debug::log(
                        "Constrainautor InCircle mismatch without tolerance " + to_string(resultWithoutTolerance) +
                        " new " + to_string(result) +
                        " points " + to_string(p1) + " " + to_string(p2) + " " + to_string(p3) + " " + to_string(px) +
                        " determinant " + to_string(determinant) +
                        " tolerance " + to_string(tolerance) +
                        " scale " + to_string(scale));
                    inCircleComparisonLoggedCount++;
                }
            }
        }

        return result;
    }

    int Constrainautor::NextEdge(int e)
    {
        return (e % 3 == 2) ? e - 2 : e + 1;
    }

    int Constrainautor::PrevEdge(int e)
    {
        return (e % 3 == 0) ? e + 2 : e - 1;
    }

    double Constrainautor::Orient2D(double ax, double ay, double bx, double by, double cx, double cy)
    {
        double acx = ax - cx;
        double bcx = bx - cx;
        double acy = ay - cy;
        double bcy = by - cy;
        return acx * bcy - acy * bcx;
    }

    double Constrainautor::InCircle(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
    {
        double adx = ax - dx;
        double ady = ay - dy;
        double bdx = bx - dx;
        double bdy = by - dy;
        double cdx = cx - dx;
        double cdy = cy - dy;

        double abdet = adx * bdy - bdx * ady;
        double bcdet = bdx * cdy - cdx * bdy;
        double cadet = cdx * ady - adx * cdy;
        double alift = adx * adx + ady * ady;
        double blift = bdx * bdx + bdy * bdy;
        double clift = cdx * cdx + cdy * cdy;

        return alift * bcdet + blift * cadet + clift * abdet;
    }

    bool Constrainautor::IntersectSegments(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y, double p4x, double p4y)
    {
        double x0 = Orient2D(p1x, p1y, p3x, p3y, p4x, p4y);
        double y0 = Orient2D(p2x, p2y, p3x, p3y, p4x, p4y);

        if ((x0 > 0 && y0 > 0) || (x0 < 0 && y0 < 0))
        {
            return false;
        }

        double x1 = Orient2D(p3x, p3y, p1x, p1y, p2x, p2y);
        double y1 = Orient2D(p4x, p4y, p1x, p1y, p2x, p2y);

        if ((x1 > 0 && y1 > 0) || (x1 < 0 && y1 < 0))
        {
            return false;
        }

        const double epsilon = 0.00000001f;
        // Check for degenerate collinear case
        // if (x0 == 0 && y0 == 0 && x1 == 0 && y1 == 0)
        if (x0 < epsilon && x0 > -epsilon &&
            y0 < epsilon && y0 > -epsilon &&
            x1 < epsilon && x1 > -epsilon &&
            y1 < epsilon && y1 > -epsilon)
        {
            return !(MathUtils::max(p3x, p4x) < MathUtils::min(p1x, p2x) ||
                     MathUtils::max(p1x, p2x) < MathUtils::min(p3x, p4x) ||
                     MathUtils::max(p3y, p4y) < MathUtils::min(p1y, p2y) ||
                     MathUtils::max(p1y, p2y) < MathUtils::min(p3y, p4y));
        }

        return true;
    }
}
