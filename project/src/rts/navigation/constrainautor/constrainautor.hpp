#ifndef CONSTRAINAUTOR_HPP
#define CONSTRAINAUTOR_HPP

#include <vector>
#include "bool_set.hpp"
#include "constraint_edge.hpp"
#include "rts/navigation/delaunator/delaunator.hpp"

using namespace std;

namespace NavigationSystemCode
{
    // Ported and based on https://github.com/kninnug/Constrainautor
    struct Constrainautor
    {
        vector<int> vertMap;
        vector<int> dirtyEdges;
        vector<bool> dirtyEdgeQueued;
        int inCircleComparisonMismatchCount;
        int inCircleComparisonLoggedCount;
        BoolSet flips;
        BoolSet consd;
        int loopMax;
        bool debugInCircleToleranceErrors;

        void Create(Delaunator &del, vector<ConstraintEdge> &edges, const string &reason);
        void ClearTemporaryLists();
        void ConstrainOne(int segP1, int segP2, Delaunator &del, const string &reason);
        void ConstrainAll(vector<ConstraintEdge> &edges, Delaunator &del, const string &reason);
        void Delaunify(bool deep, Delaunator &del);
        void EnqueueDirtyEdge(int edg, Delaunator &del);
        int Protect(int edg, Delaunator &del);
        bool MarkFlip(int edg, Delaunator &del);
        void FlipDiagonal(int edg, Delaunator &del);
        bool IsDelaunay(int edg, Delaunator &del);
        int UpdateVert(int start, Delaunator &del);
        bool IntersectSegments(int p1, int p2, int p3, int p4, Delaunator &del);
        bool InCircle(int p1, int p2, int p3, int px, Delaunator &del);
        int NextEdge(int e);
        int PrevEdge(int e);
        double Orient2D(double ax, double ay, double bx, double by, double cx, double cy);
        double InCircle(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy);
        bool IntersectSegments(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y, double p4x, double p4y);
    };
}

#endif
