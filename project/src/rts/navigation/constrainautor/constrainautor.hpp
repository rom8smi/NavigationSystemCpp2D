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
        BoolSet flips;
        BoolSet consd;
        int loopMax;

        void Create(Delaunator &del, vector<ConstraintEdge> &edges);
        void ClearTemporaryLists();
        void ConstrainOne(int segP1, int segP2, Delaunator &del);
        void ConstrainAll(vector<ConstraintEdge> &edges, Delaunator &del);
        void Delaunify(bool deep, Delaunator &del);
        int Protect(int edg, Delaunator &del);
        bool MarkFlip(int edg, Delaunator &del);
        void FlipDiagonal(int edg, Delaunator &del);
        bool IsDelaunay(int edg, Delaunator &del);
        int UpdateVert(int start, Delaunator &del);
        bool IntersectSegments(int p1, int p2, int p3, int p4, Delaunator &del);
        bool InCircle(int p1, int p2, int p3, int px, Delaunator &del);
        int NextEdge(int e);
        int PrevEdge(int e);
        float Orient2D(float ax, float ay, float bx, float by, float cx, float cy);
        float InCircle(float ax, float ay, float bx, float by, float cx, float cy, float dx, float dy);
        bool IntersectSegments(float p1x, float p1y, float p2x, float p2y, float p3x, float p3y, float p4x, float p4y);
    };
}

#endif
