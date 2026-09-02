#ifndef DELAUNATOR_HPP
#define DELAUNATOR_HPP

#include <vector>
#include "rts/math/float2.hpp"
#include "delaunator_triangle.hpp"
#include "delaunator_edge.hpp"

using namespace std;

namespace NavigationSystemCode
{
    // Ported and based on https://github.com/nol1fe/delaunator-sharp
    struct Delaunator
    {
        double EPSILON;
        vector<int> EDGE_STACK;

        /// <summary>
        /// One value per half-edge, containing the point index of where a given half edge starts.
        /// </summary>
        vector<int> triangles;

        /// <summary>
        /// One value per half-edge, containing the opposite half-edge in the adjacent triangle, or -1 if there is no adjacent triangle
        /// </summary>
        vector<int> halfedges;

        /// <summary>
        /// A list of point indices that traverses the hull of the points.
        /// </summary>
        // public int[] Hull;

        int hashSize;

        vector<int> hullPrev;
        vector<int> hullNext;
        vector<int> hullTri;
        vector<int> hullHash;

        vector<int> ids;
        vector<double> dists;

        double cx_final;
        double cy_final;

        int trianglesLen;
        vector<double> coords;
        int hullStart;
        int hullSize;

        Delaunator();
        void Create(vector<Float2> &p_points);
        void ClearTemporaryLists();
        int Legalize(int a);
        static bool InCircle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py);
        int AddTriangle(int i0, int i1, int i2, int a, int b, int c);
        void Link(int a, int b);
        int HashKey(double x, double y);
        static double PseudoAngle(double dx, double dy);
        static bool Orient(double px, double py, double qx, double qy, double rx, double ry);
        static double Circumradius(double ax, double ay, double bx, double by, double cx, double cy);
        static Float2 Circumcenter(double ax, double ay, double bx, double by, double cx, double cy);
        static double Dist(double ax, double ay, double bx, double by);
        vector<DelaunatorTriangle> GetTriangles();
        vector<DelaunatorEdge> GetEdges();
        vector<int> PointsOfTriangle(int t);
        vector<int> TrianglesAdjacentToTriangle(int t);
        static int NextHalfedge(int e);
        static vector<int> EdgesOfTriangle(int t);
        static int TriangleOfEdge(int e);
    };
}

#endif
