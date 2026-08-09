#ifndef TRIANGULATION_GRID_SEARCH_HPP
#define TRIANGULATION_GRID_SEARCH_HPP

#include <vector>
#include "aabb.hpp"
#include "rts/navigation/delaunator/delaunator.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct TriangulationGridSearch
    {
        Aabb triangulationBounds;
        float nodeDiameterX;
        float nodeDiameterY;
        int resolution;

        vector<int> trianglesStart;
        vector<int> trianglesCount;
        vector<int> trianglesByCells;

        void Create(
            Aabb &p_triangulationBounds,
            int p_resolution,
            Delaunator &p_delaunator,
            vector<Float2> &p_delaunatorPoints,
            int p_trianglesCount,
            vector<bool> &p_trianglesMask);
        int FindTriangleForPoint(
            Float2 &p_position,
            Delaunator &p_delaunator,
            vector<Float2> &p_delaunatorPoints);
        int GetGridIndexX(Float2 &position);
        int GetGridIndexY(Float2 &position);
    };
}

#endif
