#include "triangulation_grid_search.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/math/vector_utils.hpp"

namespace NavigationSystemCode
{
    void TriangulationGridSearch::Create(
        Aabb &p_triangulationBounds,
        int p_resolution,
        Delaunator &p_delaunator,
        vector<Float2> &p_delaunatorPoints,
        int p_trianglesCount,
        vector<bool> &p_trianglesMask)
    {
        resolution = p_resolution;
        triangulationBounds = p_triangulationBounds;

        int resolutionSqr = resolution * resolution;

        vector<vector<int>> trianglesGrid;
        trianglesGrid.resize(resolutionSqr);

        nodeDiameterX = (triangulationBounds.maxX - triangulationBounds.minX) / resolution;
        nodeDiameterY = (triangulationBounds.maxY - triangulationBounds.minY) / resolution;

        int cellsCount = 0;

        for (int i = 0; i < p_trianglesCount; i++)
        {
            if (p_trianglesMask[i])
            {
                int e0 = 3 * i;
                int tp1 = p_delaunator.triangles[e0];
                int tp2 = p_delaunator.triangles[e0 + 1];
                int tp3 = p_delaunator.triangles[e0 + 2];

                Float2 p1 = p_delaunatorPoints[tp1];
                Float2 p2 = p_delaunatorPoints[tp2];
                Float2 p3 = p_delaunatorPoints[tp3];

                int ip1x = GetGridIndexX(p1);
                int ip2x = GetGridIndexX(p2);
                int ip3x = GetGridIndexX(p3);

                int ip1y = GetGridIndexY(p1);
                int ip2y = GetGridIndexY(p2);
                int ip3y = GetGridIndexY(p3);

                int ip1 = ip1x * resolution + ip1y;
                int ip2 = ip2x * resolution + ip2y;
                int ip3 = ip3x * resolution + ip3y;

                cellsCount++;
                trianglesGrid[ip1].push_back(i);

                if (ip2 != ip1)
                {
                    cellsCount++;
                    trianglesGrid[ip2].push_back(i);
                }
                if (ip3 != ip1 && ip3 != ip2)
                {
                    cellsCount++;
                    trianglesGrid[ip3].push_back(i);
                }

                int minX = MathUtils::min(ip1x, ip2x);
                minX = MathUtils::min(minX, ip3x);
                int maxX = MathUtils::max(ip1x, ip2x);
                maxX = MathUtils::max(maxX, ip3x);

                int minY = MathUtils::min(ip1y, ip2y);
                minY = MathUtils::min(minY, ip3y);
                int maxY = MathUtils::max(ip1y, ip2y);
                maxY = MathUtils::max(maxY, ip3y);

                for (int ix = minX; ix <= maxX; ix++)
                {
                    for (int iy = minY; iy <= maxY; iy++)
                    {
                        int ik = ix * resolution + iy;

                        if (ik != ip1 && ik != ip2 && ik != ip3)
                        {
                            Float2 cellPos00 = Float2(nodeDiameterX * ix + triangulationBounds.minX, nodeDiameterY * iy + triangulationBounds.minY);
                            Float2 cellPos01 = Float2(nodeDiameterX * ix + triangulationBounds.minX, nodeDiameterY * (iy + 1) + triangulationBounds.minY);
                            Float2 cellPos11 = Float2(nodeDiameterX * (ix + 1) + triangulationBounds.minX, nodeDiameterY * (iy + 1) + triangulationBounds.minY);
                            Float2 cellPos10 = Float2(nodeDiameterX * (ix + 1) + triangulationBounds.minX, nodeDiameterY * iy + triangulationBounds.minY);

                            if (
                                VectorUtils::AreLineSegmentsIntersecting(p1, p2, cellPos00, cellPos01) ||
                                VectorUtils::AreLineSegmentsIntersecting(p2, p3, cellPos00, cellPos01) ||
                                VectorUtils::AreLineSegmentsIntersecting(p3, p1, cellPos00, cellPos01) ||

                                VectorUtils::AreLineSegmentsIntersecting(p1, p2, cellPos01, cellPos11) ||
                                VectorUtils::AreLineSegmentsIntersecting(p2, p3, cellPos01, cellPos11) ||
                                VectorUtils::AreLineSegmentsIntersecting(p3, p1, cellPos01, cellPos11) ||

                                VectorUtils::AreLineSegmentsIntersecting(p1, p2, cellPos11, cellPos10) ||
                                VectorUtils::AreLineSegmentsIntersecting(p2, p3, cellPos11, cellPos10) ||
                                VectorUtils::AreLineSegmentsIntersecting(p3, p1, cellPos11, cellPos10) ||

                                VectorUtils::AreLineSegmentsIntersecting(p1, p2, cellPos10, cellPos00) ||
                                VectorUtils::AreLineSegmentsIntersecting(p2, p3, cellPos10, cellPos00) ||
                                VectorUtils::AreLineSegmentsIntersecting(p3, p1, cellPos10, cellPos00))
                            {
                                cellsCount++;
                                trianglesGrid[ik].push_back(i);
                            }
                            else if (VectorUtils::PointInTriangle(cellPos00, p1, p2, p3))
                            {
                                cellsCount++;
                                trianglesGrid[ik].push_back(i);
                            }
                        }
                    }
                }
            }
        }

        trianglesStart.resize(resolutionSqr);
        trianglesCount.resize(resolutionSqr);
        trianglesByCells.resize(cellsCount);

        cellsCount = 0;

        for (int i = 0; i < resolutionSqr; i++)
        {
            int start = cellsCount;
            int count = trianglesGrid[i].size();

            trianglesStart[i] = start;
            trianglesCount[i] = count;

            for (int j = 0; j < count; j++)
            {
                trianglesByCells[cellsCount] = trianglesGrid[i][j];
                cellsCount++;
            }
        }
    }

    int TriangulationGridSearch::FindTriangleForPoint(
        Float2 &p_position,
        Delaunator &p_delaunator,
        vector<Float2> &p_delaunatorPoints)
    {
        int ix = GetGridIndexX(p_position);
        int iy = GetGridIndexY(p_position);

        if (ix > -1 && ix < resolution && iy > -1 && iy < resolution)
        {
            int k = ix * resolution + iy;

            int start = trianglesStart[k];
            int count = trianglesCount[k];

            for (int i = 0; i < count; i++)
            {
                int triangle = trianglesByCells[start + i];

                int e0 = 3 * triangle;
                int ip1 = p_delaunator.triangles[e0];
                int ip2 = p_delaunator.triangles[e0 + 1];
                int ip3 = p_delaunator.triangles[e0 + 2];

                if (VectorUtils::PointInTriangle(p_position, p_delaunatorPoints[ip1], p_delaunatorPoints[ip2], p_delaunatorPoints[ip3]))
                {
                    return triangle;
                }
            }
        }

        return -1;
    }

    int TriangulationGridSearch::GetGridIndexX(Float2 &position)
    {
        return (int)((position.x - triangulationBounds.minX) / nodeDiameterX);
    }

    int TriangulationGridSearch::GetGridIndexY(Float2 &position)
    {
        return (int)((position.y - triangulationBounds.minY) / nodeDiameterY);
    }
}
