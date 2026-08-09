#include "nav_mesh.hpp"
#include "rts/math/math_utils.hpp"
#include "rts/math/vector_utils.hpp"
#include <cmath>
#include "rts/godot_wrapper/godot_utils.hpp"
#include "rts/math/heap_sort.hpp"
#include "rts/debug/debug.hpp"
#include "rts/math/duplicate_utils.hpp"
#include "rts/math/collections_utils.hpp"

namespace NavigationSystemCode
{
    NavMesh::NavMesh()
    {
        random.seed = 8;
    }

    void NavMesh::Create(vector<Obstacle> &obstacles, Aabb &bounds)
    {
        // const auto t1 = std::chrono::high_resolution_clock::now();
        worldBounds = bounds;
        SubdivideAndBuildConstrainedTriangulation(obstacles);

        allTriangles = delaunator.GetTriangles();
        allEdges = delaunator.GetEdges();

        allTriangleBounds = vector<Aabb>();

        for (int i = 0; i < allTriangles.size(); i++)
        {
            Aabb aabb = Aabb{
                .minX = MathUtils::FLOAT_MAX_VALUE,
                .maxX = MathUtils::FLOAT_MIN_VALUE,
                .minY = MathUtils::FLOAT_MAX_VALUE,
                .maxY = MathUtils::FLOAT_MIN_VALUE,
            };
            vector<int> trianglePoints = allTriangles[i].points;

            for (int j = 0; j < trianglePoints.size(); j++)
            {
                Float2 point = allPoints[trianglePoints[j]];
                aabb.minX = MathUtils::min(aabb.minX, point.x);
                aabb.maxX = MathUtils::max(aabb.maxX, point.x);
                aabb.minY = MathUtils::min(aabb.minY, point.y);
                aabb.maxY = MathUtils::max(aabb.maxY, point.y);
            }

            allTriangleBounds.push_back(aabb);
        }

        CalculateTriangleCentroids();
        CalculateEdgesAroundPointsMap();
        FindWalkableEdges(obstacles);
        FindWalkableTriangles(obstacles);

        ResolveObstacleHullEdges(obstacles);

        CalulateHullEdgeTriangulationEdgeToObstacleIndices();
        CalculateSizeOfSmallestHullEdge();
        CreateTriangulationSearch();
        CreateVisitedTriangles();
        // const auto t2 = std::chrono::high_resolution_clock::now();

        // const double dt1 = std::chrono::duration<double, std::milli>(t2 - t1).count();

        // GodotUtils::print(to_string(dt1));
        // Times are about 20 ms for default scene.
    }

    vector<Float2> NavMesh::GetSubdividedWorldBoundEdges(vector<Obstacle> &obstacles)
    {
        vector<Float2> worldBoundCorners = GetDefaultWorldBounds();
        vector<Float2> subdividedWorldBoundCorners;

        for (int i = 0; i < worldBoundCorners.size(); i++)
        {
            int iNext = i + 1;
            if (iNext == worldBoundCorners.size())
            {
                iNext = 0;
            }

            Float2 p1 = worldBoundCorners[i];
            Float2 p2 = worldBoundCorners[iNext];
            float minDistanceSqr = MathUtils::FLOAT_MAX_VALUE;

            for (int j = 0; j < obstacles.size(); j++)
            {
                vector<Float2> obstacleCorners = obstacles[j].obstacleCorners;
                for (int k = 0; k < obstacleCorners.size(); k++)
                {
                    if (worldBounds.IsInside(obstacleCorners[k]))
                    {
                        Float2 projectedPoint = VectorUtils::FindNearestPointOnLineSegment(p1, p2, obstacleCorners[k]);
                        float distanceSqr = (obstacleCorners[k] - projectedPoint).length_squared();

                        if (distanceSqr > 0.0f && distanceSqr < minDistanceSqr)
                        {
                            minDistanceSqr = distanceSqr;
                        }
                    }
                }
            }

            int nSubdivides = (int)(0.5f * (p2 - p1).length() / sqrt(minDistanceSqr));
            if (nSubdivides > 5)
            {
                nSubdivides = 5;
            }

            Float2 diff_p2_p1 = p2 - p1;
            Float2 normal = VectorUtils::PerpendicularCounterClockwise(diff_p2_p1).normalized() * 0.001f;

            for (int j = 0; j < nSubdivides; j++)
            {
                float f1 = 1.0f * (j + 1) / (nSubdivides + 1);
                float f2 = 1.0f - f1;

                Float2 subdividedCorner = p1 * f1 + p2 * f2;
                bool isInsideObstacle = false;

                for (int k = 0; k < obstacles.size(); k++)
                {
                    Float2 subdividedCornerWithOffset = subdividedCorner + normal;

                    if (!isInsideObstacle && VectorUtils::IsPointInPolygon(subdividedCornerWithOffset, obstacles[k].obstacleCorners))
                    {
                        isInsideObstacle = true;
                    }
                }

                if (!isInsideObstacle)
                {
                    subdividedWorldBoundCorners.push_back(p1 * f1 + p2 * f2);
                }
            }
        }

        for (int i = 0; i < subdividedWorldBoundCorners.size(); i++)
        {
            worldBoundCorners.push_back(subdividedWorldBoundCorners[i]);
        }

        return worldBoundCorners;
    }

    void NavMesh::SubdivideAndBuildConstrainedTriangulation(vector<Obstacle> &obstacles)
    {
        vector<Float2> worldBoundCorners = GetSubdividedWorldBoundEdges(obstacles);
        totalNumberOfWorldBoundCorners = worldBoundCorners.size();

        allPoints.clear();
        obstacleWalkablityIndices.clear();
        isObstacleCornerIntersectingWithWorldBounds.clear();

        for (int i = 0; i < worldBoundCorners.size(); i++)
        {
            allPoints.push_back(worldBoundCorners[i]);
            obstacleWalkablityIndices.push_back(-1);
            isObstacleCornerIntersectingWithWorldBounds.push_back(false);
        }

        vector<ConstraintEdge> constraintEdges;
        if (obstacles.size() > 0)
        {
            AddObstaclesWithConstraints(
                obstacles,
                constraintEdges,
                allPoints,
                obstacleWalkablityIndices,
                obstacleIntersections,
                isObstacleCornerIntersectingWithWorldBounds);
        }

        delaunator.Create(allPoints);
        delaunator.ClearTemporaryLists();

        constrainautor.Create(delaunator, constraintEdges);
        constrainautor.ClearTemporaryLists();
    }

    void NavMesh::FindWalkableTriangles(vector<Obstacle> &obstacles)
    {
        unwalkableTriangles.clear();
        walkableTriangles.clear();
        unwalkableTrianglesObstacleIndices.clear();
        allToUnwalkableTriangleIndices.clear();

        for (int i = 0; i < allTriangles.size(); i++)
        {
            DelaunatorTriangle triangle = allTriangles[i];
            vector<int> trianglePoints = triangle.points;

            bool isWalkable = trianglesWalkability[i] == -1;

            if (!isWalkable)
            {
                allToUnwalkableTriangleIndices.push_back(unwalkableTriangles.size());
                unwalkableTriangles.push_back(i);
                unwalkableTrianglesObstacleIndices.push_back(trianglesWalkability[i]);
            }
            else
            {
                allToUnwalkableTriangleIndices.push_back(-1);
                walkableTriangles.push_back(i);
            }
        }
    }

    void NavMesh::FindWalkableEdges(vector<Obstacle> &obstacles)
    {
        edgesWalkability.resize(delaunator.trianglesLen);

        for (int i = 0; i < edgesWalkability.size(); i++)
        {
            edgesWalkability[i] = true;
        }

        trianglesWalkability.resize(allTriangles.size());

        for (int i = 0; i < allTriangles.size(); i++)
        {
            trianglesWalkability[i] = -1;
        }

        float epsilon = 0.001f;
        vector<int> neighbours;

        for (int i = 0; i < obstacles.size(); i++)
        {
            Float2 center = obstacles[i].center;
            float largestCornerDistance = obstacles[i].largest_corner_distance;

            neighbours.clear();
            allTriangleCentroidsKdTree.find_nearests_ball(center, largestCornerDistance + 2.0f * epsilon, neighbours);

            for (int j = 0; j < neighbours.size(); j++)
            {
                int triangleIndex = neighbours[j];
                if (trianglesWalkability[triangleIndex] == -1 && VectorUtils::IsPointInPolygon(allTriangleCentroids[triangleIndex], obstacles[i].obstacleCorners))
                {
                    trianglesWalkability[triangleIndex] = i;
                }
            }
        }

        for (int i = 0; i < obstacles.size(); i++)
        {
            if (obstacles[i].isWalkable)
            {
                Float2 center = obstacles[i].center;
                float largestCornerDistance = obstacles[i].largest_corner_distance;

                neighbours.clear();
                allTriangleCentroidsKdTree.find_nearests_ball(center, largestCornerDistance + 2.0f * epsilon, neighbours);

                for (int j = 0; j < neighbours.size(); j++)
                {
                    int triangleIndex = neighbours[j];
                    if (trianglesWalkability[triangleIndex] != -1 && VectorUtils::IsPointInPolygon(allTriangleCentroids[triangleIndex], obstacles[i].obstacleCorners))
                    {
                        trianglesWalkability[triangleIndex] = -1;
                    }
                }
            }
        }

        for (int i = 0; i < allTriangles.size(); i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int e = 3 * i + j;
                int opposite = delaunator.halfedges[e];

                if (opposite >= 0)
                {
                    int nextTriangle = Delaunator::TriangleOfEdge(opposite);
                    if ((trianglesWalkability[i] != -1 && trianglesWalkability[nextTriangle] != -1))
                    {
                        edgesWalkability[e] = false;
                        edgesWalkability[opposite] = false;
                    }
                }
            }
        }

        for (int i = 0; i < allEdges.size(); i++)
        {
            int p = allEdges[i].p;
            int q = allEdges[i].q;

            Float2 midPoint = (allPoints[p] + allPoints[q]) * 0.5f;

            if (!worldBounds.IsInside(midPoint))
            {
                if (isObstacleCornerIntersectingWithWorldBounds[p] && isObstacleCornerIntersectingWithWorldBounds[q])
                {
                    int e = allEdges[i].index;
                    edgesWalkability[e] = false;
                }
                else if (
                    (isObstacleCornerIntersectingWithWorldBounds[p] && q < totalNumberOfWorldBoundCorners) ||
                    (isObstacleCornerIntersectingWithWorldBounds[q] && p < totalNumberOfWorldBoundCorners))
                {
                    int e = allEdges[i].index;
                    edgesWalkability[e] = false;
                }
            }
        }
    }

    void NavMesh::CalculateSizeOfSmallestHullEdge()
    {
        smallestHullEdgeSize = worldBounds.maxX - worldBounds.minX;

        for (int i = 0; i < obstacleHullEdges.size(); i++)
        {
            int edgeIndex = obstacleHullEdges[i];
            int p = allEdges[edgeIndex].p;
            int q = allEdges[edgeIndex].q;

            float edgeSize = (allPoints[p] - allPoints[q]).length();

            smallestHullEdgeSize = MathUtils::min(smallestHullEdgeSize, edgeSize);
        }
    }

    void NavMesh::CalculateTriangleCentroids()
    {
        allTriangleCentroids.resize(allTriangles.size());

        for (int i = 0; i < allTriangleCentroids.size(); i++)
        {
            vector<int> trianglePoints = allTriangles[i].points;

            Float2 p1 = allPoints[trianglePoints[0]];
            Float2 p2 = allPoints[trianglePoints[1]];
            Float2 p3 = allPoints[trianglePoints[2]];

            allTriangleCentroids[i] = (p1 + p2 + p3) / 3.0f;
        }

        allTriangleCentroidsKdTree.make_from_points(allTriangleCentroids);
    }

    void NavMesh::CreateTriangulationSearch()
    {
        vector<bool> allTrianglesMask;
        vector<bool> walkableTrianglesMask;
        vector<bool> unwalkableTrianglesMask;

        int trianglesCount = allTriangles.size();

        allTrianglesMask.resize(trianglesCount);
        walkableTrianglesMask.resize(trianglesCount);
        unwalkableTrianglesMask.resize(trianglesCount);

        for (int i = 0; i < trianglesCount; i++)
        {
            allTrianglesMask[i] = true;

            bool isCurrentTriangleWalkable = trianglesWalkability[i] == -1;
            walkableTrianglesMask[i] = isCurrentTriangleWalkable;
            unwalkableTrianglesMask[i] = !isCurrentTriangleWalkable;
        }

        int resolution = 40;

        Aabb triangulationGridBounds = Aabb{
            .minX = worldBounds.minX - 1.0f,
            .maxX = worldBounds.maxX + 1.0f,
            .minY = worldBounds.minY - 1.0f,
            .maxY = worldBounds.maxY + 1.0f};

        allTriangulationGridSearch.Create(triangulationGridBounds, resolution, delaunator, allPoints, trianglesCount, allTrianglesMask);
        walkableTriangulationGridSearch.Create(triangulationGridBounds, resolution, delaunator, allPoints, trianglesCount, walkableTrianglesMask);
        unwalkableTriangulationGridSearch.Create(triangulationGridBounds, resolution, delaunator, allPoints, trianglesCount, unwalkableTrianglesMask);
    }

    void NavMesh::CalculateEdgesAroundPointsMap()
    {
        edgesAroundPointsMap.clear();

        for (int i = 0; i < allPoints.size(); i++)
        {
            edgesAroundPointsMap.push_back(vector<int>());
        }

        for (int i = 0; i < allEdges.size(); i++)
        {
            int p = allEdges[i].p;
            int q = allEdges[i].q;

            edgesAroundPointsMap[p].push_back(i);
            edgesAroundPointsMap[q].push_back(i);
        }
    }

    void NavMesh::CalulateHullEdgeTriangulationEdgeToObstacleIndices()
    {
        hullEdgeTriangulationEdgeToObstacleIndices.clear();
        for (int i = 0; i < delaunator.trianglesLen; i++)
        {
            hullEdgeTriangulationEdgeToObstacleIndices.push_back(-1);
        }

        for (int i = 0; i < obstacleHullEdges.size(); i++)
        {
            int edgeIndex = obstacleHullEdges[i];
            hullEdgeTriangulationEdgeToObstacleIndices[allEdges[edgeIndex].index] = obstacleHullEdgeObstacleIndices[i];
        }
    }

    void NavMesh::CreateVisitedTriangles()
    {
        visitedTriangles.resize(allTriangles.size());
        for (int i = 0; i < visitedTriangles.size(); i++)
        {
            visitedTriangles[i] = false;
        }
    }

    vector<Float2> NavMesh::GetDefaultWorldBounds()
    {
        vector<Float2> points;
        points.push_back(Float2(worldBounds.minX, worldBounds.minY));
        points.push_back(Float2(worldBounds.maxX, worldBounds.minY));
        points.push_back(Float2(worldBounds.maxX, worldBounds.maxY));
        points.push_back(Float2(worldBounds.minX, worldBounds.maxY));

        return points;
    }

    void NavMesh::AddObstacles(vector<Obstacle> &obstacles)
    {
        for (int i = 0; i < obstacles.size(); i++)
        {
            AddObstacle(obstacles, i);
        }
    }

    void NavMesh::AddObstaclesWithConstraints(
        vector<Obstacle> &obstacles,
        vector<ConstraintEdge> &constraintEdges,
        vector<Float2> &newPoints,
        vector<int> &newObstacleWalkablityIndices,
        vector<vector<int>> &newObstacleIntersections,
        vector<bool> &newIsObstacleCornerIntersectingWithWorldBounds)
    {
        vector<Float2> segmentStarts;
        vector<Float2> segmentEnds;
        vector<Float2> segmentCenters;
        vector<float> segmentRadii;
        vector<int> segmentObstacleIndices;
        vector<int> segmentCornerStartInObstacleIndices;

        vector<vector<vector<Float2>>> intersectionPointsByCorners;
        vector<vector<vector<int>>> intersectionObstacleIndicesByCorners;
        vector<vector<vector<int>>> intersectionCornerIndicesByCorners;
        vector<vector<vector<int>>> newPointsIndicesByCorners;
        vector<vector<vector<int>>> intersectionCornerIndicesByCornersNeighbours;

        float maxRadius = 0.0f;
        float epsilon = 0.001f;

        int obstaclesCount = obstacles.size();

        newObstacleIntersections.resize(obstaclesCount);

        intersectionPointsByCorners.resize(obstaclesCount);
        intersectionObstacleIndicesByCorners.resize(obstaclesCount);
        intersectionCornerIndicesByCorners.resize(obstaclesCount);
        newPointsIndicesByCorners.resize(obstaclesCount);
        intersectionCornerIndicesByCornersNeighbours.resize(obstaclesCount);

        int totalCornersCount = 0;
        for (int i = 0; i < obstaclesCount; i++)
        {
            totalCornersCount += obstacles[i].obstacleCorners.size();
        }

        segmentStarts.resize(totalCornersCount);
        segmentEnds.resize(totalCornersCount);
        segmentCenters.resize(totalCornersCount);
        segmentRadii.resize(totalCornersCount);
        segmentObstacleIndices.resize(totalCornersCount);
        segmentCornerStartInObstacleIndices.resize(totalCornersCount);

        int totalCornerIndex = 0;

        for (int i = 0; i < obstaclesCount; i++)
        {
            newObstacleIntersections[i].clear();

            int obstacleCornersCount = obstacles[i].obstacleCorners.size();

            intersectionPointsByCorners[i].resize(obstacleCornersCount);
            intersectionObstacleIndicesByCorners[i].resize(obstacleCornersCount);
            intersectionCornerIndicesByCorners[i].resize(obstacleCornersCount);
            newPointsIndicesByCorners[i].resize(obstacleCornersCount);
            intersectionCornerIndicesByCornersNeighbours[i].resize(obstacleCornersCount);

            for (int j = 0; j < obstacleCornersCount; j++)
            {
                intersectionPointsByCorners[i][j].clear();
                intersectionObstacleIndicesByCorners[i][j].clear();
                intersectionCornerIndicesByCorners[i][j].clear();
                newPointsIndicesByCorners[i][j].clear();
                intersectionCornerIndicesByCornersNeighbours[i][j].clear();

                int nextCornerIndex = j + 1;
                if (nextCornerIndex >= obstacleCornersCount)
                {
                    nextCornerIndex = 0;
                }

                Float2 segmentStart = obstacles[i].obstacleCorners[j];
                Float2 segmentEnd = obstacles[i].obstacleCorners[nextCornerIndex];
                Float2 segmentCenter = (segmentStart + segmentEnd) * 0.5f;
                float segmentRadius = (segmentCenter - segmentStart).length();
                maxRadius = MathUtils::max(maxRadius, segmentRadius);

                segmentStarts[totalCornerIndex] = segmentStart;
                segmentEnds[totalCornerIndex] = segmentEnd;
                segmentCenters[totalCornerIndex] = segmentCenter;
                segmentRadii[totalCornerIndex] = segmentRadius;
                segmentObstacleIndices[totalCornerIndex] = i;
                segmentCornerStartInObstacleIndices[totalCornerIndex] = j;

                totalCornerIndex++;
            }
        }

        KdTree2D segmentCentersKdTree;
        segmentCentersKdTree.make_from_points(segmentCenters);
        vector<int> neighbours;

        for (int i = 0; i < segmentCenters.size(); i++)
        {
            Float2 startPointI = segmentStarts[i];
            Float2 endPointI = segmentEnds[i];

            float searchDistance = maxRadius + segmentRadii[i] + 6.0f * epsilon;

            Float2 queryPoint = segmentCenters[i];
            neighbours.clear();
            segmentCentersKdTree.find_nearests_ball(queryPoint, searchDistance, neighbours);
            int neighboursCount = neighbours.size();

            int iSegmentObstacleIndex = segmentObstacleIndices[i];

            if (neighboursCount > 0)
            {
                HeapSort::Sort(neighbours);

                for (int j = 0; j < neighboursCount; j++)
                {
                    int neighbour = neighbours[j];
                    int neighbourSegmentObstacleIndex = segmentObstacleIndices[neighbour];

                    if (neighbour > i && iSegmentObstacleIndex != neighbourSegmentObstacleIndex)
                    {
                        Float2 startPointNeighbour = segmentStarts[neighbour];
                        Float2 endPointNeighbour = segmentEnds[neighbour];

                        LineSegmentsIntersectionResult result = VectorUtils::LineSegmentsIntersection(startPointI, endPointI, startPointNeighbour, endPointNeighbour, epsilon);
                        if (result.intersects &&
                            !VectorUtils::PointOnLine2D(startPointNeighbour, startPointI, endPointI, epsilon) &&
                            !VectorUtils::PointOnLine2D(endPointNeighbour, startPointI, endPointI, epsilon) &&
                            !VectorUtils::PointOnLine2D(startPointI, startPointNeighbour, endPointNeighbour, epsilon) &&
                            !VectorUtils::PointOnLine2D(endPointI, startPointNeighbour, endPointNeighbour, epsilon))
                        {
                            int iSegmentCornerStartIndex = segmentCornerStartInObstacleIndices[i];
                            int neighbourSegmentCornerStartIndex = segmentCornerStartInObstacleIndices[neighbour];

                            intersectionPointsByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].push_back(result.intersection);
                            intersectionObstacleIndicesByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].push_back(-1);
                            intersectionCornerIndicesByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].push_back(-1);
                            newPointsIndicesByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].push_back(-1);
                            intersectionCornerIndicesByCornersNeighbours[iSegmentObstacleIndex][iSegmentCornerStartIndex].push_back(-1);

                            intersectionPointsByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].push_back(result.intersection);
                            intersectionObstacleIndicesByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].push_back(iSegmentObstacleIndex);
                            intersectionCornerIndicesByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].push_back(iSegmentCornerStartIndex);
                            newPointsIndicesByCorners[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].push_back(-1);

                            intersectionCornerIndicesByCornersNeighbours[neighbourSegmentObstacleIndex][neighbourSegmentCornerStartIndex].push_back(
                                intersectionPointsByCorners[iSegmentObstacleIndex][iSegmentCornerStartIndex].size() - 1);

                            newObstacleIntersections[iSegmentObstacleIndex].push_back(neighbourSegmentObstacleIndex);
                            newObstacleIntersections[neighbourSegmentObstacleIndex].push_back(iSegmentObstacleIndex);
                        }
                    }
                }
            }
        }

        vector<int> segmentsBegin;
        vector<int> segmentsEnd;

        vector<int> sortedIndices;
        vector<float> sqrDistancesFromInitialPoint;

        for (int i = 0; i < obstacles.size(); i++)
        {
            int newPointsCountStart = newPoints.size();
            int obstacleICornersSize = obstacles[i].obstacleCorners.size();

            int currentSegmentsEndCount = 0;

            for (int j = 0; j < obstacleICornersSize; j++)
            {
                int jNext = j + 1;
                if (jNext >= obstacleICornersSize)
                {
                    jNext = 0;
                }

                newPoints.push_back(obstacles[i].obstacleCorners[j]);
                newObstacleWalkablityIndices.push_back(i);
                newIsObstacleCornerIntersectingWithWorldBounds.push_back(obstacles[i].isCornerIntersectingWithWorldBounds[j]);

                int currentNewPointsCount = newPoints.size();
                segmentsBegin.push_back(currentNewPointsCount - 1);
                segmentsEnd.push_back(currentNewPointsCount);

                int intersectionPointsByCornersIJCount = intersectionPointsByCorners[i][j].size();
                if (intersectionPointsByCornersIJCount > 0)
                {
                    sortedIndices.resize(intersectionPointsByCornersIJCount);
                    sqrDistancesFromInitialPoint.resize(intersectionPointsByCornersIJCount);

                    for (int k = 0; k < intersectionPointsByCornersIJCount; k++)
                    {
                        sortedIndices[k] = k;
                        sqrDistancesFromInitialPoint[k] = (obstacles[i].obstacleCorners[j] - intersectionPointsByCorners[i][j][k]).length_squared();
                    }

                    HeapSort::Sort(sortedIndices, sqrDistancesFromInitialPoint);

                    for (int k = 0; k < intersectionPointsByCornersIJCount; k++)
                    {
                        int kSorted = sortedIndices[k];

                        if (intersectionObstacleIndicesByCorners[i][j][kSorted] == -1)
                        {
                            newPoints.push_back(intersectionPointsByCorners[i][j][kSorted]);
                            newObstacleWalkablityIndices.push_back(i);
                            newIsObstacleCornerIntersectingWithWorldBounds.push_back(obstacles[i].isCornerIntersectingWithWorldBounds[j] && obstacles[i].isCornerIntersectingWithWorldBounds[jNext]);

                            currentNewPointsCount = newPoints.size();
                            segmentsBegin.push_back(currentNewPointsCount - 1);
                            segmentsEnd.push_back(currentNewPointsCount);
                            newPointsIndicesByCorners[i][j][kSorted] = currentNewPointsCount - 1;
                        }
                        else
                        {
                            int iNewPoint = intersectionObstacleIndicesByCorners[i][j][kSorted];
                            int jNewPoint = intersectionCornerIndicesByCorners[i][j][kSorted];
                            int kNewPoint = intersectionCornerIndicesByCornersNeighbours[i][j][kSorted];

                            int newPointsBeginIndex = newPointsIndicesByCorners[iNewPoint][jNewPoint][kNewPoint];

                            currentSegmentsEndCount = segmentsEnd.size();
                            segmentsEnd[currentSegmentsEndCount - 1] = newPointsBeginIndex;
                            segmentsBegin.push_back(newPointsBeginIndex);

                            currentNewPointsCount = newPoints.size();
                            segmentsEnd.push_back(currentNewPointsCount);
                        }
                    }
                }
            }

            currentSegmentsEndCount = segmentsEnd.size();
            segmentsEnd[currentSegmentsEndCount - 1] = newPointsCountStart;
        }

        int segmentsBeginCount = segmentsBegin.size();
        constraintEdges.resize(segmentsBeginCount);

        for (int i = 0; i < segmentsBegin.size(); i++)
        {
            constraintEdges[i] = ConstraintEdge{
                .p = segmentsBegin[i],
                .q = segmentsEnd[i]};
        }

        // Currently causes more problems than solving
        // SplitCollinearConstrainedEdges(constraintEdges, newPoints, epsilon);
    }

    void NavMesh::SplitCollinearConstrainedEdges(
        vector<ConstraintEdge> &constraintEdges,
        vector<Float2> &newPoints,
        float epsilon)
    {
        bool anyEdgesAdded = true;
        int anyEdgesAddedCount = 0;
        int maxAnyEdgesAddedCount = constraintEdges.size();
        int newPointsCount = newPoints.size();
        vector<vector<int>> edgeConnections;
        edgeConnections.resize(newPointsCount);

        while (anyEdgesAdded && anyEdgesAddedCount < maxAnyEdgesAddedCount)
        {
            int constraintEdgesCount = constraintEdges.size();
            anyEdgesAdded = false;
            anyEdgesAddedCount++;
            vector<Float2> constrainedEdgeCenters;
            vector<float> constrainedEdgeRadii;
            vector<bool> affectedSegments;
            constrainedEdgeCenters.resize(constraintEdgesCount);
            constrainedEdgeRadii.resize(constraintEdgesCount);
            affectedSegments.resize(constraintEdgesCount);

            float constrainedEdgeMaxRadius = 0.0f;

            for (int i = 0; i < newPointsCount; i++)
            {
                edgeConnections[i].clear();
            }

            for (int i = 0; i < constraintEdgesCount; i++)
            {
                int p = constraintEdges[i].p;
                int q = constraintEdges[i].q;
                Float2 center = (newPoints[p] + newPoints[q]) * 0.5f;
                float radius = (center - newPoints[p]).length();

                constrainedEdgeCenters[i] = center;
                constrainedEdgeRadii[i] = radius;
                affectedSegments[i] = false;

                constrainedEdgeMaxRadius = MathUtils::max(constrainedEdgeMaxRadius, radius);

                edgeConnections[p].push_back(q);
                edgeConnections[q].push_back(p);
            }

            KdTree2D constrainedEdgeCentersKdTree;
            constrainedEdgeCentersKdTree.make_from_points(constrainedEdgeCenters);

            vector<int> neighbours;

            for (int i = 0; i < constraintEdgesCount; i++)
            {
                if (!affectedSegments[i])
                {
                    float searchDistance = constrainedEdgeMaxRadius + constrainedEdgeRadii[i] + 6.0f * epsilon;

                    neighbours.clear();
                    constrainedEdgeCentersKdTree.find_nearests_ball(constrainedEdgeCenters[i], searchDistance, neighbours);
                    int neighboursCount = neighbours.size();

                    for (int j = 0; j < neighboursCount; j++)
                    {
                        int neighbour = neighbours[j];
                        if (neighbour != i && !affectedSegments[neighbour])
                        {
                            int ip = constraintEdges[i].p;
                            int iq = constraintEdges[i].q;
                            int neighbourp = constraintEdges[neighbour].p;
                            int neighbourq = constraintEdges[neighbour].q;

                            Float2 ipPoint = newPoints[ip];
                            Float2 iqPoint = newPoints[iq];
                            Float2 neighbourpPoint = newPoints[neighbourp];
                            Float2 neighbourqPoint = newPoints[neighbourq];

                            if (VectorUtils::AreLineSegmentsCollinearAndOverlapping(ipPoint, iqPoint, neighbourpPoint, neighbourqPoint, epsilon))
                            {
                                vector<int> selectedPointIndices;
                                vector<Float2> selectedPoints;
                                selectedPointIndices.resize(4);
                                selectedPoints.resize(4);

                                selectedPointIndices[0] = ip;
                                selectedPointIndices[1] = iq;
                                selectedPointIndices[2] = neighbourp;
                                selectedPointIndices[3] = neighbourq;

                                selectedPoints[0] = ipPoint;
                                selectedPoints[1] = iqPoint;
                                selectedPoints[2] = neighbourpPoint;
                                selectedPoints[3] = neighbourqPoint;

                                int masterStartPoint = 0;
                                float masterEndStartDistanceSqr = 0.0f;

                                for (int k = 1; k < 4; k++)
                                {
                                    float distSqr = (selectedPoints[0] - selectedPoints[k]).length_squared();
                                    if (distSqr > masterEndStartDistanceSqr)
                                    {
                                        masterEndStartDistanceSqr = distSqr;
                                        masterStartPoint = k;
                                    }
                                }

                                vector<float> distanceSqrFromMasterPoint;
                                vector<int> sortedIndicesFromMasterPoint;
                                distanceSqrFromMasterPoint.resize(4);
                                sortedIndicesFromMasterPoint.resize(4);

                                for (int k = 0; k < 4; k++)
                                {
                                    distanceSqrFromMasterPoint[k] = (selectedPoints[masterStartPoint] - selectedPoints[k]).length_squared();
                                    sortedIndicesFromMasterPoint[k] = k;
                                }

                                HeapSort::Sort(sortedIndicesFromMasterPoint, distanceSqrFromMasterPoint);

                                bool shouldSplit = true;

                                for (int k = 1; k < 4; k++)
                                {
                                    int kSortedPrevious = sortedIndicesFromMasterPoint[k - 1];
                                    int kSortedCurrent = sortedIndicesFromMasterPoint[k];

                                    int selectedPointIndexPrevious = selectedPointIndices[kSortedPrevious];
                                    int selectedPointIndexCurrent = selectedPointIndices[kSortedCurrent];

                                    if (selectedPointIndexPrevious == selectedPointIndexCurrent)
                                    {
                                        shouldSplit = false;
                                    }
                                    // if (shouldSplit && IsSegmentAdded(selectedPointIndexPrevious, selectedPointIndexCurrent, constraintEdges))
                                    if (shouldSplit && CollectionsUtils::contains(edgeConnections[selectedPointIndexPrevious], selectedPointIndexCurrent))
                                    {
                                        shouldSplit = false;
                                    }
                                }

                                if (shouldSplit)
                                {
                                    for (int k = 1; k < 4; k++)
                                    {
                                        int kSortedPrevious = sortedIndicesFromMasterPoint[k - 1];
                                        int kSortedCurrent = sortedIndicesFromMasterPoint[k];

                                        int selectedPointIndexPrevious = selectedPointIndices[kSortedPrevious];
                                        int selectedPointIndexCurrent = selectedPointIndices[kSortedCurrent];

                                        constraintEdges.push_back(ConstraintEdge{
                                            .p = selectedPointIndexPrevious,
                                            .q = selectedPointIndexCurrent});

                                        edgeConnections[selectedPointIndexPrevious].push_back(selectedPointIndexCurrent);
                                        edgeConnections[selectedPointIndexCurrent].push_back(selectedPointIndexPrevious);

                                        affectedSegments.push_back(false);

                                        affectedSegments[i] = true;
                                        affectedSegments[neighbour] = true;
                                        anyEdgesAdded = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            if (anyEdgesAdded)
            {
                int iNew = 0;
                for (int i = 0; i < constraintEdges.size(); i++)
                {
                    if (!affectedSegments[i])
                    {
                        constraintEdges[iNew] = constraintEdges[i];
                        iNew++;
                    }
                }

                constraintEdges.resize(iNew);
            }
        }

        DuplicateLineSegmentsTest(constraintEdges);
        int duplicateCount = DuplicateUtils::FindDuplicatesCountKdTree(newPoints, epsilon);
        if (duplicateCount > 0)
        {
            Debug::log("duplicateCount: " + to_string(duplicateCount));
        }
    }

    void NavMesh::DuplicateLineSegmentsTest(vector<ConstraintEdge> &constraintEdges)
    {
        for (int i = 0; i < constraintEdges.size(); i++)
        {
            int p = constraintEdges[i].p;
            int q = constraintEdges[i].q;
            for (int j = 0; j < constraintEdges.size(); j++)
            {
                if (i != j)
                {
                    if ((p == constraintEdges[j].p && q == constraintEdges[j].q) || (p == constraintEdges[j].q && q == constraintEdges[j].p))
                    {
                        Debug::log("DuplicateLineSegmentsTest: " + to_string(i) + " " + to_string(j));
                    }
                }
            }
        }
    }

    // bool NavMesh::IsSegmentAdded(int p, int q, vector<ConstraintEdge> &constraintEdges)
    // {
    //     for (int i = 0; i < constraintEdges.size(); i++)
    //     {
    //         if ((p == constraintEdges[i].p && q == constraintEdges[i].q) || (p == constraintEdges[i].q && q == constraintEdges[i].p))
    //         {
    //             return true;
    //         }
    //     }
    //     return false;
    // }

    void NavMesh::AddObstacle(vector<Obstacle> &obstacles, int obstacleIndex)
    {
        Obstacle obstacle = obstacles[obstacleIndex];
        vector<Float2> obstacleCorners = obstacle.obstacleCorners;

        obstacle.pointsIndexStart = allPoints.size();
        obstacle.pointsCount = 0;

        for (int i = 0; i < obstacleCorners.size(); i++)
        {
            allPoints.push_back(obstacleCorners[i]);
            obstacleWalkablityIndices.push_back(obstacleIndex);
            isObstacleCornerIntersectingWithWorldBounds.push_back(obstacle.isCornerIntersectingWithWorldBounds[i]);
            obstacle.pointsCount++;

            int nextCornerIndex = i + 1;
            if (nextCornerIndex >= obstacleCorners.size())
            {
                nextCornerIndex = 0;
            }

            int nSplits = obstacle.nSplits[i];

            for (int j = 0; j < nSplits; j++)
            {
                float p1 = 1.0f * (j + 1) / (nSplits + 1);
                float p2 = 1.0f - p1;

                allPoints.push_back(obstacleCorners[i] * p2 + obstacleCorners[nextCornerIndex] * p1);
                obstacleWalkablityIndices.push_back(obstacleIndex);
                isObstacleCornerIntersectingWithWorldBounds.push_back(
                    obstacle.isCornerIntersectingWithWorldBounds[i] &&
                    obstacle.isCornerIntersectingWithWorldBounds[nextCornerIndex]);

                obstacle.pointsCount++;
            }
        }

        obstacles[obstacleIndex] = obstacle;
    }

    void NavMesh::ResolveObstacleHullEdges(vector<Obstacle> &obstacles)
    {
        obstacleHullEdges.clear();
        obstacleHullEdgesByObstacles.resize(obstacles.size());
        obstacleHullEdgeObstacleIndices.clear();

        for (int i = 0; i < obstacles.size(); i++)
        {
            obstacleHullEdgesByObstacles[i].clear();
        }

        for (int i = 0; i < allEdges.size(); i++)
        {
            int e = allEdges[i].index;
            int opposite = delaunator.halfedges[e];
            if (opposite >= 0)
            {
                int triangle = Delaunator::TriangleOfEdge(e);
                int nextTriangle = Delaunator::TriangleOfEdge(opposite);

                if (trianglesWalkability[triangle] != -1 && trianglesWalkability[nextTriangle] == -1)
                {
                    int obstacleIndex = trianglesWalkability[triangle];

                    obstacleHullEdges.push_back(i);
                    obstacleHullEdgesByObstacles[obstacleIndex].push_back(i);
                    obstacleHullEdgeObstacleIndices.push_back(obstacleIndex);
                }
                else if (trianglesWalkability[triangle] == -1 && trianglesWalkability[nextTriangle] != -1)
                {
                    int obstacleIndex = trianglesWalkability[nextTriangle];

                    obstacleHullEdges.push_back(i);
                    obstacleHullEdgesByObstacles[obstacleIndex].push_back(i);
                    obstacleHullEdgeObstacleIndices.push_back(obstacleIndex);
                }
            }
        }
    }

    int NavMesh::FindTriangleForPoint(Float2 &point)
    {
        return allTriangulationGridSearch.FindTriangleForPoint(point, delaunator, allPoints);
    }

    int NavMesh::FindWalkableTriangleForPoint(Float2 &point)
    {
        return walkableTriangulationGridSearch.FindTriangleForPoint(point, delaunator, allPoints);
    }

    int NavMesh::FindUnwalkableTriangleForPoint(Float2 &point)
    {
        return unwalkableTriangulationGridSearch.FindTriangleForPoint(point, delaunator, allPoints);
    }

    GetNearestWalkablePositionResult NavMesh::TryMoveToWalkableArea(Float2 &p_position)
    {
        Float2 position = p_position;
        float epsilon = 0.001f;
        bool wasAdjusted = false;
        position = VectorUtils::AdjustForBoundaries(position, worldBounds.minX, worldBounds.maxX, worldBounds.minY, worldBounds.maxY, epsilon, wasAdjusted);

        GetNearestWalkablePositionResult nearestWalkablePositionResult = GetNearestWalkablePosition(position, epsilon);
        if (nearestWalkablePositionResult.wasMoved)
        {
            Float2 randomInsideUnitCircle = VectorUtils::random_inside_unit_circle(random);
            nearestWalkablePositionResult.position += randomInsideUnitCircle * epsilon * 0.5f;
        }

        if (wasAdjusted)
        {
            nearestWalkablePositionResult.wasMoved = true;
        }

        return nearestWalkablePositionResult;
    }

    GetNearestWalkablePositionResult NavMesh::GetNearestWalkablePosition(Float2 &position, float epsilon)
    {
        int triangle = unwalkableTriangulationGridSearch.FindTriangleForPoint(position, delaunator, allPoints);

        if (triangle != -1)
        {
            int unwalkableTriangleIndex = allToUnwalkableTriangleIndices[triangle];
            if (unwalkableTriangleIndex != -1)
            {
                int obstacleIndex = unwalkableTrianglesObstacleIndices[unwalkableTriangleIndex];

                int nearestHullEdge = -1;
                float nearestHullEdgeDistanceSqr = MathUtils::FLOAT_MAX_VALUE;
                Float2 nearestHullEdgePoint = position;

                if (obstacleIndex != -1)
                {
                    vector<int> obstacleIndicesToConsider;
                    int obstacleIndicesToConsiderCount = obstacleIntersections[obstacleIndex].size() + 1;
                    obstacleIndicesToConsider.resize(obstacleIndicesToConsiderCount);
                    obstacleIndicesToConsider[0] = obstacleIndex;

                    for (int i = 0; i < obstacleIntersections[obstacleIndex].size(); i++)
                    {
                        obstacleIndicesToConsider[i + 1] = obstacleIntersections[obstacleIndex][i];
                    }

                    for (int i = 0; i < obstacleIndicesToConsiderCount; i++)
                    {
                        int nextObstacleIndex = obstacleIndicesToConsider[i];

                        for (int j = 0; j < obstacleHullEdgesByObstacles[nextObstacleIndex].size(); j++)
                        {
                            int edgeIndex = obstacleHullEdgesByObstacles[nextObstacleIndex][j];

                            if (edgesWalkability[allEdges[edgeIndex].index])
                            {
                                int p = allEdges[edgeIndex].p;
                                int q = allEdges[edgeIndex].q;

                                Float2 nearestEdgePoint = VectorUtils::FindNearestPointOnLineSegment(allPoints[p], allPoints[q], position);
                                float sqrDistance = (nearestEdgePoint - position).length_squared();

                                if (sqrDistance < nearestHullEdgeDistanceSqr)
                                {
                                    nearestHullEdge = edgeIndex;
                                    nearestHullEdgeDistanceSqr = sqrDistance;
                                    nearestHullEdgePoint = nearestEdgePoint;
                                }
                            }
                        }
                    }
                }

                if (nearestHullEdge != -1)
                {
                    Float2 relativeDirection = nearestHullEdgePoint - position;
                    if (relativeDirection.length_squared() > 0.0f)
                    {
                        nearestHullEdgePoint += relativeDirection.normalized() * epsilon;
                    }
                    else
                    {
                        Float2 centroid = allTriangleCentroids[triangle];
                        Float2 centroidDirection = (nearestHullEdgePoint - centroid).normalized();

                        int p = allEdges[nearestHullEdge].p;
                        int q = allEdges[nearestHullEdge].q;

                        Float2 diff_p_q = allPoints[p] - allPoints[q];

                        Float2 direction = VectorUtils::PerpendicularCounterClockwise(diff_p_q).normalized();
                        if (direction.dot(centroidDirection) < 0.0f)
                        {
                            direction = -direction;
                        }

                        nearestHullEdgePoint += direction * epsilon;
                    }
                }

                return GetNearestWalkablePositionResult{
                    .wasMoved = nearestHullEdge != -1,
                    .position = nearestHullEdgePoint};
            }
        }

        return GetNearestWalkablePositionResult{
            .wasMoved = false,
            .position = position};
    }

    Float2 NavMesh::FindNearestObstacleHullEdgePointToTarget(int lowestHCostNode, Float2 &lowestHCostNodePosition, Float2 &target)
    {
        vector<int> edgesAroundPoint = edgesAroundPointsMap[lowestHCostNode];

        float smallestFinalDistanceSqr = (target - lowestHCostNodePosition).length_squared();
        Float2 finalPosition = lowestHCostNodePosition;

        for (int i = 0; i < edgesAroundPoint.size(); i++)
        {
            int edgeIndex = edgesAroundPoint[i];
            int hafEdgeIndex = allEdges[edgeIndex].index;

            if (hullEdgeTriangulationEdgeToObstacleIndices[hafEdgeIndex] != -1 && edgesWalkability[hafEdgeIndex])
            {
                int p = allEdges[edgeIndex].p;
                int q = allEdges[edgeIndex].q;
                Float2 nearestEdgePoint = VectorUtils::FindNearestPointOnLineSegment(allPoints[p], allPoints[q], target);

                float smallestDistanceSqr = (target - nearestEdgePoint).length_squared();

                if (smallestDistanceSqr < smallestFinalDistanceSqr)
                {
                    smallestFinalDistanceSqr = smallestDistanceSqr;
                    finalPosition = nearestEdgePoint;
                }
            }
        }

        return finalPosition;
    }

    bool NavMesh::CanPointsBeReachedInStraightLine(Float2 &a, Float2 &b)
    {
        Float2 direction = (b - a).normalized() * 0.001f;
        Float2 a1 = a + direction;

        direction = (a - b).normalized() * 0.001f;
        Float2 b1 = b + direction;

        vector<int> currentlyVisitedTriangles;
        int triangleToVisit = FindTriangleForPoint(a1);

        Float2 lastIntersection = a1;

        while (triangleToVisit != -1)
        {
            int triangle = triangleToVisit;
            visitedTriangles[triangle] = true;
            currentlyVisitedTriangles.push_back(triangle);
            triangleToVisit = -1;

            if (trianglesWalkability[triangle] != -1)
            {
                for (int i = 0; i < currentlyVisitedTriangles.size(); i++)
                {
                    visitedTriangles[currentlyVisitedTriangles[i]] = false;
                }
                return false;
            }

            float shortestDistanceSqr = (lastIntersection - b1).length_squared();

            for (int i = 0; i < 3; i++)
            {
                int e = 3 * triangle + i;
                int opposite = delaunator.halfedges[e];

                if (opposite >= 0)
                {
                    int nextTriangle = Delaunator::TriangleOfEdge(opposite);
                    if (!visitedTriangles[nextTriangle])
                    {
                        int p = delaunator.triangles[e];
                        int q = delaunator.triangles[Delaunator::NextHalfedge(e)];

                        LineSegmentsIntersectionResult intersectionResult = VectorUtils::LineSegmentsIntersection(
                            b1,
                            a1,
                            allPoints[p],
                            allPoints[q]);

                        if (intersectionResult.intersects)
                        {
                            float distanceSqr = (intersectionResult.intersection - b1).length_squared();

                            if (distanceSqr < shortestDistanceSqr)
                            {
                                lastIntersection = intersectionResult.intersection;
                                triangleToVisit = nextTriangle;
                                shortestDistanceSqr = distanceSqr;
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < currentlyVisitedTriangles.size(); i++)
        {
            visitedTriangles[currentlyVisitedTriangles[i]] = false;
        }
        return true;
    }
}
