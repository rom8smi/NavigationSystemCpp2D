#ifndef NAV_MESH_HPP
#define NAV_MESH_HPP

#include "rts/math/float2.hpp"
#include "rts/math/random.hpp"
#include <vector>
#include "rts/navigation/delaunator/delaunator.hpp"
#include "rts/navigation/constrainautor/constrainautor.hpp"
#include "aabb.hpp"
#include "obstacle.hpp"
#include "get_nearest_walkable_position_result.hpp"
#include "triangulation_grid_search.hpp"
#include "rts/math/kd_tree_2d.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct NavMesh
    {
        Delaunator delaunator;
        Constrainautor constrainautor;

        vector<Float2> allPoints;
        vector<DelaunatorTriangle> allTriangles;
        vector<Aabb> allTriangleBounds;
        vector<Float2> allTriangleCentroids;
        KdTree2D allTriangleCentroidsKdTree;
        vector<DelaunatorEdge> allEdges;

        vector<vector<int>> edgesAroundPointsMap;

        vector<int> unwalkableTriangles;
        vector<int> unwalkableTrianglesObstacleIndices;
        vector<int> walkableTriangles;

        vector<int> trianglesWalkability;
        vector<int> allToUnwalkableTriangleIndices;
        vector<vector<int>> obstacleIntersections;
        vector<int> obstacleWalkablityIndices;
        vector<bool> isObstacleCornerIntersectingWithWorldBounds;
        vector<int> obstacleHullEdges;
        vector<vector<int>> obstacleHullEdgesByObstacles;
        vector<int> obstacleHullEdgeObstacleIndices;
        vector<int> hullEdgeTriangulationEdgeToObstacleIndices;
        vector<bool> edgesWalkability;
        float smallestHullEdgeSize;
        Aabb worldBounds;
        int totalNumberOfWorldBoundCorners;
        Random random;
        TriangulationGridSearch allTriangulationGridSearch;
        TriangulationGridSearch walkableTriangulationGridSearch;
        TriangulationGridSearch unwalkableTriangulationGridSearch;
        vector<bool> visitedTriangles;

        NavMesh();
        void Create(vector<Obstacle> &obstacles, Aabb &bounds);
        vector<Float2> GetSubdividedWorldBoundEdges(vector<Obstacle> &obstacles);
        void SubdivideAndBuildConstrainedTriangulation(vector<Obstacle> &obstacles);
        void FindWalkableTriangles(vector<Obstacle> &obstacles);
        void FindWalkableEdges(vector<Obstacle> &obstacles);
        void CalculateSizeOfSmallestHullEdge();
        void CalculateTriangleCentroids();
        void CreateTriangulationSearch();
        void CalculateEdgesAroundPointsMap();
        void CalulateHullEdgeTriangulationEdgeToObstacleIndices();
        void CreateVisitedTriangles();
        vector<Float2> GetDefaultWorldBounds();
        void AddObstacles(vector<Obstacle> &obstacles);
        void AddObstaclesWithConstraints(
            vector<Obstacle> &obstacles,
            vector<ConstraintEdge> &constraintEdges,
            vector<Float2> &newPoints,
            vector<int> &newObstacleWalkablityIndices,
            vector<vector<int>> &newObstacleIntersections,
            vector<bool> &newIsObstacleCornerIntersectingWithWorldBounds);
        void SplitCollinearConstrainedEdges(
            vector<ConstraintEdge> &constraintEdges,
            vector<Float2> &newPoints, float epsilon);
        void DuplicateLineSegmentsTest(vector<ConstraintEdge> &constraintEdges);
        // bool IsSegmentAdded(int p, int q, vector<ConstraintEdge> &constraintEdges);
        void AddObstacle(vector<Obstacle> &obstacles,
                         int obstacleIndex);
        void ResolveObstacleHullEdges(vector<Obstacle> &obstacles);
        int FindTriangleForPoint(Float2 &point);
        int FindWalkableTriangleForPoint(Float2 &point);
        int FindUnwalkableTriangleForPoint(Float2 &point);
        GetNearestWalkablePositionResult TryMoveToWalkableArea(Float2 &position);
        GetNearestWalkablePositionResult GetNearestWalkablePosition(Float2 &position, float epsilon);
        Float2 FindNearestObstacleHullEdgePointToTarget(int lowestHCostNode, Float2 &lowestHCostNodePosition, Float2 &target);
        bool CanPointsBeReachedInStraightLine(Float2 &a, Float2 &b);
    };
}

#endif
