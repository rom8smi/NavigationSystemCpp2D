#include "debug_nav_mesh.hpp"
#include "rts/math/math_utils.hpp"
#include <chrono>
#include "rts/navigation/navigation_tests/constrainautor_tests.hpp"

namespace NavigationSystemCode
{
    void DebugNavMesh::ready()
    {
        run_edges_constrained_test = true;

        File fileCreate;
        fileCreate.open("DebugNavMeshCreate.dat");
        fileCreate.write("# allPointsCount GetSubdividedWorldBoundEdges PointsClear AddObstaclesWithConstraints Delaunator Constrainautor allTriangleBounds CalculateTriangleCentroids CalculateEdgesAroundPointsMap FindWalkableEdges FindWalkableTriangles ResolveObstacleHullEdges CalulateHullEdgeTriangulationEdgeToObstacleIndices CalculateSizeOfSmallestHullEdge CreateTriangulationSearch CreateVisitedTriangles Total\n");
        fileCreate.close();
    }

    void DebugNavMesh::Create(
        NavMesh &nav_mesh,
        vector<Obstacle> &obstacles,
        Aabb &bounds,
        const string &reason)
    {
        const auto t_initial = std::chrono::high_resolution_clock::now();
        nav_mesh.worldBounds = bounds;
        vector<Float2> world_bound_corners = nav_mesh.GetSubdividedWorldBoundEdges(obstacles);
        nav_mesh.totalNumberOfWorldBoundCorners = world_bound_corners.size();
        const auto t_get_subdivided_world_bound_edges = std::chrono::high_resolution_clock::now();

        nav_mesh.allPoints.clear();
        nav_mesh.obstacleWalkablityIndices.clear();
        nav_mesh.isObstacleCornerIntersectingWithWorldBounds.clear();
        for (int i = 0; i < world_bound_corners.size(); i++)
        {
            nav_mesh.allPoints.push_back(world_bound_corners[i]);
            nav_mesh.obstacleWalkablityIndices.push_back(-1);
            nav_mesh.isObstacleCornerIntersectingWithWorldBounds.push_back(false);
        }
        const auto t_points_clear = std::chrono::high_resolution_clock::now();

        vector<ConstraintEdge> constraint_edges;
        if (obstacles.size() > 0)
        {
            nav_mesh.AddObstaclesWithConstraints(
                obstacles,
                constraint_edges,
                nav_mesh.allPoints,
                nav_mesh.obstacleWalkablityIndices,
                nav_mesh.obstacleIntersections,
                nav_mesh.isObstacleCornerIntersectingWithWorldBounds);
        }
        const auto t_add_obstacles = std::chrono::high_resolution_clock::now();

        nav_mesh.delaunator.Create(nav_mesh.allPoints);
        nav_mesh.delaunator.ClearTemporaryLists();
        const auto t_delaunator = std::chrono::high_resolution_clock::now();
        nav_mesh.constrainautor.Create(nav_mesh.delaunator, constraint_edges, reason);
        nav_mesh.constrainautor.ClearTemporaryLists();
        const auto t_constrainautor = std::chrono::high_resolution_clock::now();

        if (run_edges_constrained_test)
        {
            ConstrainautorTests::edges_constrained_test("edges_constrained_test", nav_mesh.delaunator, constraint_edges);
        }

        nav_mesh.allTriangles = nav_mesh.delaunator.GetTriangles();
        nav_mesh.allEdges = nav_mesh.delaunator.GetEdges();
        nav_mesh.allTriangleBounds = vector<Aabb>();

        for (int i = 0; i < nav_mesh.allTriangles.size(); i++)
        {
            Aabb aabb = Aabb{
                .minX = MathUtils::FLOAT_MAX_VALUE,
                .maxX = MathUtils::FLOAT_MIN_VALUE,
                .minY = MathUtils::FLOAT_MAX_VALUE,
                .maxY = MathUtils::FLOAT_MIN_VALUE,
            };
            vector<int> trianglePoints = nav_mesh.allTriangles[i].points;

            for (int j = 0; j < trianglePoints.size(); j++)
            {
                Float2 point = nav_mesh.allPoints[trianglePoints[j]];
                aabb.minX = MathUtils::min(aabb.minX, point.x);
                aabb.maxX = MathUtils::max(aabb.maxX, point.x);
                aabb.minY = MathUtils::min(aabb.minY, point.y);
                aabb.maxY = MathUtils::max(aabb.maxY, point.y);
            }

            nav_mesh.allTriangleBounds.push_back(aabb);
        }
        const auto t_all_triangle_bounds = std::chrono::high_resolution_clock::now();

        nav_mesh.CalculateTriangleCentroids();
        const auto t_calculate_triangle_centroids = std::chrono::high_resolution_clock::now();
        nav_mesh.CalculateEdgesAroundPointsMap();
        const auto t_calculate_edges_around_points_map = std::chrono::high_resolution_clock::now();
        nav_mesh.FindWalkableEdges(obstacles);
        const auto t_find_walkable_edges = std::chrono::high_resolution_clock::now();
        nav_mesh.FindWalkableTriangles(obstacles);
        const auto t_find_walkable_triangles = std::chrono::high_resolution_clock::now();
        nav_mesh.ResolveObstacleHullEdges(obstacles);
        const auto t_resolve_obstacle_hull_edges = std::chrono::high_resolution_clock::now();
        nav_mesh.CalulateHullEdgeTriangulationEdgeToObstacleIndices();
        const auto t_calculate_hull_edge_indices = std::chrono::high_resolution_clock::now();
        nav_mesh.CalculateSizeOfSmallestHullEdge();
        const auto t_calculate_smallest_hull_edge = std::chrono::high_resolution_clock::now();
        nav_mesh.CreateTriangulationSearch();
        const auto t_create_triangulation_search = std::chrono::high_resolution_clock::now();
        nav_mesh.CreateVisitedTriangles();
        const auto t_create_visited_triangles = std::chrono::high_resolution_clock::now();

        const int allPointsCount = nav_mesh.allPoints.size();
        const double dt_GetSubdividedWorldBoundEdges = std::chrono::duration<double, std::milli>(t_get_subdivided_world_bound_edges - t_initial).count();
        const double dt_PointsClear = std::chrono::duration<double, std::milli>(t_points_clear - t_get_subdivided_world_bound_edges).count();
        const double dt_AddObstaclesWithConstraints = std::chrono::duration<double, std::milli>(t_add_obstacles - t_points_clear).count();
        const double dt_Delaunator = std::chrono::duration<double, std::milli>(t_delaunator - t_add_obstacles).count();
        const double dt_Constrainautor = std::chrono::duration<double, std::milli>(t_constrainautor - t_delaunator).count();
        const double dt_allTriangleBounds = std::chrono::duration<double, std::milli>(t_all_triangle_bounds - t_constrainautor).count();
        const double dt_CalculateTriangleCentroids = std::chrono::duration<double, std::milli>(t_calculate_triangle_centroids - t_all_triangle_bounds).count();
        const double dt_CalculateEdgesAroundPointsMap = std::chrono::duration<double, std::milli>(t_calculate_edges_around_points_map - t_calculate_triangle_centroids).count();
        const double dt_FindWalkableEdges = std::chrono::duration<double, std::milli>(t_find_walkable_edges - t_calculate_edges_around_points_map).count();
        const double dt_FindWalkableTriangles = std::chrono::duration<double, std::milli>(t_find_walkable_triangles - t_find_walkable_edges).count();
        const double dt_ResolveObstacleHullEdges = std::chrono::duration<double, std::milli>(t_resolve_obstacle_hull_edges - t_find_walkable_triangles).count();
        const double dt_CalulateHullEdgeTriangulationEdgeToObstacleIndices = std::chrono::duration<double, std::milli>(t_calculate_hull_edge_indices - t_resolve_obstacle_hull_edges).count();
        const double dt_CalculateSizeOfSmallestHullEdge = std::chrono::duration<double, std::milli>(t_calculate_smallest_hull_edge - t_calculate_hull_edge_indices).count();
        const double dt_CreateTriangulationSearch = std::chrono::duration<double, std::milli>(t_create_triangulation_search - t_calculate_smallest_hull_edge).count();
        const double dt_CreateVisitedTriangles = std::chrono::duration<double, std::milli>(t_create_visited_triangles - t_create_triangulation_search).count();
        const double dt_Total = std::chrono::duration<double, std::milli>(t_create_visited_triangles - t_initial).count();

        File fileCreate;
        fileCreate.open_append("DebugNavMeshCreate.dat");
        fileCreate.write(
            to_string(allPointsCount) + " " +
            to_string(dt_GetSubdividedWorldBoundEdges) + " " +
            to_string(dt_PointsClear) + " " +
            to_string(dt_AddObstaclesWithConstraints) + " " +
            to_string(dt_Delaunator) + " " +
            to_string(dt_Constrainautor) + " " +
            to_string(dt_allTriangleBounds) + " " +
            to_string(dt_CalculateTriangleCentroids) + " " +
            to_string(dt_CalculateEdgesAroundPointsMap) + " " +
            to_string(dt_FindWalkableEdges) + " " +
            to_string(dt_FindWalkableTriangles) + " " +
            to_string(dt_ResolveObstacleHullEdges) + " " +
            to_string(dt_CalulateHullEdgeTriangulationEdgeToObstacleIndices) + " " +
            to_string(dt_CalculateSizeOfSmallestHullEdge) + " " +
            to_string(dt_CreateTriangulationSearch) + " " +
            to_string(dt_CreateVisitedTriangles) + " " +
            to_string(dt_Total) + "\n");
        fileCreate.close();
    }
}
