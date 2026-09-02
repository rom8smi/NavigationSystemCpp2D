#ifndef NAVIGATION_FILE_EXPORT_HPP
#define NAVIGATION_FILE_EXPORT_HPP

#include "aabb.hpp"
#include "obstacle.hpp"
#include "rts/math/float2.hpp"
#include <string>
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct NavigationFileExport
    {
        static void save(const vector<Obstacle> &obstacles, Aabb &bounds, Aabb &target_bounds, const string &filename = "navigation_mesh_export_data.txt");
        static void deep_copy_obstacles(const vector<Obstacle> &original, vector<Obstacle> &copy);
        static float remap(float x, float boundsMin, float scale, float targetBoundsMin);
        static void save(const vector<Obstacle> &obstacles, Aabb &bounds, const string &filename = "navigation_mesh_export_data.txt");
        static void load(vector<Obstacle> &obstacles, Aabb &bounds, const string &filename = "navigation_mesh_export_data.txt");
    };
}

#endif
