#include "navigation_file_export.hpp"
#include <fstream>
#include <cstdint>
#include <iomanip>
#include <limits>

namespace NavigationSystemCode
{
    void NavigationFileExport::save(const vector<Obstacle> &obstacles, Aabb &bounds, Aabb &target_bounds, const string &filename)
    {
        float srcWidth = bounds.maxX - bounds.minX;
        float srcHeight = bounds.maxY - bounds.minY;
        float dstWidth = target_bounds.maxX - target_bounds.minX;
        float dstHeight = target_bounds.maxY - target_bounds.minY;

        float scaleX = (srcWidth > 0.0f) ? (dstWidth / srcWidth) : 1.0f;
        float scaleY = (srcHeight > 0.0f) ? (dstHeight / srcHeight) : 1.0f;
        float maxScale = std::max(scaleX, scaleY);

        vector<Obstacle> scaled_obstacles;
        deep_copy_obstacles(obstacles, scaled_obstacles);

        for (Obstacle &obs : scaled_obstacles)
        {
            obs.center.x = remap(obs.center.x, bounds.minX, scaleX, target_bounds.minX);
            obs.center.y = remap(obs.center.y, bounds.minY, scaleY, target_bounds.minY);
            obs.largest_corner_distance *= maxScale;

            for (Float2 &corner : obs.obstacleCorners)
            {
                corner.x = remap(corner.x, bounds.minX, scaleX, target_bounds.minX);
                corner.y = remap(corner.y, bounds.minY, scaleY, target_bounds.minY);
            }
        }

        save(scaled_obstacles, target_bounds, filename);
    }

    void NavigationFileExport::deep_copy_obstacles(const vector<Obstacle> &original, vector<Obstacle> &copy)
    {
        int original_count = original.size();
        copy.resize(original_count);

        for (int i = 0; i < original_count; i++)
        {
            copy[i].pointsIndexStart = original[i].pointsIndexStart;
            copy[i].pointsCount = original[i].pointsCount;
            copy[i].center = original[i].center;
            copy[i].largest_corner_distance = original[i].largest_corner_distance;
            copy[i].isWalkable = original[i].isWalkable;

            copy[i].obstacleCorners.resize(original[i].obstacleCorners.size());
            for (int j = 0; j < original[i].obstacleCorners.size(); j++)
            {
                copy[i].obstacleCorners[j] = original[i].obstacleCorners[j];
            }

            copy[i].isCornerIntersectingWithWorldBounds.resize(original[i].isCornerIntersectingWithWorldBounds.size());
            for (int j = 0; j < original[i].isCornerIntersectingWithWorldBounds.size(); j++)
            {
                copy[i].isCornerIntersectingWithWorldBounds[j] = original[i].isCornerIntersectingWithWorldBounds[j];
            }

            copy[i].nSplits.resize(original[i].nSplits.size());
            for (int j = 0; j < original[i].nSplits.size(); j++)
            {
                copy[i].nSplits[j] = original[i].nSplits[j];
            }
        }
    }

    float NavigationFileExport::remap(float x, float boundsMin, float scale, float targetBoundsMin)
    {
        return targetBoundsMin + (x - boundsMin) * scale;
    }

    void NavigationFileExport::save(const vector<Obstacle> &obstacles, Aabb &bounds, const string &filename)
    {
        ofstream out(filename);
        if (!out.is_open())
        {
            return;
        }

        out << std::setprecision(std::numeric_limits<float>::max_digits10);

        out << bounds.minX << " " << bounds.maxX << " "
            << bounds.minY << " " << bounds.maxY << "\n";

        out << obstacles.size() << "\n";

        for (const Obstacle &obs : obstacles)
        {
            out << obs.pointsIndexStart << " "
                << obs.pointsCount << " "
                << obs.center.x << " "
                << obs.center.y << " "
                << obs.largest_corner_distance << " "
                << (obs.isWalkable ? 1 : 0) << "\n";

            out << obs.obstacleCorners.size();
            for (const Float2 &corner : obs.obstacleCorners)
            {
                out << " " << corner.x << " " << corner.y;
            }
            out << "\n";

            out << obs.isCornerIntersectingWithWorldBounds.size();
            for (bool val : obs.isCornerIntersectingWithWorldBounds)
            {
                out << " " << (val ? 1 : 0);
            }
            out << "\n";

            out << obs.nSplits.size();
            for (int split : obs.nSplits)
            {
                out << " " << split;
            }
            out << "\n";
        }

        out.close();
    }

    void NavigationFileExport::load(vector<Obstacle> &obstacles, Aabb &bounds, const string &filename)
    {
        ifstream in(filename);
        if (!in.is_open())
        {
            return;
        }

        obstacles.clear();

        if (!(in >> bounds.minX >> bounds.maxX >> bounds.minY >> bounds.maxY))
        {
            return;
        }

        size_t obstacleCount = 0;
        if (!(in >> obstacleCount))
        {
            return;
        }

        obstacles.resize(obstacleCount);

        for (size_t i = 0; i < obstacleCount; ++i)
        {
            Obstacle &obs = obstacles[i];
            int walkableInt = 0;

            in >> obs.pointsIndexStart >> obs.pointsCount >> obs.center.x >> obs.center.y >> obs.largest_corner_distance >> walkableInt;

            obs.isWalkable = (walkableInt != 0);

            size_t cornersCount = 0;
            in >> cornersCount;
            obs.obstacleCorners.resize(cornersCount);
            for (size_t j = 0; j < cornersCount; ++j)
            {
                in >> obs.obstacleCorners[j].x >> obs.obstacleCorners[j].y;
            }

            size_t boundsIntersectionCount = 0;
            in >> boundsIntersectionCount;
            obs.isCornerIntersectingWithWorldBounds.resize(boundsIntersectionCount);
            for (size_t j = 0; j < boundsIntersectionCount; ++j)
            {
                int isIntersecting = 0;
                in >> isIntersecting;
                obs.isCornerIntersectingWithWorldBounds[j] = (isIntersecting != 0);
            }

            size_t splitsCount = 0;
            in >> splitsCount;
            obs.nSplits.resize(splitsCount);
            for (size_t j = 0; j < splitsCount; ++j)
            {
                in >> obs.nSplits[j];
            }
        }

        in.close();
    }
}
