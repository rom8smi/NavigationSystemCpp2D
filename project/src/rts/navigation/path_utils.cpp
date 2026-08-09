#include "path_utils.hpp"

namespace NavigationSystemCode
{
    float PathUtils::CalculateTotalPathLength(vector<Float2> &waypoints)
    {
        return CalculatePathLength(waypoints, 0);
    }

    float PathUtils::CalculatePathLength(vector<Float2> &waypoints, int startIndex)
    {
        float pathLength = 0.0f;

        for (int i = startIndex; i < waypoints.size() - 1; i++)
        {
            pathLength += (waypoints[i + 1] - waypoints[i]).length();
        }

        return pathLength;
    }
}
