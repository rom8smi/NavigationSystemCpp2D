#include "duplicate_utils.hpp"

namespace NavigationSystemCode
{
    int DuplicateUtils::FindDuplicatesCount(vector<Float2> &points, float epsilon)
    {
        int count = 0;
        float epsilonSqr = epsilon * epsilon;
        vector<bool> considered;
        int pointsCount = points.size();
        considered.resize(points.size());

        for (int i = 0; i < pointsCount; i++)
        {
            considered[i] = false;
        }

        for (int i = 0; i < pointsCount; i++)
        {
            for (int j = i + 1; j < pointsCount; j++)
            {
                if (!considered[j])
                {
                    float rSqr = (points[i] - points[j]).length_squared();
                    if (rSqr < epsilonSqr)
                    {
                        considered[j] = true;
                        count++;
                    }
                }
            }
        }

        return count;
    }

    int DuplicateUtils::FindDuplicatesCountKdTree(vector<Float2> &points, float epsilon)
    {
        KdTree2D kdTree;
        kdTree.make_from_points(points);

        int count = 0;
        float epsilonSqr = epsilon * epsilon;
        vector<bool> considered;

        int pointsCount = points.size();

        considered.resize(pointsCount);

        for (int i = 0; i < pointsCount; i++)
        {
            considered[i] = false;
        }

        vector<int> neighbours;

        for (int i = 0; i < pointsCount; i++)
        {
            neighbours.clear();
            kdTree.find_nearests_ball(points[i], 2.0f * epsilon, neighbours);

            for (int j = 0; j < neighbours.size(); j++)
            {
                int neighbour = neighbours[j];
                if (neighbour > i && !considered[neighbour])
                {
                    float rSqr = (points[i] - points[neighbour]).length_squared();
                    if (rSqr < epsilonSqr)
                    {
                        considered[neighbour] = true;
                        count++;
                    }
                }
            }
        }

        return count;
    }

    void DuplicateUtils::RemoveDuplicates(vector<Float2> &points, float epsilon)
    {
        KdTree2D kdTree;
        kdTree.make_from_points(points);

        float epsilonSqr = epsilon * epsilon;
        vector<bool> duplicates;

        int pointsCount = points.size();

        duplicates.resize(pointsCount);

        for (int i = 0; i < pointsCount; i++)
        {
            duplicates[i] = false;
        }

        vector<int> neighbours;

        for (int i = 0; i < pointsCount; i++)
        {
            neighbours.clear();
            kdTree.find_nearests_ball(points[i], 2.0f * epsilon, neighbours);

            for (int j = 0; j < neighbours.size(); j++)
            {
                int neighbour = neighbours[j];
                if (neighbour > i && !duplicates[neighbour])
                {
                    float rSqr = (points[i] - points[neighbour]).length_squared();
                    if (rSqr < epsilonSqr)
                    {
                        duplicates[neighbour] = true;
                    }
                }
            }
        }

        int iNew = 0;
        for (int i = 0; i < pointsCount; i++)
        {
            if (!duplicates[i])
            {
                points[iNew] = points[i];
                iNew++;
            }
        }
        points.resize(iNew);
    }
}
