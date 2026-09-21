#include "constrainautor_tests.hpp"
#include "rts/debug/debug.hpp"

namespace NavigationSystemCode
{
    void ConstrainautorTests::edges_constrained_test(const string &key, Delaunator &delaunator, vector<ConstraintEdge> &constraint_edges)
    {
        int points_count = delaunator.coords.size() / 2;
        vector<vector<int>> points_connections_map;
        points_connections_map.resize(points_count);

        for (int e = 0; e < delaunator.triangles.size(); e++)
        {
            int next = Delaunator::NextHalfedge(e);
            int p = delaunator.triangles[e];
            int q = delaunator.triangles[next];

            points_connections_map[p].push_back(q);
            points_connections_map[q].push_back(p);
        }

        int count1 = 0;
        for (int i = 0; i < constraint_edges.size(); i++)
        {
            int p = constraint_edges[i].p;
            int q = constraint_edges[i].q;

            bool found = false;
            for (int j = 0; j < points_connections_map[p].size(); j++)
            {
                if (points_connections_map[p][j] == q)
                {
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                count1++;
            }
        }

        if (count1 > 0)
        {
            Debug::log(key + " " + to_string(count1));
        }
    }
}
