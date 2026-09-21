#ifndef RTS_DEBUG_NAV_MESH_HPP
#define RTS_DEBUG_NAV_MESH_HPP

#include "nav_mesh.hpp"
#include "rts/file/file.hpp"

namespace NavigationSystemCode
{
    struct DebugNavMesh
    {
        bool run_edges_constrained_test;

        void ready();
        void Create(
            NavMesh &nav_mesh,
            vector<Obstacle> &obstacles,
            Aabb &bounds,
            const string &reason);
    };
}

#endif
