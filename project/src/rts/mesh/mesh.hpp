#ifndef MESH_HPP
#define MESH_HPP

#include "rts/math/float2.hpp"
#include "rts/math/float3.hpp"
#include <vector>

using namespace std;

namespace NavigationSystemCode
{
    struct Mesh
    {
        vector<Float3> vertices;
        vector<Float2> uvs;
        vector<int> triangles;

        void clear();
    };
}

#endif
