#ifndef RTS_CONSTRAINAUTOR_TESTS_HPP
#define RTS_CONSTRAINAUTOR_TESTS_HPP

#include <vector>
#include "rts/navigation/delaunator/delaunator.hpp"
#include "rts/navigation/constrainautor/constraint_edge.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct ConstrainautorTests
    {
        static void edges_constrained_test(const string &key, Delaunator &delaunator, vector<ConstraintEdge> &constraint_edges);;
    };
}

#endif
