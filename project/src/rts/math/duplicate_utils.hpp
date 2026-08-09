#ifndef DUPLICATE_UTILS_HPP
#define DUPLICATE_UTILS_HPP

#include "rts/math/kd_tree_2d.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct DuplicateUtils
    {
        static int FindDuplicatesCount(vector<Float2> &points, float epsilon);
        static int FindDuplicatesCountKdTree(vector<Float2> &points, float epsilon);
        static void RemoveDuplicates(vector<Float2> &points, float epsilon);
    };
}

#endif
