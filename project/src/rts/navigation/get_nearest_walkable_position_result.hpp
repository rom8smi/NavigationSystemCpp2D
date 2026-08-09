#ifndef GET_NEAREST_WALKABLE_POSITION_RESULT_HPP
#define GET_NEAREST_WALKABLE_POSITION_RESULT_HPP

#include "rts/math/float2.hpp"

namespace NavigationSystemCode
{
    struct GetNearestWalkablePositionResult
    {
        bool wasMoved;
        Float2 position;
    };
}

#endif
