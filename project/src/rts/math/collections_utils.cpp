#include "collections_utils.hpp"

namespace NavigationSystemCode
{
    bool CollectionsUtils::contains(vector<int> &data, int value)
    {
        for (int i = 0; i < data.size(); i++)
        {
            if (value == data[i])
            {
                return true;
            }
        }
        return false;
    }
}
