#include "bool_set.hpp"
#include "rts/debug/debug.hpp"

namespace NavigationSystemCode
{
    void BoolSet::Create(int len)
    {
        bs.resize(len);

        for (int i = 0; i < len; i++)
        {
            Debug::index_assert(i, bs.size(), "ccc1");
            bs[i] = false;
        }
    }

    void BoolSet::Clear()
    {
        bs.clear();
    }

    void BoolSet::Add(int idx)
    {
        Debug::index_assert(idx, bs.size(), "ccc2");
        bs[idx] = true;
    }

    void BoolSet::Remove(int idx)
    {
        Debug::index_assert(idx, bs.size(), "ccc3");
        bs[idx] = false;
    }

    bool BoolSet::Set(int idx, bool val)
    {
        Debug::index_assert(idx, bs.size(), "ccc4");
        bs[idx] = val;
        return val;
    }

    bool BoolSet::Has(int idx)
    {
        Debug::index_assert(idx, bs.size(), "ccc5");
        return bs[idx];
    }
}
