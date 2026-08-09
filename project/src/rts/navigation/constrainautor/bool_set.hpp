#ifndef BOOL_SET_HPP
#define BOOL_SET_HPP

#include <vector>

using namespace std;

namespace NavigationSystemCode
{
	struct BoolSet
	{
        vector<bool> bs;

        void Create(int len);
        void Clear();
        void Add(int idx);
        void Remove(int idx);
        bool Set(int idx, bool val);
        bool Has(int idx);
    };
}

#endif
