#ifndef TESTS_HPP
#define TESTS_HPP

#include <vector>

using namespace std;

namespace NavigationSystemCode
{
	struct HeapSort
	{
		static void Sort(vector<int> &indices, vector<float> &data);
		static void Sort(vector<int> &data);
	};
}

#endif
