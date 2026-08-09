#ifndef HEAP_HPP
#define HEAP_HPP

#include "rts/math/float2.hpp"
#include <vector>
#include "pathfinding_node.hpp"

using namespace std;

namespace NavigationSystemCode
{
    struct Heap
    {
        vector<int> items;
        int count;

        Heap();
        void Add(int item, vector<PathfindingNode> &pathfindingNodes);
        void Clear(vector<PathfindingNode> &pathfindingNodes);
        int RemoveFirst(vector<PathfindingNode> &pathfindingNodes);
        void UpdateItem(int item, vector<PathfindingNode> &pathfindingNodes);
        void SortDown(int item, vector<PathfindingNode> &pathfindingNodes);
        void SortUp(int item, vector<PathfindingNode> &pathfindingNodes);
        void Swap(int itemA, int itemB, vector<PathfindingNode> &pathfindingNodes);
        int CompareTo(int nodeA, int nodeB, vector<PathfindingNode> &pathfindingNodes);
        int InverseCompareTo(float a, float b);
        float FCost(int node, vector<PathfindingNode> &pathfindingNodes);
    };
}

#endif
