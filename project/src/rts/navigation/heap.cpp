#include "heap.hpp"

namespace NavigationSystemCode
{
    Heap::Heap()
    {
        count = 0;
    }

    void Heap::Add(int item, vector<PathfindingNode> &pathfindingNodes)
    {
        pathfindingNodes[item].heapIndex = count;

        if (items.size() == count)
        {
            items.push_back(item);
        }
        else
        {
            items[count] = item;
        }

        SortUp(item, pathfindingNodes);
        count++;
    }

    void Heap::Clear(vector<PathfindingNode> &pathfindingNodes)
    {
        if (count > 0)
        {
            for (int i = 0; i < count; i++)
            {
                int nodeIndex = items[i];
                pathfindingNodes[nodeIndex].isInClosedSet = false;
                pathfindingNodes[nodeIndex].parent = -1;
                pathfindingNodes[nodeIndex].heapIndex = -1;
            }

            items.clear();
            count = 0;
        }
    }

    int Heap::RemoveFirst(vector<PathfindingNode> &pathfindingNodes)
    {
        int firstItem = items[0];
        count--;
        items[0] = items[count];

        pathfindingNodes[items[0]].heapIndex = 0;

        SortDown(items[0], pathfindingNodes);

        pathfindingNodes[firstItem].heapIndex = -1;

        return firstItem;
    }

    void Heap::UpdateItem(int item, vector<PathfindingNode> &pathfindingNodes)
    {
        SortUp(item, pathfindingNodes);
    }

    void Heap::SortDown(int item, vector<PathfindingNode> &pathfindingNodes)
    {
        while (true)
        {
            int childIndexLeft = pathfindingNodes[item].heapIndex * 2 + 1;
            int childIndexRight = pathfindingNodes[item].heapIndex * 2 + 2;
            int swapIndex = 0;

            if (childIndexLeft < count)
            {
                swapIndex = childIndexLeft;

                if (childIndexRight < count)
                {
                    if (CompareTo(items[childIndexLeft], items[childIndexRight], pathfindingNodes) < 0)
                    {
                        swapIndex = childIndexRight;
                    }
                }

                if (CompareTo(item, items[swapIndex], pathfindingNodes) < 0)
                {
                    Swap(item, items[swapIndex], pathfindingNodes);
                }
                else
                {
                    return;
                }
            }
            else
            {
                return;
            }
        }
    }

    void Heap::SortUp(int item, vector<PathfindingNode> &pathfindingNodes)
    {
        int parentIndex = (pathfindingNodes[item].heapIndex - 1) / 2;

        while (true)
        {
            int parentItem = items[parentIndex];
            if (CompareTo(item, parentItem, pathfindingNodes) > 0)
            {
                Swap(item, parentItem, pathfindingNodes);
            }
            else
            {
                break;
            }

            parentIndex = (pathfindingNodes[item].heapIndex - 1) / 2;
        }
    }

    void Heap::Swap(int itemA, int itemB, vector<PathfindingNode> &pathfindingNodes)
    {
        items[pathfindingNodes[itemA].heapIndex] = itemB;
        items[pathfindingNodes[itemB].heapIndex] = itemA;
        int itemAIndex = pathfindingNodes[itemA].heapIndex;

        pathfindingNodes[itemA].heapIndex = pathfindingNodes[itemB].heapIndex;
        pathfindingNodes[itemB].heapIndex = itemAIndex;
    }

    int Heap::CompareTo(int nodeA, int nodeB, vector<PathfindingNode> &pathfindingNodes)
    {
        int compare = InverseCompareTo(FCost(nodeA, pathfindingNodes), FCost(nodeB, pathfindingNodes));
        if (compare == 0)
        {
            compare = InverseCompareTo(pathfindingNodes[nodeA].hCost, pathfindingNodes[nodeB].hCost);
        }
        return compare;
    }

    int Heap::InverseCompareTo(float a, float b)
    {
        if (a > b)
        {
            return -1;
        }
        if (a < b)
        {
            return 1;
        }
        return 0;
    }

    float Heap::FCost(int node, vector<PathfindingNode> &pathfindingNodes)
    {
        return pathfindingNodes[node].gCost + pathfindingNodes[node].hCost;
    }
}
