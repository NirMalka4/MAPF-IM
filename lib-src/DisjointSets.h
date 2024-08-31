#pragma once

#include "Types.h"
#include <vector>
#include <iostream>

class DisjointSets
{
public:
    using Sets = std::vector<std::vector<int>>;

    DisjointSets(int n);

    int FindSet(int x);
    void MakeSet(int x);
    void Union(int x, int y);
    void Union(int x, const std::vector<int>& ys);
    void Union(const std::vector<int>& ys);
    void Union(const AgentsIndicesSet& s);
    bool IsDisjoint(int x, int y);
    Sets GetDisjointSets(void);
    friend std::ostream& operator << (std::ostream&, DisjointSets&);

protected:
    std::vector<int> parent; // parent[i] := representative of the set contains i
    std::vector<int> rank; // rank[i] := depth of tree represents the set contains i
};