#include "DisjointSets.h"
#include <iterator>
#include <unordered_map>
#include <utility>
#include <vector>

DisjointSets::DisjointSets(int k): parent(k), rank(k)
{
    for(auto i = 0; i < k; i++)
        MakeSet(i);
}

void DisjointSets::MakeSet(int x)
{
    parent[x] = x;
    rank[x] = 0;
}

void DisjointSets::Union(int x, int y)
{
    x = FindSet(x);
    y = FindSet(y);
    if(rank[x] > rank[y])
        parent[y] = x;
    else
    {
        parent[x] = y;
        if(rank[x] == rank[y])
            rank[y] = rank[y] + 1;
    }
}

void DisjointSets::Union(int x, const std::vector<int>& ys)
{
    for(const auto& y: ys)
        Union(x, y);
}

void DisjointSets::Union(const std::vector<int>& ys)
{
    if(!ys.empty())
    {
        const auto x = ys.front();
        for(auto i = std::next(ys.begin()); i != ys.end(); i++)
        {
            Union(x, *i);
        }
    }
}

void DisjointSets::Union(const AgentsIndicesSet& s)
{
    if(!s.empty())
    {
        const auto x = *s.begin();

        for(auto i = std::next(s.begin()); i != s.end(); i++)
        {
            Union(x, *i);
        }
    }
}

int DisjointSets::FindSet(int x)
{
    if(x != parent[x])
        parent[x] = FindSet(parent[x]);
    return parent[x];
}

bool DisjointSets::IsDisjoint(int x, int y)
{
    return FindSet(x) != FindSet(y);
}

DisjointSets::Sets DisjointSets::GetDisjointSets(void)
{
    Sets ds;
    std::unordered_map<int, std::vector<int>> uniques;

    for(auto i = 0; i < parent.size(); i++)
        uniques[FindSet(i)].push_back(i);
    
    for(auto&& [i, es] : uniques)
        ds.push_back(std::forward<std::vector<int>>(es));

    return ds;
}

std::ostream& operator << (std::ostream& out, DisjointSets& dsu)
{
    auto sets = dsu.GetDisjointSets();
    out << '{';
    const auto m = sets.size();
    for(auto j = 0; j < m; j++)
    {
        const auto& s = sets[j];
        const auto n = s.size();
        out << '{';
        
        for(auto i = 0; i < n-1; i++)
            out << s[i] << ", ";

        out << std::to_string(s[n-1]) << "}";
        if(j < m - 1)
            out << ',';
    }

    out << '}';
    return out;
}