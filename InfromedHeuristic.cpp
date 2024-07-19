#include "Coordinate.h"
#include "Graph.h"
#include "InformedHeuristic.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered/unordered_map.hpp>

InformedHeuristic::InformedHeuristic(const Graph& g, const Agents& as, const EdgeSet& maybe_blocked_edges)
{
    for(const auto& a: as)
    {
        Dijkstra(g, a.goal);
    }
        
    for(const auto& e: maybe_blocked_edges)
    {
        Dijkstra(g, e.source);
    }
}

void InformedHeuristic::Dijkstra(const Graph& g, const Coordinate& goal)
{
    boost::heap::fibonacci_heap<Node, boost::heap::compare<Node::NodeComparator>> q;
    boost::unordered::unordered_map<Coordinate, float, Coordinate::Hasher, Coordinate::Equal> g_cost;

    q.push({goal, 0});
    g_cost[goal] = 0;
    dist[{goal, goal}] = 0;

    while(!q.empty())
    {
        const auto [p, p_cost] = q.top();
        q.pop();

        for(const auto& s: g.SuccessorsOf(p))
        {
            if(g_cost.find(s) == g_cost.end() || g_cost.at(s) > p_cost + g.WeightOf({p, s}))
            {
                auto s_cost = p_cost + g.WeightOf({p, s});;
                g_cost[s] = s_cost;
                q.push({s, s_cost});
                dist[{goal, s}] = s_cost;
            }
        }
    }
}

float InformedHeuristic::operator() (const Coordinate& c, const Coordinate& goal) const noexcept
{
    return dist.contains({goal, c}) ? dist.at({goal, c}) : INF;
}

std::string InformedHeuristic::ToString(void) const
{
    std::stringstream ss;
    int i = 1;

    for(const auto& [cp, d]: dist)
    {
        ss << i << ")\t" << "dist(" << cp.c1 << ", " << cp.c2 << ") = " << d << '\n';
        i += 1;
    }

    return ss.str();
}

bool InformedHeuristic::CoordinatePair::CoordinatePairEqual::operator() (const CoordinatePair& cp1, const CoordinatePair& cp2) const noexcept
{
    return cp1.c1 == cp2.c1 && cp1.c2 == cp2.c2;
}

size_t InformedHeuristic::CoordinatePair::CoordinatePairHasher::operator() (const CoordinatePair& cp) const noexcept
{
    Coordinate::Hasher coordinate_hasher;
    std::size_t seed = 0;

    boost::hash_combine(seed, (coordinate_hasher(cp.c1)));
    boost::hash_combine(seed, (coordinate_hasher(cp.c2)));

    return seed;
}