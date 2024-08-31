#pragma once
#include "Agent.h"
#include "Coordinate.h"
#include "Graph.h"
#include "Types.h"
#include <boost/unordered/unordered_map.hpp>

class InformedHeuristic
{
public:
    InformedHeuristic() = default;
    InformedHeuristic(const Graph& g, const Agents& as, const EdgeSet& maybe_blocked_edges);

    std::string ToString(void) const;

    float operator() (const Coordinate& c, const Coordinate& goal) const noexcept;
    inline operator bool () const {return !dist.empty();};
    inline friend std::ostream& operator << (std::ostream& out, const InformedHeuristic& ih) {return out << ih.ToString();}

private:
    struct CoordinatePair
    {
        const Coordinate c1, c2;

        inline bool operator != (const CoordinatePair& other) {return c1 != other.c1 || c2 != other.c2;}
        struct CoordinatePairEqual{bool operator () (const CoordinatePair& cp1, const CoordinatePair& cp2) const noexcept;};
        struct CoordinatePairHasher{size_t operator () (const CoordinatePair& cp) const noexcept;};
    };

    struct Node
    {
        Coordinate c;
        float g = INF;

        inline bool operator < (const Node& other) const noexcept {return g < other.g;}
        struct NodeComparator{bool operator () (const Node& n1, const Node& n2) const noexcept {return !(n1 < n2);}};
    };

    void Dijkstra(const Graph& g, const Coordinate& goal);
    
    // run Dijkstra from each agent goal. dist[{goal, v}] = minimum distance-to-go from v to goal (assuming undirected graph)
    boost::unordered::unordered_map<CoordinatePair, float, CoordinatePair::CoordinatePairHasher, CoordinatePair::CoordinatePairEqual> dist;
};