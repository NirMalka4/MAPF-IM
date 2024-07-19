#pragma once

#include "Graph.h"
#include "Agent.h"
#include "Types.h"
#include "Utils.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <vector>

class Astar
{
public:
    Path Plan(const Graph& g, const Agent& a, const HeuristicFunction& h = Heuristic::ManhattanDistance);

protected:
    struct Vertex;
    struct VertexComparator{ bool operator() (const Vertex* v1, const Vertex* v2) const noexcept;};

    using MinFibHeap = boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<VertexComparator>>;
    using Successors = std::vector<Vertex*>;
    using LookupTable = boost::unordered::unordered_map<Coordinate, Vertex*, Coordinate::Hasher, Coordinate::Equal>;
    
    struct Vertex
    {
        const Vertex* parent = nullptr;
        Coordinate c{};
        float g = INF;
        float h = 0;
        bool in_open = false;
        MinFibHeap::handle_type handler;
        
        bool operator < (const Vertex& v) const noexcept;
        bool operator == (const Vertex& v) const noexcept;
        inline float fVal(void) const noexcept {return g + h;}
    };

    LookupTable table;

    Vertex* Init(const Agent& a, const HeuristicFunction& h);
    Vertex* Generate(Vertex* parent, const Coordinate& successor_coordinate, const Agent& a, const HeuristicFunction& h);
    Successors Expand(Vertex* current, const Graph& g, const Agent& a, const HeuristicFunction& h);
    void Clear(void);
    bool IsGenerated(const Coordinate& c) const;
    float gCost(const Vertex* parent, const Vertex* successor, const Graph& g);
    Path ReconstructPath(const Vertex* goal) const;
};