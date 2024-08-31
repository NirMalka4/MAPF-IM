#pragma once

#include "Graph.h"
#include "Agent.h"
#include "IPolicy.h"
#include "InformedHeuristic.h"
#include "Types.h"
#include "State.h"
#include "SafeIntervals.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <vector>
#include "ILowLevelPlanner.h"

class SIPP: public ILowLevelPlanner
{
public:
    Path Plan(const Graph& g, const Agent& a, SafeIntervals& si) override;
    std::tuple<Path, unsigned long> Search(const Graph& g, const Agent& a, SafeIntervals& si) override; 
    inline void Init(IPolicy* policy, const InformedHeuristic& ih) override{this->ih = &ih; nexpansions = 0;}
    inline std::string GetName(void) const override {return "SIPP";}

protected:
    struct Vertex;
    struct VertexComparator{ bool operator() (const Vertex* v1, const Vertex* v2) const noexcept;};

    using MinFibHeap = boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<VertexComparator>>;
    using Successors = std::vector<Vertex*>;
    using LookupTable = boost::unordered::unordered_map<State, Vertex*, State::Hasher, State::Equal>;
    
    struct Vertex
    {
        const Vertex* parent = nullptr;
        State s{};
        float g = INF;
        float h = 0;
        bool in_open = false;
        MinFibHeap::handle_type handler;
        
        bool operator < (const Vertex& v) const noexcept;
        bool operator == (const Vertex& v) const noexcept;
        inline float fVal(void) const noexcept {return g + h;}
    };

    LookupTable table;
    const InformedHeuristic* ih = nullptr;
    unsigned long nexpansions = 0;

    Vertex* Init(const Agent& a, SafeIntervals& si);
    Successors Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si);
    Successors Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si);
    float EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g);
    void Clear(void);
    bool IsGenerated(const State& s) const;
    bool IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time);
    int WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const;
    Path ReconstructPath(const Vertex* goal) const;
};