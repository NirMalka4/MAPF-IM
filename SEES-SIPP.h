#pragma once

#include "Graph.h"
#include "Agent.h"
#include "Types.h"
#include "Utils.h"
#include "State.h"
#include "ILowLevelPlanner.h"
#include "SafeIntervals.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <vector>

class SEES_SIPP: public ILowLevelPlanner
{
public:
    SEES_SIPP(float w=1.000, const HeuristicFunction& h = Heuristic::ManhattanDistance);
    virtual ~SEES_SIPP() = default;

    Path Plan(const Graph& g, const Agent& a, SafeIntervals& si) override;
    std::tuple<Path, unsigned long> Search(const Graph& g, const Agent& a, SafeIntervals& si) override;
    inline void Init(IPolicy* policy, const InformedHeuristic& ih) override{this->policy = policy; this->ih = &ih; nexpansions = 0;}
    inline std::string GetName(void) const override {return "SEES-SIPP";}

protected:
    struct Vertex;
    struct VertexComparator{ bool operator() (const Vertex* v1, const Vertex* v2) const noexcept;};
    struct VertexHasher{ size_t operator() (const Vertex* v1) const noexcept;};
    struct VertexEqual{ bool operator() (const Vertex* v1, const Vertex* v2) const noexcept;};

    using MinFibHeap = boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<VertexComparator>>;
    using Successors = std::vector<Vertex*>;
    using LookupTable = boost::unordered::unordered_map<State, Vertex*, State::Hasher, State::Equal>;
    using VerticesSet = boost::unordered::unordered_set<Vertex*, VertexHasher, VertexEqual>;
    
    struct Vertex
    {
        const Vertex* parent = nullptr;
        State s{};
        float g = INF;
        float h = 0;
        float h_hat = 0;
        float d_hat = 0;
        bool in_open = false;
        MinFibHeap::handle_type handler;
        
        bool operator < (const Vertex& v) const noexcept;
        bool operator == (const Vertex& v) const noexcept;
        inline float fVal(void) const noexcept {return g + h;}
        inline float fHat(void) const noexcept {return g + h_hat;}
    };

    float w;
    unsigned long nexpansions;
    const HeuristicFunction& h;
    LookupTable table;
    const IPolicy* policy;
    const InformedHeuristic* ih;
    
    std::tuple<float, float, Vertex*> Speedy(Vertex* root, const Graph& g, const Agent& a, SafeIntervals& si, const float threshold_f, const float threshold_f_hat);
    Vertex* Init(const Agent& a, SafeIntervals& si);
    Successors Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si);
    Successors Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si);
    float EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g) const;
    void Clear(void);
    bool IsGenerated(const State& s) const;
    bool IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time) const;
    bool IsGoal(const Vertex* v, const Agent& a) const;
    int WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const;
    Path ReconstructPath(const Vertex* goal) const;
};