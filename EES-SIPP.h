#pragma once

#include "Graph.h"
#include "Agent.h"
#include "ILowLevelPlanner.h"
#include "Types.h"
#include "State.h"
#include "SafeIntervals.h"
#include "Utils.h"
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <vector>

class IPolicy;
class InformedHeuristic;

class EESSIPP: public ILowLevelPlanner
{
public:
    EESSIPP(float w=1.000, const HeuristicFunction& h = Heuristic::ManhattanDistance);
    virtual ~EESSIPP() = default;

    Path Plan(const Graph& g, const Agent& a, SafeIntervals& si) override;
    std::tuple<Path, unsigned long> Search(const Graph& g, const Agent& a, SafeIntervals& si) override;
    inline void Init(IPolicy* policy, const InformedHeuristic& ih) override{this->policy = policy; this->ih = &ih; nexpansions = 0;}
    inline std::string GetName(void) const override {return "EES-SIPP";}

protected:
    struct Vertex;
    struct CleanupComparator{bool operator () (const Vertex* v1, const Vertex* v2) const noexcept;};
    struct OpenComparator{bool operator () (const Vertex* v1, const Vertex* v2) const noexcept;};
    struct FocalComparator{bool operator () (const Vertex* v1, const Vertex* v2) const noexcept;};

    using CleanupMinFibHeap = boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<CleanupComparator>>;
    using OpenBalancedTree = boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<OpenComparator>>;
    using FocalMinFibHeap = boost::heap::fibonacci_heap<Vertex*, boost::heap::compare<FocalComparator>>;
    using Successors = std::vector<Vertex*>;
    using LookupTable = boost::unordered::unordered_map<State, Vertex*, State::Hasher, State::Equal>;
    
    struct Vertex
    {
        const Vertex* parent = nullptr;
        State s{};
        float g = INF;     // cost of travelling to a node from the root
        float h = 0;      // admisibble heurisitc to cost-to-go 
        float h_hat = 0; // inadmissible heurisitc to cost-to-go
        float d_hat = 0; // estimate (potentialy inadmissible) to distance-to-go
        bool in_cleanup = false;
        bool in_focal = false;
        bool in_closed = false;
        CleanupMinFibHeap::handle_type cleanup_handler;
        OpenBalancedTree::handle_type open_handler;
        FocalMinFibHeap::handle_type focal_handler;
        
        inline float f(void) const noexcept {return g + h;}
        inline float f_hat(void) const noexcept {return g + h_hat;}
        std::string ToString(void) const {return "[" + s.ToString() + ", g = " + std::to_string(g) + ", h = " + std::to_string(h) + "]";}
    };

    float w;
    const HeuristicFunction& h;
    LookupTable table;
    CleanupMinFibHeap cleanup;
    OpenBalancedTree open;
    FocalMinFibHeap focal;
    const IPolicy* policy;
    const InformedHeuristic* ih;
    unsigned long nexpansions;

    std::tuple<float, Vertex*> Pop();
    void Push(Vertex* parent, Vertex* successor, const Graph& g);
    void BalanceHeaps(float f_hat_min);
    bool IsGoal(const Vertex* v, const Agent& a) const;
    bool GenerateStartVertex(const Agent& a, SafeIntervals& si);
    Successors Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si);
    Successors Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si);
    float EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g) const;
    bool IsGenerated(const State& s) const;
    bool IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time) const;
    int WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const;
    Path ReconstructPath(const Vertex* goal) const;
    void Clear(void);
};