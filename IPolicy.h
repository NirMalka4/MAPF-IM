#pragma once

#include "Constants.h"
#include "Snapshot.h"
#include "Types.h"
#include "Graph.h"
#include <limits>
#include <random>
struct Coordinate;

class IPolicy
{
public:
    virtual ~IPolicy() = default;
    virtual void Init(const Snapshot& snap) = 0;
    virtual void Update(const EdgeSet& new_observed_maybe_open, const EdgeSet& new_observed_maybe_blocked) = 0;
    virtual float GetPenalty(const Edge& e, const Graph& g) const = 0;
    virtual std::string GetName(void) = 0;
};


class BaselinePolicy: public IPolicy
{
public:
    virtual ~BaselinePolicy() override = default;
    inline void Init(const Snapshot& snap) override {}
    inline float GetPenalty(const Edge& e, const Graph& g) const override {/*return CERTAIN_EDGE_PENALTY;*/ return rand() % 13;}
    inline void Update(const EdgeSet& new_observed_maybe_open, const EdgeSet& new_observed_maybe_blocked) override {}
    inline std::string GetName(void) override {return "Baseline-Policy";}

};

class RiskAversePolicy: public IPolicy
{
public:
    virtual ~RiskAversePolicy() override = default;

    void Init(const Snapshot& snap) override
    {
        for(const auto& e: snap.GetMaybeOpenEdges())
        {
            maybe_open_edges.insert(e);
        }      
    }

    void Update(const EdgeSet& new_observed_maybe_open, const EdgeSet& new_observed_maybe_blocked) override
    {
        for(const auto& e: new_observed_maybe_open)
        {
            maybe_open_edges.erase(e);
        }            
    }

    float GetPenalty(const Edge& e, const Graph& g) const override 
    {
        return maybe_open_edges.find(e) == maybe_open_edges.end() ? CERTAIN_EDGE_PENALTY : MAYBE_EDGE_PENALTY;
    }

    inline std::string GetName(void) override {return "Risk-Averse-Policy";}

protected:
    EdgeSet maybe_open_edges;
};

class ExplorativePolicy: public IPolicy
{
public:
    virtual ~ExplorativePolicy() override = default;

    void Init(const Snapshot& snap) override
    {
        for(const auto& e: snap.GetMaybeBlockedEdges())
        {
            maybe_blocked_edges.insert(e);
        }  
    }

    void Update(const EdgeSet& new_observed_maybe_open, const EdgeSet& new_observed_maybe_blocked) override
    {
        for(const auto& e: new_observed_maybe_blocked)
        {
            maybe_blocked_edges.erase(e);
        }
    }

    float GetPenalty(const Edge& e, const Graph& g) const override 
    {
        const Coordinate n = e.destination;
        const auto n_successors = g.SuccessorsOf(n);

        bool can_n_observe_uncertain_edge = std::any_of(n_successors.begin(), n_successors.end(), [&](const auto& successor){return maybe_blocked_edges.find({n, successor}) != maybe_blocked_edges.end();});
        
        return can_n_observe_uncertain_edge ? -MAYBE_EDGE_PENALTY : CERTAIN_EDGE_PENALTY;
    }

    inline std::string GetName(void) override {return "Explorative-Policy";}

protected:
    EdgeSet maybe_blocked_edges;
};

class HybridPolicy: public IPolicy
{
protected:
    RiskAversePolicy risk_averse;
    ExplorativePolicy explorative;
    
public:
    virtual ~HybridPolicy() override = default;

    void Init(const Snapshot& snap) override
    {
        risk_averse.Init(snap);
        explorative.Init(snap);
    }

    void Update(const EdgeSet& new_observed_maybe_open, const EdgeSet& new_observed_maybe_blocked) override
    {
        risk_averse.Update(new_observed_maybe_open, new_observed_maybe_blocked);
        explorative.Update(new_observed_maybe_open, new_observed_maybe_blocked);
    }

    float GetPenalty(const Edge& e, const Graph& g) const override 
    {
        float risk_averse_penalty = risk_averse.GetPenalty(e, g);

        return risk_averse_penalty == CERTAIN_EDGE_PENALTY /* no penalty is given by risk-averse policy */ ? explorative.GetPenalty(e, g) : risk_averse_penalty;
    }

    inline std::string GetName(void) override {return "Hybrid-Policy";}

protected:
    EdgeSet maybe_open_edges, maybe_blocked_edges;
};