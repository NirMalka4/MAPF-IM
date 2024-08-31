#pragma once

#include "ILowLevelPlanner.h"
#include "Types.h"
#include "Graph.h"
#include "IHighLevelPlanner.h"
#include "InformedHeuristic.h"
#include <random>

class ILowLevelPlanner;
class IPolicy;

class PP: public IHighLevelPlanner
{
public:
    PP() = default; 
    PP(ILowLevelPlanner* llp, int nshuffles=20);
    virtual ~PP() {delete llp; llp = nullptr;};

    PlanResult Plan(const Graph& g, const Agents& as, float timeout) override;
    std::tuple<bool, unsigned long> Replan(const Graph& g, const Agents& all, Paths& ongoing_plans, const AgentsIndicesSet& affected, float timeout, int current_timestep) override;
    
    inline void Init(IPolicy* policy, const InformedHeuristic& ih, size_t k) override {this->ih = &ih; llp->Init(policy, ih);}
    inline std::string GetName(void) const override {return "PP+" + (llp ? llp->GetName() : "NO-LOW-LEVEL-PLANNER");}

protected:
    ILowLevelPlanner* llp;
    int nshuffles;
    std::random_device rd;
    std::mt19937 gen;
    const InformedHeuristic* ih;
    unsigned long nexpansions;
    
    AgentsIndicesSet ReplanGroup(const Graph& g, const Agents& all, Paths& planned_paths, AgentsIndicesSet group);
    Agents Shuffle(const Agents& all, const AgentsIndicesSet& affected);
    AgentsIndicesSet BlockingAgents(const Agents& all, const AgentsIndicesSet& blocked, const Paths& ongoing_plans);
    AgentsIndicesSet Unaffected(const Paths& planned_paths);
};