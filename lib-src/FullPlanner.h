#pragma once

#include "IPlanner.h"
#include "Types.h"
#include "IHighLevelPlanner.h"

class Graph;

class FullPlanner: public IPlanner
{
public:
    FullPlanner(IPolicy* policy, IHighLevelPlanner* ihlp);
    virtual ~FullPlanner() {delete ihlp; ihlp = nullptr;}

    ScenarioResult Plan(const Snapshot& snap, const Agents& src, const InformedHeuristic& ih, float timeout) override;
    virtual inline std::string GetName(void) const override {return "Full+" + ihlp->GetName() + "+" + policy->GetName();}

protected:
    IHighLevelPlanner* ihlp;

    virtual std::tuple<bool, bool, unsigned long> Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected, float runtime, int current_timestep);
};