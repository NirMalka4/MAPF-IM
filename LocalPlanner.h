#pragma once

#include "IPlanner.h"
#include "DisjointSets.h"
#include "Types.h"
#include "IHighLevelPlanner.h"
#include "A*.h"

class Graph;

class LocalPlanner: public IPlanner
{
public:
    LocalPlanner(IPolicy* policy, IHighLevelPlanner* ihlp, int r=5);
    virtual ~LocalPlanner() {delete ihlp; ihlp = nullptr;}

    ScenarioResult Plan(const Snapshot& snap, const Agents& src, const InformedHeuristic& ih, float timeout) override;
    virtual inline std::string GetName(void) const override {return "Local+" + ihlp->GetName() + "+" + policy->GetName();}

protected:
    IHighLevelPlanner* ihlp;
    Astar astar;
    int r;

    std::vector<AgentsIndicesSet> DetectCollisions(const Paths& plans, const Agents& all);
    Agents ExtractAgentsSubset(const AgentsIndicesSet& agents_subset_indices, const Agents& all);
    std::vector<AgentsIndicesSet> Merge(const std::vector<std::vector<AgentsIndicesSet>>& groups_set, size_t K);
    std::vector<AgentsIndicesSet> BuildGroups(DisjointSets& ds);
    virtual bool Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected);
};