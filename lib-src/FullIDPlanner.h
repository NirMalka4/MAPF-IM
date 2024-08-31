#pragma once

#pragma once

#include "FullPlanner.h"
#include "IPolicy.h"
#include "Types.h"
#include <string>
#include "IHighLevelPlanner.h"

class InformedHeuristic;
class IPolicy;

class FullIDPlanner: public FullPlanner
{
public:
    FullIDPlanner(IPolicy* policy, IHighLevelPlanner* ihlp);
    virtual ~FullIDPlanner() = default; 

    inline std::string GetName(void) const override {return "Full+ID+" + ihlp->GetName() + "+" + policy->GetName();}

protected:
    std::tuple<bool, bool, unsigned long> Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected, float runtime, int current_timestep) override;
};