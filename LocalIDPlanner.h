#pragma once

#include "LocalPlanner.h"
#include "Types.h"
#include "IHighLevelPlanner.h"

class LocalIDPlanner: public LocalPlanner
{
public:
    LocalIDPlanner(IPolicy* policy, IHighLevelPlanner* ihlp, int r=5);
    virtual ~LocalIDPlanner() = default;

    inline std::string GetName(void) const override {return "Local+ID+" + ihlp->GetName() + "+" + policy->GetName();}

protected:
    bool Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected) override;
};