#pragma once

#include "Types.h"

class Graph;
class Snapshot;
class IPolicy;
class InformedHeuristic;

class IHighLevelPlanner
{
public:
    virtual PlanResult Plan(const Graph& g, const Agents& as, float timeout) = 0;
    virtual std::tuple<bool, unsigned long> Replan(const Graph& g, const Agents& all, Paths& ongoing_plans, const AgentsIndicesSet& affected, float timeout, int current_timestep) = 0;
    virtual void Init(IPolicy* policy, const InformedHeuristic& ih, size_t k) = 0;
    virtual std::string GetName(void) const = 0;
};