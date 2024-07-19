#pragma once

#include "Types.h"

class Graph;
class Snapshot;
class SafeIntervals;
class InformedHeuristic;
class IPolicy;

class ILowLevelPlanner
{
public:
    virtual Path Plan(const Graph& g, const Agent& a, SafeIntervals& si) = 0;
    virtual std::tuple<Path, unsigned long> Search(const Graph& g, const Agent& a, SafeIntervals& si) = 0;
    virtual void Init(IPolicy* policy, const InformedHeuristic& ih) = 0;
    virtual std::string GetName(void) const = 0;
};  