#include "LocalIDPlanner.h"
#include "Types.h"

LocalIDPlanner::LocalIDPlanner(IPolicy* policy, IHighLevelPlanner* ihlp, int r): LocalPlanner(policy, ihlp, r){}


bool LocalIDPlanner::Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected)
{
    if(!affected.empty())
    {
        std::for_each(affected.begin(), affected.end(), [&](const auto i){ongoing_plans[i] = astar.Plan(g, as[i]);});
        return std::none_of(affected.begin(), affected.end(), [&ongoing_plans](const auto i){return ongoing_plans[i].empty();});
    }
    return true;  
}