#include "FullIDPlanner.h"
#include "FullPlanner.h"
#include "IHighLevelPlanner.h"
#include "IPlanner.h"
#include "IPolicy.h"

FullIDPlanner::FullIDPlanner(IPolicy* policy, IHighLevelPlanner* ihlp): FullPlanner(policy, ihlp){}

std::tuple<bool, bool, unsigned long> FullIDPlanner::Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected, float runtime, const int current_timestep){
    bool is_replanning_succeed = true, is_replanning_occurred = false;
    unsigned long high_level_nexpansions = 0;

    if(!affected.empty())
    {
        std::tie(is_replanning_succeed, high_level_nexpansions) = ihlp->Replan(g, as, ongoing_plans, affected, runtime, current_timestep);
        is_replanning_occurred = true;
    }

    return {is_replanning_succeed, is_replanning_occurred, high_level_nexpansions};
}