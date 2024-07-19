#include "FullPlanner.h"
#include "Constants.h"
#include "IPlanner.h"
#include "PP+EESSIPP.h"
#include "Timer.h"
#include "Types.h"
#include "Snapshot.h"
#include "Utils.h"
#include <string>
#include <tuple>
#include "IHighLevelPlanner.h"

FullPlanner::FullPlanner(IPolicy* policy, IHighLevelPlanner* ihlp): IPlanner(policy), ihlp(ihlp) {}

ScenarioResult FullPlanner::Plan(const Snapshot& snap, const Agents& src, const InformedHeuristic& ih, const float timeout)
{

    Timer timer;
    bool is_planning_succeed, is_replanning_occurred;
    EdgeSet observed_maybe_open_edge, observed_maybe_blocked_edge;
    EdgeSet new_observed_maybe_open_edge, new_observed_maybe_blocked_edge;
    Agents as(src);
    Paths realized(src.size());
    Paths plans;
    int timestep = 0;
    float runtime;
    int replans = 0;
    unsigned long high_level_nexpansions = 0;
    unsigned long total_nexpansions = 0;
    Graph g = snap.Create();
    policy->Init(snap);
    ihlp->Init(policy, ih, src.size());
    
    timer.Start(timeout);
    
    std::tie(is_planning_succeed, plans, high_level_nexpansions, runtime) = ihlp->Plan(g, as, timer.GetRemainingRuntime());
    total_nexpansions += high_level_nexpansions;

    #ifdef LOG
        LogPlan(plans);
    #endif

    while(is_planning_succeed && !timer.ExceedsRuntime() && !Validator::AreAllAgentsReachedTheirGoals(as, realized))
    {
        std::tie(new_observed_maybe_open_edge, new_observed_maybe_blocked_edge) = Observe(as, g, snap, observed_maybe_open_edge, observed_maybe_blocked_edge);
        
        if(!new_observed_maybe_open_edge.empty() || !new_observed_maybe_blocked_edge.empty())
        {
            #ifdef LOG
                LogObservations(new_observed_maybe_open_edge, new_observed_maybe_blocked_edge, timestep);
            #endif

            UpdateGraph(g, new_observed_maybe_open_edge, new_observed_maybe_blocked_edge);
            policy->Update(new_observed_maybe_open_edge, new_observed_maybe_blocked_edge);
            std::tie(is_planning_succeed, is_replanning_occurred, high_level_nexpansions) = Replan(g, as, plans,FindAffectedAgents(g, plans, new_observed_maybe_open_edge, new_observed_maybe_blocked_edge, ih), timer.GetRemainingRuntime(), timestep);
            total_nexpansions += high_level_nexpansions;
            if(is_replanning_occurred)
            {
                replans += 1;
            }

            #ifdef LOG
                LogPlan(plans);
            #endif
        }

        if(is_planning_succeed)
        {
            Step(as, realized, plans);

            #ifdef LOG
                LogStep(realized, timestep);
            #endif
        }

        timestep++;
    }

    is_planning_succeed = is_planning_succeed && !timer.ExceedsRuntime();

    return {is_planning_succeed, is_planning_succeed ? Prune(realized): Paths(), timer.Stop(), replans, total_nexpansions};
}

std::tuple<bool, bool, unsigned long> FullPlanner::Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected, float runtime, int current_timestep)
{
    bool is_replanning_succeed;
    unsigned long high_level_nexpansions;
    float replanning_runtime;
    std::tie(is_replanning_succeed, ongoing_plans, high_level_nexpansions, replanning_runtime) = ihlp->Plan(g, as, runtime);
    return {is_replanning_succeed, true, high_level_nexpansions};
}