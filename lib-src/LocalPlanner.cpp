#include "LocalPlanner.h"

#include "Astar.h"
#include "DisjointSets.h"
#include "Graph.h"
#include "IHighLevelPlanner.h"
#include "IPlanner.h"
#include "InformedHeuristic.h"
#include "Timer.h"
#include "Types.h"
#include "Snapshot.h"
#include "Utils.h"
#include <algorithm>
#include <tuple>
#include <utility>
#include "IPolicy.h"

LocalPlanner::LocalPlanner(IPolicy* policy, IHighLevelPlanner* ihlp, const int r): IPlanner(policy), ihlp(ihlp), astar(), r(r){}

ScenarioResult LocalPlanner::Plan(const Snapshot& snap, const Agents& src, const InformedHeuristic& ih, const float timeout)
{
    Timer timer;
    const auto K = src.size();
    EdgeSet observed_maybe_open_edge, observed_maybe_blocked_edge;
    EdgeSet new_observed_maybe_open_edge, new_observed_maybe_blocked_edge;
    Agents as(src);
    Paths realized(K);
    Paths plans(K);
    int timestep = 0;
    int replans = 0;
    float runtime;
    unsigned long high_level_nexpansions = 0;
    unsigned long total_nexpansions = 0;
    Graph g = snap.Create();
    policy->Init(snap);
    ihlp->Init(policy, ih, K);

    timer.Start(timeout);

    std::for_each(as.begin(), as.end(), [&](const auto& agent){plans[agent.index] = astar.Plan(g, agent);});
    bool is_planning_succeed = std::none_of(plans.begin(), plans.end(), [](const auto& p){return p.empty();});

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

            is_planning_succeed = Replan(g, as, plans, FindAffectedAgents(g, plans, new_observed_maybe_open_edge, new_observed_maybe_blocked_edge, ih));
            replans += 1;

            #ifdef LOG
                LogPlan(plans);
            #endif
        }

        // resolve collision
        auto conflicting_groups = DetectCollisions(plans, as);
        while(!conflicting_groups.empty() && is_planning_succeed && !timer.ExceedsRuntime())
        {
            for(const auto& group: conflicting_groups)
            {
                Paths conflicting_agents_revised_plans(K);
                std::tie(is_planning_succeed, conflicting_agents_revised_plans, high_level_nexpansions, runtime) = ihlp->Plan(g, ExtractAgentsSubset(group, as), timer.GetRemainingRuntime());
                total_nexpansions += high_level_nexpansions;
                replans += 1;

                if(is_planning_succeed)
                {
                    for(const auto i: group)
                    {
                        plans[i] = std::forward<Path>(conflicting_agents_revised_plans[i]);
                    }

                    conflicting_agents_revised_plans.clear();
                }
                else
                {
                    break;
                }
            }

            if(is_planning_succeed)
            {
                auto next_conflicting_groups = DetectCollisions(plans, as);
                if(!next_conflicting_groups.empty())
                {
                    conflicting_groups = Merge({conflicting_groups, next_conflicting_groups}, K);
                }
                else
                {
                    conflicting_groups.clear();
                }
            }
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

std::vector<AgentsIndicesSet> LocalPlanner::DetectCollisions(const Paths& plans, const Agents& all)
{
    const auto K = all.size();
    std::vector<AgentsIndicesSet> groups;
    DisjointSets ds(K);
    auto conflicts = Validator::FindConflicts(plans);

    for(auto* c: conflicts)
    {
        if(c->timestep <= r)
        {
            ds.Union(c->agents_indices);
        }
    }

    Validator::Free(conflicts);

    return BuildGroups(ds);
}

Agents LocalPlanner::ExtractAgentsSubset(const AgentsIndicesSet& agents_subset_indices, const Agents& all)
{
    Agents subset(all.size());

    for(auto i: agents_subset_indices)
    {
        subset[i] = all[i];
    }

    return subset;
}

std::vector<AgentsIndicesSet> LocalPlanner::Merge(const std::vector<std::vector<AgentsIndicesSet>>& groups_set, const size_t K)
{
    DisjointSets ds(K);
    for(const auto& set: groups_set)
    {
        for(const auto& group: set)
        {
            ds.Union(group);
        }
    }

    return BuildGroups(ds);
}

std::vector<AgentsIndicesSet> LocalPlanner::BuildGroups(DisjointSets& ds)
{
    std::vector<AgentsIndicesSet> groups;

    for(const auto& group: ds.GetDisjointSets())
    {
        if(group.size() > 1)
        {
            groups.push_back(AgentsIndicesSet{group.begin(), group.end()});
        }
    }

    return groups;
}

bool LocalPlanner::Replan(const Graph& g, const Agents& as, Paths& ongoing_plans, const AgentsIndicesSet& affected)
{
    std::for_each(as.begin(), as.end(), [&](const auto& a){ongoing_plans[a.index] = astar.Plan(g, a);});
    return std::none_of(ongoing_plans.begin(), ongoing_plans.end(), [](const auto& p){return p.empty();});
}