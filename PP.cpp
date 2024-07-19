#include "PP.h"
#include "Agent.h"
#include "InformedHeuristic.h"
#include "SafeIntervals.h"
#include "Timer.h"
#include "Types.h"
#include <cassert>
#include <iterator>
#include <utility>
#include <algorithm>
#include "ILowLevelPlanner.h"
#include <algorithm>

PP::PP(ILowLevelPlanner* llp, const int nshuffles): llp(llp), nshuffles(nshuffles), rd(), gen(rd()), ih(nullptr), nexpansions(0){}

PlanResult PP::Plan(const Graph& g, const Agents& as, const float timeout)
{
    Timer timer;
    const auto K = as.size();
    nexpansions = 0;
    Paths ps(K);
    bool is_plan_found;

    timer.Start(timeout);

    AgentsIndicesSet S;
    for(const auto& a: as)
    {
        if(!Agent::IsPlaceholderAgent(a))
        {
            S.insert(a.index);
        }
    }

    std::tie(is_plan_found, nexpansions) = Replan(g, as, ps, S, timeout, 0);

    return {is_plan_found, is_plan_found ? ps : Paths{}, nexpansions, timer.Stop()};
}


std::tuple<bool, unsigned long> PP::Replan(const Graph& g, const Agents& all, Paths& planned_paths, const AgentsIndicesSet& affected, const float timeout, const int current_timestep)
{
    Timer timer;
    nexpansions = 0;
    const int N = std::max(1, static_cast<int>(0.1 * all.size()));
    AgentsIndicesSet blocked, blocking, blocking_subset;

    timer.Start(timeout);

    blocked = ReplanGroup(g, all, planned_paths, affected);

    while(!blocked.empty() && !timer.ExceedsRuntime())
    {
        blocking = BlockingAgents(all, blocked, planned_paths);
        blocking_subset.clear();

        while(!blocked.empty() && !timer.ExceedsRuntime())
        {
            std::sample(blocking.begin(), blocking.end(), std::inserter(blocking_subset, blocking_subset.begin()), N, gen);
            std::for_each(blocking_subset.begin(), blocking_subset.end(), [&planned_paths, &blocking](const auto i){planned_paths[i].clear(); blocking.erase(i);});

            blocked = ReplanGroup(g, all, planned_paths, blocked);
            
            // Print(Yellow, "blocking_subset = ", blocking_subset.size(), ", blocked = ", blocked.size(), ", blocking = ", blocking.size(), ", unaffected = ", Unaffected(planned_paths).size(), '\n');
            // Print(blocked);
        }

        blocked = ReplanGroup(g, all, planned_paths, blocking_subset);
    }

    return {blocked.empty() && !timer.ExceedsRuntime(), nexpansions};
}

AgentsIndicesSet PP::ReplanGroup(const Graph& g, const Agents& all, Paths& planned_paths, AgentsIndicesSet to_replan)
{
    std::for_each(to_replan.begin(), to_replan.end(), [&planned_paths](const auto i){planned_paths[i].clear();});
    SafeIntervals si(planned_paths);
    
    for(auto&& a: Shuffle(all, to_replan))
    {
        auto&& [p, low_level_nexpansions] = llp->Search(g, a, si);
        nexpansions += low_level_nexpansions;

        if(!p.empty())
        {
            si.Add(p);
            planned_paths[a.index] = std::forward<Path>(p);
            to_replan.erase(a.index);
        }
    }
    
    return to_replan;
}

AgentsIndicesSet PP::BlockingAgents(const Agents& all, const AgentsIndicesSet& blocked, const Paths& ongoing_plans)
{
    AgentsIndicesSet blocking;
    AgentsIndicesSet unaffeceted = Unaffected(ongoing_plans);

    for(const auto i: blocked)
    {
        const auto& a = all[i];

        assert(unaffeceted.find(i) == unaffeceted.end());

        auto iter = unaffeceted.begin();
        while(iter != unaffeceted.end())
        {
            const auto j = *iter;
            const auto& b = all[j];
            const auto& p = ongoing_plans[j];
            const auto h_val = (*ih)(a.start, b.goal) + (*ih)(b.goal, a.goal);
            
            // agent a is blocked by agent b whose "sitting on" its goal. i.e, the minimum-cost path a.start->b.goal->a.goal is blocked.
            // agent a is "run over" at its starting location
            if(h_val < INF || std::any_of(p.begin(), p.end(), [&a](const auto& c){return c == a.start;}))
            {
                blocking.insert(j);
                iter = unaffeceted.erase(iter);
            }
            else
            {
                iter++;
            }
        }
    }
    
    return blocking;
}

AgentsIndicesSet PP::Unaffected(const Paths& planned_paths)
{
    AgentsIndicesSet unaffeceted;

    for(int i = 0; i < (int)planned_paths.size(); i++)
    {
        if(!planned_paths[i].empty())
        {
            unaffeceted.insert(i);
        }
    }
    
    return unaffeceted;
}

Agents PP::Shuffle(const Agents& all, const AgentsIndicesSet& group)
{
    Agents shuffled;
    
    for(const auto agent_index: group)
    {
        shuffled.push_back(all[agent_index]);
    }
    
    std::shuffle(shuffled.begin(), shuffled.end(), gen);
    
    return shuffled;
}