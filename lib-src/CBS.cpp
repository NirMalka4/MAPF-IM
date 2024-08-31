#include "CBS.h"
#include "Agent.h"
#include "IConflict.h"
#include "SafeIntervals.h"
#include "Timer.h"
#include "Printer.h"
#include "ILowLevelPlanner.h"
#include "Types.h"
#include "Utils.h"
#include <algorithm>
#include <sstream>
#include <utility>
#include "DisjointSets.h"

CBS::CTNode::CTNode(const Paths& ps, const size_t cost): paths(ps), cost(cost){}

bool CBS::CTNode::operator == (const CBS::CTNode& other) const noexcept
{
    return constraints == other.constraints;
}

bool CBS::CTNode::operator < (const CBS::CTNode& other) const noexcept
{
    return cost < other.cost || (cost == other.cost && h < other.h);
}

CBS::CTNode::operator bool() const
{
    return !paths.empty();
}

std::string CBS::CTNode::ToString(void) const
{
    std::stringstream ss;
    ss << "CT node. Cost: " << cost << '\n';

    for(const auto& c: constraints)
    {
        ss << c << '\n';
    }

    return ss.str();
}

bool CBS::CTNode::Comparator::operator()(const CTNode& n1, const CTNode& n2) const noexcept
{
    return !(n1 < n2);
}

size_t CBS::CTNode::Hasher::operator()(const Constraints& cs) const noexcept
{
    // we apply xor since SAME constraints might appear in a different order at DIFFERENT ConstraintTreeNode
    std::size_t seed = 0, xor_hashes = 0;
    const Constraint::Hasher constraint_hasher;

    std::for_each(cs.begin(), cs.end(), [&](const auto& constraint){xor_hashes = xor_hashes ^ constraint_hasher(constraint);});

    boost::hash_combine(seed, xor_hashes);
    return seed;
}

CBS::CBS(ILowLevelPlanner* llp): llp(llp), lookup(), high_level_nexpansions(0), low_level_nexpansions(0), K(300), previous_constraints(){}

PlanResult CBS::Plan(const Graph& g, const Agents& as, const float timeout)
{
    Timer timer;
    timer.Start(timeout);

    high_level_nexpansions = 0;
    low_level_nexpansions = 0;
    previous_constraints.clear();

    auto&& [is_plan_found, paths, constraints] = Search(g, as, timeout);

    return {is_plan_found, paths, high_level_nexpansions, timer.Stop()};
}

std::tuple<bool, unsigned long> CBS::Replan(const Graph& g, const Agents& all, Paths& ongoing_plans, const AgentsIndicesSet& affected, const float timeout, const int current_timestep)
{
    Timer timer;
    timer.Start(timeout);

    high_level_nexpansions = 0;
    low_level_nexpansions = 0;
    
    auto disjoint_groups = Partition(all, current_timestep);
    Groups next_disjoint_groups;
    
    bool is_replanning_succeed = ReplanAffectedGroups(disjoint_groups, g, all, ongoing_plans, affected, timer.GetRemainingRuntime(), current_timestep);
    bool exist_conflicting_group = true;

    while(!timer.ExceedsRuntime() && exist_conflicting_group && is_replanning_succeed)
    {
        exist_conflicting_group = false;
        
        for(const auto& conflicting_groups_indicies: ConflictingGroups(disjoint_groups, ongoing_plans, all))
        {
            if(conflicting_groups_indicies.size() > 1)
            {
                auto merged_conflicting_group = Merge(disjoint_groups, conflicting_groups_indicies);
                is_replanning_succeed = ReplanGroup(merged_conflicting_group, g, all, ongoing_plans, timer.GetRemainingRuntime(), current_timestep);
                
                if(!is_replanning_succeed)
                {
                    break;
                }
                
                next_disjoint_groups.push_back(merged_conflicting_group);
                exist_conflicting_group = true;
            }
            else
            {
                next_disjoint_groups.push_back(AgentsIndicesSet(conflicting_groups_indicies.begin(), conflicting_groups_indicies.end()));
            }
        }

        disjoint_groups = std::forward<Groups>(next_disjoint_groups);
        next_disjoint_groups.clear();
    }

    
    return {!timer.ExceedsRuntime() && !exist_conflicting_group && is_replanning_succeed, high_level_nexpansions};
}

std::tuple<bool, Paths, Constraints> CBS::Search(const Graph& g, const Agents& as, const float timeout)
{
    CTNode goal;
    MinFibHeap open;
    Timer timer;
    bool is_plan_found = false;
    auto root = Init(g, as);
    if(root)
    {
        open.push(root);
    }
    
    timer.Start(timeout);

    while(!open.empty() && !timer.ExceedsRuntime() && !is_plan_found)
    {
        auto n = open.top();
        open.pop();

        auto conflicts = Validator::FindConflicts(n.paths);
        is_plan_found = conflicts.empty();
        if(is_plan_found)
        {
            goal = std::forward<CTNode>(n);
        }
        else
        {
            IConflict* c = conflicts.front();
            for(auto&& s: Expand(n, c, g, as))
            {
                open.push(s);
            }
            Validator::Free(conflicts);
        }
    }
    
    lookup.clear();
    
    return {is_plan_found && !timer.ExceedsRuntime(), goal.paths, goal.constraints};
}

CBS::CTNode CBS::Init(const Graph& g, const Agents& as)
{
    CTNode root;
    root.paths.resize(K);
    SafeIntervals si;
    bool is_solvable_scenario = true;

    for(const auto& a: as)
    {
        if(!Agent::IsPlaceholderAgent(a))
        {
            auto&& [p, low_level_nexpansions] = llp->Search(g, a, si);
            low_level_nexpansions += low_level_nexpansions;

            if(!p.empty())
            {
                root.paths[a.index] = std::forward<Path>(p);
            }
            else
            {
                is_solvable_scenario = false;
                Print(Red, "NO PLAN FOUND FOR: ", agent, '\n');
                break;
            }
        }
    }

    if(is_solvable_scenario)
    {
        root.cost = ObjectiveFunction::SumOfCost(root.paths);
        high_level_nexpansions += 1;
    }
    else
    {
        root.paths.clear();
    }
    
    return root;
}

CBS::Successors CBS::Expand(const CTNode& n, const IConflict* conf, const Graph& g, const Agents& as)
{
    Successors ss;

    for(const auto& new_constraint: conf->Resolve())
    {
        Constraints successor_constraints = n.constraints;
        successor_constraints.insert(new_constraint);

        if(!lookup.contains(h(successor_constraints))) // no CTNode with the same constraints is generated
        {
            high_level_nexpansions += 1;
            auto successor = Generate(g, as[new_constraint.constrained_agent], n.paths, std::forward<Constraints>(successor_constraints));

            if(successor)
            {
                assert(Validator::GetCoordinateAt(successor.paths[new_constraint.constrained_agent], new_constraint.timestep) != new_constraint.c);
                lookup.insert(h(successor.constraints));
                ss.push_back(successor);
            }
        }
    }

    return ss;
}

CBS::CTNode CBS::Generate(const Graph& g, const Agent& a, const Paths& ps, Constraints&& cs)
{
    CTNode n;
    SafeIntervals si(cs, a.index);
    auto&& [p, low_level_nexpansions] = llp->Search(g, a, si);
    low_level_nexpansions += low_level_nexpansions;

    if(!p.empty())
    {
        n.paths = ps;
        n.paths[a.index] = std::forward<Path>(p);
        n.constraints = std::forward<Constraints>(cs);
        n.cost = ObjectiveFunction::SumOfCost(n.paths);
        n.h = NumberOfConflicts(n.paths);
    }

    return n;
}

int CBS::NumberOfConflicts(const Paths& ps)
{
    auto confs = Validator::FindConflicts(ps);
    auto n = confs.size();
    Validator::Free(confs);
    return n;
}

CBS::Groups CBS::Partition(const Agents& all, const int current_timestep)
{
    Groups disjoint_groups;
    DisjointSets ds(K);
    auto iter = previous_constraints.begin();

    while(iter != previous_constraints.end())
    {
        const auto& c = *iter;
        if(c.timestep >= current_timestep)
        {
            if(!Agent::IsPlaceholderAgent(all[c.constrained_agent]) && !Agent::IsPlaceholderAgent(all[c.conflicted_agent]))
            {
                ds.Union(c.constrained_agent, c.conflicted_agent);
            }
            iter = previous_constraints.erase(iter);
        }
        else
        {
            iter++;
        }
    }

    for(const auto& agents_indicies: ds.GetDisjointSets())
    {
        disjoint_groups.push_back(AgentsIndicesSet(agents_indicies.begin(), agents_indicies.end()));
    }

    return disjoint_groups;
}

bool CBS::ReplanAffectedGroups(const Groups& disjoint_groups, const Graph& g, const Agents& all, Paths& ongoing_plans, const AgentsIndicesSet& affected, const float timeout, const int current_timestep)
{
    Timer timer;
    timer.Start(timeout);
    bool is_replanning_succeed = true;

    for(const auto& group: disjoint_groups)
    {
        if(std::any_of(group.begin(), group.end(), [&affected](const auto i){return affected.contains(i);}))
        {
            is_replanning_succeed = ReplanGroup(group, g, all, ongoing_plans, timer.GetRemainingRuntime(), current_timestep);
            if(!is_replanning_succeed)
            {
                break;
            }
        }
    }

    return is_replanning_succeed;
}

bool CBS::ReplanGroup(const AgentsIndicesSet& group, const Graph& g, const Agents& all, Paths& ongoing_plans, const float timeout, const int current_timestep)
{
    // // remove previous group constraints
    auto iter = previous_constraints.begin();
    while(iter != previous_constraints.end())
    {
        const auto& c = *iter;
        if(group.contains(c.constrained_agent) || group.contains(c.conflicted_agent))
        {
            iter = previous_constraints.erase(iter);
        }
        else
        {
            iter++;
        }
    }

    // plan for the group
    auto&& [is_plan_found, paths, current_constraints] = Search(g, ExtractGroupAgents(group, all), timeout);

    if(is_plan_found)
    {
        for(const auto i: group)
        {
            ongoing_plans[i] = std::forward<Path>(paths[i]);
        }
        // CBS planning from timestep 0, although current timestep is later.
        for(const auto& c: current_constraints)
        {
            previous_constraints.insert({c.constrained_agent, c.conflicted_agent, c.c, c.timestep + current_timestep});
        }
    }
    return is_plan_found;
}

Agents CBS::ExtractGroupAgents(const AgentsIndicesSet& group, const Agents& all)
{
    Agents group_agents(K);
    for(const auto i: group)
    {
        group_agents[i] = all[i];
    }
    return group_agents;
}

std::vector<std::vector<int>> CBS::ConflictingGroups(const Groups& disjoint_groups, const Paths& ongoing_plans, const Agents& all)
{
    const int N = disjoint_groups.size();
    DisjointSets ds(N);
    
    for(int i = 0; i < N; i++)
    {
        for(int j = i + 1; j < N; j++)
        {
            Paths groups_merged_paths;
            const auto& g1 = disjoint_groups[i], g2 = disjoint_groups[j];

            std::for_each(g1.begin(), g1.end(), [&groups_merged_paths, &ongoing_plans](const auto i){groups_merged_paths.push_back(ongoing_plans[i]);});
            std::for_each(g2.begin(), g2.end(), [&groups_merged_paths, &ongoing_plans](const auto i){groups_merged_paths.push_back(ongoing_plans[i]);});

            if(Validator::ExistsConflict(groups_merged_paths))
            {
                ds.Union(i, j);
            }
        }
    }

    return ds.GetDisjointSets();
}

AgentsIndicesSet CBS::Merge(const Groups& groups, const std::vector<int>& conflicting_groups_indices)
{
    AgentsIndicesSet merged;

    for(const auto i: conflicting_groups_indices)
    {
        merged.insert(groups[i].begin(), groups[i].end());
    }

    return merged;
}