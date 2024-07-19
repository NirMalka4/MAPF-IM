#pragma once

#include "Graph.h"
#include "IConflict.h"
#include "ILowLevelPlanner.h"
#include "Types.h"
#include "IHighLevelPlanner.h"
#include <boost/heap/fibonacci_heap.hpp>

class ILowLevelPlanner;
class IPolicy;
class InformedHeuristic;

class CBS: public IHighLevelPlanner
{
public:
    CBS(ILowLevelPlanner* llp);
    virtual ~CBS() {delete llp; llp = nullptr;}
    
    PlanResult Plan(const Graph& g, const Agents& as, float timeout) override;
    std::tuple<bool, unsigned long> Replan(const Graph& g, const Agents& all, Paths& ongoing_plans, const AgentsIndicesSet& affected, float timeout, int current_timestep) override;
    inline void Init(IPolicy* policy, const InformedHeuristic& ih, const size_t k) override {this->K = k; llp->Init(policy, ih);};
    inline std::string GetName(void) const override {return "CBS+" + (llp ? llp->GetName() : "NO-LOW-LEVEL-PLANNER");}

protected:
    struct CTNode
    {
        CTNode() = default;
        CTNode(const Paths&, size_t cost);
        
        Paths paths;
        Constraints constraints;
        size_t cost = LONG_INF;
        int h = 0; // breaking-tie in favour of nodes with lower number of conflicts

        std::string ToString(void) const;

        inline friend std::ostream& operator << (std::ostream& out, const CTNode* n){return out << n->ToString();}
        bool operator == (const CTNode& other) const noexcept;
        bool operator < (const CTNode& other) const noexcept;
        explicit operator bool() const;

        // Hash a CT node according to its constraints.
        struct Hasher{size_t operator() (const Constraints&) const noexcept;};
        struct Comparator{bool operator() (const CTNode&, const CTNode&) const noexcept;};
    };

    using MinFibHeap = boost::heap::fibonacci_heap<CTNode, boost::heap::compare<CTNode::Comparator>>;
    using CTNodeSet = boost::unordered_set<size_t>; // set of hash of CTNode constraints, used for duplicates detection
    using Successors = std::vector<CTNode>;
    using Groups = std::vector<AgentsIndicesSet>;

    ILowLevelPlanner* llp;
    CTNodeSet lookup;
    unsigned long high_level_nexpansions;
    unsigned long low_level_nexpansions;
    size_t K;
    Constraints previous_constraints;
    static constexpr CTNode::Hasher h;

    // CBS methods
    std::tuple<bool, Paths, Constraints> Search(const Graph& g, const Agents& as, float timeout);
    CTNode Init(const Graph& g, const Agents& as);
    Successors Expand(const CTNode& n, const IConflict* c, const Graph& g, const Agents& as);
    CTNode Generate(const Graph& g, const Agent& a, const Paths& ps, Constraints&& cs);
    int NumberOfConflicts(const Paths& ps);

    // ID+CBS methods
    Groups Partition(const Agents& all, int current_timestep);
    bool ReplanAffectedGroups(const Groups& disjoint_groups, const Graph& g, const Agents& all, Paths& ongoing_plans, const AgentsIndicesSet& affected, float timeout, int current_timestep);
    bool ReplanGroup(const AgentsIndicesSet& group, const Graph& g, const Agents& all, Paths& ongoing_plans, float timeout, int current_timestep);
    Agents ExtractGroupAgents(const AgentsIndicesSet& g, const Agents& all);
    std::vector<std::vector<int>> ConflictingGroups(const Groups& disjoint_groups, const Paths& ongoing_plans, const Agents& all);
    AgentsIndicesSet Merge(const Groups& groups, const std::vector<int>& conflicting_groups_indices);
};