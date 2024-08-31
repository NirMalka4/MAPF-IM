#include "EES-SIPP.h"
#include "Agent.h"
#include "Graph.h"
#include "InformedHeuristic.h"
#include "SafeIntervals.h"
#include "Types.h"
#include "Printer.h"
#include "IPolicy.h"

EESSIPP::EESSIPP(const float w, const HeuristicFunction& h): w(w), h(h), table(), cleanup(), open(), focal(), policy(nullptr), ih(nullptr), nexpansions(0){}

bool EESSIPP::CleanupComparator::operator () (const Vertex* v1, const Vertex* v2) const noexcept
{
    if(v1->f() == v2->f())
    {
        if(v1->h == v2->h)
        {
            return v1->g <= v2->g;
        }
        return v1->h >= v2->h;
    }
    return v1->f() >= v2->f();
}

bool EESSIPP::OpenComparator::operator () (const Vertex* v1, const Vertex* v2) const noexcept
{
    if(v1->f_hat() == v2->f_hat())
    {
        if(v1->h == v2->h)
        {
            return v1->g <= v2->g;
        }
        return v1->h >= v2->h;
    }
    return v1->f_hat() >= v2->f_hat();
}

bool EESSIPP::FocalComparator::operator () (const Vertex* v1, const Vertex* v2) const noexcept
{
    if(v1->d_hat == v2->d_hat)
    {
        if(v1->h == v2->h)
        {
            return v1->g <= v2->g;
        }
        return v1->h >= v2->h;
    }
    return v1->d_hat >= v2->d_hat;
}

Path EESSIPP::Plan(const Graph& g, const Agent& a, SafeIntervals& si)
{
    nexpansions = 0;
    GenerateStartVertex(a, si);
    Vertex* v;
    float best_f_hat;
    bool is_goal_found = false;

    while(!focal.empty() && !is_goal_found)
    {
        std::tie(best_f_hat, v) = Pop();
        v->in_closed = true;

        is_goal_found = IsGoal(v, a);
        
        if(!is_goal_found)
        {
            for(Vertex* successor: Expand(v, g, a, si))
            {
                Push(v, successor, g);
            }

            BalanceHeaps(best_f_hat);
        }
    }

    auto path = is_goal_found ? ReconstructPath(v) : Path{};;
    Clear();
    return path;
}

std::tuple<Path, unsigned long> EESSIPP::Search(const Graph& g, const Agent& a, SafeIntervals& si)
{
    return {Plan(g, a, si), nexpansions};
}

std::tuple<float, EESSIPP::Vertex*> EESSIPP::Pop()
{
    float best_f = cleanup.top()->f();
    float best_f_hat = open.top()->f_hat();
    Vertex* out = nullptr;

    if(focal.top()->f_hat() <= w * best_f)
    {
        out = focal.top();
        focal.pop();
        out->in_focal = false;

        open.erase(out->open_handler);
        cleanup.erase(out->cleanup_handler);
        out->in_cleanup = false;
    }
    else if(open.top()->f_hat() <= w * best_f)
    {
        out = open.top();
        open.pop();

        cleanup.erase(out->cleanup_handler);
        out->in_cleanup = false;

        focal.erase(out->focal_handler);
        out->in_focal = false;
    }
    else
    {
        out = cleanup.top();
        cleanup.pop();
        out->in_cleanup = false;

        if(out->in_focal && out->f_hat() <= w * open.top()->f_hat())
        {
            focal.erase(out->focal_handler);
            out->in_focal = false;
        }

        open.erase(out->open_handler);
    }

    return {best_f_hat, out};
}

void EESSIPP::Push(Vertex* parent, Vertex* successor, const Graph& g)
{
    if(parent != nullptr && successor != nullptr)
    {
        const auto successor_arriving_time = EarliestArrivingTime(parent, successor->s, g);

        if(successor_arriving_time < successor->g)
        {
            successor->g = successor_arriving_time;
            successor->parent = parent;
            successor->d_hat = policy ? (successor->h + policy->GetPenalty({parent->s.c, successor->s.c}, g)) : successor->h;

            if(successor->in_cleanup)
            {
                cleanup.decrease(successor->cleanup_handler, successor);
                open.decrease(successor->open_handler, successor);
            }
            else
            {
                successor->cleanup_handler = cleanup.push(successor);
                successor->open_handler = open.push(successor);
                successor->in_cleanup = true;
            }

            if(successor->f_hat() <= w * open.top()->f_hat())
            {
                if(successor->in_focal)
                {
                    focal.update(successor->focal_handler);
                }
                else
                {
                    successor->focal_handler = focal.push(successor);
                    successor->in_focal = true;
                }
            }
        }
    }
}

void EESSIPP::BalanceHeaps(const float best_f_hat)
{
    if(!open.empty() && best_f_hat < open.top()->f_hat())
    {
        for(auto* v: open)
        {
            if(v->f_hat() <= w * open.top()->f_hat())
            {
                if(!v->in_focal)
                {
                    v->focal_handler = focal.push(v);
                    v->in_focal = true;
                }
            }
            else if(v->in_focal)
            {
                focal.erase(v->focal_handler);
                v->in_focal = false;
            }
        }
    }
}

bool EESSIPP::IsGoal(const Vertex* v, const Agent& a) const
{
    return v != nullptr && v->s.c == a.goal && v->s.i.IsUnbounded();
}

bool EESSIPP::GenerateStartVertex(const Agent& a, SafeIntervals& si)
{
    Vertex* start = nullptr;

    const auto start_safe_interval = si.FirstSafeInterval(a.start, 0);
    if(start_safe_interval.IsIntersects(0) && !start_safe_interval.IsEmpty())
    {
        start = new Vertex();
        start->s = {a.start, start_safe_interval};
        start->g = 0;
        start->h = ih ? std::max((*ih)(a.start, a.goal), si.IntervalsOf(a.goal).rbegin()->start) : h(a.start, a.goal);
        start->h_hat = w * start->h;
        start->d_hat = start->h;
        table[start->s] = start;

        start->cleanup_handler = cleanup.push(start);
        start->open_handler = open.push(start);
        start->in_cleanup = true;

        start->focal_handler = focal.push(start);
        start->in_focal = true;
    }

    return start != nullptr;
}

EESSIPP::Successors EESSIPP::Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si)
{
    Successors successors;

    for(const auto& successor_safe_interval: si.IntervalsOf(successor_coordinate))
    {
        State successor_state{successor_coordinate, successor_safe_interval};
        float successor_arriving_time = EarliestArrivingTime(parent, successor_state, g);

        if(IsTransitionAllowed(parent, successor_state, successor_arriving_time))
        {
            Vertex* successor;
            if(IsGenerated(successor_state))
            {
                successor = table.at(successor_state);
            }
            else
            {
                successor = new Vertex();
                successor->s = successor_state;
                successor->g = INF;
                successor->h = ih ? std::max((*ih)(successor_coordinate, a.goal), si.IntervalsOf(a.goal).rbegin()->start - successor_arriving_time) : h(successor_coordinate, a.goal);
                successor->h_hat = w * successor->h;
                table[successor_state] = successor;
            }

            if(!successor->in_closed)
            {
                successors.push_back(successor);
            }
        }
    }
    
    return successors;
}

EESSIPP::Successors EESSIPP::Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si)
{
    Successors successors;

    for(const auto& successor_coordinate: g.SuccessorsOf(current->s.c))
    {
        auto&& successor_safe_states = Generate(current, successor_coordinate, g, a, si);
        successors.insert(successors.end(), successor_safe_states.begin(), successor_safe_states.end());
    }

    nexpansions += 1;
    
    return successors;
}

float EESSIPP::EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g) const
{
    const auto successor_arriving_time = parent->g + g.WeightOf({parent->s.c, successor_state.c});
    return std::max(successor_state.i.start, successor_arriving_time);
}

bool EESSIPP::IsGenerated(const State& s) const
{
    return table.find(s) != table.end();
}

bool EESSIPP::IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time) const
{
    return (parent->s.c != successor_state.c) && successor_state.i.IsIntersects(arriving_time) && arriving_time <= parent->s.i.end;
}

int EESSIPP::WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const
{    
    return parent ? (successor->g - parent->g) : 0;
}

Path EESSIPP::ReconstructPath(const Vertex* goal) const
{
    Path path;
    const Vertex* curr = goal;

    while(curr)
    {
        path.emplace_back(curr->s.c);
        auto parent = curr->parent;
        int n = WaitingTimeAtParent(parent, curr);

        while(n > 1)
        {
            path.emplace_back(parent->s.c);
            n -= 1;
        }
        
        curr = parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void EESSIPP::Clear(void)
{
    for(auto& [state, vertex]: table)
        delete(vertex);

    table.clear();
    cleanup.clear();
    open.clear();
    focal.clear();
}
