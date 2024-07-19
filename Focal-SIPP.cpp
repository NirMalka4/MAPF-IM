#include "Focal-SIPP.h"
#include "Agent.h"
#include "Graph.h"
#include "SafeIntervals.h"
#include "Utils.h"
#include "Printer.h"
#include "IPolicy.h"

FocalSIPP::FocalSIPP(float w, const HeuristicFunction& h): w(w), h(h), table(), open(), focal(), policy(nullptr), ih(nullptr), nexpansions(0){}


bool FocalSIPP::OpenComparator::operator () (const Vertex* v1, const Vertex* v2) const noexcept
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

bool FocalSIPP::FocalComparator::operator () (const Vertex* v1, const Vertex* v2) const noexcept
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

Path FocalSIPP::Plan(const Graph& g, const Agent& a, SafeIntervals& si)
{
    nexpansions = 0;
    GenerateStartVertex(a, si);
    Vertex* v;
    float fmin;
    bool is_goal_found = false;

    while(!focal.empty() && !is_goal_found)
    {
        std::tie(fmin, v) = Pop();
        v->in_closed = true;
        
        is_goal_found = IsGoal(v, a);

        if(!is_goal_found)
        {
            for(Vertex* successor: Expand(v, g, a, si))
            {
                Push(v, successor, g);
            }

            BalanceHeaps(fmin);
        }
    }

    auto path = is_goal_found ? ReconstructPath(v) : Path{};;
    Clear();
    return path;
}

std::tuple<Path, unsigned long> FocalSIPP::Search(const Graph& g, const Agent& a, SafeIntervals& si)
{
    return {Plan(g, a, si), nexpansions};
}

std::tuple<float, FocalSIPP::Vertex*> FocalSIPP::Pop()
{
    float fmin = open.top()->f();

    Vertex* top = focal.top();
    focal.pop();
    top->in_focal = false;

    open.erase(top->open_handler);
    top->in_open = false;

    return {fmin, top};
}

void FocalSIPP::Push(Vertex* parent, Vertex* successor, const Graph& g)
{
    if(parent != nullptr && successor != nullptr)
    {
        const auto successor_arriving_time = EarliestArrivingTime(parent, successor->s, g);

        if(successor_arriving_time < successor->g)
        {
            successor->g = successor_arriving_time;
            successor->parent = parent;
            successor->d_hat = policy ? (successor->h + policy->GetPenalty({parent->s.c, successor->s.c}, g)) : successor->h;

            if(successor->in_open)
            {
                open.decrease(successor->open_handler, successor);
            }
            else
            {
                successor->open_handler = open.push(successor);
                successor->in_open = true;
            }

            if(successor->f() <= w * open.top()->f())
            {
                if(successor->in_focal)
                {
                    focal.decrease(successor->focal_handler);
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

void FocalSIPP::BalanceHeaps(const float fmin)
{
    if(!open.empty() && fmin < w * open.top()->f())
    {
        const auto old_bound = w * fmin;
        const auto new_bound = w * open.top()->f();

        for(auto* v: open)
        {
            if(!v->in_focal && v->f() > old_bound && v->f() <= new_bound)
            {
                v->focal_handler = focal.push(v);
                v->in_focal = true;
            }
        }
    }
}

bool FocalSIPP::IsGoal(const Vertex* v, const Agent& a) const
{
    return v != nullptr && v->s.c == a.goal && v->s.i.IsUnbounded();
}

bool FocalSIPP::GenerateStartVertex(const Agent& a, SafeIntervals& si)
{
    Vertex* start = nullptr;

    const auto start_safe_interval = si.FirstSafeInterval(a.start, 0);
    if(!start_safe_interval.IsEmpty())
    {
        start = new Vertex();
        start->s = {a.start, start_safe_interval};
        start->g = 0;
        start->h = ih ? std::max((*ih)(a.start, a.goal), si.IntervalsOf(a.goal).rbegin()->start) : h(a.start, a.goal);
        start->d_hat = start->h;
        table[start->s] = start;

        start->open_handler = open.push(start);
        start->in_open = true;

        start->focal_handler = focal.push(start);
        start->in_focal = true;
    }

    return start != nullptr;
}

FocalSIPP::Successors FocalSIPP::Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si)
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
                successor->h = ih ? std::max((*ih)(successor->s.c, a.goal), si.IntervalsOf(a.goal).rbegin()->start - successor_arriving_time) : h(successor->s.c, a.goal);
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

FocalSIPP::Successors FocalSIPP::Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si)
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

float FocalSIPP::EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g) const
{
    const auto successor_arriving_time = parent->g + g.WeightOf({parent->s.c, successor_state.c});
    return std::max(successor_state.i.start, successor_arriving_time);
}

bool FocalSIPP::IsGenerated(const State& s) const
{
    return table.find(s) != table.end();
}

bool FocalSIPP::IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time) const
{
    return parent->s.c != successor_state.c && successor_state.i.IsIntersects(arriving_time) && arriving_time <= parent->s.i.end;
}

int FocalSIPP::WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const
{
    assert(!parent || successor->g >= parent->g);
    
    return parent ? (successor->g - parent->g) : 0;
}

Path FocalSIPP::ReconstructPath(const Vertex* goal) const
{
    Path path;
    const Vertex* curr = goal;

    while(curr)
    {
        path.emplace_back(curr->s.c);
        auto parent = curr->parent;
        int n = WaitingTimeAtParent(parent, curr);

        while(n-- > 1)
            path.emplace_back(parent->s.c);

        curr = parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void FocalSIPP::Clear(void)
{
    for(auto& [state, vertex]: table)
        delete(vertex);

    table.clear();
    open.clear();
    focal.clear();
}
