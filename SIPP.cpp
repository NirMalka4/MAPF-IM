#include "SIPP.h"
#include "SafeIntervals.h"
#include "State.h"
#include "Printer.h"
#include <algorithm>
#include <cassert>
#include "InformedHeuristic.h"
#include "Utils.h"

bool SIPP::VertexComparator::operator() (const Vertex* v1, const Vertex* v2) const noexcept
{
    assert(v1 && v2);
    return !(*v1 < *v2);
}

bool SIPP::Vertex::operator < (const Vertex& other) const noexcept
{
    if(fVal() == other.fVal())
    {
        if(h == other.h)
            return g > other.h;  // break tie in a depth-first manner: prefer node further from root
        return h < other.h;// prefer node with LOWER h cost: closer to destination
    }
    return fVal() < other.fVal();
}

bool SIPP::Vertex::operator == (const Vertex& other) const noexcept
{
    return s == other.s;
}

Path SIPP::Plan(const Graph& g, const Agent& a, SafeIntervals& si)
{
    auto root = Init(a, si);
    nexpansions = 0;
    MinFibHeap open;
    Vertex* current;
    bool is_goal_found = false;
    if(root)
        root->handler = open.push(root);

    while(!open.empty() && !is_goal_found)
    {
        current = open.top();
        open.pop();
        current->in_open = false;

        is_goal_found = (current->s.c == a.goal && current->s.i.IsUnbounded());

        if(!is_goal_found)
        {
            for(Vertex* successor: Expand(current, g, a, si))
            {
                assert(successor != nullptr);

                if(EarliestArrivingTime(current, successor->s, g) < successor->g) // a shorter path to successor is found
                {
                    successor->g = EarliestArrivingTime(current, successor->s, g);
                    successor->parent = current;

                    if(successor->in_open)
                        open.decrease(successor->handler, successor);
                    else
                    {
                        successor->handler = open.push(successor);
                        successor->in_open = true;
                    }
                        
                }
            }
        }
    }
    
    auto path = is_goal_found ? ReconstructPath(current) : Path{};
    Clear();
    return path;
}

std::tuple<Path, unsigned long> SIPP::Search(const Graph& g, const Agent& a, SafeIntervals& si)
{
    return {Plan(g, a, si), nexpansions};
}

SIPP::Successors SIPP::Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si)
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

bool SIPP::IsGenerated(const State& s) const
{
    return table.find(s) != table.end();
}

float SIPP::EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g)
{
    assert(parent != nullptr);

    const auto successor_arriving_time = parent->g + g.WeightOf({parent->s.c, successor_state.c});

    return std::max(successor_state.i.start, successor_arriving_time);
}

bool SIPP::IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time)
{
    assert(parent != nullptr);

    return parent->s.c != successor_state.c && successor_state.i.IsIntersects(arriving_time) && arriving_time <= parent->s.i.end;
}

SIPP::Successors SIPP::Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si)
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
                successor = table.at(successor_state);
            else
            {
                successor = new Vertex();
                successor->s = successor_state;
                successor->h = ih ? std::max((*ih)(successor_coordinate, a.goal), si.IntervalsOf(a.goal).rbegin()->start - successor_arriving_time) : Heuristic::ManhattanDistance(successor_coordinate, a.goal);
                table[successor->s] = successor;
            }
            successors.push_back(successor);
        }
    }
    
    return successors;
}

SIPP::Vertex* SIPP::Init(const Agent& a, SafeIntervals& si)
{
    Vertex* start = nullptr;
    const auto start_safe_interval = *si.IntervalsOf(a.start).begin();

    if(start_safe_interval.IsIntersects(0))// Expected starting state to be reachable at the beginning
    {
        start = new Vertex();
        start->parent = nullptr;

        start->s = State(a.start, start_safe_interval);
        assert(start->s.i.IsIntersects(0)); 

        start->g = 0;
        start->h = ih ? std::max((*ih)(a.start, a.goal), si.IntervalsOf(a.goal).rbegin()->start) : Heuristic::ManhattanDistance(a.start, a.goal);
        table[start->s] = start;
    }

    return start;
}

int SIPP::WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const
{
    assert(!parent || successor->g >= parent->g);
    
    return parent ? (successor->g - parent->g) : 0;
}


Path SIPP::ReconstructPath(const Vertex* goal) const
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

void SIPP::Clear(void)
{
    for(auto& [state, vertex]: table)
        delete(vertex);

    table.clear();
}