#include "SEES-SIPP.h"
#include "Agent.h"
#include "Printer.h"
#include "InformedHeuristic.h"
#include "IPolicy.h"

bool SEES_SIPP::VertexComparator::operator() (const Vertex* v1, const Vertex* v2) const noexcept
{
    return !(*v1 < *v2);
}

bool SEES_SIPP::VertexEqual::operator() (const Vertex* v1, const Vertex* v2) const noexcept
{
    return (*v1 == *v2);
}

size_t SEES_SIPP::VertexHasher::operator() (const Vertex* v) const noexcept
{
    return State::Hasher{}(v->s);
}

bool SEES_SIPP::Vertex::operator < (const Vertex& other) const noexcept
{
    if(d_hat == other.d_hat)
    {
        if(h == other.h)
            return  g > other.h; 
        return h < other.h;
    }
    return d_hat < other.d_hat;
}

bool SEES_SIPP::Vertex::operator == (const Vertex& other) const noexcept
{
    return s == other.s;
}

SEES_SIPP::SEES_SIPP(const float w, const HeuristicFunction& h): w(w), nexpansions(0), h(h), table(), policy(nullptr), ih(nullptr){}

Path SEES_SIPP::Plan(const Graph& g, const Agent& a, SafeIntervals& si)
{
    auto root = Init(a, si);
    Vertex* goal = nullptr;

    float threshold_f = (root != nullptr ? root->h : INF);
    float threshold_f_hat = (root != nullptr ? root->h_hat : INF);

    while(threshold_f < INF && goal == nullptr)
    {
        std::tie(threshold_f, threshold_f_hat, goal) = Speedy(root, g, a, si, threshold_f, threshold_f_hat);
    }

    auto path = goal != nullptr ? ReconstructPath(goal) : Path{};
    Clear();
    return path;
}

std::tuple<float, float, SEES_SIPP::Vertex*> SEES_SIPP::Speedy(Vertex* root, const Graph& g, const Agent& a, SafeIntervals& si, const float threshold_f, const float threshold_f_hat)
{
    MinFibHeap open;
    Vertex* current;
    VerticesSet closed;
    float threshold_f_next = INF;
    float threshold_f_hat_next = INF;
    bool is_goal_found = false;

    if(root)
    {
        root->handler = open.push(root);
    }

    while(!open.empty() && !is_goal_found)
    {
        current = open.top();
        open.pop();
        current->in_open = false;
        closed.insert(current);

        is_goal_found = IsGoal(current, a);

        if(!is_goal_found)
        {
            for(Vertex* successor: Expand(current, g, a, si))
            {
                const auto successor_arriving_time = EarliestArrivingTime(current, successor->s, g);

                if(successor_arriving_time < successor->g)
                {
                    successor->g = successor_arriving_time;
                    successor->parent = current;
                    successor->d_hat = policy ? (successor->h + policy->GetPenalty({successor->s.c, successor->s.c}, g)) : successor->h;
                }

                if(successor->fVal() > w * threshold_f || successor->fHat() > w * threshold_f_hat)
                {
                    threshold_f_next = std::min(threshold_f_next, successor->fVal());
                    threshold_f_hat_next = std::min(threshold_f_hat_next, successor->fHat());
                }

                if(successor->in_open)
                {
                    open.decrease(successor->handler, successor);
                }
                else if(closed.find(successor) == closed.end())
                {
                    successor->handler = open.push(successor);
                    successor->in_open = true;
                }
            }
        }
    }

    return {threshold_f_next, threshold_f_hat_next, is_goal_found ? current : nullptr};
}

std::tuple<Path, unsigned long> SEES_SIPP::Search(const Graph& g, const Agent& a, SafeIntervals& si)
{
    return {Plan(g, a, si), nexpansions};
}

SEES_SIPP::Vertex* SEES_SIPP::Init(const Agent& a, SafeIntervals& si)
{
    Vertex* start = nullptr;
    const auto first_safe_interval = *si.IntervalsOf(a.start).begin();

    if(first_safe_interval.IsIntersects(0) && !first_safe_interval.IsEmpty())
    {
        start = new Vertex();
        start->s = {a.start, first_safe_interval};
        start->g = 0;
        start->h = ih ? std::max((*ih)(a.start, a.goal), si.IntervalsOf(a.goal).rbegin()->start) : h(a.start, a.goal);
        start->h_hat = w * start->h;
        table[start->s] = start;
    }
    
    return start;
}

SEES_SIPP::Successors SEES_SIPP::Generate(Vertex* parent, const Coordinate& successor_coordinate, const Graph& g, const Agent& a, SafeIntervals& si)
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
                successor->h = ih ? std::max((*ih)(successor->s.c, a.goal), si.IntervalsOf(a.goal).rbegin()->start - successor_arriving_time) : h(successor->s.c, a.goal);
                successor->h_hat = w * successor->h;
                table[successor->s] = successor;
            }

            successors.push_back(successor);
        }
    }
    
    return successors;
}

SEES_SIPP::Successors SEES_SIPP::Expand(Vertex* current, const Graph& g, const Agent& a, SafeIntervals& si)
{
    Successors successors;

    for(const auto& successor_coordinate: g.SuccessorsOf(current->s.c))
    {
        auto&& successor_safe_states = Generate(current, successor_coordinate, g, a, si);
        successors.insert(successors.end(), successor_safe_states.begin(), successor_safe_states.end());
    }

    return successors;
}

float SEES_SIPP::EarliestArrivingTime(const Vertex* parent, const State& successor_state, const Graph& g) const
{
    const auto successor_arriving_time = parent->g + g.WeightOf({parent->s.c, successor_state.c});
    return std::max(successor_state.i.start, successor_arriving_time);
}

void SEES_SIPP::Clear(void)
{
    for(auto& [state, vertex]: table)
        delete(vertex);

    table.clear();
}

bool SEES_SIPP::IsGenerated(const State& s) const
{
    return table.find(s) != table.end();
}

bool SEES_SIPP::IsTransitionAllowed(const Vertex* parent, const State& successor_state, float arriving_time) const
{
    return successor_state.i.IsIntersects(arriving_time) && arriving_time <= parent->s.i.end;
}

bool SEES_SIPP::IsGoal(const Vertex* v, const Agent& a) const
{
    return v != nullptr && v->s.c == a.goal && v->s.i.IsUnbounded();
}

int SEES_SIPP::WaitingTimeAtParent(const Vertex* parent, const Vertex* successor) const
{    
    return parent ? (successor->g - parent->g) : 0;
}

Path SEES_SIPP::ReconstructPath(const Vertex* goal) const
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
            n = n - 1;
        }
        
        curr = parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}