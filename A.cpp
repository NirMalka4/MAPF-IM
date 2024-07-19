#include "A*.h"
#include <cassert>

bool Astar::VertexComparator::operator() (const Vertex* v1, const Vertex* v2) const noexcept
{
    assert(v1 && v2);
    return !(*v1 < *v2);
}

bool Astar::Vertex::operator < (const Vertex& other) const noexcept
{
    if(fVal() == other.fVal())
    {
        if(g == other.g)
            return  h < other.h; // prefer node with LOWER h cost: closer to destination
        return g > other.g;  // break tie in a depth-first manner: prefer node further from root
    }
    return fVal() < other.fVal();
}

bool Astar::Vertex::operator == (const Vertex& other) const noexcept
{
    return c == other.c;
}

Path Astar::Plan(const Graph& g, const Agent& a, const HeuristicFunction& h)
{
    auto root = Init(a, h);
    MinFibHeap open;
    Vertex* current;
    bool is_goal_found = false;
    root->handler = open.push(root);

    while(!open.empty() && !is_goal_found)
    {
        current = open.top();
        open.pop();
        current->in_open = false;

        is_goal_found = (current->c == a.goal);

        if(!is_goal_found)
        {
            for(Vertex* successor: Expand(current, g, a, h))
            {
                if(gCost(current, successor, g) < successor->g)
                {
                    successor->g = gCost(current, successor, g);
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

Astar::Successors Astar::Expand(Vertex* current, const Graph& g, const Agent& a, const HeuristicFunction& h)
{
    Successors successors;

    for(const auto& successor_coordinate: g.SuccessorsOf(current->c))
        successors.push_back(Generate(current, successor_coordinate, a, h));

    return successors;
}

bool Astar::IsGenerated(const Coordinate& c) const
{
    return table.find(c) != table.end();
}

Astar::Vertex* Astar::Generate(Vertex* parent, const Coordinate& successor_coordinate, const Agent& a, const HeuristicFunction& h)
{
    Vertex* successor;
    if(IsGenerated(successor_coordinate))
    {
        successor = table.at(successor_coordinate);
    }
    else
    {
        successor = new Vertex();
        successor->c = successor_coordinate;
        successor->h = h(successor_coordinate, a.goal);
        table[successor_coordinate] = successor;
    }
    return successor;
}

Astar::Vertex* Astar::Init(const Agent& a, const HeuristicFunction& h)
{
    Vertex* start = new Vertex();
    start->parent = nullptr;
    start->c = a.start;
    start->g = 0;
    start->h = h(a.start, a.goal);
    table[start->c] = start;
    return start;
}

float Astar::gCost(const Vertex* parent, const Vertex* successor, const Graph& g)
{
    return parent->g + g.WeightOf({parent->c, successor->c});
}

Path Astar::ReconstructPath(const Vertex* goal) const
{
    Path path;
    const Vertex* curr = goal;

    while(curr)
    {
        path.emplace_back(curr->c);
        curr = curr->parent;
    }

    std::reverse(path.begin(), path.end());
    return path;
}

void Astar::Clear(void)
{
    for(auto& [coordinate, vertex]: table)
        delete(vertex);
    table.clear();
}