#include "Utils.h"
#include "Map.h"
#include "Printer.h"
#include "Snapshot.h"
#include "Terrain.h"
#include "Types.h"
#include "VertexConflict.h"
#include "EdgeConflict.h"
#include <algorithm>
#include <cstdlib>
#include "A*.h"

CoordinateSet Neighborhood::FourPrincipleDirection(const Coordinate& c, size_t R)
{
    CoordinateSet s;

    for(int r = 1; r <= R; r++)
    {
        s.emplace(c.row - r, c.column); // up
        s.emplace(c.row + r, c.column); // down
        s.emplace(c.row, c.column - r); // left
        s.emplace(c.row, c.column + r); // right
        s.emplace(c.row, c.column); // wait
    }
    
    return s;
}

CoordinateSet Neighborhood::EightPrincipleDirection(const Coordinate& c, size_t R)
{
    CoordinateSet s = FourPrincipleDirection(c, R);

    for(int r = 1; r <= R; r++)
    {
        s.emplace(c.row - r, c.column - r); // up-left
        s.emplace(c.row - r, c.column + r); // up-right
        s.emplace(c.row + r, c.column - r); // down-left
        s.emplace(c.row + r, c.column + r); // down-right
    }
    
    return s;
}

float Heuristic::ManhattanDistance(const Coordinate& c1, const Coordinate& c2)
{
    return abs(c1.row - c2.row) + abs(c1.column - c2.column);
}

float Heuristic::ChebyshevDistance(const Coordinate& c1, const Coordinate& c2)
{
    return std::max(abs(c1.row - c2.row), abs(c1.column - c2.column));
}

long ObjectiveFunction::PathLength(const Path& p)
{
    return p.empty() ? 0 : (long)p.size() - 1;
}

long ObjectiveFunction::SumOfCost(const Paths& ps)
{
    long soc = 0;

    for(const auto& p: ps)
    {
        soc += PathLength(p);
    }
    
    return soc;
}

long ObjectiveFunction::Makespan(const Paths& ps)
{
    long longest = -1;

    for(const auto& p: ps)
    {
        longest = std::max(longest, PathLength(p));
    }

    return longest;
}

Conflicts Validator::FindConflicts(const Paths& ps)
{
    Conflicts conflicts;
    CoordinateMap cmap;
    EdgeMap emap;

    const auto max_time_step = ObjectiveFunction::Makespan(ps);
    const auto number_of_agents = ps.size();

    for(size_t timestep = 0; timestep <= max_time_step; timestep++)
    {
        for(size_t i = 0; i < number_of_agents; i++)
        {
            const auto& p = ps.at(i);
            if(!p.empty())
            {
                cmap[GetCoordinateAt(p, timestep)].push_back(i);
                emap[GetEdgeAt(p, timestep)] = {i, false};
            }
        }
        AddVertexConflictsAt(conflicts, timestep, cmap);
        AddEdgeConflictsAt(conflicts, timestep, emap);
    }

    return conflicts;
}

void Validator::AddVertexConflictsAt(Conflicts& cs, const int timestep, CoordinateMap& cmap)
{
    for(const auto& [coordinate, agents_indices]: cmap)
    {
        if(agents_indices.size() > 1)
            cs.push_back(new VertexConflict(agents_indices, timestep, coordinate));
    }

    cmap.clear();
}

void Validator::AddEdgeConflictsAt(Conflicts& cs, int timestep, EdgeMap& emap)
{
    for(auto& [edge, edge_status]: emap)
    {
        auto& [agent, edge_flag] = edge_status;
        if(!edge.IsSelfLoopEdge())
        {
            for(const auto& crossing_edge: edge.GetCrossingEdges())
            {
                if(emap.find(crossing_edge) != emap.end())
                {
                    auto& [conflicted_agent, crossing_edge_flag] = emap[crossing_edge];
                    if(!crossing_edge_flag)
                    {
                        cs.push_back(new EdgeConflict({agent, conflicted_agent}, timestep + 1, edge, crossing_edge));
                        crossing_edge_flag = edge_flag = true;
                    }
                }
            }
        }
    }

    emap.clear();
}

Coordinate Validator::GetCoordinateAt(const Path& p, const int timestep)
{
    return timestep < p.size() ? p.at(timestep): p.back();
}

Edge Validator::GetEdgeAt(const Path& p, const int timestep)
{
    const auto there_is_edge_to_cross = (timestep + 1) < p.size();
    return there_is_edge_to_cross ? Edge{p.at(timestep), p.at(timestep + 1)} : Edge();
}

bool Validator::IsLegalPlan(const Snapshot& snap, const Paths& ps, const Agents& as, const bool verbose)
{
    auto no_conflicts = NoConflictsExists(ps, verbose);
    auto arrived_at_goal = AreAllAgentsReachedTheirGoals(as, ps, verbose);
    auto no_invalid_transitions = AreAllPathsContainsOnlyValidTransitions(snap, ps, as, verbose);
    return no_conflicts && no_invalid_transitions && arrived_at_goal;
}

bool Validator::AreAllAgentsReachedTheirGoals(const Agents& as, const Paths& ps, const bool verbose)
{
    assert(as.size() == ps.size());
    
    const int k = as.size();
    bool is_valid = true;

    for(int i = 0; i < k; i++)
    {
        is_valid &= (!ps[i].empty() && ps[i].back() == as[i].goal);
        if(!is_valid)
        {
            if(verbose)
            {
                Print(Red, as[i], " has not reached its goal", '\n');
            }
            else
                return false;
        }
    }

    return is_valid;
}

bool Validator::AreAllPathsContainsOnlyValidTransitions(const Snapshot& snap, const Paths& ps, const Agents& as, const bool verbose)
{
    auto is_path_contains_only_valid_transition = [&snap, verbose](const Path& p, const Agent& a)
    {
        auto is_valid = true;
        const int n = p.size();

        for(int timestep = 0; timestep < n - 1; timestep++)
        {
            const auto& curr = p[timestep];
            const auto& next = p[timestep + 1];
            Edge e{curr, next};
            auto is_valid_transition = snap.IsOpenEdge(e) || snap.IsMaybeBlockedEdge(e);

            if(!is_valid_transition)
            {
                if(verbose)
                {
                    Print(Red, a, " executed invalid transition over ", (snap.IsMaybeOpenEdge(e) ? "maybe_open edge " : "blocked edge "), e.ToString(), ", at ", timestep, '\n');
                    // assert(false);
                }
            }
                
            is_valid = is_valid && is_valid_transition;
        }

        return is_valid;
    };

    const int k = as.size();
    bool out = true;

    for(int i = 0; i < k; i++)
        out = out && is_path_contains_only_valid_transition(ps[i], as[i]);
    
    return out;
}

bool Validator::NoConflictsExists(const Paths& ps, const bool verbose)
{
    const auto cs = FindConflicts(ps);
    bool out = cs.empty();

    if(!out && verbose)
    {
        Print(Red,"The following conflicts were detected:\n");
        for(int i = 0; i < (int)cs.size(); i++)
            Print(Red, '#', i, ")\t", cs[i], '\n');
    }

    Free(cs);
    return out;
}

bool Validator::ExistsConflict(const Paths& ps)
{
    const auto cs = FindConflicts(ps);
    bool out = !cs.empty();
    Free(cs);
    return out;
}

void Validator::Free(const Conflicts& conflicts)
{
    for(auto conflict: conflicts)
    {
        delete conflict;
        conflict = nullptr;
    }
}

Agents Validator::ValidAgents(const Agents& src, const Map& m)
{
    Astar astar;
    Agents valid_agents;
    Graph g = m.CreateGraph(false, false);

    for(const auto& a: src)
    {
        if(m.IsValidCoordinate(a.start) && m.IsValidCoordinate(a.goal))
        {
            auto agent_start_terrain = m.GetTerrain(a.start), agent_goal_terrain = m.GetTerrain(a.goal);
            bool is_agent_start_at_legal_coordinates = (agent_start_terrain == Terrain::empty && agent_goal_terrain == Terrain::empty);
            
            if( is_agent_start_at_legal_coordinates && 
                !astar.Plan(g, a).empty() && 
                std::none_of(valid_agents.begin(), valid_agents.end(), [&a](const auto& b){return b.start == a.start || b.goal == a.goal;}))
            {
                valid_agents.push_back(a);
            }
        }
    }

    for(int i = 0; i < (int)valid_agents.size(); i++)
    {
        valid_agents[i].index = i;
    }

    return valid_agents;
}