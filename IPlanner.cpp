#include "IPlanner.h"
#include "Constants.h"
#include "Graph.h"
#include "InformedHeuristic.h"
#include "Types.h"
#include "Utils.h"
#include "A*.h"
#include "Printer.h"
#include "Map.h"
#include <algorithm>
#include <string>
#include <filesystem>
#include <utility>

IPlanner::IPlanner(IPolicy* policy): policy(policy) {}

std::tuple<EdgeSet, EdgeSet> IPlanner::Observe(const Agents& as, const Graph& g, const Snapshot& snap, EdgeSet& observed_maybe_open_edge, EdgeSet& observed_maybe_blocked_edge) const
{
    EdgeSet new_observed_maybe_open_edge, new_observed_maybe_blocked_edge;

    for(const auto& a: as)
    {
        for(const auto& successor_coordinate: g.SuccessorsOf(a.start))
        {
            Edge e{a.start, successor_coordinate};
            if(snap.IsMaybeOpenEdge(e) && observed_maybe_open_edge.find(e) == observed_maybe_open_edge.end())
            {
                new_observed_maybe_open_edge.insert(e);
                observed_maybe_open_edge.insert(e);
            }
            else if(snap.IsMaybeBlockedEdge(e) && observed_maybe_blocked_edge.find(e) == observed_maybe_blocked_edge.end())
            {
                new_observed_maybe_blocked_edge.insert(e);
                observed_maybe_blocked_edge.insert(e);
            }
        }
    }

    return {new_observed_maybe_open_edge, new_observed_maybe_blocked_edge};
}

void IPlanner::Step(Agents& as, Paths& realized, Paths& planned) const
{
    for(int i = 0; i < as.size(); i++)
    {
        // track agent current location
        realized[i].push_back(planned[i].front()); 
        // if the agent is currently at its goal, its planned to stay there (i.e, wait) until all agents reach their goals.
        planned[i] = (planned[i].size() > 1) ? Path(std::move_iterator(std::next(planned[i].begin())), std::move_iterator(planned[i].end())) : Path{planned[i].back()};
        // set the rest of the planned path
        // move the agent to its next planned coordinate
        as[i].start = planned[i].front();
    }
}

void IPlanner::UpdateGraph(Graph& g, const EdgeSet& new_observed_maybe_open_edge, const EdgeSet& new_observed_maybe_blocked_edge) const
{
    for(const auto& actually_blocked_edge: new_observed_maybe_open_edge)
        g.RemoveEdge(actually_blocked_edge);
        
    for(const auto& actually_open_edge: new_observed_maybe_blocked_edge)
        g.AddEdge(actually_open_edge);
}

Paths IPlanner::Prune(const Paths& ps) const
{
    Paths pruned;

    constexpr auto last_time_goal_visited = [](const Path& p)
    {
        const auto goal = p.back();
        const auto n = p.size();
        auto j = INF;
        for(auto i = 0; i < n; i++)
        {
            if(p[i] == goal)
            {
                if(j == INF)
                    j = i;
            }
            else if(j != INF)
                j = INF;
        }
        return j;
    };

    std::transform(ps.begin(), ps.end(), std::back_inserter(pruned),
        [&last_time_goal_visited](const Path& p){return Path(p.begin(), p.begin() + last_time_goal_visited(p) + 1);});
    
    return pruned;
}

AgentsIndicesSet IPlanner::FindAffectedAgents(const Graph& g, const Paths& planned_paths, const EdgeSet& observed_maybe_open_edge, const EdgeSet& observed_maybe_blocked_edge, const InformedHeuristic& ih)
{
    AgentsIndicesSet affected;

    auto&& affected_by_new_observed_blocked_edge = FindAffectedByNewObservedBlockedEdge(planned_paths, observed_maybe_open_edge);
    affected.insert(affected_by_new_observed_blocked_edge.begin(), affected_by_new_observed_blocked_edge.end());

    auto&& affected_by_new_observed_open_edge = FindAffectedByNewObservedOpenEdge(g, planned_paths, observed_maybe_blocked_edge, ih);
    affected.insert(affected_by_new_observed_open_edge.begin(), affected_by_new_observed_open_edge.end()); 

    return affected;
}

AgentsIndicesSet IPlanner::FindAffectedByNewObservedOpenEdge(const Graph& g, const Paths& planned_paths, const EdgeSet& new_observed_maybe_blocked_edge, const InformedHeuristic& ih)
{
    AgentsIndicesSet affected;
    Astar astar;
    const int k = planned_paths.size();

    for(int i = 0; i < k; i++)
    {
        const auto& p = planned_paths[i];
        assert(!p.empty());

        Agent a{p.front(), p.back(), i};
        const long current_path_length = ObjectiveFunction::PathLength(p);

        for(const auto& e: new_observed_maybe_blocked_edge)
        {
            const long alternative_path_length = ih(a.start, e.source) + g.WeightOf(e) + ih(e.destination, a.goal);
            if(alternative_path_length < current_path_length)
            {
                affected.insert(i);
                break; // continue with next agent
            }
        }
    }

    return affected;
}

AgentsIndicesSet IPlanner::FindAffectedByNewObservedBlockedEdge(const Paths& planned_paths, const EdgeSet& observed_maybe_open_edge)
{
    AgentsIndicesSet affected;
    const int k = planned_paths.size();

    for(int i = 0; i < k; i++)
    {
        const auto& p = planned_paths[i];

        for(int t = 0; t < p.size() - 1; t++)
        {
            Edge e{p[t], p[t + 1]};

            if(observed_maybe_open_edge.find(e) != observed_maybe_open_edge.end())
            {
                affected.insert(i);
                break; // continue with next agent
            }
        }
    }

    return affected;
}

void IPlanner::LogPlan(const Paths& planned_paths)
{
    log << log_plan_title << log_lines_delimiter;

    const auto k = planned_paths.size();

    for(auto i = 0; i < k; i++)
    {
        log << log_Agent_prefix << std::to_string(i + 1);

        for(const auto& c: planned_paths[i])
        {
            log << log_path_coordinates_delimiter << c;
        }

        log << log_lines_delimiter;
    }

    log << log_lines_delimiter;
}

void IPlanner::LogStep(const Paths& realized_paths, const int timestep)
{
    log << log_steps_title << log_lines_delimiter;

    log << "#step" << log_lines_delimiter << std::to_string(timestep);

    for(const auto& p: realized_paths)
    {
        log << log_path_coordinates_delimiter << p.back();
    }
        
    log << log_lines_delimiter;
}

void IPlanner::LogObservations(const EdgeSet& new_observed_maybe_open_edges, const EdgeSet& new_observed_maybe_blocked_edges, const int timestep, const int agent_index)
{
    std::for_each(new_observed_maybe_open_edges.begin(), new_observed_maybe_open_edges.end(), [&](const auto& edge){
        log << ObservationToString(edge, true, timestep, agent_index) << log_lines_delimiter;
    });

    std::for_each(new_observed_maybe_blocked_edges.begin(), new_observed_maybe_blocked_edges.end(), [&](const auto& edge){
        log << ObservationToString(edge, false, timestep, agent_index) << log_lines_delimiter;
    });

    log << log_lines_delimiter;
}

std::string IPlanner::ObservationToString(const Edge& edge, bool is_maybe_open, int timestep, int agent_index)
{
    return std::string(log_observation_prefix) + std::string(log_lines_delimiter) + (is_maybe_open ? std::string(log_observation_maybe_open) : std::string(log_observation_maybe_closed)) + 
            " obstacle is observed by " + "Agent" + std::to_string(agent_index) + " at " + std::to_string(timestep) + " on " + edge.ToStringLog();
}

void IPlanner::LogMap(const Map& m)
{
    m.LogGrid(log, log_grid_cell_delimiter, log_lines_delimiter);
}

void IPlanner::LogAgents(const Agents& as)
{
    const auto k = as.size();
    if(k > 0)
    {
        log << log_Agents_title << log_lines_delimiter;

        for(auto i = 0; i < k; i++)
        {
            log << log_Agent_prefix << std::to_string(i+1) << log_path_coordinates_delimiter << as[i].start << log_path_coordinates_delimiter << as[i].goal << log_lines_delimiter;
        }
            
        log << log_lines_delimiter;
    }
}

void IPlanner::LogSnapshot(const Snapshot& snap)
{
    log << log_Maybe_Edeges_title << log_lines_delimiter;
    LogMaybeEdges(snap.GetMaybeBlockedEdges());
    LogMaybeEdges(snap.GetMaybeOpenEdges());
    log << log_lines_delimiter;
}

void IPlanner::LogMaybeEdges(const EdgeSet& maybe_edges)
{
    int i = 1;

    for(const auto& e: maybe_edges)
    {
        log << std::to_string(i) << log_path_coordinates_delimiter << e.ToStringLog() << log_lines_delimiter;
        i += 1;
    }
}

void IPlanner::InitLogFile(const std::string& output_directory_folder_path, const std::string& filename)
{
    std::filesystem::create_directories(output_directory_folder_path);

    const auto filepath = output_directory_folder_path + '/' + filename;
    std::fstream file{filepath, file.out};

    if(!file)
    {
        Print(Red, __PRETTY_FUNCTION__," Failed to open ", filename , "!\n");
    }

    log = std::forward<std::fstream>(file);
}