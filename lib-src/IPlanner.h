#pragma once

#include "Edge.h"
#include "Graph.h"
#include "IPolicy.h"
#include "Types.h"
#include "Snapshot.h"
#include <string>
#include <fstream>

class IPolicy;
class InformedHeuristic;
class Map;

class IPlanner
{
public:
    IPlanner(IPolicy* policy);
    virtual ~IPlanner() {delete policy; policy = nullptr;}

    virtual ScenarioResult Plan(const Snapshot& snap, const Agents& src, const InformedHeuristic& ih, float timeout) = 0;
    virtual std::string GetName(void) const = 0;

    void LogMap(const Map&);
    void LogAgents(const Agents&);
    void LogSnapshot(const Snapshot&);
    inline void CloseLogFile(void) {log.close();}

    void InitLogFile(const std::string& directory_folder_path, const std::string& filename);

protected:
    IPolicy* policy;
    std::fstream log{};
    static constexpr auto log_path_coordinates_delimiter = " ";
    static constexpr auto log_grid_cell_delimiter = " ";
    static constexpr auto log_lines_delimiter = "\n";
    static constexpr auto log_steps_title = "#Steps";
    static constexpr auto log_plan_title = "#Plan";
    static constexpr auto log_Agents_title = "#Agents";
    static constexpr auto log_Maybe_Edeges_title = "#UncertainEdges";
    static constexpr auto log_Maybe_Opened_title = "Maybe opened edges (E_o?)";
    static constexpr auto log_Maybe_blocked_title = "Maybe blocked edges (E_b?)";
    static constexpr auto log_observation_prefix = "#observation";
    static constexpr auto log_Agent_prefix = "Agent";
    static constexpr auto log_observation_maybe_opened = "maybe opened";
    static constexpr auto log_observation_maybe_blocked = "maybe blocked";
    static constexpr auto log_file_suffix = "log";

    std::tuple<EdgeSet, EdgeSet> Observe(const Agents& as, const Graph& g, const Snapshot& snap, EdgeSet& observed_maybe_open_edge, EdgeSet& observed_maybe_blocked_edge) const;
    void Step(Agents& as, Paths& realized, Paths& planned) const;
    void UpdateGraph(Graph& g, const EdgeSet& new_observed_maybe_open_edge, const EdgeSet& new_observed_maybe_blocked_edge) const;
    Paths Prune(const Paths& ps) const;

    AgentsIndicesSet FindAffectedAgents(const Graph& g, const Paths& planned_paths, const EdgeSet& observed_maybe_open_edge, const EdgeSet& observed_maybe_blocked_edge, const InformedHeuristic& ih);
    AgentsIndicesSet FindAffectedByNewObservedBlockedEdge(const Paths& planned_paths, const EdgeSet& new_observed_maybe_open_edge);
    AgentsIndicesSet FindAffectedByNewObservedOpenEdge(const Graph& g, const Paths& planned_paths, const EdgeSet& new_observed_maybe_blocked_edge, const InformedHeuristic& ih);

    void LogPlan(const Paths& planned_paths);
    void LogStep(const Paths& realized_paths, int timestep);
    void LogObservations(const EdgeSet& new_observed_maybe_open_edges, const EdgeSet& new_observed_maybe_blocked_edges, int timestep, int agent_index = 0);
    std::string ObservationToString(const Edge& edge, bool is_maybe_open, int timestep, int agent_index);
    void LogMaybeEdges(const EdgeSet& maybe_edges, const std::string& title);
};