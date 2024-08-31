#include "../lib-src/IPlanner.h"
#include "../lib-src/LocalIDPlanner.h"
#include "../lib-src/Printer.h"
#include "../lib-src/Map.h"
#include "../lib-src/PP.h"
#include "../lib-src/FullPlanner.h"
#include "../lib-src/SIPP.h"
#include "../lib-src/EES-SIPP.h"
#include "../lib-src/CBS.h"
#include "../lib-src/FullIDPlanner.h"
#include "../lib-src/Scenario.h"
#include <cstdlib>
#include <string>

IPlanner* CreatePlanner(const std::string& framework_name, const std::string& high_level_planner_name, const std::string& low_level_planner_name, const std::string& policy_name);
IPlanner* CreateFramework(const std::string& framework_name, IHighLevelPlanner* high_level_planner, IPolicy* policy);
IHighLevelPlanner* CreateHighLevelPlanner(const std::string& high_level_planner_name, ILowLevelPlanner* low_level_planner);
ILowLevelPlanner* CreateLowLevelPlanner(const std::string& low_level_planner_name);
IPolicy* CreatePolicy(const std::string& policy_name);
Agents Sample(const Agents& all_valid_agents, const int nsamples);


int main(int argc, const char* argv[])
{
    if(argc != 12)
    {
        Print(Red, "Expected 11 arguments in the following order: map_file_path, scenario_file_path, output_directory_path, number_of_agents, framework_name, high_level_planner_name, low_level_planner_name, policy_name, runtime, visualize_path, print_path. Given: ", argc, " arguments", '\n');
        exit(EXIT_FAILURE);
    }

    const std::string map_file_path(argv[1]), scenario_file_path(argv[2]), output_directory_path(argv[3]);
    const int number_of_agents = atoi(argv[4]);

    Map m(map_file_path);
    Scenario s(scenario_file_path);
    IPlanner* planner = CreatePlanner(argv[5], argv[6], argv[7], argv[8]);
    const float timeout = atoi(argv[9]);
    const int visualize_path = atoi(argv[10]);
    const int print_path = atoi(argv[11]);

    // retrieve agents that has path in the true graph
    Agents agents = Validator::ValidAgents(s.GetAgents(), m);
    Agents agents_subset(agents.begin(), std::next(agents.begin(), number_of_agents));
    for(int i = 0; i < number_of_agents; i += 1)
    {
        agents_subset[i].index = i;
    }

    Snapshot snap = m.CreateSnapshot();
    InformedHeuristic ih(snap.Create(true, true), agents_subset, snap.GetMaybeBlockedEdges());
    const std::string filename = m.GetName() + '_' + s.GetName() + '_' + planner->GetName() + '_' + "number_of_agents=" + std::to_string(number_of_agents) + '_' + "number_of_uncertain_edges=" + std::to_string(m.GetNumberOfUncertiandEdge()) + ".log";

    planner->InitLogFile(output_directory_path, filename);
    planner->LogMap(m);
    planner->LogAgents(agents_subset);
    planner->LogSnapshot(snap);

    const auto& [is_planning_succeed, paths, runtime, replans, nexpansions] = planner->Plan(snap, agents_subset, ih, timeout);
    bool is_legal_plan = !is_planning_succeed || Validator::IsLegalPlan(snap, paths, agents_subset);
    const auto solution_cost = is_planning_succeed ? ObjectiveFunction::SumOfCost(paths) : -1;

    Print(Default, "Planner: ", planner->GetName(), '\n', "Map: ", m.GetName(), '\n', "Scenario: ", s.ToString(), '\n', "IsLegal: ", (is_legal_plan ? "True" : "False"), '\n',  "K: ", number_of_agents, \
    '\n', "|Eo?|: ", snap.GetNumberOfMaybeOpenEdge(), '\n', "|Eb?|: ", snap.GetNumberOfMaybeBlockedEdge(), '\n', \
    "SOC: ", solution_cost, '\n', "#Replans: ", replans, '\n', "#Expansions: ", nexpansions, '\n', "Runtime: ", runtime, '\n', '\n');

    if(is_planning_succeed && is_legal_plan && visualize_path)
    {
        for(int i = 0; i < number_of_agents; i += 1)
        {
            Print(Default, "Visualize path of ", agents_subset[i], '\n');
            std::cout << m.ToString(paths[i]) << '\n';

            if(print_path)
            {
                Print(paths[i]);
            }
        }
    }

    planner->CloseLogFile();
    delete planner;

    exit(EXIT_SUCCESS);
}

IPlanner* CreatePlanner(const std::string& framework_name, const std::string& high_level_planner_name, const std::string& low_level_planner_name, const std::string& policy_name)
{
    IPolicy* policy = CreatePolicy(policy_name);
    ILowLevelPlanner* low_level_planner = CreateLowLevelPlanner(low_level_planner_name);
    IHighLevelPlanner* high_level_planner = CreateHighLevelPlanner(high_level_planner_name, low_level_planner);
    return CreateFramework(framework_name, high_level_planner, policy);
}

IPlanner* CreateFramework(const std::string& framework_name, IHighLevelPlanner* high_level_planner, IPolicy* policy)
{
    std::unordered_map<std::string, std::function<IPlanner*(IHighLevelPlanner* high_level_planner, IPolicy* policy)>> frameworkMap = {
        {"full_planner", [](IHighLevelPlanner* high_level_planner, IPolicy* policy){return new FullPlanner(policy, high_level_planner);}},
        {"full_id_planner", [](IHighLevelPlanner* high_level_planner, IPolicy* policy){return new FullIDPlanner(policy, high_level_planner);}},
        {"local_planner", [](IHighLevelPlanner* high_level_planner, IPolicy* policy){return new LocalPlanner(policy, high_level_planner);}},
        {"local_id_planner", [](IHighLevelPlanner* high_level_planner, IPolicy* policy){return new LocalIDPlanner(policy, high_level_planner);}}
    };

    auto it = frameworkMap.find(framework_name);
    if (it != frameworkMap.end())
    {
        return it->second(high_level_planner, policy);
    }
    else
    {
        throw std::invalid_argument("Invalid framework_name: " + framework_name);
    }
}


IHighLevelPlanner* CreateHighLevelPlanner(const std::string& high_level_planner_name, ILowLevelPlanner* low_level_planner)
{
    std::unordered_map<std::string, std::function<IHighLevelPlanner*(ILowLevelPlanner* low_level_planner)>> highLevelPlannerMap = {
        {"pp", [](ILowLevelPlanner* low_level_planner) {return new PP(low_level_planner);}},
        {"cbs", [](ILowLevelPlanner* low_level_planner) {return new CBS(low_level_planner);}}
    };

    auto it = highLevelPlannerMap.find(high_level_planner_name);
    if (it != highLevelPlannerMap.end())
    {
        return it->second(low_level_planner);
    }
    else
    {
        throw std::invalid_argument("Invalid high_level_planner_name: " + high_level_planner_name);
    } 
}

ILowLevelPlanner* CreateLowLevelPlanner(const std::string& low_level_planner_name)
{
    std::unordered_map<std::string, std::function<ILowLevelPlanner*(void)>> lowLevelPlannerMap = {
        {"sipp", [](){return new SIPP();}},
        {"ees_sipp", [](){return new EESSIPP();}}
    };

    auto it = lowLevelPlannerMap.find(low_level_planner_name);
    if (it != lowLevelPlannerMap.end())
    {
        return it->second();
    }
    else
    {
        throw std::invalid_argument("Invalid low_level_planner_name: " + low_level_planner_name);
    } 
}

IPolicy* CreatePolicy(const std::string& policy_name)
{
    std::unordered_map<std::string, std::function<IPolicy*(void)>> policyMap = {
        {"risk_averse", [](){return new RiskAversePolicy();}},
        {"explorative", [](){return new ExplorativePolicy();}},
        {"hybrid", [](){return new HybridPolicy();}},
        {"baseline", [](){return new BaselinePolicy();}}
    };

    auto it = policyMap.find(policy_name);
    if (it != policyMap.end())
    {
        return it->second();
    } 
    else
    {
        throw std::invalid_argument("Invalid policy_name: " + policy_name);
    }
}

Agents Sample(const Agents& all_valid_agents, const int nsamples)
{
    Agents sampled;
    std::sample(all_valid_agents.begin(), all_valid_agents.end(), std::back_inserter(sampled), nsamples, std::mt19937 {std::random_device{}()});

    for(int i = 0; i < nsamples; i++)
    {
        sampled[i].index = i;
    }
        
    return sampled;            
}