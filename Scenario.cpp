#include "Scenario.h"
#include "Printer.h"
#include "Map.h"
#include <algorithm>
#include <fstream>
#include <filesystem>

Scenario::Scenario(const std::string& scenario_file_path): name(ExtractFileName<std::string>(scenario_file_path))
{
    std::ifstream scenario_file(scenario_file_path, std::ios::in);

    if(!scenario_file.is_open())
    {
        Print(Red, "Invalid path for scenario file. Given: ", scenario_file_path, "PWD :", std::filesystem::current_path(), '\n');
        exit(EXIT_FAILURE);
    }

    const auto version = ParseVersion(scenario_file);
    as = version == 1.0 ? ParseScenarioVersionOne(scenario_file) : ParseScenarioVersionZero(scenario_file);
}

bool Scenario::IsVersionSupported(const float version)
{
    return version == 0.0 || version == 1.0;
}

float Scenario::ParseVersion(std::ifstream &scenario_file)
{
    float version;
    std::string first;
    scenario_file >> first;
    if (first != "version")
    {
        version = 0.0;
        scenario_file.seekg(0,std::ios::beg);
    }
    else
    {
        scenario_file >> version;
    }
    if(!IsVersionSupported(version))
    {
        Print(Red, "Invalid scenario version number. Given:", version, ". Expected either 1.0 or 0.0\n");
        exit(EXIT_FAILURE);
    }
    return version;
}

Agents Scenario::ParseScenarioVersionZero(std::ifstream& scenario_file)
{
    Agents agents;
    // version 0.0 row format:
    // Bucket  map name start x-coordinate  start y-coordinate  goal x-coordinate  goal y-coordinate  optimal length
    int bucket, start_x_coordinate, start_y_coordinate, goal_x_coordinate, goal_y_coordinate;
    double optimal_length;
    std::string map_name;

    while(scenario_file>>bucket>>map_name>>start_x_coordinate>>start_y_coordinate>>goal_x_coordinate>>goal_y_coordinate>>optimal_length)
        agents.emplace_back(Coordinate(start_x_coordinate, start_y_coordinate), Coordinate(goal_x_coordinate, goal_y_coordinate), agents.size());
    
    return agents;
}

Agents Scenario::ParseScenarioVersionOne(std::ifstream& scenario_file)
{
    Agents agents;
    // version 1.0 row format: 
    // Bucket  map name  map width  map height  start x-coordinate  start y-coordinate  goal x-coordinate  goal y-coordinate  optimal length
    int bucket, map_width, map_height, start_x_coordinate, start_y_coordinate, goal_x_coordinate, goal_y_coordinate;
    double optimal_length;
    std::string map_name;

    while(scenario_file>>bucket>>map_name>>map_width>>map_height>>start_x_coordinate>>start_y_coordinate>>goal_x_coordinate>>goal_y_coordinate>>optimal_length)
        agents.emplace_back(Coordinate(start_x_coordinate, start_y_coordinate), Coordinate(goal_x_coordinate, goal_y_coordinate), agents.size());
    
    return agents;
}

Scenario& Scenario::operator += (const Scenario& other)
{
    if(this != &other)
    {
        as.insert(as.end(), other.as.begin(), other.as.end());
        name += (", " + other.name);
    }
    return *this;
}

std::vector<Scenario> Scenario::LoadAllScenarios(const std::string& scenarios_directory_path)
{
    std::vector<Scenario> scenarios;

    for(const auto& dir_enrty: std::filesystem::directory_iterator(scenarios_directory_path))
        scenarios.push_back(Scenario(dir_enrty.path()));

    return scenarios;
}

Agents Scenario::LoadAllValidAgents(const std::string& scenarios_directory_path, const Map& m)
{
    auto all_scenarios = LoadAllScenarios(scenarios_directory_path);
    Agents all_valid_agents;

    for(const auto& s: all_scenarios)
    {
        const auto& scenario_valid_agents = Validator::ValidAgents(s.GetAgents(), m);
        
        for(const auto& a: scenario_valid_agents)
        {
            if(std::none_of(all_valid_agents.begin(), all_valid_agents.end(), [&a](const auto& b){return b.start == a.start || b.goal == a.goal;}))
            {
                all_valid_agents.push_back(a);
            }
        }
    }
    
    for(int i = 0; (int)i < all_valid_agents.size(); i++)
    {
        all_valid_agents[i].index = i;
    }

    return all_valid_agents;
}