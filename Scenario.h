#pragma once

#include "Types.h"
#include <string>
class Map;

class Scenario
{
public:
    Scenario() = default;
    Scenario(const std::string& scenario_file_path);
    
    static std::vector<Scenario> LoadAllScenarios(const std::string& scenarios_directory_path);
    static Agents LoadAllValidAgents(const std::string& scenarios_directory_path, const Map& m);

    inline std::string ToString(void) const {return name;};
    inline friend std::ostream& operator << (std::ostream& out, const Scenario& s) {return out << s.ToString();}
    Scenario& operator += (const Scenario& other);
    inline const Agents& GetAgents(void) const {return as;}
    inline const std::string& GetName(void) {return name;}

private:
    std::string name;
    Agents as;

    Agents ParseScenarioVersionZero(std::ifstream& scenario_file);
    Agents ParseScenarioVersionOne(std::ifstream& scenario_file);
    float ParseVersion(std::ifstream &scenario_file);
    bool IsVersionSupported(float version);  
};