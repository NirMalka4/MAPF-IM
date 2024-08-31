#include "Constraint.h"
#include <boost/unordered_map.hpp>
#include <string>

Constraint::Constraint(int constrained_agent, const Coordinate& c, int timestep): constrained_agent(constrained_agent), c(c), timestep(timestep){}

Constraint::Constraint(int constrained_agent, int conflicted_agent, const Coordinate& c, int timestep): 
constrained_agent(constrained_agent), conflicted_agent(conflicted_agent), c(c), timestep(timestep){}

std::string Constraint::ToString(void) const
{
    return "Constraint on Agent" + std::to_string(constrained_agent) + ", on: " + c.ToString() + ", at: " + std::to_string(timestep) + ". Conflicted with Agent" + std::to_string(conflicted_agent);
}

std::ostream& operator << (std::ostream& out, const Constraint& c)
{
    return out << c.ToString();
}

bool Constraint::operator == (const Constraint &other) const noexcept
{
    return constrained_agent == other.constrained_agent && c == other.c && timestep == other.timestep;
}

size_t Constraint::Hasher::operator()(const Constraint& c) const noexcept
{
    Coordinate::Hasher coordinate_hasher;
    std::size_t seed = 0;
    boost::hash_combine(seed, c.constrained_agent);
    boost::hash_combine(seed, coordinate_hasher(c.c));
    boost::hash_combine(seed, c.timestep);
    return seed;
}

bool Constraint::Equal::operator()(const Constraint& c1, const Constraint& c2) const noexcept
{
    return c1 == c2;
}