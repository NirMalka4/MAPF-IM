#include "Agent.h"
#include "Coordinate.h"
#include <boost/unordered/unordered_map.hpp>

Agent::Agent(Coordinate&& start, Coordinate&& goal, const int index):
start(std::move(start)), goal(std::move(goal)), index(index){}

Agent::Agent(const Coordinate& start, const Coordinate& goal, const int index):
start(start), goal(goal), index(index){}

Agent::Agent(const Agent& other): start(other.start), goal(other.goal), index(other.index){}

std::string Agent::ToString(void) const
{
    return "Agent" + std::to_string(index) + ": start " + start.ToString() + " goal " + goal.ToString();
}

std::ostream& operator << (std::ostream& out, const Agent& agent)
{
    return out << agent.ToString();
}

bool Agent::operator==(const Agent &other) const noexcept
{
    return start == other.start && goal == other.goal;
}

bool Agent::operator < (const Agent& other) const noexcept
{
    return index < other.index;
}

Agent& Agent::operator = (const Agent &other)
{
    if(this != &other)
    {
        start = other.start;
        goal = other.goal;
        index = other.index;
    }
    return *this;
}