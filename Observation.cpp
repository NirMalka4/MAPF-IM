#include "Observation.h"
#include <string>

size_t Observation::Hasher::operator() (const Observation& o) const noexcept
{
    Edge::Hasher h;
    return h(o.e);
}

bool Observation::Equal::operator() (const Observation& o1, const Observation& o2) const noexcept
{
    return o1.e == o2.e;
}

std::string Observation::ToString(void) const
{
    return "Edge " + e.ToString() + " is observed by " + std::to_string(agent_index) + " at " + std::to_string(timestep);
}

std::ostream& operator << (std::ostream& out, const Observation& o)
{
    return out << o.ToString();
}