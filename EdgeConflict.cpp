#include "EdgeConflict.h"
#include <sstream>
#include <vector>
#include <cassert>

EdgeConflict::EdgeConflict(const std::vector<int>& agents_indices, const int timestep, const Edge& e1, const Edge& e2): IConflict(agents_indices, timestep), e1(e1), e2(e2){}

std::string EdgeConflict::ToString(void) const
{
    std::stringstream ss;
    ss << "Edge-Conflict between: " << "Agent" << agents_indices[0] << ':' << e1 << ", Agent" << agents_indices[1] << ':' << e2 << ", at " << timestep << '.';
    return ss.str();
}

std::vector<Constraint> EdgeConflict::Resolve(void) const
{
    assert(agents_indices.size() == 2);
    return {{agents_indices[0], agents_indices[1], e1.destination, timestep}, {agents_indices[1], agents_indices[0], e2.destination, timestep}};
}