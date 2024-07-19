#include "VertexConflict.h"
#include "Constraint.h"
#include <sstream>
#include <cassert>

VertexConflict::VertexConflict(const std::vector<int>& agents_indices, const int timestep, const Coordinate& c): IConflict(agents_indices, timestep), c(c){}


std::string VertexConflict::ToString(void) const
{
    std::stringstream ss;

    ss << "Vertex-Conflict on: " << c << ", at: " << timestep << ". Agents: ";
    for(const auto& i: agents_indices)
        ss << i << ' ';

    return ss.str();
}

std::vector<Constraint> VertexConflict::Resolve(void) const
{
    assert(agents_indices.size() >= 2);
    return {{agents_indices[0], agents_indices[1], c, timestep}, {agents_indices[1], agents_indices[0], c, timestep}};
}