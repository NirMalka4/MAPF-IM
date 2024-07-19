#pragma once

#include "IConflict.h"
#include "Coordinate.h"

struct VertexConflict: public IConflict
{
    const Coordinate c;

    VertexConflict(const std::vector<int>& agents_indices, int timestep, const Coordinate& c);
    virtual ~VertexConflict() = default;

    virtual std::string ToString(void) const override;
    std::vector<Constraint> Resolve(void) const override;
};