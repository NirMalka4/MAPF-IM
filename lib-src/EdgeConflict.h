#pragma once

#include "IConflict.h"
#include "Edge.h"

struct EdgeConflict: public IConflict
{
    Edge e1, e2;

    EdgeConflict(const std::vector<int>& agents_indices, const int timestep, const Edge& e1, const Edge& e2);
    virtual ~EdgeConflict() = default;
    
    std::string ToString(void) const override;
    std::vector<Constraint> Resolve(void) const override;
};