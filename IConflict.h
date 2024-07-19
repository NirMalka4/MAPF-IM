#pragma once

#include "Constraint.h"
#include <vector>
#include <string>

struct IConflict
{
    std::vector<int> agents_indices;
    int timestep;

    IConflict(const std::vector<int>& agents_indices, int timestep);
    virtual ~IConflict() = default;

    virtual std::string ToString(void) const = 0;
    virtual std::vector<Constraint> Resolve(void) const = 0;

    friend std::ostream& operator << (std::ostream& out, const IConflict* conflict);
};