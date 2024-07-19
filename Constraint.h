#pragma once

#include "Coordinate.h"
#include <string>

struct Constraint
{
    int constrained_agent = -1;
    int conflicted_agent = -1;
    Coordinate c{};
    int timestep = -1;

    Constraint(int constrained_agent, const Coordinate& c, int timestep);
    Constraint(int constrained_agent, int conflicted_agent, const Coordinate& c, int timestep);

    std::string ToString(void) const;

    friend std::ostream& operator << (std::ostream& out, const Constraint&);
    bool operator == (const Constraint&) const noexcept;

    struct Hasher{size_t operator()(const Constraint& constraint) const noexcept;};
    struct Equal{bool operator()(const Constraint& c1, const Constraint& c2) const noexcept;};
};