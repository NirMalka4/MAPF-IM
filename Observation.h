#pragma once

#include "Edge.h"
#include <string.h>

struct Observation
{
    Edge e;
    int timestep = -1;
    int agent_index = -1;
    
    std::string ToString(void) const;

    friend std::ostream& operator << (std::ostream& out, const Observation&);
    struct Hasher{size_t operator()(const Observation&) const noexcept;};
    struct Equal{bool operator()(const Observation&, const Observation&) const noexcept;};
};