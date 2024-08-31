#pragma once

#include "Coordinate.h"
#include <string>

struct Agent
{
    Coordinate start, goal;
    int index = -1;

    Agent() = default;
    Agent(Coordinate&& start, Coordinate&& goal, int index = 0);
    Agent(const Coordinate& start, const Coordinate& goal, int index = 0);
    Agent(const Agent&);
    
    Agent& operator = (const Agent& other);
    bool operator == (const Agent&) const noexcept;
    bool operator < (const Agent&) const noexcept;
    std::string ToString(void) const;
    friend std::ostream& operator << (std::ostream&, const Agent&);

    static inline bool IsPlaceholderAgent(const Agent& a){return a.start == Coordinate() || a.goal == Coordinate();}
};

