#pragma once

#include "TimeInterval.h"
#include "Coordinate.h"
#include <string>

struct State
{
    Coordinate c;
    TimeInterval i;

    State() = default;
    State(const State& other);
    State(const Coordinate& c);
    State(const Coordinate& c, const TimeInterval& i);
    virtual ~State() = default;

    bool IsUnbounded(void) const;
    std::string ToString(void) const;

    bool operator == (const State& other) const noexcept;
    bool operator != (const State& other) const noexcept;
    bool operator < (const State& other) const noexcept;
    State& operator = (const State& other);
    friend std::ostream& operator << (std::ostream&, const State&);
    
    struct Hasher{ size_t operator() (const State& s) const noexcept; };
    struct Equal{ bool operator() (const State& s1, const State& s2) const noexcept; };
};