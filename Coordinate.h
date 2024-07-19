#pragma once

#include <iostream>

struct Coordinate
{
    int row = -1, column = -1;

    Coordinate() = default;
    Coordinate(int row, int column);
    Coordinate(const Coordinate&);

    bool operator == (const Coordinate&) const;
    bool operator != (const Coordinate&) const;
    operator bool() const {return row >= 0 && column >= 0;}
    Coordinate& operator = (const Coordinate&);
    Coordinate& operator = (Coordinate&&) noexcept ;
    std::string ToString(void) const;
    friend std::ostream& operator << (std::ostream&, const Coordinate&);

    struct Hasher{ size_t operator()(const Coordinate &) const noexcept; };
    struct Equal{ bool operator()(const Coordinate&, const Coordinate&) const noexcept; };
};