#include "Coordinate.h"
#include <boost/unordered_map.hpp>

Coordinate::Coordinate(const int row, const int column): row(row), column(column){}
Coordinate::Coordinate(const Coordinate& other): row(other.row), column(other.column){}

bool Coordinate::operator == (const Coordinate &other) const
{
    return row == other.row && column == other.column;
}

bool Coordinate::operator != (const Coordinate &other) const
{
    return !(*this == other);
}

std::string Coordinate::ToString(void) const
{
    return "(" + std::to_string(row) + ", " + std::to_string(column) + ")";
}

std::ostream& operator << (std::ostream& out,const Coordinate& coordinate)
{
    return out << coordinate.ToString();
}

Coordinate& Coordinate::operator=(const Coordinate& other)
{
    if(this != &other)
    {
        row = other.row;
        column = other.column;
    }
    return *this;
}

Coordinate& Coordinate::operator=(Coordinate&& other)
noexcept {
    if(this != &other)
    {
        row = std::forward<int>(other.row);
        column = std::forward<int>(other.column);
    }
    return *this;
}

size_t Coordinate::Hasher::operator()(const Coordinate& coordinate) const noexcept
{
    std::size_t seed = 0;
    boost::hash_combine(seed, coordinate.row);
    boost::hash_combine(seed, coordinate.column);
    return seed;
}

bool Coordinate::Equal::operator()(const Coordinate &c1, const Coordinate &c2) const noexcept
{
    return c1 == c2;
}