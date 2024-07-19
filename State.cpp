#include "State.h"
#include <boost/unordered_map.hpp>

State::State(const Coordinate& c): c(c), i(){}

State::State(const State& other): c(other.c), i(other.i){}

State::State(const Coordinate& c, const TimeInterval& i): c(c), i(i){}

bool State::operator == (const State& other) const noexcept
{
    return (c == other.c) && (i == other.i);
}

bool State::operator != (const State& other) const noexcept
{
    return (c != other.c) || (i != other.i);
}

bool State::operator < (const State& other) const noexcept
{
    return i < other.i;
}

State& State::operator = (const State& other)
{
    if(this != &other)
    {
        c = other.c;
        i = other.i;
    }
    return *this;
}

std::string State::ToString(void) const
{
    return "<" + c.ToString() + ", " + i.ToString() + ">";
}

std::ostream& operator << (std::ostream& out, const State& s)
{
    return out << s.ToString();
}

size_t State::Hasher::operator()(const State &s) const noexcept
{
    size_t seed = 0;
    Coordinate::Hasher coordinate_hasher;
    boost::hash_combine(seed, coordinate_hasher(s.c));
    boost::hash_combine(seed, s.i.start);
    boost::hash_combine(seed, s.i.end);
    return seed;
}

bool State::Equal::operator()(const State &s1, const State &s2) const noexcept
{
    return s1 == s2;
}