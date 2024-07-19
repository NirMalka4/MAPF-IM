#pragma once

#include "TimeInterval.h"
#include "Types.h"
#include "Coordinate.h"
#include <string>
#include <boost/unordered_map.hpp>

class SafeIntervals
{
public:
    SafeIntervals() = default;
    SafeIntervals(const SafeIntervals& other);
    SafeIntervals(SafeIntervals&& other);
    SafeIntervals(const Paths& ps);
    SafeIntervals(const Constraints& cs, int constrained_agent_index);
    virtual ~SafeIntervals() = default;

    std::tuple<TimeInterval, TimeInterval> Add(const Coordinate& c, float collision_time);
    void Add(const Path& p);
    const Intervals& IntervalsOf(const Coordinate&);
    TimeInterval FirstSafeInterval(const Coordinate& c, float collision_time);
    std::string ToString(void) const;
    
    friend std::ostream& operator << (std::ostream&, const SafeIntervals&) noexcept;
    SafeIntervals& operator = (SafeIntervals&& other);
    SafeIntervals& operator = (const SafeIntervals& other);
    
protected:
    Configurations configurations;

    Intervals& _IntervalsOf(const Coordinate&);
    std::tuple<TimeInterval, TimeInterval>  _Add(Intervals& intervals, float start, float collision_time, float end);
};