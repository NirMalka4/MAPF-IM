#include "SafeIntervals.h"
#include "Constants.h"
#include "TimeInterval.h"
#include "Types.h"
#include <sstream>
#include <cassert>
#include <utility>

SafeIntervals::SafeIntervals(const SafeIntervals& other): configurations(other.configurations){}

SafeIntervals::SafeIntervals(SafeIntervals&& other): configurations(std::forward<Configurations>(other.configurations)){}

SafeIntervals::SafeIntervals(const Paths& ps): configurations()
{
    for(const auto& p: ps)
    {
        if(!p.empty())
        {
            Add(p);
        }
    } 
}

SafeIntervals::SafeIntervals(const Constraints& cs, const int constrained_agent_index)
{
    for(const auto& c: cs)
    {
        if(c.constrained_agent == constrained_agent_index)
        {
            Add(c.c, c.timestep);
        }
    }
}

std::tuple<TimeInterval, TimeInterval> SafeIntervals::Add(const Coordinate& c, float collision_time)
{
    auto& intervals = _IntervalsOf(c);
    assert(!intervals.empty());

    // find first interaval contains collision time
    auto iter = intervals.begin();
    while(iter != intervals.end() && iter->end < collision_time)
    {
        ++iter;
    }
       
    if(iter != intervals.end() && iter->IsIntersects(collision_time))
    {
        float start = iter->start, end = iter->end;
        intervals.erase(iter);
        return _Add(intervals, start, collision_time, end);
    }

    return {TimeInterval::CreateEmptyInterval(), TimeInterval::CreateEmptyInterval()};
}

void SafeIntervals::Add(const Path& p)
{
    const int n = p.size();

    for(auto t = 0; t < n; t++)
    {
        Add(p[t], t); // avoid vertex conflict
        if(t > 0)
            Add(p[t - 1], t); // avoid edge conflict
    }

    if(n > 0)
    {
        // avoid target conflicts
        auto& intervals = _IntervalsOf(p.back());
        auto last_interval = *intervals.rbegin();
        last_interval.end = n; // allow traverse agent goal until he reached it
        intervals.erase(std::prev(intervals.end()));
        intervals.insert(last_interval);
    }
}

std::tuple<TimeInterval, TimeInterval> SafeIntervals::_Add(Intervals& intervals, float start, float collision_time, float end)
{
    TimeInterval left = (collision_time > start) ? TimeInterval{start, collision_time} : TimeInterval::CreateEmptyInterval();
    TimeInterval right = (collision_time + 1 < end) ? TimeInterval{collision_time + 1, end} : TimeInterval::CreateEmptyInterval();

    if(!left.IsEmpty())
        intervals.insert(left);
    if(!right.IsEmpty())
        intervals.insert(right);

    return {left, right};
}

TimeInterval SafeIntervals::FirstSafeInterval(const Coordinate& c, const float collision_time)
{
    const auto& intervals = IntervalsOf(c);

    auto iter = intervals.begin();
    while(iter != intervals.end() && iter->end < collision_time)
        ++iter;

    return iter != intervals.end() && iter->IsIntersects(collision_time) ? *iter : TimeInterval::CreateEmptyInterval();
}

const Intervals& SafeIntervals::IntervalsOf(const Coordinate& c)
{
    return _IntervalsOf(c);
}

Intervals& SafeIntervals::_IntervalsOf(const Coordinate& c)
{
    if(configurations.find(c) == configurations.end())
        configurations[c] = {};

    auto& intervals = configurations[c];
    if(intervals.empty())
        intervals.emplace(0, INF);

    return intervals;
}

std::string SafeIntervals::ToString(void) const
{
    std::stringstream ss;
    unsigned i = 1;

    for(const auto& [coordinate, intervals] : configurations)
    {
        ss << "Safe intervals for " << coordinate << '\n';
        for(const auto& interval : intervals)
            ss << '#' << i++ << "\t" << interval << '\n';
    }

    return ss.str(); 
}

SafeIntervals& SafeIntervals::operator = (SafeIntervals&& other)
{
    if(this != &other)
    {
        configurations = std::forward<Configurations>(other.configurations);
    }
    return *this;
}

SafeIntervals& SafeIntervals::operator = (const SafeIntervals& other)
{
    if(this != &other)
    {
        configurations =other.configurations;
    }
    return *this;
}

std::ostream& operator << (std::ostream& out, const SafeIntervals& si) noexcept
{
    return out << si.ToString();
}