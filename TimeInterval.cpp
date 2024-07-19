#include "TimeInterval.h"
#include "Constants.h"

TimeInterval::TimeInterval(): start(0), end(INF){}

TimeInterval::TimeInterval(float start, float end): start(start), end(end){}

TimeInterval::TimeInterval(const TimeInterval& other): start(other.start), end(other.end){}

TimeInterval TimeInterval::CreateEmptyInterval(void)
{
    return {INF, INF};
}

bool TimeInterval::IsIntersects(const float time) const 
{
    return time >= start && time < end;
}

bool TimeInterval::IsUnbounded(void) const 
{
    return start < INF && end == INF;
}

bool TimeInterval::IsEmpty(void) const 
{
    return start == end;
}

void TimeInterval::SetEmpty(void) 
{
    start = INF;
    end = INF;
}

std::string TimeInterval::ToString(void) const 
{
    return "[" + std::to_string(start) + ", " + std::to_string(end) + ")";
}


bool TimeInterval::operator < (const TimeInterval& other) const noexcept 
{
    return start < other.start;
}

bool TimeInterval::operator == (const TimeInterval& other) const noexcept 
{
    return start == other.start && end == other.end;
}

bool TimeInterval::operator != (const TimeInterval& other) const noexcept 
{
    return start != other.start || end != other.end;
}

std::ostream& operator << (std::ostream& out, const TimeInterval& ti)
{
    return out << ti.ToString();
}

TimeInterval& TimeInterval::operator = (const TimeInterval& other)
{
    if(this != &other)
    {
        start = other.start;
        end = other.end;
    }
    return *this;
}

bool TimeInterval::Comparator::operator () (const TimeInterval& t1, const TimeInterval& t2) const noexcept 
{
    return t1 < t2;
};