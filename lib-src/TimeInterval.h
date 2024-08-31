#pragma once

#include <iostream>
#include <string>
#include "Constants.h"

struct TimeInterval
{
    float start = 0;
    float end = INF;

    TimeInterval();
    TimeInterval(float start, float end);
    TimeInterval(const TimeInterval& other);
    virtual ~TimeInterval() = default;
    static TimeInterval CreateEmptyInterval(void);

    bool IsIntersects(const float time) const;
    bool IsUnbounded(void) const;
    bool IsEmpty(void) const;
    void SetEmpty(void);
    std::string ToString(void) const;
    
    bool operator < (const TimeInterval& other) const noexcept;
    bool operator == (const TimeInterval& other) const noexcept;
    bool operator != (const TimeInterval& other) const noexcept;
    TimeInterval& operator = (const TimeInterval& other);
    friend std::ostream& operator << (std::ostream& out, const TimeInterval& ti);

    struct Comparator{ bool operator () (const TimeInterval& t1, const TimeInterval& t2) const noexcept;};
};