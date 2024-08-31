#pragma once

#include <chrono>

struct Timer
{
    using Clock =  std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    
    double runtime_in_seconds = 0.000;
    TimePoint start;

    Timer() = default;
    bool ExceedsRuntime(void) const;
    double GetRuntime(void) const;
    double GetRemainingRuntime(void) const;
    void Start(double runtime_in_seconds);
    void Start(void);
    double Stop(void);
};