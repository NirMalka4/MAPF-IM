#include "Timer.h"

void Timer::Start(const double t)
{
    runtime_in_seconds = t;
    start = Clock::now();
}

void Timer::Start(void)
{
    start = Clock::now();
}

double Timer::Stop(void)
{
    return GetRuntime();
}

bool Timer::ExceedsRuntime(void) const
{
    return GetRuntime() > runtime_in_seconds;
}

double Timer::GetRuntime(void) const
{
    std::chrono::duration<double> runtime = Clock::now() - start;
    return runtime.count();
}

double Timer::GetRemainingRuntime(void) const
{
    return runtime_in_seconds - GetRuntime();
}