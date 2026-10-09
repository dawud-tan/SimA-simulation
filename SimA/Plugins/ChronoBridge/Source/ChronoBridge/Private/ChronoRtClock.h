// ============================================================================
//  ChronoRtClock.h — absolute-deadline timing helpers shared by the physics
//  and wheel threads.
//
//  These started as a private anonymous namespace in each .cpp. That works
//  under a normal build and breaks under Unreal's UNITY BUILD, which
//  concatenates a module's translation units into one file to cut compile time:
//  two anonymous namespaces then merge into a single one and the identical
//  helpers collide. Sharing one header is the fix, and is what the duplication
//  was asking for anyway.
// ============================================================================
#pragma once

#include <time.h>

namespace ChronoRt
{
    /** Advance an absolute deadline. Sleeping for a DURATION drifts, because
     *  each wakeup is late by some epsilon and the epsilons accumulate.
     *  Advancing an absolute deadline by exactly one period keeps the loop
     *  phase-locked to the clock however late any individual wakeup was. */
    inline void AddNs(timespec& T, long Ns)
    {
        T.tv_nsec += Ns;
        while (T.tv_nsec >= 1000000000L) { T.tv_nsec -= 1000000000L; ++T.tv_sec; }
    }

    inline double DiffUs(const timespec& A, const timespec& B)
    {
        return double(A.tv_sec - B.tv_sec) * 1e6 + double(A.tv_nsec - B.tv_nsec) / 1e3;
    }

    /** Exponential moving average; cheap and allocation-free. */
    inline double Ema(double Prev, double Sample, double Alpha = 0.001)
    {
        return Prev <= 0.0 ? Sample : Prev + Alpha * (Sample - Prev);
    }

    /** Sleep to (Deadline - SpinMarginNs), then busy-wait to the deadline.
     *
     *  A plain sleep lets the core enter a deep C-state, and on this CPU that
     *  exit latency IS the lateness: C6 costs 85 us and C7s 124 us, against a
     *  measured mean lateness of 85.8 us under a plain sleep. Spinning the last
     *  stretch holds the core in C0/POLL and takes the mean to 0.1 us.
     *  Costs about 20% of one core out of eight. Pass 0 to disable. */
    template <typename TStopPredicate>
    inline void SleepUntil(const timespec& Deadline, long SpinMarginNs, TStopPredicate&& ShouldStop)
    {
        timespec Early = Deadline;
        Early.tv_nsec -= SpinMarginNs;
        if (Early.tv_nsec < 0) { Early.tv_nsec += 1000000000L; --Early.tv_sec; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &Early, nullptr);

        timespec Now;
        do
        {
            clock_gettime(CLOCK_MONOTONIC, &Now);
            // On x86 this emits PAUSE: lowers power draw in a spin-wait and
            // avoids a memory-order-violation pipeline flush on loop exit.
            __builtin_ia32_pause();
        }
        while (DiffUs(Now, Deadline) < 0.0 && !ShouldStop());
    }
}
