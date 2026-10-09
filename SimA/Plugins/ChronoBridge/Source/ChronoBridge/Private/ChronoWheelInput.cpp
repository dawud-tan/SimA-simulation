#include "ChronoWheelInput.h"

#include "wheel_device.h"
#include "ChronoRtClock.h"

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <cstring>
#include <cmath>

DEFINE_LOG_CATEGORY_STATIC(LogChronoWheel, Log, All);

namespace
{
    // The G923's mechanical limit. The Innova needs more: 34.6 deg of road-wheel
    // lock times a 17.5 steering ratio is 605 deg per side, 1211 deg
    // lock-to-lock, or 3.4 turns. The wheel gives 2.5.
    //
    // The axis is normalised against its own reported range, so device full
    // lock always maps to car full lock whatever this is set to. What the
    // shortfall actually costs is the RATIO: the driver reaches full lock after
    // 450 deg of hand movement instead of 605, so the rack is 1.35x quicker
    // than the real car's.
    //
    // That is the right trade for this test. Parallel parking and the tightest
    // slalom gates need FULL LOCK to be reachable; a compressed hand-over-hand
    // feel is a fidelity loss, an unreachable steering lock is a broken
    // exercise. It is a hardware limit, not a tuning choice -- 1211 deg cannot
    // be had from a 900 deg wheel.
    constexpr int    kDeviceRangeDeg = 900;

    // G923 peak output. Used only to scale into the device's 16-bit range; the
    // soft clip in wheel_ffb_set does the rest.
    constexpr double kPeakTorqueNm = 2.3;

    using ChronoRt::AddNs;
    using ChronoRt::DiffUs;
    using ChronoRt::Ema;
}

FChronoWheelThread::FChronoWheelThread() {}
FChronoWheelThread::~FChronoWheelThread() { Shutdown(); }

void FChronoWheelThread::Launch()
{
    if (Thread) { return; }
    Thread = FRunnableThread::Create(this, TEXT("ChronoWheel"), 128 * 1024, TPri_TimeCritical);
}

void FChronoWheelThread::Shutdown()
{
    if (!Thread) { return; }
    Stop();
    Thread->WaitForCompletion();
    delete Thread;
    Thread = nullptr;
}

bool FChronoWheelThread::Init()
{
    // Priority 79: one BELOW the physics thread. If both are runnable on the
    // same core the simulation must go first -- a late torque update is a small
    // fidelity loss, a late physics step is a missed deadline.
    {
        sched_param Sp{};
        Sp.sched_priority = 79;
        const int Rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &Sp);
        bRealtimeGranted.store(Rc == 0, std::memory_order_relaxed);
    }

    char Err[256] = {};
    if (wheel_open(&Device, nullptr, Err, sizeof Err) != 0)
    {
        // NOT a failure. Returning false here would skip Run() and leave the
        // sim with no input path at all; a keyboard fallback is far better than
        // a dead simulator, and the wheel may be plugged in later.
        UE_LOG(LogChronoWheel, Warning,
               TEXT("No steering wheel: %hs -- continuing without one."), Err);
        Device = nullptr;
        return true;
    }

    const WheelCaps* C = wheel_caps(Device);
    bConnected.store(true, std::memory_order_relaxed);

    UE_LOG(LogChronoWheel, Log,
           TEXT("Wheel '%hs' at %hs (%04x:%04x), %hs."),
           C->name, C->path, C->vendor, C->product,
           C->writable ? "read-write" : "READ-ONLY, no force feedback");
    UE_LOG(LogChronoWheel, Log,
           TEXT("  axes: steer=%d throttle=%d brake=%d clutch=%d   gates=%d/7"),
           C->steer.code, C->throttle.code, C->brake.code, C->clutch.code, C->gate_count);
    UE_LOG(LogChronoWheel, Log,
           TEXT("  ff: %hs%hs%hs%hs slots=%d"),
           C->ff_constant ? "CONSTANT " : "", C->ff_spring ? "SPRING " : "",
           C->ff_damper ? "DAMPER " : "", C->ff_autocenter ? "AUTOCENTER " : "",
           C->ff_slots);

    if (wheel_set_range(Device, kDeviceRangeDeg) == 0)
    {
        UE_LOG(LogChronoWheel, Log, TEXT("  rotation range set to %d deg."), kDeviceRangeDeg);
    }
    else
    {
        // hid-logitech-hidpp does not expose the lg4ff `range` sysfs attribute.
        // Not fatal: the axis is normalised against whatever range the device
        // reports, so full lock still maps to full lock.
        UE_LOG(LogChronoWheel, Log,
               TEXT("  no sysfs 'range' attribute; using the device's own range."));
    }

    if (wheel_ffb_init(Device, kPeakTorqueNm, Err, sizeof Err) == 0)
    {
        // 1 kHz in, 250 Hz on the wire, 64/32767 deadband. See the note in
        // wheel_device.h: a full-speed HID endpoint cannot absorb 1000
        // updates a second, and queueing them in the driver adds latency
        // rather than fidelity.
        wheel_ffb_set_rate(Device, 4, 64);
        bFfbActive.store(true, std::memory_order_relaxed);
        UE_LOG(LogChronoWheel, Log, TEXT("  force feedback active, peak %.1f Nm."), kPeakTorqueNm);
    }
    else
    {
        UE_LOG(LogChronoWheel, Warning, TEXT("  force feedback unavailable: %hs"), Err);
    }
    return true;
}

uint32 FChronoWheelThread::Run()
{
    const long PeriodNs = 1000000L;     // 1 kHz, matching the physics thread
    const long SpinMarginNs = 200000L;  // C-state margin, same reasoning as M3

    timespec Next;
    clock_gettime(CLOCK_MONOTONIC, &Next);

    double MeanPoll = 0.0, MaxPoll = 0.0;
    uint64 Ticks = 0;
    FChronoDriverState State;
    State.bConnected = bConnected.load(std::memory_order_relaxed);

    while (!bStopRequested.load(std::memory_order_relaxed))
    {
        AddNs(Next, PeriodNs);
        {
            timespec Early = Next;
            Early.tv_nsec -= SpinMarginNs;
            if (Early.tv_nsec < 0) { Early.tv_nsec += 1000000000L; --Early.tv_sec; }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &Early, nullptr);

            timespec Now;
            do
            {
                clock_gettime(CLOCK_MONOTONIC, &Now);
                __builtin_ia32_pause();
            }
            while (DiffUs(Now, Next) < 0.0 &&
                   !bStopRequested.load(std::memory_order_relaxed));
        }

        timespec T0; clock_gettime(CLOCK_MONOTONIC, &T0);

        if (Device)
        {
            WheelState W;
            if (wheel_poll(Device, &W) < 0)
            {
                // Read error means the device went away mid-session (unplugged,
                // or the USB port reset). Drop it and keep the thread alive so
                // the sim degrades to no-input rather than dying.
                UE_LOG(LogChronoWheel, Warning, TEXT("Wheel disconnected."));
                wheel_close(Device);
                Device = nullptr;
                bConnected.store(false, std::memory_order_relaxed);
                bFfbActive.store(false, std::memory_order_relaxed);
                State = FChronoDriverState{};
            }
            else
            {
                State.Steer      = W.steer;      // device convention: +clockwise
                State.Throttle   = W.throttle;
                State.Brake      = W.brake;
                State.Clutch     = W.clutch;
                State.Gear       = W.gear;
                State.GlanceDir  = W.dpad_x;
                State.bConnected = true;
            }
            Inbound.Publish(State);

            // Newest commanded torque. If the physics thread happened to be
            // mid-write for all eight retries, hold the previous value: a
            // repeated force for one millisecond is imperceptible, whereas
            // dropping to zero would be a click the driver can feel.
            double Nm = 0.0;
            if (bFfbActive.load(std::memory_order_relaxed) && Outbound.TryRead(Nm))
            {
                wheel_ffb_set(Device, Nm);
            }
        }

        timespec T1; clock_gettime(CLOCK_MONOTONIC, &T1);
        const double PollUs = DiffUs(T1, T0);
        MeanPoll = Ema(MeanPoll, PollUs);
        if (PollUs > MaxPoll) { MaxPoll = PollUs; }
        ++Ticks;

        if ((Ticks & 63u) == 0)
        {
            StatMeanPollUs.store(MeanPoll, std::memory_order_relaxed);
            StatMaxPollUs .store(MaxPoll,  std::memory_order_relaxed);
            StatTicks     .store(Ticks,    std::memory_order_relaxed);
        }
    }
    return 0;
}

void FChronoWheelThread::Stop() { bStopRequested.store(true, std::memory_order_relaxed); }

void FChronoWheelThread::Exit()
{
    if (Device)
    {
        // Zero the force BEFORE releasing the effect. Tearing down an effect
        // slot that is still commanding torque leaves some wheels holding the
        // last value until they are power-cycled.
        wheel_ffb_set(Device, 0.0);
        wheel_ffb_stop(Device);
        wheel_close(Device);
        Device = nullptr;
    }
    UE_LOG(LogChronoWheel, Log, TEXT("Wheel thread exited after %llu ticks."),
           (unsigned long long)StatTicks.load());
}

FChronoWheelStats FChronoWheelThread::GetStats() const
{
    FChronoWheelStats S;
    S.bConnected = bConnected.load(std::memory_order_relaxed);
    S.bFfbActive = bFfbActive.load(std::memory_order_relaxed);
    S.bRealtime  = bRealtimeGranted.load(std::memory_order_relaxed);
    S.MeanPollUs = StatMeanPollUs.load(std::memory_order_relaxed);
    S.MaxPollUs  = StatMaxPollUs .load(std::memory_order_relaxed);
    S.Ticks      = StatTicks     .load(std::memory_order_relaxed);
    if (Device)
    {
        uint64_t Calls = 0; double TotalUs = 0.0;
        wheel_ffb_stats(Device, &Calls, &TotalUs);
        S.FfbCalls  = Calls;
        S.MeanFfbUs = Calls ? TotalUs / double(Calls) : 0.0;
        const WheelCaps* C = wheel_caps(Device);
        S.RangeDeg = C->range_deg;
        std::snprintf(S.DeviceName, sizeof S.DeviceName, "%s", C->name);
    }
    return S;
}
