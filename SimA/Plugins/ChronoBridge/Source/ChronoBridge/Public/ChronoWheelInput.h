// ============================================================================
//  ChronoWheelInput.h — the G923 on its own real-time thread.
//
//  WHY A SEPARATE THREAD AND NOT THE PHYSICS THREAD:
//  Reading the wheel is nearly free -- an empty non-blocking drain of the evdev
//  queue measured 0.23 us, and the whole poll-plus-normalise step measured
//  1.31 us mean over 5000 iterations. That part could live anywhere.
//
//  WRITING force feedback is the problem. Each update is an EVIOCSFF ioctl that
//  descends into ff-core, then into hid-logitech or hid-logitech-hidpp, and
//  ends as a USB interrupt-OUT submission. Its cost is UNMEASURED on this
//  machine, because no wheel has ever been attached to it. Putting an unmeasured
//  syscall on a SCHED_FIFO thread that must not miss a 1 ms deadline -- when
//  that thread is already spending 179 us on the vehicle -- is a bet with no
//  upside. The physics thread stays pure computation.
//
//  The cost of the split is one frame of latency in each direction: up to 1 ms
//  for an input to reach physics and up to 1 ms for a torque to reach the wheel.
//  Against the ~10 ms at which a driver starts to perceive force feedback as
//  disconnected, that is not a meaningful expense.
//
//  Priority is 79, one below the physics thread's 80, so that if both are
//  runnable the simulation wins.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

#include <atomic>

/** Driver state as the simulation wants it. Steering is in the DEVICE
 *  convention here: +1 = wheel turned clockwise. The flip to Chrono's
 *  positive-left convention happens in ChronoPhysicsThread.cpp, once. */
struct FChronoDriverState
{
    double Steer    = 0.0;      // -1 .. +1, clockwise positive
    double Throttle = 0.0;
    double Brake    = 0.0;
    double Clutch   = 0.0;      // 1 = pedal down = disengaged (matches Chrono)
    int32  Gear     = 0;        // -1 R, 0 N, 1..5
    int32  GlanceDir = 0;       // D-pad held: -1 left, 0, +1 right (task 17)
    bool   bConnected = false;  // false => no wheel; caller should fall back
};

/**
 * Single-slot seqlock for one small POD, newest-value-wins.
 *
 * The state ring exists because INTERPOLATION needs the two snapshots that
 * bracket a render timestamp. Nothing here needs history: the physics thread
 * wants the newest pedal positions, and the wheel wants the newest torque.
 * A ring would only add a stale-index question that has no useful answer.
 */
template <typename T>
class TChronoSeqSlot
{
public:
    void Publish(const T& In)
    {
        const uint64 S = Seq.load(std::memory_order_relaxed);
        Seq.store(S + 1, std::memory_order_relaxed);          // odd: writing
        std::atomic_thread_fence(std::memory_order_release);
        Data = In;
        std::atomic_thread_fence(std::memory_order_release);
        Seq.store(S + 2, std::memory_order_relaxed);          // even: stable
    }

    /** Returns false if the writer kept winning; caller reuses its last value.
     *  Bounded retries, like the ring: an unbounded spin here would hand the
     *  writer the power to stall the reader indefinitely. */
    bool TryRead(T& Out) const
    {
        for (int Attempt = 0; Attempt < 8; ++Attempt)
        {
            const uint64 Before = Seq.load(std::memory_order_relaxed);
            if (Before & 1) { continue; }
            std::atomic_thread_fence(std::memory_order_acquire);
            Out = Data;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (Seq.load(std::memory_order_relaxed) == Before) { return true; }
        }
        return false;
    }

private:
    mutable std::atomic<uint64> Seq{0};
    T                           Data{};
};

/** Diagnostics for the HUD and the log. */
struct FChronoWheelStats
{
    bool   bConnected     = false;
    bool   bFfbActive     = false;
    bool   bRealtime      = false;
    double MeanPollUs     = 0.0;
    double MaxPollUs      = 0.0;
    double MeanFfbUs      = 0.0;   // per EVIOCSFF actually issued
    uint64 FfbCalls       = 0;
    uint64 Ticks          = 0;
    int32  RangeDeg       = -1;
    char   DeviceName[128] = {};
};

class CHRONOBRIDGE_API FChronoWheelThread : public FRunnable
{
public:
    FChronoWheelThread();
    virtual ~FChronoWheelThread() override;

    virtual bool   Init() override;
    virtual uint32 Run() override;
    virtual void   Stop() override;
    virtual void   Exit() override;

    void Launch();
    void Shutdown();

    /** Physics thread reads this. Never blocks. */
    bool ReadDriverState(FChronoDriverState& Out) const { return Inbound.TryRead(Out); }

    /** Physics thread writes this, in newton-metres at the rim, in the DEVICE
     *  convention (+ = clockwise). Never blocks. */
    void CommandTorque(double Nm) { Outbound.Publish(Nm); }

    FChronoWheelStats GetStats() const;

    /** True once a device with an ABS_X axis was opened. */
    bool IsConnected() const { return bConnected.load(std::memory_order_relaxed); }

private:
    struct WheelDevice* Device = nullptr;

    TChronoSeqSlot<FChronoDriverState> Inbound;    // wheel -> physics
    TChronoSeqSlot<double>             Outbound;   // physics -> wheel

    FRunnableThread*  Thread = nullptr;
    std::atomic<bool> bStopRequested{false};
    std::atomic<bool> bConnected{false};
    std::atomic<bool> bFfbActive{false};
    std::atomic<bool> bRealtimeGranted{false};

    std::atomic<double> StatMeanPollUs{0.0};
    std::atomic<double> StatMaxPollUs{0.0};
    std::atomic<uint64> StatTicks{0};
};
