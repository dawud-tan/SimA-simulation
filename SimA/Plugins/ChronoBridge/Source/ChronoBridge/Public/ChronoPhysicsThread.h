// ============================================================================
//  ChronoPhysicsThread.h — the 1 kHz simulation thread.
//
//  WHY A DEDICATED THREAD AT ALL:
//  Unreal ticks the game thread once per frame, at whatever rate the renderer
//  achieves. Running physics there ties the timestep to the framerate: the
//  simulation would behave differently at 60 fps than at 30, and a single slow
//  frame would produce a single enormous timestep. For a stiff SMC contact
//  model that is not merely inaccurate -- a 33 ms step with a contact stiffness
//  tuned for 1 ms is numerically explosive.
//
//  The simulation therefore owns its own clock and never consults the renderer.
//
//  WHY 1000 Hz SPECIFICALLY:
//  Three constraints meet at roughly this number.
//    1. SMC penalty contact is stiff. Penetration depth must stay small, which
//       forces k large, which bounds the stable timestep.
//    2. Force feedback needs new torque values faster than the wheel's own
//       control loop, which the G923 runs at ~1 kHz. Feeding it at 60 Hz makes
//       the wheel feel notchy no matter how good the physics is.
//    3. Static friction transitions -- the clutch bite point on a hill start --
//       resolve over a few milliseconds. At 60 Hz that is one or two samples.
//
//  Measured on this machine: a 240-DOF rigid vehicle steps in 7.4 us mean
//  against a 1000 us budget, so the thread is idle ~99% of the time. It is not
//  a CPU hog; it is a latency-sensitive, mostly-sleeping thread.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"

#include "ChronoStateRing.h"

/**
 * The physics -> game ring's depth, in 1 ms samples. It is how long the game
 * thread may stall before the referee loses samples (REF_SAMPLE_GAP): a
 * frame that takes longer than this cannot be drained whole. 64 (64 ms) lost
 * a sample on ordinary frame hitches -- measured at ~27 ms frames (task 5).
 * 256 is a quarter of a second, ~435 KB; the writer still publishes one
 * slot a step, so the depth costs the physics thread nothing.
 */
constexpr uint32 GChronoRingSize = 256;
using FChronoRing = TChronoStateRing<GChronoRingSize>;
#include "ChronoWheelInput.h"

#include "driver_source.h"

/**
 * Which of the three input sources is actually driving the car this step.
 *
 * They are not interchangeable, and the difference is WHO OWNS THE CLOCK:
 *   Wheel    -- the device's clock, reaching us through a seqlock.
 *   Keyboard -- the renderer's clock for intent, ours for the integration.
 *   Script   -- ours alone, keyed to step_index. The only reproducible one.
 */
enum class EChronoInputSource : uint8
{
    None = 0,
    Keyboard,
    Wheel,
    Script,
};

/** Rolling statistics, published to the game thread for the HUD/logs. */
struct FChronoThreadStats
{
    double   MeanStepUs   = 0.0;
    double   MaxStepUs    = 0.0;
    double   MeanLateUs   = 0.0;   // wakeup lateness vs the absolute deadline
    double   MaxLateUs    = 0.0;
    uint64   MissedTicks  = 0;     // steps whose work exceeded the period
    uint64   StepCount    = 0;
    bool     bRealtime    = false; // SCHED_FIFO actually granted?

    // Milestone 5: what the driver is doing and what the wheel is being told.
    double   SteerTorqueNm = 0.0;  // at the rim, device convention (+ = CW)
    double   SpeedMps      = 0.0;
    double   EngineRpm     = 0.0;
    int32    Gear          = 0;
    // The clutch PEDAL as commanded, 1 = down = disengaged. The referee's
    // stall rule needs it to tell a stalled engine from one declutched at a
    // stop; without it every stop in gear with the clutch down read as
    // "drive connected".
    double   Clutch        = 1.0;
    // Task 5, for the HUD: the other pedals and the wheel as commanded.
    // Steering in DEVICE convention (+ = clockwise = right), -1..+1.
    double   Throttle      = 0.0;
    double   Brake         = 0.0;
    double   Steering      = 0.0;
    // Task 17: where the driver is looking, -1 left door mirror, 0 ahead,
    // +1 right; the head, not the car -- it never reaches Chrono.
    double   Glance        = 0.0;
    bool     bVehicle      = false;

    // Milestone 6: where the inputs came from, and how far through a scripted
    // run we are. ScriptTime is derived from the step counter, not from a
    // wall clock, so two runs of the same script report the same numbers.
    EChronoInputSource Source      = EChronoInputSource::None;
    double             ScriptTime  = 0.0;
    double             ScriptLen   = 0.0;
    bool               bScriptEnded = false;
};

/**
 * Owns the Chrono world and steps it at a fixed rate.
 *
 * THREAD OWNERSHIP RULES -- these are the whole safety argument:
 *   - The ChsWorld handle is touched ONLY by this thread. The game thread never
 *     calls Chs_Step or Chs_GetSnapshot.
 *   - Communication out is exclusively through the lock-free ring.
 *   - Communication in is exclusively through atomics (input state, stop flag).
 * No Unreal object (UObject, AActor, FString) is touched from this thread:
 * UObjects are not thread-safe and the GC can move or destroy them at any time.
 */
class CHRONOBRIDGE_API FChronoPhysicsThread : public FRunnable
{
public:
    explicit FChronoPhysicsThread(double InStepHz = 1000.0);
    virtual ~FChronoPhysicsThread() override;

    /** FRunnable. Called on the NEW thread before Run(). */
    virtual bool Init() override;
    /** FRunnable. The thread body; returns when Stop() is requested. */
    virtual uint32 Run() override;
    /** FRunnable. Called from the REQUESTING thread, asynchronously. */
    virtual void Stop() override;
    /** FRunnable. Called on the new thread after Run() returns. */
    virtual void Exit() override;

    /** Starts the underlying OS thread. Safe to call once. */
    void Launch();
    /** Requests stop and blocks until the thread has exited. */
    void Shutdown();

    /** Consumer access to the state ring. Safe from any thread. */
    const FChronoRing& GetRing() const { return Ring; }

    /** Restart the timing statistics' max and missed count (the mean is an
     *  EMA and forgets on its own). The session start calls it, so the
     *  health line measures steady rendering, not asset loading. */
    void ResetStats() { bStatsReset.store(true, std::memory_order_relaxed); }

    /** Snapshot of timing stats. Safe from any thread (torn reads are benign). */
    FChronoThreadStats GetStats() const;

    /** Attach the wheel thread. Must be called BEFORE Launch(): the pointer is
     *  read without synchronisation from the physics thread, which is only safe
     *  because it is written once, before that thread exists. */
    void SetWheelThread(FChronoWheelThread* InWheel) { Wheel = InWheel; }

    /**
     * Arms a scripted driver. `NameOrPath` is a built-in name (see
     * driver_source.cpp) or a path to a keyframe file.
     *
     * MUST be called BEFORE Launch(), and this is not a convention -- it is
     * the reason no lock is needed. drv_script_load mallocs and can realloc a
     * table that the physics thread scans on every one of its 1000 steps a
     * second. Publishing that table to a running thread would need either a
     * lock on the hot path or an RCU-shaped handoff, and neither is worth
     * building to change a test fixture. Arming before the thread exists makes
     * the table immutable-once-visible, which is the same argument that makes
     * the raw Wheel pointer above safe.
     *
     * Re-running the SAME script is a different question and is safe at any
     * time: see RestartScript().
     *
     * @return false with OutError set if the script could not be resolved.
     */
    bool ArmScript(const FString& NameOrPath, FString& OutError);

    /** Rewinds the script clock to the current step. Safe from any thread: it
     *  writes one atomic and touches no table. Use this to repeat a run
     *  without tearing down the world -- the vehicle state does NOT reset, so
     *  it repeats the INPUTS, not the experiment. */
    void RestartScript();

    /**
     * A scripted drive is HELD at its t = 0 keyframe from Launch() until the
     * game releases it (task 5): the thread starts with the world, and the
     * first rendered frame can come a minute later, after asset loading and
     * shader compilation. Unheld, a script ran out during the loading screen
     * and the first frame showed the car long gone. Callable from any thread.
     */
    void ReleaseScript() { bScriptHold.store(false, std::memory_order_relaxed); }

    /** True if a script was armed and resolved. */
    bool IsScriptArmed() const { return bScriptArmed; }

    /**
     * Publishes the keyboard's held-key bitmask (DRV_KEY_*). Called from the
     * game thread once per frame; the physics thread integrates it into axes
     * at 1 kHz. See driver_source.h for why intent crosses rather than axes.
     */
    void PublishKeys(uint32 Keys) { KeyboardIn.Publish(Keys); }

private:
    void  BuildScene();

    FChronoWheelThread* Wheel = nullptr;
    bool                bVehicle = false;

    // ---- Milestone 6 input sources -------------------------------------
    // Both live ON the physics thread and are touched by nothing else. The
    // keyboard's *intent* arrives through a seqlock; its integration state
    // does not cross a thread boundary at all.
    DrvKeyboard Kbd{};
    DrvScript   Script{};
    bool        bScriptArmed = false;
    uint32      LastKeys     = 0;

    TChronoSeqSlot<uint32> KeyboardIn;    // game thread -> physics

    // Step index at which the script clock reads zero. Written and read only
    // by the physics thread, so it is a plain uint64. RestartScript() from
    // another thread raises the flag below instead of writing this directly:
    // that keeps the per-step cost to one relaxed LOAD of an otherwise
    // read-only cache line, rather than an atomic RMW on the hot path.
    uint64            ScriptEpochStep = 0;
    std::atomic<bool> bScriptRestart{false};
    std::atomic<bool> bScriptHold{true};      // see ReleaseScript()

    // Last driver state successfully read from the wheel thread. Held so that a
    // failed seqlock read reuses it rather than commanding zero throttle for a
    // millisecond -- which at 1 kHz would be an audible stumble in the engine.
    FChronoDriverState  LastDriver;
    // The wheel's glance, integrated here from its held D-pad direction like
    // the keyboard's from its keys (drv_glance_step). Physics thread only.
    double              WheelGlance = 0.0;

    FChronoRing Ring;
    std::atomic<bool> bStatsReset{false};

    void*   World      = nullptr;   // ChsWorld, opaque
    double  StepHz     = 1000.0;
    double  StepSeconds= 0.001;

    FRunnableThread*        Thread = nullptr;
    std::atomic<bool>       bStopRequested{false};

    // Written by the physics thread, read by the game thread. Individually
    // atomic; the set is not a consistent snapshot, which is fine for stats.
    std::atomic<double> StatMeanStepUs{0.0};
    std::atomic<double> StatMaxStepUs{0.0};
    std::atomic<double> StatMeanLateUs{0.0};
    std::atomic<double> StatMaxLateUs{0.0};
    std::atomic<uint64> StatMissed{0};
    std::atomic<uint64> StatSteps{0};
    std::atomic<bool>   bRealtimeGranted{false};
    std::atomic<double> StatSteerTorque{0.0};
    std::atomic<double> StatSpeed{0.0};
    std::atomic<double> StatRpm{0.0};
    std::atomic<int32>  StatGear{0};
    std::atomic<double> StatClutch{1.0};
    std::atomic<double> StatThrottle{0.0};
    std::atomic<double> StatBrake{0.0};
    std::atomic<double> StatSteer{0.0};
    std::atomic<double> StatGlance{0.0};
    std::atomic<uint8>  StatSource{uint8(EChronoInputSource::None)};
    std::atomic<double> StatScriptTime{0.0};
    std::atomic<bool>   bStatScriptEnded{false};
};
