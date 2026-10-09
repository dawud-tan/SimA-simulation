// ============================================================================
//  ChronoWorldSubsystem.h — the game-thread owner of the physics thread.
//
//  A UWorldSubsystem is created with the UWorld and destroyed with it, so the
//  physics thread's lifetime is tied to the level rather than to any actor.
//  That matters: an actor can be garbage collected or streamed out mid-frame,
//  and a thread owned by one would outlive its owner.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"

#include "ChronoPhysicsThread.h"
#include "ChronoInterpolator.h"
#include "ChronoWheelInput.h"
#include "referee.h"

#include "ChronoWorldSubsystem.generated.h"

UCLASS()
class CHRONOBRIDGE_API UChronoWorldSubsystem : public UTickableWorldSubsystem
{
    GENERATED_BODY()

public:
    // USubsystem
    virtual void Initialize(FSubsystemCollectionBase& Collection) override;
    virtual void Deinitialize() override;

    /**
     * UWorldSubsystem. The default accepts Game, Editor AND PIE
     * (WorldSubsystem.cpp:29), which meant the editor's preview world was
     * already standing up a Chrono world, a 1 kHz SCHED_FIFO physics thread
     * and a second SCHED_FIFO wheel thread just for having the editor open --
     * and pressing Play then created a SECOND set, two Innovas at ~180 us a
     * step. Nothing noticed while there was no way to drive, because both were
     * simulating a stationary car. Milestone 6 makes it expensive and visible.
     *
     * The editor world also has no PlayerController, so it could never take
     * keyboard input anyway.
     */
    virtual bool DoesSupportWorldType(const EWorldType::Type WorldType) const override
    {
        return WorldType == EWorldType::Game || WorldType == EWorldType::PIE;
    }

    // FTickableGameObject
    virtual void Tick(float DeltaTime) override;
    virtual TStatId GetStatId() const override;
    virtual bool IsTickable() const override { return PhysicsThread != nullptr; }

    /** Interpolated poses for the frame just ticked. */
    const FChronoInterpResult& GetFrameState() const { return FrameState; }

    /** Timing statistics from the physics thread. */
    FChronoThreadStats GetPhysicsStats() const;

    /** Wheel/force-feedback diagnostics. */
    FChronoWheelStats GetWheelStats() const;

    /** Referee events accumulated so far this session. */
    int32 GetRefereeEvents(RefEvent* Out, int32 Max) const
    {
        return bRefereeReady ? ref_get_events(&Referee, Out, Max) : 0;
    }

    /** The test so far: overall verdict, and each exercise's state. Pure
     *  reads of the referee, cheap enough to call every frame (task 5's HUD).
     *  Returns false when no course is loaded. */
    bool GetRefereeVerdict(RefVerdict& Out) const
    {
        if (!bRefereeReady) { return false; }
        ref_verdict(&Referee, &Out);
        return true;
    }
    const RefExerciseResult* GetExerciseResult(int32 Index) const
    {
        return (bRefereeReady && Index >= 0 && Index < Course.n_exercises)
            ? &Referee.ex[Index] : nullptr;
    }
    const RefCourse* GetCourse() const { return bRefereeReady ? &Course : nullptr; }

    /** False until frames have been steady long enough to start judging and
     *  release a scripted drive -- see Tick. For the HUD. */
    bool IsSessionStarted() const { return bSessionStarted; }

private:
    /**
     * Feeds the referee EVERY physics sample published since the last frame.
     *
     * This DRAINS the ring; it does not sample it. The interpolated pose the
     * renderer draws is a function of frame timing, so a verdict derived from
     * it would depend on the framerate and two runs of the same input could
     * score differently. Draining costs ~17 evaluations per frame at 60 Hz
     * against a 64-slot ring, so the margin is 3.7x -- and if the game thread
     * ever does fall far enough behind to lose samples, ref_update reports a
     * REF_SAMPLE_GAP rather than quietly scoring a trajectory with holes.
     */
    void DrainRingIntoReferee();

    /**
     * Samples the keyboard into a DRV_KEY_* mask.
     *
     * This polls APlayerController::IsInputKeyDown directly instead of binding
     * through Enhanced Input, and that is a deliberate consequence of where
     * the project is: SimA/Content is EMPTY. Enhanced Input needs a
     * UInputMappingContext and UInputAction, both of which are content
     * assets, and none exists -- so a binding-based path could not be run at
     * all today. Polling needs nothing but the PlayerController that PIE
     * spawns for any world, empty level included.
     *
     * Migrate to Enhanced Input once there is a pawn and a mapping context to
     * hang it off; the physics side will not notice, because what crosses the
     * thread boundary is a bitmask either way.
     */
    uint32 BuildKeyMask() const;

    // Declared BEFORE the physics thread so that member destruction order --
    // reverse of declaration -- tears the physics thread down first. The
    // physics thread holds a raw pointer to the wheel thread; the reverse is
    // not true, so the holder must die first.
    TUniquePtr<FChronoWheelThread>   WheelThread;
    TUniquePtr<FChronoPhysicsThread> PhysicsThread;
    FChronoInterpolator              Interpolator;
    FChronoInterpResult              FrameState;

    // The game thread's estimate of the physics clock. Advanced by real
    // DeltaTime each frame and gently corrected toward the newest snapshot,
    // rather than simply copied from it -- see the .cpp for why.
    // Referee. Owned here because it is per-world state, like the threads.
    RefCourse Course;
    RefState  Referee;
    bool      bRefereeReady = false;
    uint64    LastJudgedStep = 0;
    uint64    NextRingIndex  = 0;   // absolute ring index the drain reads next
    int32     LastReportedEvents = 0;

    double GameSimClock  = 0.0;
    bool   bClockPrimed  = false;
    double LogAccumulator = 0.0;
    // Rendered frame times over the health line's window (task 9): what the
    // scene costs the GPU, which the physics line cannot show.
    double FrameSumS = 0.0, FrameMaxS = 0.0;
    int32  FrameCount = 0;
    // Where each frame went, as `stat unit` splits it (task 18): game,
    // render and RHI thread and GPU, summed in ms over the same window.
    double GameSumMs = 0.0, RenderSumMs = 0.0, RhiSumMs = 0.0, GpuSumMs = 0.0;

    // Session start (task 5), see Tick.
    bool   bSessionStarted = false;
    double SteadySeconds   = 0.0;
};
