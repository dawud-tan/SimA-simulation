#include "ChronoWorldSubsystem.h"

#include "ChronoBridgeModule.h"
#include "driver_source.h"

#include "DynamicRHI.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "RenderTimer.h"
#include "InputCoreTypes.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogChronoSubsystem, Log, All);

namespace
{
    const TCHAR* SourceName(EChronoInputSource S)
    {
        switch (S)
        {
        case EChronoInputSource::Keyboard: return TEXT("keyboard");
        case EChronoInputSource::Wheel:    return TEXT("wheel");
        case EChronoInputSource::Script:   return TEXT("script");
        default:                           return TEXT("none");
        }
    }
}

void UChronoWorldSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
    Super::Initialize(Collection);

    if (!FChronoBridgeModule::Get().IsChronoReady())
    {
        UE_LOG(LogChronoSubsystem, Error,
               TEXT("Chrono bridge not ready; physics thread not started."));
        return;
    }

    // Order matters. The wheel thread must exist and be running before the
    // physics thread is told about it, because SetWheelThread writes a pointer
    // that the physics thread then reads every step without synchronisation --
    // safe only because the write happens before that thread is created.
    WheelThread = MakeUnique<FChronoWheelThread>();
    WheelThread->Launch();

    PhysicsThread = MakeUnique<FChronoPhysicsThread>(1000.0);
    PhysicsThread->SetWheelThread(WheelThread.Get());

    // -ChronoScript=<built-in name | path> arms the deterministic driver.
    //
    // A command-line switch rather than a config setting or an editor widget,
    // because the point of the scripted driver is unattended, repeatable runs:
    // the thing that eventually launches it is a shell script, not a person
    // clicking. It is parsed HERE, before Launch(), which is the only window
    // in which arming is safe -- see FChronoPhysicsThread::ArmScript.
    FString ScriptName;
    if (FParse::Value(FCommandLine::Get(), TEXT("ChronoScript="), ScriptName))
    {
        FString ScriptError;
        if (!PhysicsThread->ArmScript(ScriptName, ScriptError))
        {
            // Deliberately loud and deliberately NOT fatal. An unattended run
            // that silently fell back to a keyboard nobody is touching would
            // produce a stationary car and a passing-looking log.
            UE_LOG(LogChronoSubsystem, Error,
                   TEXT("-ChronoScript=%s could not be armed: %s"),
                   *ScriptName, *ScriptError);
        }
    }

    // The referee's course. Sibling directory, same convention as the vehicle
    // JSON and the driver scripts: shared with a host harness, not UE content.
    {
        const FString CourseFile = FPaths::ConvertRelativePathToFull(
            FPaths::Combine(FPaths::ProjectDir(), TEXT("../course/simA_referee.txt")));
        char Err[256] = {};
        if (FPaths::FileExists(CourseFile) &&
            ref_load_course(&Course, TCHAR_TO_UTF8(*CourseFile), Err, sizeof Err) == 0)
        {
            ref_init(&Referee, &Course);
            bRefereeReady = true;
            UE_LOG(LogChronoSubsystem, Log,
                   TEXT("Referee: %d cones, %d lines, %d gates from %s."),
                   Course.n_cones, Course.n_lines, Course.n_gates, *CourseFile);
        }
        else
        {
            UE_LOG(LogChronoSubsystem, Warning,
                   TEXT("No referee course (%hs). Run 'bash course/run.sh gen'."), Err);
        }
    }

    PhysicsThread->Launch();
    Interpolator.SetDelaySeconds(0.002);

    UE_LOG(LogChronoSubsystem, Log,
           TEXT("Chrono world subsystem initialised (world '%s', input: %s)."),
           *GetWorld()->GetName(),
           PhysicsThread->IsScriptArmed() ? TEXT("scripted")
           : (WheelThread->IsConnected() ? TEXT("wheel") : TEXT("keyboard")));
}

void UChronoWorldSubsystem::Deinitialize()
{
    // Join before anything else tears down. Shutdown() blocks until the thread
    // has run Exit(), which destroys the Chrono world -- so by the time this
    // returns nothing is touching the shim any more.
    if (PhysicsThread)
    {
        PhysicsThread->Shutdown();
        PhysicsThread.Reset();
    }
    // Only now is it safe to drop the wheel: nothing is left that could call
    // CommandTorque. Its Exit() zeroes the force before releasing the effect
    // slot, so the wheel does not stay loaded after the editor stops.
    if (WheelThread)
    {
        WheelThread->Shutdown();
        WheelThread.Reset();
    }
    Super::Deinitialize();
}

uint32 UChronoWorldSubsystem::BuildKeyMask() const
{
    const UWorld* W = GetWorld();
    const APlayerController* PC = W ? W->GetFirstPlayerController() : nullptr;
    if (!PC) { return 0; }

    uint32 K = 0;

    // Steering. WASD and the arrow keys are both live: the arrows are what a
    // person reaches for when the other hand is on the number row shifting.
    if (PC->IsInputKeyDown(EKeys::A) || PC->IsInputKeyDown(EKeys::Left))  K |= DRV_KEY_LEFT;
    if (PC->IsInputKeyDown(EKeys::D) || PC->IsInputKeyDown(EKeys::Right)) K |= DRV_KEY_RIGHT;
    if (PC->IsInputKeyDown(EKeys::W) || PC->IsInputKeyDown(EKeys::Up))    K |= DRV_KEY_THROTTLE;
    if (PC->IsInputKeyDown(EKeys::S) || PC->IsInputKeyDown(EKeys::Down))  K |= DRV_KEY_BRAKE;

    // Clutch on Left Shift, the sim convention. Left Ctrl is the fine
    // modifier -- quarter rate on every axis, which is what makes the
    // centimetre-scale corrections of a parking exercise possible from a key.
    if (PC->IsInputKeyDown(EKeys::LeftShift))   K |= DRV_KEY_CLUTCH;
    if (PC->IsInputKeyDown(EKeys::LeftControl)) K |= DRV_KEY_FINE;

    // There is no auto-centre (see driver_source.cpp), so straightening up is
    // an explicit request.
    if (PC->IsInputKeyDown(EKeys::Q)) K |= DRV_KEY_CENTER;

    // Glance at a door mirror, held (task 17). Two pairs for the same reason
    // steering has two: Z / C for the free left hand when the arrows steer,
    // comma / period for the free right hand when WASD does.
    if (PC->IsInputKeyDown(EKeys::Z) || PC->IsInputKeyDown(EKeys::Comma))  K |= DRV_KEY_GLANCE_L;
    if (PC->IsInputKeyDown(EKeys::C) || PC->IsInputKeyDown(EKeys::Period)) K |= DRV_KEY_GLANCE_R;

    // H-pattern on the number row. Edge detection happens on the physics
    // thread inside drv_keyboard_step; this is a level-triggered mirror.
    if (PC->IsInputKeyDown(EKeys::Zero))  K |= DRV_KEY_GEAR_N;
    if (PC->IsInputKeyDown(EKeys::R))     K |= DRV_KEY_GEAR_R;
    if (PC->IsInputKeyDown(EKeys::One))   K |= DRV_KEY_GEAR_1;
    if (PC->IsInputKeyDown(EKeys::Two))   K |= DRV_KEY_GEAR_2;
    if (PC->IsInputKeyDown(EKeys::Three)) K |= DRV_KEY_GEAR_3;
    if (PC->IsInputKeyDown(EKeys::Four))  K |= DRV_KEY_GEAR_4;
    if (PC->IsInputKeyDown(EKeys::Five))  K |= DRV_KEY_GEAR_5;

    return K;
}

void UChronoWorldSubsystem::Tick(float DeltaTime)
{
    if (!PhysicsThread) { return; }

    // Publish key INTENT every frame. Not axis values -- the integration to an
    // axis happens on the physics thread at 1 kHz, because a boolean sampled
    // at 60 Hz and used directly as a steering position commands a steering
    // rate of 605 deg per millisecond. See driver_source.h.
    PhysicsThread->PublishKeys(BuildKeyMask());

    const FChronoRing& Ring = PhysicsThread->GetRing();

    ChsSnapshot Newest;
    if (!Ring.Read(Newest, 0)) { return; }   // nothing published yet

    if (!bClockPrimed)
    {
        GameSimClock = Newest.sim_time;
        bClockPrimed = true;
    }
    else
    {
        // Advance by real elapsed time. Do NOT simply assign Newest.sim_time:
        // that would make the render clock jump in 1 ms increments at whatever
        // moment the frame happens to sample it, reintroducing exactly the
        // beat-frequency stutter that interpolation exists to remove.
        GameSimClock += DeltaTime;

        // Slew-correct toward the producer instead of snapping. Drift comes
        // from DeltaTime being the renderer's measure of a frame, which is
        // never exactly the physics clock's measure of the same interval.
        // Correcting 2% per frame converges in ~50 frames without a visible
        // discontinuity.
        const double Error = Newest.sim_time - GameSimClock;
        if (FMath::Abs(Error) > 0.25)
        {
            GameSimClock = Newest.sim_time;   // hitch or hard stall: resync
        }
        else
        {
            GameSimClock += Error * 0.02;
        }
    }

    FrameState = Interpolator.Evaluate(Ring, GameSimClock);

    // SESSION START (task 5). The physics thread runs from world creation,
    // and the first frames can come a minute later -- asset loading, shader
    // compilation -- during which nothing drains the ring. A scripted drive
    // ran out in that time (the first frame showed the car long gone, the
    // script finished), and the referee, fed across the gap, flagged every
    // verdict as missing samples. So the session starts when frames have been
    // STEADY: under 40 ms -- inside the ring's 64 ms of history, so no sample
    // can be lost between two drains -- for half a second. Then a scripted
    // drive is released and the referee judges from that sample on.
    if (!bSessionStarted)
    {
        SteadySeconds = DeltaTime < 0.040f ? SteadySeconds + DeltaTime : 0.0;
        if (SteadySeconds >= 0.5)
        {
            bSessionStarted = true;
            PhysicsThread->ReleaseScript();
            PhysicsThread->ResetStats();
            if (bRefereeReady)
            {
                ref_init(&Referee, &Course);
                LastJudgedStep = Newest.step_index;
                NextRingIndex = Ring.Count();
                LastReportedEvents = 0;
            }
            UE_LOG(LogChronoSubsystem, Log,
                   TEXT("Session started at physics step %llu (%.1f s after the world): ")
                   TEXT("frames steady; script released, referee judging from here."),
                   (unsigned long long)Newest.step_index, Newest.sim_time);
        }
    }
    if (bSessionStarted)
    {
        DrainRingIntoReferee();
    }

    // Periodic health line. At 60 Hz this is once every 5 s.
    LogAccumulator += DeltaTime;
    FrameSumS += DeltaTime;
    FrameMaxS  = FMath::Max(FrameMaxS, (double)DeltaTime);
    ++FrameCount;
    // The previous frame's split, as the engine measured it: the threads run
    // a frame apart, so a frame's time is the slowest of the four, and the
    // slowest is the one worth making cheaper (task 18).
    GameSumMs   += FPlatformTime::ToMilliseconds(GGameThreadTime);
    RenderSumMs += FPlatformTime::ToMilliseconds(GRenderThreadTime);
    RhiSumMs    += FPlatformTime::ToMilliseconds(GRHIThreadTime);
    GpuSumMs    += FPlatformTime::ToMilliseconds(RHIGetGPUFrameCycles(0));
    if (LogAccumulator >= 5.0)
    {
        LogAccumulator = 0.0;
        const FChronoThreadStats S = PhysicsThread->GetStats();
        UE_LOG(LogChronoSubsystem, Log,
               TEXT("physics: %llu steps, step %.1f/%.1f us (mean/max), ")
               TEXT("late %.1f/%.1f us, missed %llu, rt=%s | render: alpha %.3f span %d, ")
               TEXT("frame %.2f/%.2f ms (mean/max) = game %.2f render %.2f rhi %.2f gpu %.2f"),
               (unsigned long long)S.StepCount, S.MeanStepUs, S.MaxStepUs,
               S.MeanLateUs, S.MaxLateUs, (unsigned long long)S.MissedTicks,
               S.bRealtime ? TEXT("yes") : TEXT("no"),
               FrameState.Alpha, FrameState.SpanSteps,
               1000.0 * FrameSumS / FMath::Max(FrameCount, 1), 1000.0 * FrameMaxS,
               GameSumMs / FMath::Max(FrameCount, 1), RenderSumMs / FMath::Max(FrameCount, 1),
               RhiSumMs / FMath::Max(FrameCount, 1), GpuSumMs / FMath::Max(FrameCount, 1));
        FrameSumS = FrameMaxS = 0.0;
        GameSumMs = RenderSumMs = RhiSumMs = GpuSumMs = 0.0;
        FrameCount = 0;

        const FChronoWheelStats W = GetWheelStats();
        if (W.bConnected)
        {
            UE_LOG(LogChronoSubsystem, Log,
                   TEXT("wheel: '%hs' poll %.2f/%.2f us, ffb %llu calls @ %.2f us, ")
                   TEXT("rt=%s | car: %.1f km/h, %.0f rpm, gear %d, %.2f Nm"),
                   W.DeviceName, W.MeanPollUs, W.MaxPollUs,
                   (unsigned long long)W.FfbCalls, W.MeanFfbUs,
                   W.bRealtime ? TEXT("yes") : TEXT("no"),
                   S.SpeedMps * 3.6, S.EngineRpm, S.Gear, S.SteerTorqueNm);
        }
        else if (S.bVehicle)
        {
            UE_LOG(LogChronoSubsystem, Log,
                   TEXT("input: %s | car: %.1f km/h, %.0f rpm, gear %d"),
                   SourceName(S.Source), S.SpeedMps * 3.6, S.EngineRpm, S.Gear);
        }

        if (S.Source == EChronoInputSource::Script)
        {
            UE_LOG(LogChronoSubsystem, Log,
                   TEXT("script: t=%.3f / %.2f s%s"),
                   S.ScriptTime, S.ScriptLen,
                   S.bScriptEnded ? TEXT("  (ended, holding last inputs)") : TEXT(""));
        }
    }
}

void UChronoWorldSubsystem::DrainRingIntoReferee()
{
    if (!bRefereeReady || !PhysicsThread) { return; }

    const FChronoRing& Ring = PhysicsThread->GetRing();
    const FChronoThreadStats S = PhysicsThread->GetStats();

    // Walk every sample published since the last drain, OLDEST-FIRST, by
    // ABSOLUTE index (ReadAt). Indexing back from the head skipped a sample
    // whenever the producer published mid-loop -- see ReadAt. Samples the
    // ring no longer holds are simply absent, and the referee reports the
    // step gap. Already-judged steps are still skipped by step index.
    const uint64 Published = Ring.Count();
    uint64 First = NextRingIndex;
    if (Published > First + (GChronoRingSize - 1))
    {
        First = Published - (GChronoRingSize - 1);     // older ones are gone
    }
    NextRingIndex = Published;
    for (uint64 Index = First; Index < Published; ++Index)
    {
        ChsSnapshot Snap;
        if (!Ring.ReadAt(Index, Snap)) { continue; }
        if (Snap.step_index <= LastJudgedStep || Snap.body_count < 1) { continue; }

        // Yaw from the chassis quaternion, in CHRONO's frame. The referee and
        // the course are both in Chrono coordinates, so nothing converts here;
        // the handedness flip belongs to the renderer alone.
        const ChsQuat& Q = Snap.bodies[0].rotation;
        const double Yaw = FMath::Atan2(2.0 * (Q.w * Q.z + Q.x * Q.y),
                                        1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z));

        RefVehicleState V;
        FMemory::Memzero(&V, sizeof V);
        V.t    = Snap.sim_time;
        V.step = Snap.step_index;
        V.x    = Snap.bodies[0].position.x;
        V.y    = Snap.bodies[0].position.y;
        V.yaw  = Yaw;
        // SIGNED SPEED ALONG THE BODY X AXIS, which is what the referee's
        // rules are written against -- and what Chrono's own
        // ChChassis::GetSpeed() computes, as dot(world velocity, body X).
        // The snapshot's velocity is in WORLD axes (rf.GetPosDt()), and this
        // used to take its x component: right only while the car points
        // along world X. Reversing into the bay the car moves along world Y,
        // so it read ~0 m/s and looked parked while moving. Body X is the
        // first column of the rotation, straight from the quaternion.
        const ChsVec3& Vw = Snap.bodies[0].linear_velocity;
        const double Bx = 1.0 - 2.0 * (Q.y * Q.y + Q.z * Q.z);
        const double By = 2.0 * (Q.x * Q.y + Q.w * Q.z);
        const double Bz = 2.0 * (Q.x * Q.z - Q.w * Q.y);
        V.speed = Vw.x * Bx + Vw.y * By + Vw.z * Bz;

        // How far the WHEELS roll, for the rollback rule (task 7): a braked car
        // settles back on its tyres' compliance with its wheels locked, and
        // that is not the car rolling. The rear spindles (slots 3, 4; their
        // angular velocity is in WORLD axes) spinning about the body's lateral
        // axis, less the body's own rotation about it (slot 0, BODY axes).
        // ref_wheel_speed does the arithmetic, so this and the headless
        // harness compute the same number from the same snapshot.
        if (Snap.body_count >= 5)
        {
            const double Qd[4] = { Q.w, Q.x, Q.y, Q.z };
            const ChsVec3& Wb = Snap.bodies[0].angular_velocity;
            const double Wbd[3] = { Wb.x, Wb.y, Wb.z };
            const double Ws[2][3] = {
                { Snap.bodies[3].angular_velocity.x, Snap.bodies[3].angular_velocity.y,
                  Snap.bodies[3].angular_velocity.z },
                { Snap.bodies[4].angular_velocity.x, Snap.bodies[4].angular_velocity.y,
                  Snap.bodies[4].angular_velocity.z },
            };
            V.wheel_speed = ref_wheel_speed(Qd, Wbd, Ws, 2, Course.r_roll);
        }

        // Engine speed, gear and clutch are NOT in the snapshot -- the ring
        // carries poses. They come from the thread's stats, which are the
        // latest values rather than this sample's, so they are up to one frame
        // stale. That is acceptable for the only rule that uses them: stall
        // detection has 0.75 s of hysteresis, against 16.7 ms of staleness.
        // It would NOT be acceptable for anything edge-triggered, and any such
        // rule must move these into the snapshot instead.
        V.engine_rpm = S.EngineRpm;
        V.gear       = S.Gear;
        // The pedal, not a constant. This was 0.0 -- "always engaged" -- which
        // the stall rule read as drive connected at every stop in gear, so a
        // driver waiting at the hill's stop line with the clutch down and the
        // engine under REF_STALL_RPM scored a STALL. Harmless while the
        // referee only logged; with a verdict (task 4) it fails the test.
        V.clutch     = S.Clutch;

        ref_update(&Referee, &V);
        LastJudgedStep = Snap.step_index;
    }

    // Report only what is new, so the log does not repeat the whole session.
    RefEvent Events[REF_MAX_EVENTS];
    const int32 N = ref_get_events(&Referee, Events, REF_MAX_EVENTS);
    bool bExerciseEnded = false;
    for (int32 i = LastReportedEvents; i < N; ++i)
    {
        UE_LOG(LogChronoSubsystem, Warning,
               TEXT("REFEREE  t=%.3f  %hs  object=%d  detail=%.4f"),
               Events[i].t, ref_event_name(Events[i].type),
               Events[i].object_id, Events[i].detail);
        bExerciseEnded |= Events[i].type == REF_EX_PASS || Events[i].type == REF_EX_FAIL;
    }
    LastReportedEvents = N;

    // The running verdict, once per exercise that ends -- not per frame.
    if (bExerciseEnded)
    {
        RefVerdict Verdict;
        ref_verdict(&Referee, &Verdict);
        UE_LOG(LogChronoSubsystem, Warning,
               TEXT("VERDICT  %hs  passed %d/%d  failed %d  unfinished %d  general faults %d%s"),
               ref_result_name(Verdict.result), Verdict.passed, Verdict.total,
               Verdict.failed, Verdict.unfinished, Verdict.general_faults,
               Verdict.suspect ? TEXT("  (SUSPECT: samples missed)") : TEXT(""));
    }
}

TStatId UChronoWorldSubsystem::GetStatId() const
{
    RETURN_QUICK_DECLARE_CYCLE_STAT(UChronoWorldSubsystem, STATGROUP_Tickables);
}

FChronoThreadStats UChronoWorldSubsystem::GetPhysicsStats() const
{
    return PhysicsThread ? PhysicsThread->GetStats() : FChronoThreadStats{};
}

FChronoWheelStats UChronoWorldSubsystem::GetWheelStats() const
{
    return WheelThread ? WheelThread->GetStats() : FChronoWheelStats{};
}
