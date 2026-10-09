#include "ChronoGameMode.h"

#include "ChronoCourseActor.h"
#include "ChronoHUD.h"
#include "ChronoMirrorComponent.h"
#include "ChronoVehicleActor.h"

#include "Components/DirectionalLightComponent.h"
#include "Components/SkyAtmosphereComponent.h"
#include "Components/SkyLightComponent.h"
#include "Engine/DirectionalLight.h"
#include "Engine/SkyLight.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/DefaultPawn.h"
#include "GameFramework/GameUserSettings.h"
#include "GameFramework/HUD.h"
#include "GameFramework/PlayerController.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "UnrealClient.h"
#include "TimerManager.h"
#include "UObject/UObjectIterator.h"

DEFINE_LOG_CATEGORY_STATIC(LogChronoGameMode, Log, All);

AChronoGameMode::AChronoGameMode()
{
    // A plain DefaultPawn exists only so a PlayerController is spawned: the
    // keyboard path polls APlayerController::IsInputKeyDown directly (see
    // UChronoWorldSubsystem::BuildKeyMask, which does so because Enhanced
    // Input would need content assets that do not exist). The pawn itself is
    // never seen -- the view target is the car.
    DefaultPawnClass = ADefaultPawn::StaticClass();
    // Speed, gear, revs, pedals, the test's progress and the referee's calls
    // (task 5). A canvas HUD: code, not a UMG asset.
    HUDClass = AChronoHUD::StaticClass();
}

void AChronoGameMode::StartPlay()
{
    Super::StartPlay();

    UWorld* W = GetWorld();
    if (!W) { return; }

    FActorSpawnParameters P;
    P.SpawnCollisionHandlingOverride =
        ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    // IDEMPOTENT BY CONSTRUCTION.
    //
    // scripts/20-make-level.py bakes a course and a vehicle into
    // /Game/Maps/SimACourse so they can be refined in the editor. Spawning
    // unconditionally would then produce a SECOND course and a second car on
    // that map -- two vehicles reading the same physics stream and drawn on
    // top of each other, which looks like a rendering bug rather than a
    // duplicate actor.
    //
    // Checking the world beats a config flag someone has to remember to flip
    // when they change maps: the right thing happens on a bare map and on an
    // authored one, with no setting to get wrong.
    AChronoCourseActor*  ExistingCourse  = nullptr;
    AChronoVehicleActor* ExistingVehicle = nullptr;
    // `if` and not `for`: every one of these wants the FIRST match and breaks
    // immediately, which is what TActorIterator's operator bool already says.
    // The loop form also trips -Wunreachable-code-loop-increment, which is an
    // error here -- it only stayed hidden while this file was being folded
    // into the unity build.
    TActorIterator<AChronoCourseActor> CourseIt(W);
    if (CourseIt) { ExistingCourse = *CourseIt; }
    TActorIterator<AChronoVehicleActor> VehicleIt(W);
    if (VehicleIt) { ExistingVehicle = *VehicleIt; }

    if (bSpawnCourse && !ExistingCourse)
    {
        W->SpawnActor<AChronoCourseActor>(AChronoCourseActor::StaticClass(),
                                          FTransform::Identity, P);
    }

    AChronoVehicleActor* Car = ExistingVehicle;
    if (!Car && bSpawnVehicle)
    {
        Car = W->SpawnActor<AChronoVehicleActor>(AChronoVehicleActor::StaticClass(),
                                                 FTransform::Identity, P);
    }

    // An unlit course is indistinguishable from a broken one, and a bare map
    // brings no lighting of its own.
    // Force any light the level supplies to Movable and to a usable intensity.
    // The level's lights were placed by 20-make-level.py at engine defaults,
    // which with static lighting disabled contributed nothing.
    bool bHaveLight = false;
    TActorIterator<ADirectionalLight> SunIt(W);
    if (SunIt)
    {
        bHaveLight = true;
        SunIt->SetMobility(EComponentMobility::Movable);
        SunIt->GetLightComponent()->SetIntensity(8.0f);
        SunIt->SetActorRotation(FRotator(-55.0f, 35.0f, 0.0f));
        if (UDirectionalLightComponent* D = Cast<UDirectionalLightComponent>(SunIt->GetLightComponent()))
        {
            D->SetAtmosphereSunLight(true);
        }
    }
    TActorIterator<ASkyLight> SkyIt(W);
    if (SkyIt)
    {
        SkyIt->GetLightComponent()->SetMobility(EComponentMobility::Movable);
        SkyIt->GetLightComponent()->SetIntensity(3.0f);
        SkyIt->GetLightComponent()->RecaptureSky();
    }
    if (bSpawnLighting && !bHaveLight)
    {
        if (ADirectionalLight* Sun = W->SpawnActor<ADirectionalLight>(
                ADirectionalLight::StaticClass(),
                FTransform(FRotator(-55.0f, 35.0f, 0.0f)), P))
        {
            Sun->SetMobility(EComponentMobility::Movable);
            Sun->GetLightComponent()->SetIntensity(6.0f);
            if (UDirectionalLightComponent* D = Cast<UDirectionalLightComponent>(Sun->GetLightComponent()))
            {
                D->SetAtmosphereSunLight(true);
            }
        }
        if (ASkyLight* Sky = W->SpawnActor<ASkyLight>(ASkyLight::StaticClass(),
                                                      FTransform::Identity, P))
        {
            Sky->GetLightComponent()->SetMobility(EComponentMobility::Movable);
            Sky->GetLightComponent()->SetIntensity(1.0f);
        }
    }

    // A SKY (task 5). There was none: above the horizon the view was black,
    // which made every screenshot look broken and hid the horizon a driver
    // steers by. ASkyAtmosphere is engine code, no asset, and it draws the
    // sky from the directional light flagged as its sun above. It also shows
    // in the mirrors, which is right -- they reflect the sky too.
    TActorIterator<ASkyAtmosphere> SkyAtmoIt(W);
    if (!SkyAtmoIt && bSpawnLighting)
    {
        W->SpawnActor<ASkyAtmosphere>(ASkyAtmosphere::StaticClass(), FTransform::Identity, P);
    }

    // Look through the car. The DefaultPawn keeps input focus, which is all
    // the keyboard path needs.
    if (Car)
    {
        if (APlayerController* PC = W->GetFirstPlayerController())
        {
            PC->SetViewTarget(Car);
            // Hide the DefaultPawn. It exists only so a PlayerController is
            // spawned for the keyboard path; its debug sphere was rendering in
            // mid-air beside the car.
            if (APawn* HiddenPawn = PC->GetPawn())
            {
                HiddenPawn->SetActorHiddenInGame(true);
            }
        }
    }

    // 60 FRAMES A SECOND, NOT MORE (task 18). The display is 60 Hz
    // (CLAUDE.md, hard constraints), and with task 18's renderer settings the
    // GPU needs ~8.5 ms a frame, so an uncapped game ran ~100 frames a second
    // -- and the frames nobody can see cost the 1 kHz physics loop: over
    // 113,000 steps, 10 missed deadlines and a 2.25 ms worst step, where at
    // 35 frames a second it had missed none (task 9). Set on the user
    // settings, not as t.MaxFPS in an ini: UGameUserSettings re-applies its
    // own FrameRateLimit (0, unlimited) over the cvar at the same priority,
    // and a saved GameUserSettings.ini would win over a project default.
    // -ChronoMaxFPS=<n> overrides it; 0 is uncapped.
    if (UGameUserSettings* Settings = GEngine ? GEngine->GetGameUserSettings() : nullptr)
    {
        float MaxFps = 60.0f;
        FParse::Value(FCommandLine::Get(), TEXT("ChronoMaxFPS="), MaxFps);
        Settings->SetFrameRateLimit(MaxFps);
        Settings->ApplyNonResolutionSettings();
        UE_LOG(LogChronoGameMode, Log, TEXT("Frame rate limit: %s."),
               MaxFps > 0.0f ? *FString::Printf(TEXT("%.0f Hz"), MaxFps) : TEXT("none"));
    }

    // -ChronoShot=<seconds> renders for that long, saves a screenshot and
    // quits. It exists because what a mirror shows can only be confirmed from
    // a rendered frame, and an automated run that produces an image is the
    // difference between "compiles" and "works". With -ChronoMirrorProbe the
    // confirmation is a number: mirror/probe.py, via `mirror/run.sh render`.
    float ShotDelay = 0.0f;
    if (FParse::Value(FCommandLine::Get(), TEXT("ChronoShot="), ShotDelay) &&
        ShotDelay > 0.0f)
    {
        FTimerHandle H;
        W->GetTimerManager().SetTimer(H, [this]()
        {
            UWorld* TW = GetWorld();
            APlayerController* PC = TW ? TW->GetFirstPlayerController() : nullptr;
            if (PC)
            {
                UE_LOG(LogChronoGameMode, Log, TEXT("Taking screenshot."));

                // Dump every mirror's render target beside the screenshot.
                //
                // A screenshot alone cannot distinguish "the capture rendered
                // nothing" from "the capture rendered fine and the glass is
                // not showing it" -- both are a black rectangle, and three
                // render attempts were spent not telling them apart. The
                // targets on disk settle it: a scene in the PNG means
                // everything upstream of the glass works. Found by component
                // type rather than through the vehicle actor, so this also
                // covers mirrors placed by hand in the level.
                //
                // With -ChronoMirrorProbe each mirror also logs where its
                // markers must appear, in its target and on screen, from the
                // same frame the dumps come from; mirror/probe.py compares.
                for (TObjectIterator<UChronoMirrorComponent> It; It; ++It)
                {
                    if (It->GetWorld() == TW)
                    {
                        It->DumpTargetToDisk();
                        It->LogProbeExpectations(PC);
                    }
                }
                // And where the steering wheel's stripe must be, for a
                // -ChronoLookWheel frame (cockpit/probe.py).
                for (TActorIterator<AChronoVehicleActor> It(TW); It; ++It)
                {
                    It->LogWheelExpectation(PC);
                    It->LogGlance();
                }

                // -ChronoShotUI takes the viewport AS DRAWN, the HUD
                // included, to Saved/Screenshots/ChronoHUD.png. Otherwise the
                // frame is the 3-D scene ALONE, which is what the mirror and
                // wheel probes measure -- and that has to be made so:
                // HighResShot leaves out Slate, not the canvas HUD, which is
                // drawn into the viewport. Task 5 recorded the scene-only
                // frame as given; task 9's frames showed the panels in it.
                if (FParse::Param(FCommandLine::Get(), TEXT("ChronoShotUI")))
                {
                    // Where each glass is, and what covers it, in this frame.
                    if (AChronoHUD* Hud = Cast<AChronoHUD>(PC->GetHUD()))
                    {
                        Hud->ReportMirrorsNextFrame();
                    }
                    FScreenshotRequest::RequestScreenshot(
                        FPaths::ScreenShotDir() / TEXT("ChronoHUD.png"),
                        /*bInShowUI*/ true, /*bAddUniqueSuffix*/ false);
                }
                else
                {
                    if (PC->MyHUD) { PC->MyHUD->bShowHUD = false; }
                    PC->ConsoleCommand(TEXT("HighResShot 1280x720"), true);
                }
                FTimerHandle Q;
                TW->GetTimerManager().SetTimer(Q, [](){
                    FGenericPlatformMisc::RequestExit(false);
                }, 3.0f, false);
            }
        }, ShotDelay, false);
        UE_LOG(LogChronoGameMode, Log,
               TEXT("Screenshot scheduled at t=%.1fs, then quit."), ShotDelay);

        // -ChronoProfileGPU: one frame's GPU time, pass by pass, into the log
        // (UE's ProfileGPU), 5 s before the shot -- steady play, clear of the
        // screenshot's own stall. Task 18.
        if (FParse::Param(FCommandLine::Get(), TEXT("ChronoProfileGPU")) && ShotDelay > 5.0f)
        {
            FTimerHandle ProfileTimer;
            W->GetTimerManager().SetTimer(ProfileTimer, [this]()
            {
                UWorld* TW = GetWorld();
                if (APlayerController* PC = TW ? TW->GetFirstPlayerController() : nullptr)
                {
                    UE_LOG(LogChronoGameMode, Log, TEXT("Profiling one frame on the GPU."));
                    PC->ConsoleCommand(TEXT("ProfileGPU"), true);
                }
            }, ShotDelay - 5.0f, false);
        }
    }

    UE_LOG(LogChronoGameMode, Log,
           TEXT("Scene ready. course: %s, vehicle: %s, lighting: %s."),
           ExistingCourse  ? TEXT("from level") : (bSpawnCourse  ? TEXT("spawned") : TEXT("none")),
           ExistingVehicle ? TEXT("from level") : (bSpawnVehicle ? TEXT("spawned") : TEXT("none")),
           bHaveLight      ? TEXT("from level") : (bSpawnLighting ? TEXT("spawned") : TEXT("none")));
}
