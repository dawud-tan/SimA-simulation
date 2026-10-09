// ============================================================================
//  ChronoHUD.h — what the driver sees besides the road (task 5).
//
//  Everything shown is already published: the physics thread's stats (speed,
//  rpm, gear, pedals, loop timing) and the referee (events, exercise results,
//  the verdict), both through UChronoWorldSubsystem. The HUD owns no state of
//  its own beyond a few seconds of message history and reads nothing from
//  Chrono.
//
//  WHY A CANVAS HUD AND NOT UMG:
//  UMG widgets are content assets, and SimA/Content holds only what two
//  generator scripts write. A canvas HUD is code: engine fonts, rectangles and
//  text, drawn every frame. Same reasoning as keyboard input polling
//  IsInputKeyDown instead of Enhanced Input.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"

#include "ChronoHUD.generated.h"

UCLASS()
class CHRONOBRIDGE_API AChronoHUD : public AHUD
{
    GENERATED_BODY()

public:
    virtual void DrawHUD() override;

    /** Logs the mirror report in the next frame drawn even if nothing
     *  changed, marked "at the shot": the -ChronoShot timer calls it, so
     *  cockpit/probe.py measures the frame it is given, not the last frame
     *  in which a mirror happened to go on or off screen (task 17). */
    void ReportMirrorsNextFrame() { bReportNextFrame = true; }

    /** Seconds a referee message stays on screen. */
    UPROPERTY(EditAnywhere, Category = "Chrono")
    float MessageSeconds = 8.0f;

private:
    void DrawInstruments(const struct FChronoThreadStats& S, float K);
    void DrawTest(class UChronoWorldSubsystem* Sub, float K);
    void DrawMessages(class UChronoWorldSubsystem* Sub, const struct FChronoThreadStats& S, float K);
    void DrawTiming(const struct FChronoThreadStats& S, float K);
    /** A panel's backdrop; also recorded for CheckMirrorsClear. */
    void DrawPanel(float X, float Y, float W, float H);
    /** Logs how much of each mirror's glass the panels cover, when a mirror
     *  goes off screen, clear or covered. */
    void CheckMirrorsClear();

    /** This frame's panels, screen pixels. */
    TArray<FBox2D> Panels;
    FString LastMirrorState;
    bool    bReportNextFrame = false;

    /** Referee events already turned into messages, by count. */
    int32 SeenEvents = 0;
    struct FMessage { FString Text; FLinearColor Color; double ShownAt; };
    TArray<FMessage> Messages;
};
