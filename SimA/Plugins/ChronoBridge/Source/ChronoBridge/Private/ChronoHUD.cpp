#include "ChronoHUD.h"

#include "ChronoMirrorComponent.h"
#include "ChronoPhysicsThread.h"
#include "ChronoWorldSubsystem.h"
#include "referee.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Engine/Font.h"
#include "Engine/World.h"
#include "UObject/UObjectIterator.h"

DEFINE_LOG_CATEGORY_STATIC(LogChronoHUD, Log, All);

namespace
{
// Laid out at 1280 x 720 and scaled with the canvas height.
constexpr float kDesignHeight = 720.0f;

const FLinearColor kPanel (0.00f, 0.00f, 0.00f, 0.55f);
const FLinearColor kText  (0.95f, 0.95f, 0.95f, 1.0f);
const FLinearColor kDim   (0.62f, 0.62f, 0.62f, 1.0f);
const FLinearColor kGood  (0.35f, 0.90f, 0.45f, 1.0f);
const FLinearColor kBad   (1.00f, 0.35f, 0.30f, 1.0f);
const FLinearColor kWarn  (1.00f, 0.80f, 0.25f, 1.0f);
const FLinearColor kAccent(0.40f, 0.72f, 1.00f, 1.0f);
const FLinearColor kTrack (1.00f, 1.00f, 1.00f, 0.15f);

// The engine's redline, gen_innova.py's torque map ends at 4400 rpm.
constexpr double kRpmMax = 4400.0;

FString GearName(int32 G)
{
    return G < 0 ? FString(TEXT("R")) : (G == 0 ? FString(TEXT("N")) : FString::FromInt(G));
}

// Named from the course's own data, so the HUD holds no copy of the course:
// the gated exercise is the slalom and the hill kind is the hill; a parking
// zone's heading tells the two parks apart -- parked along the road (0 deg)
// is the parallel park, reversed into a bay (-90 deg) is the bay park.
FString ExerciseName(const RefCourse* C, const RefExercise& E)
{
    switch (E.kind)
    {
    case REF_EXK_GATED: return TEXT("Slalom");
    case REF_EXK_HILL:  return TEXT("Hill start");
    case REF_EXK_PARK:
        for (int32 i = 0; i < C->n_zones; ++i)
        {
            if (C->zones[i].id == E.zone)
            {
                return FMath::Abs(FMath::Sin(C->zones[i].heading)) < 0.5
                    ? TEXT("Parallel park") : TEXT("Bay park");
            }
        }
        return TEXT("Park");
    default:
        return FString::Printf(TEXT("Exercise %d"), E.id);
    }
}

FString ExerciseNameById(const RefCourse* C, int32 Id)
{
    for (int32 i = 0; i < C->n_exercises; ++i)
    {
        if (C->exercises[i].id == Id) { return ExerciseName(C, C->exercises[i]); }
    }
    return FString::Printf(TEXT("exercise %d"), Id);
}

// A referee event in words, or empty for the ones a driver does not need to
// be told about as they happen (gates passed in order, sample gaps).
FString EventText(const RefCourse* C, const RefEvent& E, FLinearColor& Color)
{
    Color = kText;
    switch (E.type)
    {
    case REF_CONE_STRIKE:  Color = kBad;  return TEXT("Cone struck");
    case REF_LINE_CROSS:   Color = kBad;  return TEXT("Line crossed");
    case REF_GATE_SKIP:    Color = kWarn; return TEXT("Gate taken out of order");
    case REF_GATE_MISS:    Color = kBad;  return TEXT("Gate missed");
    case REF_STALL:        Color = kBad;  return TEXT("Engine stalled");
    case REF_PARK_OK:      Color = kGood;
        return FString::Printf(TEXT("Parked: %.0f cm clear of every line"), E.detail * 100.0);
    case REF_PARK_OUTSIDE: Color = kWarn;
        return FString::Printf(TEXT("Not in the space (%.0f cm)"), E.detail * 100.0);
    case REF_PARK_ANGLE:   Color = kWarn;
        return FString::Printf(TEXT("Not square to the space (%.1f deg)"), E.detail);
    case REF_ROLLBACK:     Color = kBad;
        return FString::Printf(TEXT("Rolled back on the hill (%.0f cm)"), -E.detail * 100.0);
    case REF_EX_START:     Color = kAccent;
        return ExerciseNameById(C, E.object_id) + TEXT(": started");
    case REF_EX_PASS:      Color = kGood;
        return FString::Printf(TEXT("%s: PASSED (%.1f s)"), *ExerciseNameById(C, E.object_id), E.detail);
    case REF_EX_FAIL:      Color = kBad;
        return FString::Printf(TEXT("%s: FAILED -- %s"), *ExerciseNameById(C, E.object_id),
                               UTF8_TO_TCHAR(ref_fail_reason_name((int32_t)E.detail)));
    default:
        return FString();
    }
}
} // namespace

void AChronoHUD::DrawHUD()
{
    Super::DrawHUD();
    if (!Canvas) { return; }
    UWorld* W = GetWorld();
    UChronoWorldSubsystem* Sub = W ? W->GetSubsystem<UChronoWorldSubsystem>() : nullptr;
    if (!Sub) { return; }

    const FChronoThreadStats S = Sub->GetPhysicsStats();
    const float K = Canvas->ClipY / kDesignHeight;
    Panels.Reset();
    if (S.bVehicle) { DrawInstruments(S, K); }
    DrawTest(Sub, K);
    DrawMessages(Sub, S, K);
    DrawTiming(S, K);
    CheckMirrorsClear();
}

void AChronoHUD::DrawPanel(float X, float Y, float W, float H)
{
    DrawRect(kPanel, X, Y, W, H);
    Panels.Add(FBox2D(FVector2D(X, Y), FVector2D(X + W, Y + H)));
}

/* A panel over a mirror hides what the mirror is there to show. Until task 9
 * the test panel sat squarely on the interior mirror: the glass was put in
 * the cockpit after the HUD was laid out, and nothing compared the two. Now
 * every frame projects each glass's corners through this frame's view and
 * measures how much of it the panels cover; the log gets a line whenever
 * a mirror goes off screen, clear or covered. Where the glass falls depends on the seat, the FOV and the
 * aspect ratio, so this is checked rather than laid out by hand. In a
 * -ChronoLookMirror view the glass fills the frame and is reported covered,
 * which is true and harmless there. */
void AChronoHUD::CheckMirrorsClear()
{
    const FBox2D Screen(FVector2D(0.0, 0.0), FVector2D(Canvas->ClipX, Canvas->ClipY));
    // Logged when the coarse state changes -- off screen, clear or covered,
    // per mirror -- not the numbers, which move every frame the car does.
    FString State, Report;
    for (TObjectIterator<UChronoMirrorComponent> It; It; ++It)
    {
        if (It->GetWorld() != GetWorld()) { continue; }
        // The glass is centred on the component: +Y across it, +Z up it.
        const FTransform& X = It->GetComponentTransform();
        const FVector C = X.GetLocation();
        const FVector R = X.TransformVectorNoScale(FVector::RightVector) * (It->MirrorWidth * 0.5);
        const FVector U = X.TransformVectorNoScale(FVector::UpVector) * (It->MirrorHeight * 0.5);
        FBox2D Glass(ForceInit);
        bool bInFront = true;
        for (const FVector& P : { C + R + U, C - R + U, C + R - U, C - R - U })
        {
            const FVector S = Project(P, /*bClampToZeroPlane*/ true);
            if (S.Z <= 0.0) { bInFront = false; break; }   // behind the eye
            Glass += FVector2D(S.X, S.Y);
        }
        if (!bInFront || !Glass.Intersect(Screen))
        {
            State += It->GetName() + TEXT(":off ");
            Report += FString::Printf(TEXT(" %s off-screen;"), *It->GetName());
            continue;
        }
        Glass = Glass.Overlap(Screen);
        // Panels do not overlap one another, so their overlaps add.
        double Covered = 0.0;
        for (const FBox2D& P : Panels)
        {
            if (P.Intersect(Glass)) { Covered += P.Overlap(Glass).GetArea(); }
        }
        State += It->GetName() + (Covered > 0.0 ? TEXT(":covered ") : TEXT(":clear "));
        Report += FString::Printf(TEXT(" %s at (%.0f,%.0f)-(%.0f,%.0f) %.1f%% covered;"),
                                  *It->GetName(), Glass.Min.X, Glass.Min.Y, Glass.Max.X,
                                  Glass.Max.Y, 100.0 * Covered / FMath::Max(Glass.GetArea(), 1.0));
    }
    if (State != LastMirrorState || bReportNextFrame)
    {
        UE_LOG(LogChronoHUD, Log, TEXT("HUD over mirrors%s (%d panels, %.0fx%.0f):%s"),
               bReportNextFrame ? TEXT(" at the shot") : TEXT(""),
               Panels.Num(), Canvas->ClipX, Canvas->ClipY, *Report);
        LastMirrorState = State;
        bReportNextFrame = false;
    }
}

void AChronoHUD::DrawInstruments(const FChronoThreadStats& S, float K)
{
    UFont* Big = GEngine->GetLargeFont();
    UFont* Med = GEngine->GetMediumFont();
    UFont* Small = GEngine->GetSmallFont();
    const float X = 20.0f * K, W = 340.0f * K, H = 200.0f * K;
    const float Y = Canvas->ClipY - H - 20.0f * K;
    DrawPanel(X, Y, W, H);

    // Speed and gear, large. |speed|: reversing reads as speed, R says which way.
    DrawText(FString::Printf(TEXT("%.0f"), FMath::Abs(S.SpeedMps) * 3.6), kText,
             X + 16.0f * K, Y + 8.0f * K, Big, 2.2f * K);
    DrawText(TEXT("km/h"), kDim, X + 118.0f * K, Y + 44.0f * K, Med, 1.0f * K);
    DrawText(GearName(S.Gear), kAccent, X + 250.0f * K, Y + 8.0f * K, Big, 2.2f * K);
    DrawText(TEXT("gear"), kDim, X + 292.0f * K, Y + 44.0f * K, Small, 1.0f * K);

    // Revs: a bar to the redline. The governor holds idle at ~780 rpm.
    const float BarX = X + 16.0f * K, BarY = Y + 82.0f * K, BarW = W - 32.0f * K, BarH = 10.0f * K;
    const double Rpm = FMath::Max(0.0, S.EngineRpm);
    DrawRect(kTrack, BarX, BarY, BarW, BarH);
    DrawRect(Rpm > 0.9 * kRpmMax ? kBad : kAccent, BarX, BarY,
             BarW * float(FMath::Min(1.0, Rpm / kRpmMax)), BarH);
    DrawText(FString::Printf(TEXT("%.0f rpm"), Rpm), kDim, BarX, BarY + 12.0f * K, Small, 1.0f * K);

    // A dead engine reads 0 rpm (task 8: it dies under 300 rpm and restarts
    // after a second with the pedal down or in neutral).
    if (Rpm < 50.0)
    {
        DrawText(TEXT("ENGINE STALLED -- clutch down or neutral to restart"), kBad,
                 BarX, BarY + 30.0f * K, Small, 1.0f * K);
    }

    // Pedals, as commanded: clutch (1 = down), brake, throttle.
    const float PY = Y + 130.0f * K, PH = 56.0f * K, PW = 18.0f * K;
    const struct { const TCHAR* L; double V; FLinearColor C; } Pedals[3] = {
        { TEXT("C"), S.Clutch,   kWarn  },
        { TEXT("B"), S.Brake,    kBad   },
        { TEXT("A"), S.Throttle, kGood  },
    };
    for (int32 i = 0; i < 3; ++i)
    {
        const float PX = BarX + i * 30.0f * K;
        const float V = float(FMath::Clamp(Pedals[i].V, 0.0, 1.0));
        DrawRect(kTrack, PX, PY, PW, PH);
        DrawRect(Pedals[i].C, PX, PY + PH * (1.0f - V), PW, PH * V);
        DrawText(Pedals[i].L, kDim, PX + 4.0f * K, PY + PH + 1.0f * K, Small, 1.0f * K);
    }

    // The wheel: a centred bar, device convention (+ = right).
    const float SX = BarX + 110.0f * K, SW = W - 142.0f * K, SY = PY + 20.0f * K;
    const float Mid = SX + 0.5f * SW;
    const float St = float(FMath::Clamp(S.Steering, -1.0, 1.0));
    DrawRect(kTrack, SX, SY, SW, 12.0f * K);
    DrawRect(kAccent, St < 0.0f ? Mid + St * 0.5f * SW : Mid, SY, FMath::Abs(St) * 0.5f * SW, 12.0f * K);
    DrawRect(kText, Mid - 1.0f * K, SY - 4.0f * K, 2.0f * K, 20.0f * K);
    DrawText(FString::Printf(TEXT("steering %+.0f%%"), St * 100.0f), kDim, SX, SY + 16.0f * K,
             Small, 1.0f * K);
}

void AChronoHUD::DrawTest(UChronoWorldSubsystem* Sub, float K)
{
    UFont* Med = GEngine->GetMediumFont();
    UFont* Small = GEngine->GetSmallFont();
    const RefCourse* C = Sub->GetCourse();
    // On the dashboard, bottom right, above the timing line: a part of the
    // view the car already blocks. It was top left, squarely over the
    // interior mirror (task 9; CheckMirrorsClear measures it).
    const float W = 360.0f * K, Row = 22.0f * K;
    const float X = Canvas->ClipX - W - 20.0f * K;
    if (!C)
    {
        const float Y = Canvas->ClipY - 30.0f * K - 60.0f * K;
        DrawPanel(X, Y, W, 30.0f * K);
        DrawText(TEXT("Referee: no course loaded"), kWarn, X + 10.0f * K, Y + 6.0f * K, Small, 1.0f * K);
        return;
    }
    const float H = 44.0f * K + Row * C->n_exercises;
    const float Y = Canvas->ClipY - H - 60.0f * K;
    DrawPanel(X, Y, W, H);

    RefVerdict V;
    Sub->GetRefereeVerdict(V);
    FString Verdict;
    FLinearColor VC = kText;
    if (V.result == REF_RESULT_PASS)      { Verdict = TEXT("PASS"); VC = kGood; }
    else if (V.result == REF_RESULT_FAIL) { Verdict = TEXT("FAIL"); VC = kBad;  }
    else { Verdict = FString::Printf(TEXT("%d of %d passed"), V.passed, V.total); }
    DrawText(TEXT("SIM A test"), kText, X + 10.0f * K, Y + 8.0f * K, Med, 1.0f * K);
    if (!Sub->IsSessionStarted())
    {
        Verdict = TEXT("starting...");      // frames not yet steady: nothing judged
        VC = kWarn;
    }
    DrawText(Verdict, VC, X + 170.0f * K, Y + 8.0f * K, Med, 1.0f * K);
    if (V.suspect)
    {
        DrawText(TEXT("(samples missed)"), kWarn, X + 260.0f * K, Y + 12.0f * K, Small, 1.0f * K);
    }

    for (int32 i = 0; i < C->n_exercises; ++i)
    {
        const RefExerciseResult* R = Sub->GetExerciseResult(i);
        if (!R) { continue; }
        const float RY = Y + 38.0f * K + i * Row;
        FString State;
        FLinearColor SC = kDim;
        switch (R->state)
        {
        case REF_EX_RUNNING: State = TEXT("in progress"); SC = kAccent; break;
        case REF_EX_PASSED:
            State = FString::Printf(TEXT("passed  %.1f s"), R->t_end - R->t_start); SC = kGood; break;
        case REF_EX_FAILED:
            State = FString::Printf(TEXT("FAILED  %s"),
                                    UTF8_TO_TCHAR(ref_fail_reason_name(R->fail_reason)));
            SC = kBad; break;
        default: State = TEXT("-"); break;
        }
        if (C->exercises[i].kind == REF_EXK_HILL && R->rollback < -0.005)
        {
            State += FString::Printf(TEXT("  (back %.0f cm)"), -R->rollback * 100.0);
        }
        DrawText(FString::Printf(TEXT("%d  %s"), i + 1, *ExerciseName(C, C->exercises[i])),
                 R->state == REF_EX_RUNNING ? kText : kDim, X + 10.0f * K, RY, Small, 1.0f * K);
        DrawText(State, SC, X + 170.0f * K, RY, Small, 1.0f * K);
    }
}

void AChronoHUD::DrawMessages(UChronoWorldSubsystem* Sub, const FChronoThreadStats& S, float K)
{
    UWorld* W = GetWorld();
    const double Now = W ? W->GetRealTimeSeconds() : 0.0;

    // New referee events become messages. The log is append-only, so a count
    // is all the bookkeeping needed.
    static RefEvent Events[REF_MAX_EVENTS];
    const int32 N = Sub->GetRefereeEvents(Events, REF_MAX_EVENTS);
    const RefCourse* C = Sub->GetCourse();
    for (int32 i = SeenEvents; i < N && C; ++i)
    {
        FLinearColor Color;
        const FString Text = EventText(C, Events[i], Color);
        if (!Text.IsEmpty()) { Messages.Add({ Text, Color, Now }); }
    }
    SeenEvents = N;
    Messages.RemoveAll([&](const FMessage& M) { return Now - M.ShownAt > MessageSeconds; });

    // A scripted run says where it is.
    if (S.Source == EChronoInputSource::Script)
    {
        const FString T = S.bScriptEnded
            ? FString(TEXT("script finished"))
            : FString::Printf(TEXT("script %.1f / %.1f s"), S.ScriptTime, S.ScriptLen);
        Messages.RemoveAll([](const FMessage& M) { return M.Text.StartsWith(TEXT("script ")); });
        Messages.Insert({ T, kDim, Now }, 0);
    }

    UFont* Small = GEngine->GetSmallFont();
    const int32 Shown = FMath::Min(Messages.Num(), 7);
    if (Shown == 0) { return; }
    const float Row = 22.0f * K, W2 = 400.0f * K;
    const float X = Canvas->ClipX - W2 - 20.0f * K, Y = 20.0f * K;
    DrawPanel(X, Y, W2, 12.0f * K + Row * Shown);
    for (int32 i = 0; i < Shown; ++i)
    {
        const FMessage& M = Messages[Messages.Num() - Shown + i];
        DrawText(M.Text, M.Color, X + 10.0f * K, Y + 6.0f * K + i * Row, Small, 1.0f * K);
    }
}

void AChronoHUD::DrawTiming(const FChronoThreadStats& S, float K)
{
    UFont* Small = GEngine->GetSmallFont();
    const TCHAR* Src = S.Source == EChronoInputSource::Keyboard ? TEXT("keyboard")
                     : S.Source == EChronoInputSource::Wheel    ? TEXT("wheel")
                     : S.Source == EChronoInputSource::Script   ? TEXT("script") : TEXT("-");
    const FString T = FString::Printf(
        TEXT("physics %.0f us mean, %.0f max | late %.1f us | missed %llu | %s | input %s"),
        S.MeanStepUs, S.MaxStepUs, S.MeanLateUs, (unsigned long long)S.MissedTicks,
        S.bRealtime ? TEXT("RT") : TEXT("not RT"), Src);
    float TW = 0.0f, TH = 0.0f;
    GetTextSize(T, TW, TH, Small, 1.0f * K);
    const float X = Canvas->ClipX - TW - 30.0f * K, Y = Canvas->ClipY - TH - 26.0f * K;
    DrawPanel(X - 8.0f * K, Y - 4.0f * K, TW + 16.0f * K, TH + 8.0f * K);
    DrawText(T, S.MissedTicks ? kWarn : kDim, X, Y, Small, 1.0f * K);
}
