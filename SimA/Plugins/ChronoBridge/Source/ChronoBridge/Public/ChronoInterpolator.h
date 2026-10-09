// ============================================================================
//  ChronoInterpolator.h — turning 1000 Hz of samples into smooth 60 Hz motion.
//
//  THE PROBLEM, CONCRETELY:
//  Physics publishes every 1.000 ms. The renderer draws every 16.667 ms. The
//  ratio is 16.667 -- NOT an integer. So the newest snapshot at draw time is
//  somewhere between 0 and 1 ms old, and that age walks:
//
//      frame 0 draws snapshot 16, which is 0.667 ms stale
//      frame 1 draws snapshot 33, which is 0.334 ms stale
//      frame 2 draws snapshot 50, which is 0.001 ms stale
//      frame 3 draws snapshot 66, which is 0.668 ms stale   <- wraps
//
//  A car at 60 km/h covers 16.7 mm/ms. That wrapping staleness makes the car
//  advance 16.7 mm, 16.7 mm, 16.7 mm, then 0 mm, then 16.7 mm... a periodic
//  stutter at the beat frequency between the two rates. It is small, perfectly
//  regular, and extremely visible -- the eye is far better at detecting
//  irregular motion than absolute position error.
//
//  Taking "the latest sample" is therefore NOT free just because samples are
//  plentiful. Oversampling does not fix a phase mismatch.
//
//  THE FIX:
//  Render at a deliberate delay behind the newest sample, and interpolate
//  between the two snapshots that bracket that render time. Positions become a
//  continuous function of wall-clock time, so equal wall-clock intervals give
//  equal visual motion -- which is exactly what "smooth" means.
//
//  COST: one render delay of latency, here 2 ms. That is a fifth of one 60 Hz
//  frame and imperceptible, whereas the stutter it removes is not. Extrapolation
//  would avoid the latency but must guess future state; on a collision it
//  guesses wrong and the correction is a visible snap. For a parking test full
//  of low-speed contact, interpolation is the right trade.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "ChronoStateRing.h"

/** One body's pose in Unreal's coordinate system and units. */
struct FChronoPose
{
    FVector  Location = FVector::ZeroVector;
    FQuat    Rotation = FQuat::Identity;
    FVector  LinearVelocity = FVector::ZeroVector;
};

struct FChronoInterpResult
{
    TArray<FChronoPose> Poses;
    double  RenderTime   = 0.0;
    double  Alpha        = 0.0;   // blend factor actually used, 0..1
    int32   SpanSteps    = 0;     // how many physics steps the two snapshots span
    bool    bExtrapolated = false;// true if we ran past the newest sample
    bool    bValid       = false;
};

/**
 * Reads the ring and produces interpolated poses for the current frame.
 * Lives on the game thread; holds no locks.
 */
class CHRONOBRIDGE_API FChronoInterpolator
{
public:
    /** Render this far behind the newest sample. Two physics steps at 1 kHz. */
    void SetDelaySeconds(double In) { DelaySeconds = In; }

    /**
     * @param Ring          the producer's ring
     * @param NowSimSeconds current sim-clock estimate on the game thread
     */
    template <uint32 N>
    FChronoInterpResult Evaluate(const TChronoStateRing<N>& Ring, double NowSimSeconds)
    {
        FChronoInterpResult Out;

        ChsSnapshot Newest;
        if (!Ring.Read(Newest, 0)) { return Out; }

        const double RenderTime = NowSimSeconds - DelaySeconds;
        Out.RenderTime = RenderTime;

        // Walk backwards for the first snapshot at or before RenderTime. The
        // ring is small and the answer is nearly always 1-3 slots back, so a
        // linear scan beats any cleverer structure.
        ChsSnapshot Older = Newest, NewerS = Newest;
        bool bFound = false;
        for (uint32 Back = 0; Back < N; ++Back)
        {
            ChsSnapshot S;
            if (!Ring.Read(S, Back)) { break; }
            if (S.sim_time <= RenderTime) { Older = S; bFound = true; break; }
            NewerS = S;
        }

        if (!bFound)
        {
            // RenderTime is older than everything we hold: the game thread
            // stalled for longer than the ring covers (64 ms at 1 kHz). Snap to
            // the oldest we have rather than extrapolate wildly.
            Out.Poses = ToPoses(NewerS);
            Out.Alpha = 0.0;
            Out.bValid = true;
            return Out;
        }

        const double Span = NewerS.sim_time - Older.sim_time;
        double Alpha = (Span > 1e-9) ? (RenderTime - Older.sim_time) / Span : 0.0;

        if (Alpha > 1.0)
        {
            // RenderTime is beyond the newest sample -- physics has fallen
            // behind. Clamp rather than extrapolate: a frozen frame reads as a
            // tiny hitch, whereas extrapolating a contact-rich scene produces
            // interpenetration that then snaps back.
            Alpha = 1.0;
            Out.bExtrapolated = true;
        }
        Alpha = FMath::Clamp(Alpha, 0.0, 1.0);

        Out.Alpha     = Alpha;
        Out.SpanSteps = int32(NewerS.step_index - Older.step_index);
        Out.Poses     = Blend(Older, NewerS, Alpha);
        Out.bValid    = true;
        return Out;
    }

private:
    double DelaySeconds = 0.002;   // 2 ms

    // Chrono is right-handed, Z-up, metres. Unreal is LEFT-handed, Z-up,
    // centimetres. The handedness flip is a Y negation, and it must be applied
    // to the quaternion as well -- mirroring a rotation negates the components
    // whose axes are NOT flipped. Getting this wrong yields a car that steers
    // the wrong way, which is a very confusing bug to chase from the visuals.
    static FVector ToUE(const ChsVec3& V)
    {
        return FVector(V.x * 100.0, -V.y * 100.0, V.z * 100.0);
    }
    static FQuat ToUE(const ChsQuat& Q)
    {
        return FQuat(-Q.x, Q.y, -Q.z, Q.w);
    }

    static TArray<FChronoPose> ToPoses(const ChsSnapshot& S)
    {
        TArray<FChronoPose> P;
        P.SetNum(S.body_count);
        for (int32 i = 0; i < S.body_count; ++i)
        {
            P[i].Location       = ToUE(S.bodies[i].position);
            P[i].Rotation       = ToUE(S.bodies[i].rotation);
            P[i].LinearVelocity = ToUE(S.bodies[i].linear_velocity);
        }
        return P;
    }

    static TArray<FChronoPose> Blend(const ChsSnapshot& A, const ChsSnapshot& B, double T)
    {
        const int32 N = FMath::Min(A.body_count, B.body_count);
        TArray<FChronoPose> P;
        P.SetNum(N);
        for (int32 i = 0; i < N; ++i)
        {
            const FVector PA = ToUE(A.bodies[i].position);
            const FVector PB = ToUE(B.bodies[i].position);
            P[i].Location = FMath::Lerp(PA, PB, T);

            // Slerp, not Lerp, for rotation: a normalised linear blend takes
            // the chord instead of the arc, so angular velocity varies through
            // the interval. Over 1 ms the error is tiny, but it is systematic,
            // and it is exactly the kind of thing that makes a wheel look like
            // it is rotating unevenly.
            const FQuat QA = ToUE(A.bodies[i].rotation);
            const FQuat QB = ToUE(B.bodies[i].rotation);
            P[i].Rotation = FQuat::Slerp(QA, QB, T).GetNormalized();

            P[i].LinearVelocity = FMath::Lerp(ToUE(A.bodies[i].linear_velocity),
                                              ToUE(B.bodies[i].linear_velocity), T);
        }
        return P;
    }
};
