// ============================================================================
//  ChronoMirrorComponent.h — one exact planar mirror.
//
//  Drives a USceneCaptureComponent2D from mm_ue_mirror_capture in
//  mirror_math.h -- position, orientation AND projection, all from one function
//  that mirror/run.sh selftest verifies host-side (no GPU). What needs pixels
//  is measured by `mirror/run.sh render`, which places coloured markers where
//  the geometry says each mirror must show them and finds them in the frame.
//
//  WHY THIS IS NOT UE's BUILT-IN PLANAR REFLECTION:
//  UPlanarReflectionComponent renders a full symmetric frustum and resolves
//  through screen-space UVs. That is the right trade on a GPU with headroom.
//  This machine has an RX 550 driving three mirrors at 60 Hz, and measured on
//  the door-mirror geometry, a symmetric frustum wide enough to contain the
//  mirror needs 3.8x the horizontal pixels of the off-axis frustum that
//  contains exactly the mirror and nothing else.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "Components/SceneComponent.h"

#include "ChronoMirrorComponent.generated.h"

class APlayerController;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;
class UStaticMeshComponent;
class UMaterialInstanceDynamic;

UCLASS(ClassGroup = (Chrono), meta = (BlueprintSpawnableComponent))
class CHRONOBRIDGE_API UChronoMirrorComponent : public USceneComponent
{
    GENERATED_BODY()

public:
    UChronoMirrorComponent();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
                               FActorComponentTickFunction* ThisTickFunction) override;

    /** Glass size in centimetres (Unreal units). The component's local +X is
     *  the mirror normal, +Y its width axis, +Z its height axis. */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    float MirrorWidth = 25.0f;

    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    float MirrorHeight = 6.0f;

    /**
     * Render one frame in N. 1 = every frame.
     *
     * The budget argument, not a quality knob: three mirrors are three extra
     * scene traversals, and on this GPU the draw-call cost dominates the pixel
     * cost. A door mirror at 30 Hz against a 60 Hz main view is a far better
     * trade than dropping the main view, and the content of a mirror at
     * parking speed changes slowly. Raise it for the interior mirror last --
     * that is the one used at speed.
     */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror", meta = (ClampMin = "1"))
    int32 UpdateDivisor = 2;

    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror", meta = (ClampMin = "16"))
    int32 TargetWidth = 512;

    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror", meta = (ClampMin = "16"))
    int32 TargetHeight = 128;

    // There used to be a bFlipOffAxisSkew here: one sign, shared by both off-
    // centre terms, documented as a one-bit unknown to settle by eye. It was
    // not an unknown and it could never have been right. The capture renders
    // in a proper rotation, a mirror image is left-handed, and so the two
    // terms need OPPOSITE signs relative to the Kooima extents -- see
    // mm_ue_mirror_capture and the test pinning it. The extents are now
    // measured in the capture's own axes and nothing is left to flip.

    /** Where the driver's eye is, in world space. Set once per frame by
     *  whatever owns the camera. A mirror is only correct for ONE eye
     *  position, which is why this is an input and not a guess. */
    UFUNCTION(BlueprintCallable, Category = "Chrono|Mirror")
    void SetEyeLocation(const FVector& InEyeWorld) { EyeWorld = InEyeWorld; }

    /** The render target the mirror material should sample. */
    UFUNCTION(BlueprintCallable, Category = "Chrono|Mirror")
    UTextureRenderTarget2D* GetMirrorTarget() const { return Target; }

    /**
     * Writes this mirror's render target to Saved/MirrorDumps/<name>.png.
     *
     * THE POINT IS TO SPLIT ONE SYMPTOM INTO TWO FAULTS. "The mirror is
     * black" can mean the capture produced nothing, or that it produced a
     * perfectly good image the glass is not showing. Looking at the glass in
     * a rendered frame cannot tell those apart -- and three render attempts
     * were spent not telling them apart. The target on disk can: if this PNG
     * has a scene in it, everything upstream of the glass works.
     */
    void DumpTargetToDisk() const;

    /** Draw a glass plane showing the reflection. Off if the car mesh supplies
     *  its own mirror geometry and just wants the render target. */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    bool bBuildGlass = true;

    /**
     * FALSE, and it was briefly set true for a reason that turned out wrong.
     *
     * Because the frustum IS the mirror aperture, the render target maps 1:1
     * onto the glass UVs with no screen-space projection; only the directions
     * are in question. The capture's image +x is the glass's +Y (its `right`,
     * see mm_ue_mirror_capture), so target u = 1 belongs on the glass's +Y
     * edge. MEASURED with -ChronoMirrorUV (swap on, both flips off): the +Y
     * edge -- the driver's screen-LEFT on the interior mirror -- already
     * sampled u = 1. So the raw mapping is right and U needs no flip.
     *
     * It was set true from the premise that target u = 0 sits at pa, the +Y
     * edge. That holds in Kooima's basis, which is left-handed; no Unreal
     * capture renders in it. With it true the reflection is left-right
     * reversed. `mirror/run.sh render` measures the result rather than
     * trusting either argument.
     */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    bool bFlipMirrorU = false;

    /**
     * The vertical counterpart, and it exists because the glass is stood up
     * by a rotation whose sign was wrong in the first version: the plane's
     * +Z normal maps to the mirror's -X under a +90 pitch, not +X. That is
     * now handled by a two-sided material rather than by re-deriving the
     * rotation -- which means the plane is viewed from its BACK face, and a
     * face viewed from behind has its V running the other way.
     *
     * MEASURED the same way: the glass's screen-TOP edge sampled V=1 and its
     * screen-BOTTOM edge V=0. Target V=0 is the image's TOP row -- the glass's
     * +Z edge, since the capture's up is the glass's up -- so the glass was
     * showing the reflection upside down, visible directly as the road above
     * the sky. Hence true.
     *
     * Both flags are real parameters in M_Mirror now; before this task the
     * component set a "FlipU" scalar that the material did not have, so the
     * flag could never have done anything at all.
     */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    bool bFlipMirrorV = true;

    /**
     * Transposes the sampled UV before the two flips.
     *
     * REQUIRED, not a third guess. The glass is an /Engine Plane stood up by
     * a +90 pitch, which puts the plane's local X on the mirror's HEIGHT axis
     * and its local Y on the WIDTH axis -- so the plane's U runs down the
     * glass while the render target's U runs across it. Measured from a
     * dumped target against the same frame's glass: content occupying the
     * target's left 66% at full height appeared on the glass across the full
     * width in the bottom 66%. That is a 90 degree rotation, and two axis
     * flips cannot produce one.
     *
     * Swap plus the two flips span all eight orientations, which is what
     * makes this settleable in one render rather than one render per guess.
     */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    bool bSwapMirrorUV = true;

    /**
     * Show the sampled coordinate as colour instead of the reflection: red
     * rises with U, green with V, so the corner that renders BLACK is target
     * (0,0). Armed with -ChronoMirrorUV. One render reads the whole mapping
     * off, instead of guessing at a grey quad seen obliquely.
     */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    bool bDebugUV = false;

    /**
     * THE MEASUREMENT, armed with -ChronoMirrorProbe.
     *
     * Puts two coloured markers in the world exactly where the geometry says
     * this mirror must show them: each on the ray from the reflected eye
     * through a chosen, off-centre point of the glass, some metres beyond it.
     * By the law of reflection the driver then sees that marker AT that glass
     * point, and the render target must hold it at the matching (u, v).
     *
     * LogProbeExpectations writes both predictions; mirror/probe.py finds the
     * markers in the dumped target and in the screenshot and reports the
     * error in pixels. That one number covers the frustum, the capture's
     * orientation, the clip plane, the glass's UV swap and both flips -- the
     * things that were previously settled by looking at a grey quad.
     */
    UPROPERTY(EditAnywhere, Category = "Chrono|Mirror")
    bool bProbe = false;

    /** Hides this mirror's probe markers from `Other`'s capture, so every
     *  target shows only its own markers. No-op without probes. */
    void HideProbesFrom(const UChronoMirrorComponent* Other) const;

    /** Logs where each probe marker must appear: in this mirror's target,
     *  and -- projected through `PC`'s camera -- on screen, on the glass.
     *  One "MirrorProbe" line per marker, parsed by mirror/probe.py. */
    void LogProbeExpectations(APlayerController* PC) const;

private:
    /** Logs which way the drawn glass face points, so "the mirror is black"
     *  can be told apart from "the glass is culled" without rendering. */
    void LogGlassFacing() const;

    /** Projects the glass's corners and centre through the capture's REAL
     *  view-projection -- built from the capture's transform the way
     *  SceneCaptureRendering.cpp builds it -- and checks each lands on its
     *  own NDC corner. mm_ue_mirror_capture is tested on the host; this is
     *  the check that Unreal was handed the same thing. */
    void CheckCornersLandOnNdcEdges() const;

    void BuildProbes();
    /** Re-places the markers on their rays; `EyeRefl` is the capture position. */
    void PlaceProbes(const FVector& EyeRefl);
    /** Glass point of marker `Index`, world space. */
    FVector ProbeGlassPoint(int32 Index) const;

    UPROPERTY(Transient) TObjectPtr<USceneCaptureComponent2D> Capture;
    UPROPERTY(Transient) TObjectPtr<UTextureRenderTarget2D>   Target;
    UPROPERTY(Transient) TObjectPtr<UStaticMeshComponent>     Glass;
    UPROPERTY(Transient) TObjectPtr<UMaterialInstanceDynamic> GlassMID;
    UPROPERTY(Transient) TArray<TObjectPtr<UStaticMeshComponent>> Probes;

    FVector EyeWorld = FVector::ZeroVector;
    int32   FrameCounter = 0;
    /** +1 or -1: which way along the glass's +Y is outboard, so door-mirror
     *  markers sit clear of the car's own flank. */
    double  ProbeSide = 1.0;
    /** The NDC corner check is a setup property, not a per-frame one. */
    bool    bCheckedNdc = false;
    bool    bWarnedGeometry = false;
};
