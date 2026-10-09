#include "ChronoMirrorComponent.h"

#include "mirror_math.h"

#include "Components/SceneCaptureComponent2D.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GameFramework/Actor.h"
#include "Camera/PlayerCameraManager.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "Misc/Paths.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"

DEFINE_LOG_CATEGORY_STATIC(LogChronoMirror, Log, All);

namespace
{
    // Must match TEX_PARAM in scripts/21-make-mirror-material.py. A mismatch
    // is silent: the mirror renders the parameter's default texture forever.
    const TCHAR* kMirrorMaterial   = TEXT("/Game/Materials/M_Mirror.M_Mirror");
    const FName  kMirrorTextureParam(TEXT("MirrorTexture"));
    const FName  kFlipUParam(TEXT("FlipU"));
    const FName  kFlipVParam(TEXT("FlipV"));
    const FName  kSwapUVParam(TEXT("SwapUV"));
    const FName  kUVDebugParam(TEXT("UVDebug"));

    // How far past the glass the capture's near plane sits, cm. THE NEAR PLANE
    // IS THE MIRROR'S CLIP PLANE: the capture looks along the glass normal, so
    // the glass is parallel to the image plane and one near plane just past it
    // removes the glass and everything between the reflected eye and it. It
    // must be past the glass -- the glass lies exactly on the aperture, and
    // left in, it fills its own capture (see bEnableClipPlane in BeginPlay).
    constexpr double kNearPastGlass = 0.5;

    // -ChronoMirrorProbe markers. S runs along the glass's +Y and T along its
    // +Z, both in [-1, 1] across the glass. ASYMMETRIC IN BOTH AXES on purpose:
    // a mirrored, flipped or transposed image moves every one of them, and a
    // symmetric layout could hide exactly the error being looked for. S is
    // multiplied by ProbeSide so door-mirror markers sit outboard, clear of
    // the car's own flank.
    struct FProbeSpec { double S, T; FLinearColor Colour; const TCHAR* Name; };
    const FProbeSpec kProbes[] = {
        { +0.55, +0.40, FLinearColor(1.0f, 0.0f, 1.0f), TEXT("magenta") },
        { +0.15, -0.45, FLinearColor(0.0f, 1.0f, 0.0f), TEXT("green")   },
    };
    // How far beyond the glass each marker floats, cm. Far enough to be
    // unambiguously "behind the car", near enough to stay above the road.
    constexpr double kProbeDistance = 400.0;
    // Marker radius as a fraction of the glass half-height, in angle as seen
    // from the reflected eye -- so every marker is the same size in its
    // target (about 15 px across in a 128 px tall one).
    constexpr double kProbeSize = 0.12;

    inline MmVec3 ToMm(const FVector& V) { MmVec3 R; R.x = V.X; R.y = V.Y; R.z = V.Z; return R; }
    inline FVector ToUE(const MmVec3& V) { return FVector(V.x, V.y, V.z); }
}

UChronoMirrorComponent::UChronoMirrorComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    // After the camera has moved: a mirror computed from last frame's eye
    // position lags the head by a frame, which is visible when looking around.
    PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
}

void UChronoMirrorComponent::BeginPlay()
{
    Super::BeginPlay();

    // -ChronoMirrorUV arms the UV read-off on every mirror at once, so the
    // mapping can be settled without editing a default and rebuilding.
    if (FParse::Param(FCommandLine::Get(), TEXT("ChronoMirrorUV")))
    {
        bDebugUV = true;
    }
    // -ChronoMirrorProbe arms the end-to-end measurement (see bProbe). Not
    // combinable with -ChronoMirrorUV: the markers are drawn with the mirror
    // material, which would then paint them as UV colours too.
    if (FParse::Param(FCommandLine::Get(), TEXT("ChronoMirrorProbe")))
    {
        bProbe = true;
    }

    Target = NewObject<UTextureRenderTarget2D>(this);
    // No HDR and no alpha. A mirror is displayed straight onto a surface in
    // the same scene at the same exposure; an HDR target would cost bandwidth
    // this GPU does not have for a range nothing downstream uses.
    Target->RenderTargetFormat = RTF_RGBA8;
    Target->InitAutoFormat(TargetWidth, TargetHeight);
    Target->UpdateResourceImmediate(true);

    Capture = NewObject<USceneCaptureComponent2D>(this);
    Capture->SetupAttachment(this);
    Capture->RegisterComponent();
    Capture->TextureTarget = Target;
    Capture->CaptureSource = SCS_FinalColorLDR;

    // Driven explicitly from TickComponent, so the divisor actually controls
    // the cost. Leaving bCaptureEveryFrame on would render every frame no
    // matter what UpdateDivisor says, which is a silent way to lose the whole
    // budget saving.
    Capture->bCaptureEveryFrame = false;
    Capture->bCaptureOnMovement = false;

    // NOT Unreal's capture clip plane, deliberately OFF. It was on from the
    // start and never did anything: bEnableClipPlane needs
    // r.AllowGlobalClipPlane, which defaults to 0, is read-only at runtime,
    // and costs a full shader recompile plus ~15% base-pass time to enable.
    // Nothing noticed while the frustum was wrong, because the glass covered
    // only part of it. Once the frustum became exactly the aperture, each
    // two-sided glass filled its OWN capture with its own black target -- a
    // feedback loop, measured as 100% black on all three mirrors.
    //
    // The clip it was meant to do -- everything between the reflected eye and
    // the glass, the car's own nose included -- is done by the NEAR PLANE,
    // which mm_ue_mirror_capture puts just past the glass (kNearPastGlass).
    // That is exact, and free, only because the capture looks along the glass
    // normal: every glass point is then at the same view depth.
    Capture->bEnableClipPlane = false;

    Capture->bUseCustomProjectionMatrix = true;

    // ---- the glass -----------------------------------------------------
    if (bBuildGlass)
    {
        UStaticMesh* Plane = LoadObject<UStaticMesh>(
            nullptr, TEXT("/Engine/BasicShapes/Plane.Plane"));
        UMaterialInterface* MirrorMat =
            LoadObject<UMaterialInterface>(nullptr, kMirrorMaterial);

        if (Plane)
        {
            Glass = NewObject<UStaticMeshComponent>(this);
            Glass->SetStaticMesh(Plane);
            Glass->SetupAttachment(this);
            Glass->RegisterComponent();
            // The engine Plane lies in XY with its normal along local +Z, and
            // this pitches it up to stand in the glass plane.
            //
            // NOTE THE DIRECTION, because the comment here used to claim the
            // opposite and it cost three render attempts. A +90 pitch in
            // Unreal maps +X -> +Z and +Z -> -X. So the plane's normal ends up
            // on the mirror's local -X -- pointing AWAY from the driver, who
            // is on +X. With the single-sided material this asset used to
            // have, the face was culled and the driver looked straight through
            // the glass. Nothing was wrong with the capture, the frustum or
            // the render target; there was simply no surface being drawn.
            //
            // The fix is a two-sided material (21-make-mirror-material.py)
            // rather than a corrected rotation, deliberately: two-sided is
            // right whichever way the plane ends up facing, whereas flipping
            // the pitch to -90 would also invert the plane's in-plane X axis
            // and silently flip the image vertically. bFlipMirrorV exists for
            // that question and is settled by looking, not by deriving.
            Glass->SetRelativeRotation(FRotator(90.0f, 0.0f, 0.0f));
            Glass->SetRelativeScale3D(
                FVector(MirrorHeight / 100.0f, MirrorWidth / 100.0f, 1.0f));
            Glass->SetCollisionEnabled(ECollisionEnabled::NoCollision);
            Glass->SetCastShadow(false);

            if (MirrorMat)
            {
                GlassMID = UMaterialInstanceDynamic::Create(MirrorMat, this);
                if (GlassMID)
                {
                    GlassMID->SetTextureParameterValue(kMirrorTextureParam, Target);
                    Glass->SetMaterial(0, GlassMID);

                    // PROVE THE BIND, do not assume it. SetTextureParameterValue
                    // on a name the material does not have is silent, and that
                    // is precisely the failure this mirror spent three render
                    // attempts inside. Read it back: if the material really
                    // carries the parameter, the value returned is our target.
                    UTexture* Bound = nullptr;
                    const bool bGot =
                        GlassMID->GetTextureParameterValue(kMirrorTextureParam, Bound);
                    if (!bGot || Bound != Target)
                    {
                        UE_LOG(LogChronoMirror, Error,
                               TEXT("Mirror '%s': '%s' did NOT take on %s. The "
                                    "glass will show the sampler default (the "
                                    "engine 'Bad' texture), not a reflection. "
                                    "Re-run scripts/21-make-mirror-material.py."),
                               *GetName(), *kMirrorTextureParam.ToString(),
                               kMirrorMaterial);
                    }
                    else
                    {
                        UE_LOG(LogChronoMirror, Log,
                               TEXT("Mirror '%s': '%s' bound to a %dx%d target."),
                               *GetName(), *kMirrorTextureParam.ToString(),
                               TargetWidth, TargetHeight);
                    }
                }
            }
            else
            {
                // Not fatal, and worth saying plainly: the capture still runs
                // and GetMirrorTarget() still works, there is just nothing
                // showing it.
                UE_LOG(LogChronoMirror, Warning,
                       TEXT("Mirror '%s': %s not found, so the glass shows the "
                            "default material. Run "
                            "scripts/21-make-mirror-material.py once."),
                       *GetName(), kMirrorMaterial);
            }
        }
    }

    UE_LOG(LogChronoMirror, Log,
           TEXT("Mirror '%s': %.1f x %.1f cm glass, %dx%d target, 1-in-%d frames."),
           *GetName(), MirrorWidth, MirrorHeight, TargetWidth, TargetHeight,
           UpdateDivisor);

    LogGlassFacing();

    if (bProbe) { BuildProbes(); }
}

/* Which way the drawn surface points, as a NUMBER rather than as an argument
 * about Unreal's rotation conventions.
 *
 * "No reflection has ever been seen" is a symptom three different faults
 * produce -- glass culled, parameter unbound, capture black -- and a black
 * rectangle looks the same in all three. This one is checkable at BeginPlay
 * without rendering anything, so it should never again cost a render attempt.
 *
 * A negative dot means the drawn face points away from the driver. That is
 * survivable now only because the material is two-sided; it is still worth
 * saying out loud, because it is the thing that was wrong. */
void UChronoMirrorComponent::LogGlassFacing() const
{
    if (!Glass) { return; }

    const FVector GlassNormal =
        Glass->GetComponentTransform().TransformVectorNoScale(FVector::UpVector);
    const FVector MirrorNormal =
        GetComponentTransform().TransformVectorNoScale(FVector::ForwardVector);
    const FVector ToEye = EyeWorld - GetComponentLocation();

    UE_LOG(LogChronoMirror, Log,
           TEXT("Mirror '%s': glass face . eye = %+.2f, mirror normal . eye = "
                "%+.2f, face . normal = %+.2f%s"),
           *GetName(),
           FVector::DotProduct(GlassNormal, ToEye),
           FVector::DotProduct(MirrorNormal, ToEye),
           FVector::DotProduct(GlassNormal, MirrorNormal),
           FVector::DotProduct(GlassNormal, ToEye) < 0.0
               ? TEXT("  [drawn face points AWAY from the driver -- visible "
                      "only because M_Mirror is two-sided]")
               : TEXT(""));
}

/* Projects the glass's corners and centre through the capture's REAL
 * view-projection and checks each lands on its own NDC corner.
 *
 * mm_ue_mirror_capture is proven on the host. What the host cannot see is the
 * hand-off: whether the rotation this component builds from the capture's
 * basis, and the FMatrix it copies the projection into, are what Unreal then
 * renders with. So the view here is rebuilt exactly the way
 * SceneCaptureRendering.cpp builds a capture's (UE 5.8.2: the component
 * transform's inverse, translation and scale removed, then the x=z, y=x, z=y
 * swap), from the capture's actual transform.
 *
 * The expected corners come from the MIRROR's own axes, independently of
 * mirror_math: +Y is the image's +x and +Z its +y. Checking each corner
 * against its own target -- not merely |ndc| = 1 -- is what catches a
 * mirrored or rotated frustum, which also puts four corners on the edges.
 *
 * This check once logged the glass corners 1.8 to 10.8 NDC units off; that is
 * how the look-at orientation was caught. */
void UChronoMirrorComponent::CheckCornersLandOnNdcEdges() const
{
    if (!Capture) { return; }

    FTransform View = Capture->GetComponentToWorld();
    const FVector ViewOrigin = View.GetTranslation();
    View.SetTranslation(FVector::ZeroVector);
    View.SetScale3D(FVector::OneVector);
    const FMatrix ViewProj =
        FTranslationMatrix(-ViewOrigin) *
        View.ToInverseMatrixWithScale() *
        FMatrix(FPlane(0, 0, 1, 0), FPlane(1, 0, 0, 0),
                FPlane(0, 1, 0, 0), FPlane(0, 0, 0, 1)) *
        Capture->CustomProjectionMatrix;

    const FTransform& X = GetComponentTransform();
    const FVector C0 = X.GetLocation();
    const FVector R  = X.TransformVectorNoScale(FVector::RightVector) * (MirrorWidth * 0.5);
    const FVector U  = X.TransformVectorNoScale(FVector::UpVector)    * (MirrorHeight * 0.5);

    struct FPt { FVector P; double Nx, Ny; const TCHAR* Name; };
    const FPt Pts[5] = {
        { C0 + R + U, +1.0, +1.0, TEXT("+Y+Z") },
        { C0 - R + U, -1.0, +1.0, TEXT("-Y+Z") },
        { C0 + R - U, +1.0, -1.0, TEXT("+Y-Z") },
        { C0 - R - U, -1.0, -1.0, TEXT("-Y-Z") },
        { C0,          0.0,  0.0, TEXT("centre") },
    };

    double WorstErr = 0.0;
    // Reversed-Z depth of the glass: it must come out > 1, i.e. nearer than
    // the near plane and CLIPPED. At <= 1 the glass fills its own capture --
    // the black-mirror feedback loop -- whatever the corners say.
    double GlassZ = 0.0;
    FString Line;
    for (const FPt& Pt : Pts)
    {
        const FVector4 Clip = ViewProj.TransformFVector4(FVector4(Pt.P, 1.0));
        if (FMath::Abs(Clip.W) < KINDA_SMALL_NUMBER)
        {
            Line += FString::Printf(TEXT("%s=(degenerate) "), Pt.Name);
            WorstErr = BIG_NUMBER;
            continue;
        }
        const double Nx = Clip.X / Clip.W;
        const double Ny = Clip.Y / Clip.W;
        WorstErr = FMath::Max(WorstErr, FMath::Max(FMath::Abs(Nx - Pt.Nx),
                                                   FMath::Abs(Ny - Pt.Ny)));
        Line += FString::Printf(TEXT("%s=(%+.3f,%+.3f) "), Pt.Name, Nx, Ny);
        if (Pt.Nx == 0.0 && Pt.Ny == 0.0) { GlassZ = Clip.Z / Clip.W; }
    }

    const TCHAR* Verdict =
        WorstErr >= 0.01 ? TEXT("FRUSTUM DOES NOT MATCH THE GLASS")
        : GlassZ <= 1.0  ? TEXT("GLASS NOT CLIPPED -- it will fill the mirror")
                         : TEXT("FRUSTUM IS THE APERTURE");
    UE_LOG(LogChronoMirror, Log,
           TEXT("Mirror '%s': corner NDC %s| worst error %.4f  glass ndc z %.5f  %s"),
           *GetName(), *Line, WorstErr, GlassZ, Verdict);
}

/* Two small unlit spheres, one per kProbes entry, coloured by the mirror
 * material itself sampling a 4x4 target cleared to the marker colour -- so
 * they need no content asset of their own, and are unlit: a lit sphere's
 * shadowed half drops out of a colour threshold and drags the measured
 * centroid toward the sun. */
void UChronoMirrorComponent::BuildProbes()
{
    UStaticMesh* Sphere = LoadObject<UStaticMesh>(
        nullptr, TEXT("/Engine/BasicShapes/Sphere.Sphere"));
    UMaterialInterface* Mat = LoadObject<UMaterialInterface>(nullptr, kMirrorMaterial);
    if (!Sphere || !Mat)
    {
        UE_LOG(LogChronoMirror, Error,
               TEXT("Mirror '%s': probe needs /Engine/BasicShapes/Sphere and %s."),
               *GetName(), kMirrorMaterial);
        return;
    }

    // Outboard, for door mirrors: the side of the glass away from the car's
    // centreline, expressed as a sign along the glass's +Y. The interior
    // mirror is on the centreline and either side will do.
    ProbeSide = 1.0;
    if (const AActor* Car = GetOwner())
    {
        const FVector Side = Car->GetActorRightVector();
        const double Lateral =
            FVector::DotProduct(GetComponentLocation() - Car->GetActorLocation(), Side);
        if (FMath::Abs(Lateral) > 20.0)
        {
            const FVector Outboard = Side * FMath::Sign(Lateral);
            const FVector GlassY =
                GetComponentTransform().TransformVectorNoScale(FVector::RightVector);
            ProbeSide = FVector::DotProduct(GlassY, Outboard) >= 0.0 ? 1.0 : -1.0;
        }
    }

    for (const FProbeSpec& Spec : kProbes)
    {
        UTextureRenderTarget2D* Solid = NewObject<UTextureRenderTarget2D>(this);
        Solid->RenderTargetFormat = RTF_RGBA8;
        Solid->ClearColor = Spec.Colour;
        Solid->InitAutoFormat(4, 4);
        Solid->UpdateResourceImmediate(true);

        UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(Mat, this);
        MID->SetTextureParameterValue(kMirrorTextureParam, Solid);

        UStaticMeshComponent* M = NewObject<UStaticMeshComponent>(this);
        M->SetStaticMesh(Sphere);
        M->SetupAttachment(this);
        M->SetMobility(EComponentMobility::Movable);
        M->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        M->SetCastShadow(false);
        M->SetMaterial(0, MID);
        M->RegisterComponent();
        Probes.Add(M);
    }

    UE_LOG(LogChronoMirror, Log,
           TEXT("Mirror '%s': %d probe markers, outboard sign %+.0f."),
           *GetName(), Probes.Num(), ProbeSide);
}

FVector UChronoMirrorComponent::ProbeGlassPoint(int32 Index) const
{
    const FTransform& X = GetComponentTransform();
    return X.GetLocation()
        + X.TransformVectorNoScale(FVector::RightVector)
              * (kProbes[Index].S * ProbeSide * MirrorWidth * 0.5)
        + X.TransformVectorNoScale(FVector::UpVector)
              * (kProbes[Index].T * MirrorHeight * 0.5);
}

void UChronoMirrorComponent::PlaceProbes(const FVector& EyeRefl)
{
    for (int32 i = 0; i < Probes.Num(); ++i)
    {
        if (!Probes[i]) { continue; }
        // On the ray from the reflected eye through the glass point, beyond
        // the glass: by the law of reflection, exactly what the driver sees
        // at that point of the glass.
        const FVector G = ProbeGlassPoint(i);
        const FVector Ray = G - EyeRefl;
        const FVector M = G + Ray.GetSafeNormal() * kProbeDistance;
        const double Radius =
            kProbeSize * MirrorHeight * 0.5 * (M - EyeRefl).Size() / FMath::Max(Ray.Size(), 1.0);
        Probes[i]->SetWorldLocation(M);
        // The engine Sphere is 100 uu across, i.e. radius 50.
        Probes[i]->SetWorldScale3D(FVector(Radius / 50.0));
    }
}

void UChronoMirrorComponent::HideProbesFrom(const UChronoMirrorComponent* Other) const
{
    if (!Other || Other == this || !Other->Capture) { return; }
    for (UStaticMeshComponent* M : Probes)
    {
        if (M) { Other->Capture->HideComponent(M); }
    }
}

void UChronoMirrorComponent::LogProbeExpectations(APlayerController* PC) const
{
    if (!bProbe || Probes.Num() == 0) { return; }

    int32 ViewX = 0, ViewY = 0;
    if (PC) { PC->GetViewportSize(ViewX, ViewY); }

    // REFRESH THE CAMERA FIRST. This runs from a timer, and in UWorld::Tick
    // (LevelTick.cpp, UE 5.8.2) timers fire AFTER TG_PostPhysics has moved the
    // car but BEFORE UpdateCameraManager refreshes the view. Projecting now
    // puts this frame's glass through LAST frame's camera, while the
    // screenshot renders this frame's glass through this frame's camera.
    // MEASURED: with the car creeping, every marker came out 11 to 21 px off
    // on screen -- the whole glass translated with it -- while the same
    // markers sat within 0.2 px in their targets. A zero-delta update just
    // recomputes the view from the camera component, which the car's own
    // tick has already placed this frame; our camera has no lag to disturb.
    if (PC && PC->PlayerCameraManager) { PC->PlayerCameraManager->UpdateCamera(0.0f); }

    // This glass's bounding box on screen. The marker must be found ON THIS
    // glass: other mirrors' glass can share the frame at its edge -- in the
    // interior-mirror view the left door mirror sits about 26.6 degrees off
    // axis against a 27.6 degree half-FOV -- and it carries its OWN markers,
    // in the same two colours.
    FVector2D Lo(TNumericLimits<double>::Max(), TNumericLimits<double>::Max());
    FVector2D Hi(-TNumericLimits<double>::Max(), -TNumericLimits<double>::Max());
    bool bGlassOnScreen = PC != nullptr;
    {
        const FTransform& X = GetComponentTransform();
        const FVector R = X.TransformVectorNoScale(FVector::RightVector) * (MirrorWidth * 0.5);
        const FVector U = X.TransformVectorNoScale(FVector::UpVector)    * (MirrorHeight * 0.5);
        const FVector C0 = X.GetLocation();
        const FVector Corners[4] = { C0 + R + U, C0 - R + U, C0 + R - U, C0 - R - U };
        for (const FVector& P : Corners)
        {
            FVector2D S2;
            if (!PC || !PC->ProjectWorldLocationToScreen(P, S2, false))
            {
                bGlassOnScreen = false;
                break;
            }
            Lo = FVector2D(FMath::Min(Lo.X, S2.X), FMath::Min(Lo.Y, S2.Y));
            Hi = FVector2D(FMath::Max(Hi.X, S2.X), FMath::Max(Hi.Y, S2.Y));
        }
    }
    if (!bGlassOnScreen) { Lo = Hi = FVector2D(-1.0, -1.0); }

    for (int32 i = 0; i < Probes.Num(); ++i)
    {
        // In the target: image +x is the glass's +Y and image +y its +Z
        // (mm_ue_mirror_capture), and row 0 is the TOP of a render target.
        const double S = kProbes[i].S * ProbeSide;
        const double T = kProbes[i].T;
        const double U = 0.5 * (S + 1.0);
        const double V = 0.5 * (1.0 - T);

        // On screen: the reflection must sit ON the glass at the glass point.
        // Only meaningful for the mirror the camera is looking at.
        FVector2D Screen(-1.0, -1.0);
        const bool bOnScreen =
            PC && PC->ProjectWorldLocationToScreen(ProbeGlassPoint(i), Screen, false);

        UE_LOG(LogChronoMirror, Log,
               TEXT("MirrorProbe mirror=%s marker=%s target_uv=%.5f,%.5f "
                    "target_size=%d,%d screen=%.2f,%.2f onscreen=%d viewport=%d,%d "
                    "glass_box=%.1f,%.1f,%.1f,%.1f"),
               *GetName(), kProbes[i].Name, U, V, TargetWidth, TargetHeight,
               Screen.X, Screen.Y, bOnScreen ? 1 : 0, ViewX, ViewY,
               Lo.X, Lo.Y, Hi.X, Hi.Y);
    }
}

void UChronoMirrorComponent::DumpTargetToDisk() const
{
    if (!Target) { return; }

    const FString Dir = FPaths::ProjectSavedDir() / TEXT("MirrorDumps");
    const FString File = GetName() + TEXT(".png");

    UKismetRenderingLibrary::ExportRenderTarget(
        const_cast<UChronoMirrorComponent*>(this), Target, Dir, File);

    UE_LOG(LogChronoMirror, Log, TEXT("Mirror '%s': target written to %s"),
           *GetName(), *(Dir / File));
}

void UChronoMirrorComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                           FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
    if (!Capture || !Target) { return; }

    if (++FrameCounter < FMath::Max(1, UpdateDivisor)) { return; }
    FrameCounter = 0;

    // Local +X is the glass normal (toward the driver), +Y its width axis,
    // +Z its height axis.
    const FTransform& X = GetComponentTransform();
    const FVector Origin = X.GetLocation();
    const FVector Normal = X.TransformVectorNoScale(FVector::ForwardVector);
    const FVector Up     = X.TransformVectorNoScale(FVector::UpVector);

    // THE WHOLE CAPTURE COMES FROM ONE TESTED FUNCTION: the reflected eye, a
    // view basis Unreal can hold, and the off-axis projection measured in
    // that basis. mirror/run.sh selftest asserts what it does.
    //
    // This used to be assembled here by hand: Kooima extents from
    // mm_offaxis_extents, a look-at rotation toward the glass centre, and a
    // skew whose sign was a documented guess. The extents and the rotation
    // were in different bases, and the glass corners landed 1.8 to 10.8 NDC
    // units off the edges whichever way the guess went.
    MmUeCapture C;
    const int Rc = mm_ue_mirror_capture(ToMm(EyeWorld), ToMm(Origin), ToMm(Normal),
                                        ToMm(Up), MirrorWidth * 0.5,
                                        MirrorHeight * 0.5, kNearPastGlass, &C);
    if (Rc != 0)
    {
        // -2 is the eye-behind-the-glass case: a placement error, not a
        // transient. Warn once: a mirror the driver has walked behind would
        // otherwise spam the log every frame.
        if (!bWarnedGeometry)
        {
            bWarnedGeometry = true;
            // UE_LOG validates its format string at compile time and needs a
            // literal, so the explanation is selected into a variable first
            // rather than chosen inside the call.
            const TCHAR* Why = (Rc == -2)
                ? TEXT("the driver's eye is behind the glass")
                : TEXT("degenerate glass size, bad near plane, or the eye in the glass plane");
            UE_LOG(LogChronoMirror, Warning,
                   TEXT("Mirror '%s': no valid frustum (rc=%d) -- %s."),
                   *GetName(), Rc, Why);
        }
        return;
    }
    bWarnedGeometry = false;

    // 1. THE VIRTUAL EYE, looking along the glass normal. Rendering from the
    //    reflected eye is not an approximation of the mirror image -- for flat
    //    glass it IS the mirror image. MakeFromXZ builds Y = Z x X, the same
    //    `right` mm_ue_mirror_capture measured the extents along; a quaternion
    //    rather than an FRotator keeps the Euler round trip out of it.
    Capture->SetWorldLocationAndRotation(
        ToUE(C.position),
        FRotationMatrix::MakeFromXZ(ToUE(C.forward), ToUE(C.up)).ToQuat());

    // 2. Clip at the glass: done by the projection's near plane, just past the
    //    glass (see kNearPastGlass). Nothing to set here.

    // 3. THE OFF-AXIS FRUSTUM, copied row by row: mm_ue_mirror_capture emits
    //    FMatrix's own layout (row-major, row vectors, reversed-Z, infinite
    //    far -- TReversedZPerspectiveMatrix's pattern plus the off-centre
    //    terms), so there is nothing to transpose or re-sign here.
    Capture->CustomProjectionMatrix = FMatrix(
        FPlane(C.proj[0],  C.proj[1],  C.proj[2],  C.proj[3]),
        FPlane(C.proj[4],  C.proj[5],  C.proj[6],  C.proj[7]),
        FPlane(C.proj[8],  C.proj[9],  C.proj[10], C.proj[11]),
        FPlane(C.proj[12], C.proj[13], C.proj[14], C.proj[15]));

    if (!bCheckedNdc)
    {
        bCheckedNdc = true;
        CheckCornersLandOnNdcEdges();
    }

    if (bProbe) { PlaceProbes(ToUE(C.position)); }

    if (GlassMID)
    {
        // Sampling is 1:1 with the glass because the frustum IS the mirror
        // aperture (see mirror_math.h); these only orient the glass mesh's UVs
        // to the target. All are real parameters in M_Mirror -- this call used
        // to set a "FlipU" the material did not have, which is silent -- and
        // `mirror/run.sh render` measures that their defaults are right.
        GlassMID->SetScalarParameterValue(kSwapUVParam, bSwapMirrorUV ? 1.0f : 0.0f);
        GlassMID->SetScalarParameterValue(kFlipUParam, bFlipMirrorU ? 1.0f : 0.0f);
        GlassMID->SetScalarParameterValue(kFlipVParam, bFlipMirrorV ? 1.0f : 0.0f);
        GlassMID->SetScalarParameterValue(kUVDebugParam, bDebugUV ? 1.0f : 0.0f);
    }

    Capture->CaptureScene();
}
