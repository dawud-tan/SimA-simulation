#include "ChronoCourseActor.h"

#include "referee.h"
#include "course_terrain.h"
#include "chrono_shim.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "Misc/Paths.h"
#include "UObject/ConstructorHelpers.h"

DEFINE_LOG_CATEGORY_STATIC(LogChronoCourse, Log, All);

namespace
{
    // Chrono is metres, right-handed, +Y LEFT. Unreal is centimetres,
    // left-handed, +Y RIGHT. Same conversion as ChronoInterpolator.h, and it
    // lives in both places for the same reason the flip is written once there:
    // getting it wrong here would put the course on the wrong side of the car.
    inline FVector ChronoToUE(double x, double y, double z)
    {
        return FVector(x * 100.0, -y * 100.0, z * 100.0);
    }
}

AChronoCourseActor::AChronoCourseActor()
{
    PrimaryActorTick.bCanEverTick = false;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AChronoCourseActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    // Rebuild in the editor viewport as well as at runtime, so moving the
    // actor or editing LineWidth shows immediately.
    RebuildCourse();
}

void AChronoCourseActor::BeginPlay()
{
    Super::BeginPlay();
    if (Built.Num() == 0) { RebuildCourse(); }
}

void AChronoCourseActor::ClearBuilt()
{
    for (UStaticMeshComponent* C : Built)
    {
        if (C) { C->DestroyComponent(); }
    }
    Built.Reset();
}

UStaticMeshComponent* AChronoCourseActor::AddShape(
    const TCHAR* MeshPath, const FVector& Location, const FRotator& Rotation,
    const FVector& Scale, const FLinearColor& Colour, const TCHAR* Tag)
{
    UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, MeshPath);
    if (!Mesh) { return nullptr; }

    UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(this);
    C->SetStaticMesh(Mesh);
    C->SetupAttachment(RootComponent);
    // MOVABLE, and set BEFORE registering.
    //
    // Components created in OnConstruction default to STATIC mobility. With
    // r.AllowStaticLighting=False (this GPU has no ray tracing, so Lumen is
    // off and there are no lightmaps) a static primitive receives no lighting
    // at all and renders BLACK. That is precisely what the first two runs
    // showed: 23 course meshes built and logged, and a black screen with only
    // the runtime-created chassis visible.
    C->SetMobility(EComponentMobility::Movable);
    C->RegisterComponent();
    C->SetRelativeLocation(Location);
    C->SetRelativeRotation(Rotation);
    C->SetRelativeScale3D(Scale);
    C->ComponentTags.Add(FName(Tag));

    // COLLISION OFF, on everything. The course is drawn by Unreal and
    // simulated by Chrono, and those are separate worlds. Leaving Unreal
    // collision enabled would let Chaos push the car around behind Chrono's
    // back -- two physics engines disagreeing about the same scene, which is
    // exactly the failure the ABI wall exists to keep out of this project.
    C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    C->SetCastShadow(false);

    // The engine basic shapes ship with a WORLD GRID material -- a grey
    // checker -- so making the dynamic instance from the mesh's own material
    // inherits the checker and no colour parameter. Load the plain
    // BasicShapeMaterial explicitly instead.
    UMaterialInterface* Base = LoadObject<UMaterialInterface>(
        nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
    if (!Base) { Base = C->GetMaterial(0); }
    if (Base)
    {
        UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(Base, this);
        if (MID)
        {
            // BasicShapeMaterial exposes "Color". If a future base material
            // does not, this is a harmless no-op rather than an error.
            MID->SetVectorParameterValue(FName("Color"), Colour);
            MID->SetVectorParameterValue(FName("BaseColor"), Colour);
            MID->SetVectorParameterValue(FName("Tint"), Colour);
            C->SetMaterial(0, MID);
        }
    }

    Built.Add(C);
    return C;
}

void AChronoCourseActor::RebuildCourse()
{
    ClearBuilt();

    const FString Dir = FPaths::ConvertRelativePathToFull(
        FPaths::Combine(FPaths::ProjectDir(), TEXT("../course")));
    const FString RefFile = FPaths::Combine(Dir, TEXT("simA_referee.txt"));
    const FString TerFile = FPaths::Combine(Dir, TEXT("simA_terrain.txt"));

    static const TCHAR* kCone = TEXT("/Engine/BasicShapes/Cone.Cone");
    static const TCHAR* kCube = TEXT("/Engine/BasicShapes/Cube.Cube");

    // ---- terrain: the ground and the ramp ------------------------------
    // Built from the SAME patch records Chs_CreateVehicleEx collides against,
    // so a ramp the car climbs is a ramp the driver sees.
    ChsTerrainPatch Patches[32];
    char Err[256] = {};
    const int32 NumPatches = FPaths::FileExists(TerFile)
        ? course_load_terrain(TCHAR_TO_UTF8(*TerFile), Patches,
                              UE_ARRAY_COUNT(Patches), Err, sizeof Err)
        : -1;

    if (bBuildGround && NumPatches > 0)
    {
        for (int32 i = 0; i < NumPatches; ++i)
        {
            const ChsTerrainPatch& P = Patches[i];
            // The record's centre is the centre of the DRIVING SURFACE. A UE
            // Cube is centred on its own volume, so it is pushed down by half
            // its thickness to put its top face on that surface.
            const double Thick = 0.20;               // metres
            const FVector Loc = ChronoToUE(P.center_x, P.center_y,
                                           P.center_z - Thick * 0.5);
            // Chrono pitch is about +Y (left); Unreal pitch is about +Y
            // (right), so the handedness flip negates it.
            const FRotator Rot(-FMath::RadiansToDegrees(P.pitch_rad), 0.0f, 0.0f);
            const FVector Scale(P.length / 1.0, P.width / 1.0, Thick / 1.0);
            const bool bRamp = FMath::Abs(P.pitch_rad) > 1e-6;
            AddShape(kCube, Loc, Rot, Scale,
                     bRamp ? FLinearColor(0.45f, 0.42f, 0.40f)
                           : FLinearColor(0.28f, 0.29f, 0.30f),
                     bRamp ? TEXT("ramp") : TEXT("ground"));
        }

        // SCENERY past the course's edges (task 9). The patches are all the
        // ground there is, and past them a view fell into the void below the
        // horizon: a black band across the windscreen and every mirror. The
        // scenery is drawn, not simulated -- no Chrono patch is behind it, so
        // a car that left the course would still fall -- and olive, not
        // green: mirror/probe.py finds its green marker by hue.
        //
        // It is laid AROUND the patches, not under them. One plane 2 cm under
        // the driving surface won the depth test against it everywhere --
        // measured: a red plane covered the whole approach, and at 1 m down
        // it did not. Why was not isolated (a 4 km triangle's depth precision
        // is the suspect); it does not matter, because nothing here overlaps
        // anything: the plane is cut on the patches' own edges, and a cell no
        // patch covers is a box.
        // The patches carry no yaw, so each footprint is a rectangle.
        struct FFoot { double X0, X1, Y0, Y1; };
        TArray<FFoot> Foot;
        TArray<double> Xs = { -2000.0, 2000.0 }, Ys = { -2000.0, 2000.0 };
        for (int32 i = 0; i < NumPatches; ++i)
        {
            const ChsTerrainPatch& P = Patches[i];
            const double Hx = 0.5 * P.length * FMath::Cos(P.pitch_rad);
            const FFoot F{ P.center_x - Hx, P.center_x + Hx,
                           P.center_y - 0.5 * P.width, P.center_y + 0.5 * P.width };
            Foot.Add(F);
            Xs.Append({ F.X0, F.X1 });
            Ys.Append({ F.Y0, F.Y1 });
        }
        Xs.Sort();
        Ys.Sort();
        auto Covered = [&Foot](double X, double Y)
        {
            for (const FFoot& F : Foot)
            {
                if (X > F.X0 && X < F.X1 && Y > F.Y0 && Y < F.Y1) { return true; }
            }
            return false;
        };
        const double Thick = 0.20, Top = -0.02;   // metres
        int32 NumScenery = 0;
        for (int32 j = 0; j + 1 < Ys.Num(); ++j)
        {
            const double Y0 = Ys[j], Y1 = Ys[j + 1];
            if (Y1 - Y0 < 0.01) { continue; }
            // Consecutive uncovered cells along x are one box.
            for (int32 i = 0; i + 1 < Xs.Num();)
            {
                if (Xs[i + 1] - Xs[i] < 0.01 ||
                    Covered(0.5 * (Xs[i] + Xs[i + 1]), 0.5 * (Y0 + Y1)))
                {
                    ++i;
                    continue;
                }
                int32 k = i + 1;
                while (k + 1 < Xs.Num() &&
                       !Covered(0.5 * (Xs[k] + Xs[k + 1]), 0.5 * (Y0 + Y1)))
                {
                    ++k;
                }
                const double X0 = Xs[i], X1 = Xs[k];
                AddShape(kCube, ChronoToUE(0.5 * (X0 + X1), 0.5 * (Y0 + Y1), Top - 0.5 * Thick),
                         FRotator::ZeroRotator, FVector(X1 - X0, Y1 - Y0, Thick),
                         FLinearColor(0.18f, 0.21f, 0.10f), TEXT("scenery"));
                ++NumScenery;
                i = k;
            }
        }
        UE_LOG(LogChronoCourse, Log, TEXT("Scenery: %d boxes around %d patches."),
               NumScenery, NumPatches);
    }

    // ---- the course proper ---------------------------------------------
    RefCourse Course;
    if (!FPaths::FileExists(RefFile) ||
        ref_load_course(&Course, TCHAR_TO_UTF8(*RefFile), Err, sizeof Err) != 0)
    {
        UE_LOG(LogChronoCourse, Warning,
               TEXT("No course to build (%hs). Run 'bash course/run.sh gen'."), Err);
        return;
    }

    // Cones. The engine Cone mesh is 100 uu tall on a 100 uu base diameter, so
    // scale is (diameter, diameter, height) in metres-as-hundreds.
    for (int32 i = 0; i < Course.n_cones; ++i)
    {
        const RefCone& K = Course.cones[i];
        AddShape(kCone, ChronoToUE(K.x, K.y, 0.0), FRotator::ZeroRotator,
                 FVector(K.radius * 2.0, K.radius * 2.0, 0.70),
                 FLinearColor(1.0f, 0.35f, 0.02f), TEXT("cone"));
    }

    // Boundary lines, as thin painted boxes lying on the ground.
    for (int32 i = 0; i < Course.n_lines; ++i)
    {
        const RefLine& L = Course.lines[i];
        const FVector A = ChronoToUE(L.x0, L.y0, 0.0);
        const FVector B = ChronoToUE(L.x1, L.y1, 0.0);
        const FVector Mid = (A + B) * 0.5f + FVector(0, 0, 1.0f);
        const float Len = FVector::Dist(A, B);
        if (Len < 1.0f) { continue; }
        const FRotator Rot = (B - A).Rotation();
        AddShape(kCube, Mid, Rot,
                 FVector(Len / 100.0f, LineWidth / 100.0f, 0.02f),
                 FLinearColor::White, TEXT("line"));
    }

    // Gates, drawn low and dim: they are progress markers, not obstacles, and
    // a driver should not mistake one for something to avoid.
    for (int32 i = 0; i < Course.n_gates; ++i)
    {
        const RefGate& G = Course.gates[i];
        const FVector A = ChronoToUE(G.x0, G.y0, 0.0);
        const FVector B = ChronoToUE(G.x1, G.y1, 0.0);
        const float Len = FVector::Dist(A, B);
        if (Len < 1.0f) { continue; }
        AddShape(kCube, (A + B) * 0.5f + FVector(0, 0, 0.5f), (B - A).Rotation(),
                 FVector(Len / 100.0f, 0.04f, 0.005f),
                 FLinearColor(0.15f, 0.55f, 0.95f), TEXT("gate"));
    }

    UE_LOG(LogChronoCourse, Log,
           TEXT("Course built: %d cones, %d lines, %d gates, %d patches -> %d meshes."),
           Course.n_cones, Course.n_lines, Course.n_gates, NumPatches, Built.Num());
}
