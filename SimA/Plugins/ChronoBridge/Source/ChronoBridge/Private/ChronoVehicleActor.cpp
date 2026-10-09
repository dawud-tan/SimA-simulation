#include "ChronoVehicleActor.h"

#include "ChronoMirrorComponent.h"
#include "ChronoWorldSubsystem.h"
#include "driver_source.h"
#include "referee.h"

#include "Camera/CameraComponent.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Dom/JsonObject.h"
#include "Engine/Scene.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/PlayerController.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"

DEFINE_LOG_CATEGORY_STATIC(LogChronoVehicle, Log, All);

namespace
{
    const TCHAR* const kCube = TEXT("/Engine/BasicShapes/Cube.Cube");
    const TCHAR* const kCyl  = TEXT("/Engine/BasicShapes/Cylinder.Cylinder");

    // Tyre 205/65 R16 and the car's height, from gen_innova.py. Drawn only:
    // the tyre Chrono simulates is sized by its own JSON, and nothing judged
    // depends on the roof.
    constexpr double kTyreRadius = 0.33645;
    constexpr double kTyreWidth  = 0.205;
    constexpr double kRimRadius  = 0.2032;    // 16 in
    constexpr double kHeight     = 1.795;     // [SPEC]

    // Height of the chassis frame's origin above the ground at design ride
    // height. The origin is at wheel-centre height (task 14), and the wheel
    // centre sits the loaded radius up: gen_innova.py's R_LOADED, 0.3264;
    // `attitude` measures 0.3259. The body below is laid out in heights
    // above the ground, which is how a car's dimensions are published.
    constexpr double kWheelCentreH = 0.326;

    // The steering wheel's rim: the radius of the tube's centreline, metres.
    constexpr double kSteeringRimR = 0.19;
    // -ChronoLookWheel: the camera's distance behind the rim, down the column.
    constexpr double kLookWheelDist = 0.60;

    // The mirrors, in BuildMirrors' order.
    constexpr int32 kMirrorInterior = 0, kMirrorLeft = 1, kMirrorRight = 2;

    /** Chrono chassis frame (x forward, y LEFT) and a height above the
     *  ground, to this actor's local frame: Unreal axes (y RIGHT), cm. */
    FVector Local(double X, double Y, double H)
    {
        return FVector(X * 100.0, -Y * 100.0, (H - kWheelCentreH) * 100.0);
    }

    UStaticMeshComponent* MakeMesh(AActor* Owner, USceneComponent* Parent,
                                   const TCHAR* Path, const FLinearColor& Colour,
                                   const TCHAR* Name = TEXT("Mesh"))
    {
        UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, Path);
        if (!Mesh) { return nullptr; }
        UStaticMeshComponent* C = NewObject<UStaticMeshComponent>(
            Owner, MakeUniqueObjectName(Owner, UStaticMeshComponent::StaticClass(),
                                        FName(Name)));
        C->SetStaticMesh(Mesh);
        C->SetupAttachment(Parent);
        C->SetMobility(EComponentMobility::Movable);
        C->RegisterComponent();
        // Chrono owns the physics; Unreal must not also simulate this body.
        C->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        UMaterialInterface* Base = LoadObject<UMaterialInterface>(
            nullptr, TEXT("/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial"));
        if (!Base) { Base = C->GetMaterial(0); }
        if (Base)
        {
            if (UMaterialInstanceDynamic* MID =
                    UMaterialInstanceDynamic::Create(Base, Owner))
            {
                MID->SetVectorParameterValue(FName("Color"), Colour);
                MID->SetVectorParameterValue(FName("BaseColor"), Colour);
                MID->SetVectorParameterValue(FName("Tint"), Colour);
                C->SetMaterial(0, MID);
            }
        }
        return C;
    }

    USceneComponent* MakePivot(AActor* Owner, USceneComponent* Parent, const TCHAR* Name)
    {
        USceneComponent* S = NewObject<USceneComponent>(Owner, FName(Name));
        S->SetupAttachment(Parent);
        S->SetMobility(EComponentMobility::Movable);
        S->RegisterComponent();
        return S;
    }

    TSharedPtr<FJsonObject> ReadJson(const FString& Path)
    {
        FString Text;
        TSharedPtr<FJsonObject> Obj;
        if (!FFileHelper::LoadFileToString(Text, *Path)) { return nullptr; }
        if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Obj))
        {
            return nullptr;
        }
        return Obj;
    }
}

AChronoVehicleActor::AChronoVehicleActor()
{
    PrimaryActorTick.bCanEverTick = true;
    // After the subsystem has ticked and produced this frame's interpolated
    // poses. Reading them in the default group would show last frame's car.
    PrimaryActorTick.TickGroup = TG_PostPhysics;
    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
}

void AChronoVehicleActor::BeginPlay()
{
    Super::BeginPlay();
    LoadCarData();
    BuildShell();
    BuildSteeringWheel();
    BuildWheels();
    BuildMirrors();
    BuildCamera();
}

void AChronoVehicleActor::LoadCarData()
{
    const FString Root = FPaths::ConvertRelativePathToFull(
        FPaths::Combine(FPaths::ProjectDir(), TEXT("..")));

    // The footprint the referee judges: the same file ref_load_course feeds
    // the subsystem, so the body drawn is the body scored.
    const FString RefFile = FPaths::Combine(Root, TEXT("course/simA_referee.txt"));
    RefCourse Course;
    char Err[256] = {};
    if (FPaths::FileExists(RefFile) &&
        ref_load_course(&Course, TCHAR_TO_UTF8(*RefFile), Err, sizeof Err) == 0)
    {
        FrontOverhang = Course.front_overhang;
        RearOverhang  = Course.rear_overhang;
        Wheelbase     = Course.wheelbase;
        Width         = Course.car_width;
    }
    else
    {
        UE_LOG(LogChronoVehicle, Warning,
               TEXT("No course CAR record (%s: %hs); drawing the built-in footprint."),
               *RefFile, Err);
    }

    const FString Dir = FPaths::Combine(Root, TEXT("vehicle/innova"));
    bool bEye = false, bSteer = false;
    if (const TSharedPtr<FJsonObject> Chassis =
            ReadJson(FPaths::Combine(Dir, TEXT("innova_Chassis.json"))))
    {
        const TSharedPtr<FJsonObject>* Pos = nullptr;
        const TArray<TSharedPtr<FJsonValue>>* Loc = nullptr;
        if (Chassis->TryGetObjectField(TEXT("Driver Position"), Pos) &&
            (*Pos)->TryGetArrayField(TEXT("Location"), Loc) && Loc->Num() == 3)
        {
            DriverEyeChrono = FVector((*Loc)[0]->AsNumber(), (*Loc)[1]->AsNumber(),
                                      (*Loc)[2]->AsNumber());
            bEye = true;
        }
    }
    if (const TSharedPtr<FJsonObject> Steering =
            ReadJson(FPaths::Combine(Dir, TEXT("innova_Steering.json"))))
    {
        const TSharedPtr<FJsonObject>* Pinion = nullptr;
        bSteer = Steering->TryGetObjectField(TEXT("Pinion"), Pinion) &&
                 (*Pinion)->TryGetNumberField(TEXT("Maximum Angle (deg)"), SteeringMaxDeg);
    }
    UE_LOG(LogChronoVehicle, Log,
           TEXT("Car: footprint %.3f + %.3f + %.3f x %.3f m; eye (%.3f, %.3f, %.3f) = %.2f m "
                "above the ground%s; steering wheel +-%.1f deg%s."),
           FrontOverhang, Wheelbase, RearOverhang, Width,
           DriverEyeChrono.X, DriverEyeChrono.Y, DriverEyeChrono.Z,
           DriverEyeChrono.Z + kWheelCentreH,
           bEye ? TEXT(" (innova_Chassis.json)") : TEXT(" (BUILT-IN: no innova_Chassis.json)"),
           SteeringMaxDeg,
           bSteer ? TEXT(" (innova_Steering.json)") : TEXT(" (BUILT-IN: no innova_Steering.json)"));
}

/* THE SHELL (task 9). The box it replaces had no windows and put the
 * driver's eye inside solid bodywork: the view out worked only because a
 * box's inside faces are culled, and a mirror aimed through the cabin saw a
 * closed box. The shell is panels with real openings -- windscreen, side
 * windows, the tailgate glass -- so the interior mirror sees out through the
 * back of the car and the door mirrors are seen through the side windows,
 * both of which `mirror/run.sh render` now measures with the body in place.
 *
 * Laid out in the chassis frame: x forward from the front axle, y LEFT, h
 * the height above the ground at design ride height. The footprint is the
 * referee's; every station below is [EST], read off side views of the AN140,
 * and chosen where it matters by what the driver sees from the seat: the
 * header ~28 deg above the eye, the road first visible ~7 m ahead of it
 * over the bonnet, the door mirrors clear of the A-pillars. */
void AChronoVehicleActor::BuildShell()
{
    Body = MakePivot(this, RootComponent, TEXT("Body"));

    const FLinearColor Paint(0.05f, 0.20f, 0.45f);
    const FLinearColor Trim (0.16f, 0.15f, 0.13f);   // door cards, headliner
    const FLinearColor Dark (0.05f, 0.05f, 0.05f);   // dashboard
    const FLinearColor Seal (0.02f, 0.02f, 0.02f);   // window seals

    auto Box = [this](const TCHAR* Name, double X0, double X1, double Y0, double Y1,
                      double H0, double H1, const FLinearColor& C)
    {
        UStaticMeshComponent* M = MakeMesh(this, Body, kCube, C, Name);
        if (!M) { return; }
        // The engine Cube is 100 uu on a side and centred on itself.
        M->SetRelativeLocation(Local(0.5 * (X0 + X1), 0.5 * (Y0 + Y1), 0.5 * (H0 + H1)));
        M->SetRelativeScale3D(FVector(FMath::Abs(X1 - X0), FMath::Abs(Y1 - Y0),
                                      FMath::Abs(H1 - H0)));
    };
    // Both sides of the car at once: y from YIn to YOut, and its mirror image.
    auto BoxLR = [&Box](const TCHAR* Name, double X0, double X1, double YIn, double YOut,
                        double H0, double H1, const FLinearColor& C)
    {
        Box(Name, X0, X1,  YIn,  YOut, H0, H1, C);
        Box(Name, X0, X1, -YOut, -YIn, H0, H1, C);
    };
    // A member from (X0, H0) to (X1, H1) in the side view, at lateral Y,
    // `Wide` across the car and `Thick` through itself.
    auto Strut = [this](const TCHAR* Name, double X0, double H0, double X1, double H1,
                        double Y, double Wide, double Thick, const FLinearColor& C)
    {
        UStaticMeshComponent* M = MakeMesh(this, Body, kCube, C, Name);
        if (!M) { return; }
        const FVector A = Local(X0, Y, H0), B = Local(X1, Y, H1);
        M->SetRelativeLocation(0.5 * (A + B));
        M->SetRelativeRotation(FRotationMatrix::MakeFromXZ(B - A, FVector::UpVector).Rotator());
        M->SetRelativeScale3D(FVector((B - A).Size() / 100.0, Wide, Thick));
    };

    // Footprint, from the referee.
    const double XF = FrontOverhang;
    const double XR = -(Wheelbase + RearOverhang);
    const double XA = -Wheelbase;                   // rear axle
    const double Half = 0.5 * Width;

    // Wheel arches: open over each axle so the wheels show. The tyre is
    // 0.336 in radius, its top 0.66 up, its inner face 0.67 from the
    // centreline; the arch clears it by ~6 cm, the floor passes inside it.
    const double ArchX = 0.42, ArchTop = 0.72, WellY = 0.60;

    // Stations, [EST].
    const double Sill = 0.20, Floor = 0.42, Belt = 1.10;
    const double Liner = kHeight - 0.06;             // underside of the roof
    const double CowlX = -0.10, CowlH = 1.06;        // foot of the windscreen
    const double HeadX = -0.72, HeadH = 1.72;        // top of the windscreen
    const double Glass = 0.80;                       // half-width of the glasshouse
    const double TailBelt = 1.08, TailGlass = 0.66;  // tailgate glass: bottom, half-width
    const double BPillarX = -1.70, CPillarX = -2.45;
    const double T = 0.04;                           // panel thickness

    // Front: the nose, the engine bay between the front wheels, the wings
    // over them, and the bonnet rising to the windscreen.
    Box  (TEXT("Nose"),       ArchX, XF, -Half, Half, 0.22, 0.93, Paint);
    Box  (TEXT("EngineBay"), -ArchX, ArchX, -WellY, WellY, 0.25, 0.93, Paint);
    BoxLR(TEXT("Wing"),      -ArchX, ArchX, WellY, Half, ArchTop, 0.93, Paint);
    Strut(TEXT("Bonnet"), XF, 0.93 - 0.04, CowlX, 1.05 - 0.04, 0.0, Width, 0.08, Paint);
    BoxLR(TEXT("Scuttle"),   -ArchX, CowlX, Glass, Half, 0.93, Belt, Paint);

    // The lower body: floor, sills, the rear wheel wells, doors and quarters.
    // Open above the beltline everywhere but the pillars.
    Box  (TEXT("Floor"),    XR, -ArchX, -WellY, WellY, Sill, Floor, Paint);
    BoxLR(TEXT("Sill"),     XA + ArchX, -ArchX, WellY, Half, Sill, Floor, Paint);
    BoxLR(TEXT("RearSill"), XR, XA - ArchX, WellY, Half, Sill, Floor, Paint);
    BoxLR(TEXT("RearArch"), XA - ArchX, XA + ArchX, WellY - 0.02, Half, ArchTop, ArchTop + 0.08, Paint);
    BoxLR(TEXT("RearWell"), XA - ArchX, XA + ArchX, WellY - 0.02, WellY, Floor, ArchTop, Trim);
    BoxLR(TEXT("Door"),     XA + ArchX, -ArchX, Half - T, Half, Floor, Belt, Paint);
    BoxLR(TEXT("Quarter"),  XA - ArchX, XA + ArchX, Half - T, Half, ArchTop + 0.08, Belt, Paint);
    BoxLR(TEXT("RearQuarter"), XR, XA - ArchX, Half - T, Half, Floor, Belt, Paint);
    BoxLR(TEXT("DoorCard"), BPillarX, -ArchX, Half - T - 0.012, Half - T, Floor + 0.10, Belt - 0.03, Trim);
    BoxLR(TEXT("Shoulder"), XR, -ArchX, Glass, Half, Belt - 0.03, Belt, Seal);
    Box  (TEXT("Tail"),     XR, XR + T, -(Half - T), Half - T, Sill, TailBelt, Paint);

    // Inside: the dashboard, and the hood over the driver's instruments --
    // on the RIGHT, in front of the seat.
    const double Ey = DriverEyeChrono.Y;
    Box(TEXT("Dashboard"), -0.72, CowlX, -(Half - T), Half - T, 0.55, 1.08, Dark);
    Box(TEXT("Binnacle"),  -0.82, -0.62, Ey - 0.20, Ey + 0.20, 1.02, 1.14, Dark);

    // The glasshouse: A-pillars along the windscreen's edges, the header over
    // it, the roof, B/C pillars, and D-pillars framing the tailgate glass.
    Strut(TEXT("APillar"), CowlX, CowlH, HeadX, HeadH + 0.04,  Glass, 0.08, 0.08, Paint);
    Strut(TEXT("APillar"), CowlX, CowlH, HeadX, HeadH + 0.04, -Glass, 0.08, 0.08, Paint);
    Box  (TEXT("Header"),    HeadX - 0.08, HeadX + 0.02, -Glass, Glass, HeadH, kHeight, Paint);
    Box  (TEXT("Roof"),      XR, HeadX, -(Glass + 0.02), Glass + 0.02, Liner, kHeight, Paint);
    Box  (TEXT("Headliner"), XR + T, HeadX, -(Glass - 0.02), Glass - 0.02, Liner - 0.01, Liner, Trim);
    BoxLR(TEXT("BPillar"), BPillarX - 0.06, BPillarX + 0.06, Glass - 0.04, Glass + 0.04, Belt, Liner, Paint);
    BoxLR(TEXT("CPillar"), CPillarX - 0.06, CPillarX + 0.06, Glass - 0.04, Glass + 0.04, Belt, Liner, Paint);
    BoxLR(TEXT("DPillar"), XR, XR + 0.16, TailGlass, Glass + 0.04, TailBelt, Liner, Paint);
}

/* A 380 mm wheel on a column raised 28 degrees, in front of the seat, its
 * rim ~0.55 m from the eye [EST]. It turns with the input at the car's own
 * ratio: the steering input is the pinion's travel as a fraction of its
 * maximum, and the pinion IS the column, so the rim turns
 * input x SteeringMaxDeg -- 612.5 deg at full lock, 1.7 turns each way.
 * The pale segment at the top is the stripe a rally wheel carries, and for
 * the same reason: it is what makes the rim's angle readable. */
void AChronoVehicleActor::BuildSteeringWheel()
{
    constexpr double ColumnDeg = 28.0, CentreX = -0.93, CentreH = 1.00;
    constexpr double RimR = kSteeringRimR, Tube = 0.017;
    constexpr int32  Segments = 24;
    const FLinearColor Grip  (0.03f, 0.03f, 0.03f);
    const FLinearColor Stripe(0.75f, 0.72f, 0.62f);

    // The column's frame: local +X runs from the driver INTO the dashboard,
    // down the column. Looking along +X is the driver's view of the wheel,
    // and a positive roll about it is clockwise in that view (Unreal's
    // FRotator: "Roll ... +Clockwise"), which is the device convention of
    // FChronoThreadStats::Steering, + = right. Local +Y is then the car's
    // right, +Z up the face of the wheel toward its top.
    const double C = FMath::Cos(FMath::DegreesToRadians(ColumnDeg));
    const double S = FMath::Sin(FMath::DegreesToRadians(ColumnDeg));
    USceneComponent* Column = MakePivot(this, Body, TEXT("SteeringColumn"));
    Column->SetRelativeLocation(Local(CentreX, DriverEyeChrono.Y, CentreH));
    Column->SetRelativeRotation(
        FRotationMatrix::MakeFromXZ(FVector(C, 0.0, -S), FVector::UpVector).Rotator());
    SteeringColumn = Column;

    // The shroud does not turn.
    const FRotator AlongX = FRotationMatrix::MakeFromZ(FVector::ForwardVector).Rotator();
    if (UStaticMeshComponent* Shroud = MakeMesh(this, Column, kCyl, Grip, TEXT("Shroud")))
    {
        Shroud->SetRelativeLocation(FVector(14.5, 0.0, 0.0));
        Shroud->SetRelativeRotation(AlongX);
        Shroud->SetRelativeScale3D(FVector(0.055, 0.055, 0.23));
    }

    SteeringWheel = MakePivot(this, Column, TEXT("SteeringWheel"));

    // The rim, a ring of short cylinders. Angle Phi runs clockwise from the
    // top as the driver sees it: (y, z) = (sin, cos).
    for (int32 i = 0; i < Segments; ++i)
    {
        const double Phi = 2.0 * PI * i / Segments;
        UStaticMeshComponent* Seg = MakeMesh(this, SteeringWheel, kCyl,
                                             i == 0 ? Stripe : Grip, TEXT("Rim"));
        if (!Seg) { continue; }
        Seg->SetRelativeLocation(FVector(0.0, FMath::Sin(Phi), FMath::Cos(Phi)) * RimR * 100.0);
        Seg->SetRelativeRotation(FRotationMatrix::MakeFromZ(
            FVector(0.0, FMath::Cos(Phi), -FMath::Sin(Phi))).Rotator());
        Seg->SetRelativeScale3D(FVector(2.0 * Tube, 2.0 * Tube,
                                        1.15 * 2.0 * RimR * FMath::Sin(PI / Segments)));
        Seg->SetCastShadow(false);
    }
    if (UStaticMeshComponent* Hub = MakeMesh(this, SteeringWheel, kCyl, Grip, TEXT("Hub")))
    {
        Hub->SetRelativeLocation(FVector(-1.0, 0.0, 0.0));
        Hub->SetRelativeRotation(AlongX);
        Hub->SetRelativeScale3D(FVector(0.14, 0.14, 0.06));
    }
    // Three spokes: right, bottom, left.
    for (const double Deg : {90.0, 180.0, 270.0})
    {
        const double Phi = FMath::DegreesToRadians(Deg);
        const FVector Radial(0.0, FMath::Sin(Phi), FMath::Cos(Phi));
        UStaticMeshComponent* Spoke = MakeMesh(this, SteeringWheel, kCube, Grip, TEXT("Spoke"));
        if (!Spoke) { continue; }
        Spoke->SetRelativeLocation(Radial * 11.75);
        Spoke->SetRelativeRotation(
            FRotationMatrix::MakeFromXZ(Radial, FVector::ForwardVector).Rotator());
        Spoke->SetRelativeScale3D(FVector(0.115, 0.045, 0.02));
    }
}

/* Each wheel is a pivot the spindle's pose is written onto, carrying a tyre,
 * a silver rim and one dark spoke. The spoke is what makes a wheel's spin
 * visible; the rim is what makes the wheel visible at all -- a near-black
 * cylinder in the car's shadow was not. */
void AChronoVehicleActor::BuildWheels()
{
    const FLinearColor Rubber(0.06f, 0.06f, 0.06f);
    const FLinearColor Silver(0.55f, 0.56f, 0.58f);
    const FLinearColor Spoke (0.12f, 0.12f, 0.12f);
    // The engine Cylinder's axis is Z. A wheel spins about the spindle's
    // lateral axis, so the cylinders are rolled 90 degrees onto Y.
    const FRotator OntoY(0.0f, 0.0f, 90.0f);

    Wheels.Reset();
    for (int32 i = 0; i < 4; ++i)
    {
        USceneComponent* W = MakePivot(
            this, RootComponent, *FString::Printf(TEXT("Wheel%d"), i));
        if (UStaticMeshComponent* Tyre = MakeMesh(this, W, kCyl, Rubber, TEXT("Tyre")))
        {
            Tyre->SetRelativeRotation(OntoY);
            Tyre->SetRelativeScale3D(FVector(2.0 * kTyreRadius, 2.0 * kTyreRadius, kTyreWidth));
        }
        if (UStaticMeshComponent* Rim = MakeMesh(this, W, kCyl, Silver, TEXT("Rim")))
        {
            Rim->SetRelativeRotation(OntoY);
            Rim->SetRelativeScale3D(FVector(2.0 * kRimRadius, 2.0 * kRimRadius, kTyreWidth + 0.01));
        }
        if (UStaticMeshComponent* Bar = MakeMesh(this, W, kCube, Spoke, TEXT("Spoke")))
        {
            Bar->SetRelativeScale3D(FVector(1.8 * kRimRadius, kTyreWidth + 0.02, 0.05));
        }
        Wheels.Add(W);
    }
}

void AChronoVehicleActor::BuildMirrors()
{
    // Positions are [EST] and in the chassis reference frame, now judged
    // against the cockpit (task 9): the interior mirror hangs from the
    // header behind the windscreen, ~0.8 m from the eye; the door mirrors
    // stand off the front corners of the front doors, seen through the side
    // windows behind the A-pillars. The geometry that USES them is exact
    // either way.
    struct FMirrorSpec { const TCHAR* Name; FVector PosChrono, AimChrono; float W, H; };
    // AIMED, NOT ANGLED. `AimChrono` is the point on the road behind the car
    // that this mirror should show, and the glass angle is DERIVED from it by
    // the law of reflection: a mirror shows `Aim` to `Eye` exactly when its
    // normal bisects them,
    //
    //     N = normalize( normalize(Eye - M) + normalize(Aim - M) )
    //
    // which is what a driver does by hand when adjusting a mirror.
    //
    // This replaces a hand-set yaw with no pitch at all, and the missing
    // pitch was not cosmetic. MEASURED on the previous constants: the
    // interior mirror's reflected-eye frustum spanned +14.7 to +21.8 degrees
    // of elevation -- ENTIRELY above the horizon. It was rendering nothing
    // but sky, and this scene's sky is black, so its render target came back
    // 100% black (mean luminance 0.1 over 512x128). That is not a mirror bug;
    // a flat mirror 15 cm ABOVE an eye 45 cm away, with zero pitch, looks up
    // by exactly the angle the eye looks up at it. Real rear-view mirrors are
    // tilted down, which is precisely what the bisector produces.
    //
    // It also makes the old yaw-sign note automatic rather than a rule to
    // remember: a normal that bisects eye and aim faces the eye by
    // construction, so the door mirrors can no longer end up pointing
    // outboard with the driver behind the glass (the rc = -2 case).
    //
    // Chrono frame: X forward, Y LEFT, Z up, metres, origin at the FRONT
    // AXLE at wheel-centre height. Aim points are still [EST] -- they are a
    // driver's preference, not a derived quantity -- but they are now a
    // preference about WHAT THE MIRROR SHOWS, which is the thing worth
    // expressing.
    const double Z = -kWheelCentreH;   // + a height above the ground
    const FMirrorSpec Specs[] = {
        // 12 m back and near the road, not 25 m: the aim point sets where the
        // mirror's cone LANDS, and at 25 m it grazed so shallowly that it
        // cleared the terrain entirely.
        // In kMirrorInterior, kMirrorLeft, kMirrorRight order.
        { TEXT("MirrorInterior"), FVector(-0.71,  0.00, 1.60 + Z),
                                  FVector(-12.0,  0.00, 0.40), 25.0f,  6.0f },
        { TEXT("MirrorLeft"),     FVector(-0.38,  1.00, 1.22 + Z),
                                  FVector(-12.0,  2.60, 0.50), 16.0f,  9.0f },
        { TEXT("MirrorRight"),    FVector(-0.38, -1.00, 1.22 + Z),
                                  FVector(-12.0, -2.60, 0.50), 16.0f,  9.0f },
    };
    const FLinearColor Housing[] = {
        FLinearColor(0.02f, 0.02f, 0.02f),
        FLinearColor(0.05f, 0.20f, 0.45f), FLinearColor(0.05f, 0.20f, 0.45f),
    };

    Mirrors.Reset();
    for (int32 si = 0; si < UE_ARRAY_COUNT(Specs); ++si)
    {
        const FMirrorSpec& S = Specs[si];
        UChronoMirrorComponent* M = NewObject<UChronoMirrorComponent>(this, S.Name);
        M->SetupAttachment(RootComponent);

        // EVERY PROPERTY MUST BE SET BEFORE RegisterComponent(). Registering a
        // component into a world that has already begun play runs its
        // BeginPlay immediately, and this component's BeginPlay is what sizes
        // the glass and allocates the render target. Setting the size after
        // registering therefore had no effect: the first run logged all three
        // mirrors at the 25.0 x 6.0 DEFAULT, including the two that were meant
        // to be 16 x 9 door mirrors.
        const FVector PosLocal(S.PosChrono.X * 100.0,
                               -S.PosChrono.Y * 100.0,
                                S.PosChrono.Z * 100.0);
        const FVector AimLocal(S.AimChrono.X * 100.0,
                               -S.AimChrono.Y * 100.0,
                                S.AimChrono.Z * 100.0);
        const FVector EyeLocalCm(DriverEyeChrono.X * 100.0,
                                 -DriverEyeChrono.Y * 100.0,
                                  DriverEyeChrono.Z * 100.0);
        M->SetRelativeLocation(PosLocal);

        // Local +X is the glass normal, pointing back at the driver. Derived
        // by the bisector law above rather than set by hand.
        const FVector ToEyeDir = (EyeLocalCm - PosLocal).GetSafeNormal();
        const FVector ToAimDir = (AimLocal - PosLocal).GetSafeNormal();
        FVector Normal = (ToEyeDir + ToAimDir).GetSafeNormal();
        if (Normal.IsNearlyZero())
        {
            // Eye and aim exactly opposite: no orientation reflects one into
            // the other. Fall back to facing the driver, which at least keeps
            // the frustum valid.
            Normal = ToEyeDir;
        }
        // MakeFromXZ, not MakeFromX: MakeFromX leaves the roll about the
        // normal arbitrary, and that roll IS the glass's width/height axes --
        // which the capture's image axes follow (mm_ue_mirror_capture). Pinning
        // Z to world up keeps the glass upright and its width horizontal.
        M->SetRelativeRotation(
            FRotationMatrix::MakeFromXZ(Normal, FVector::UpVector).Rotator());
        M->MirrorWidth  = S.W;
        M->MirrorHeight = S.H;
        // The interior mirror is the one used at speed, so it updates most
        // often; the door mirrors matter most while parking, which is slow.
        M->UpdateDivisor = (si == kMirrorInterior) ? 1 : 2;

        // Seed the eye BEFORE the first tick. Left at the origin, the very
        // first frame computes a frustum from an eye that is nowhere near the
        // driver's head, which is what produced the rc=-2 "wound away from the
        // eye" warning on the first run.
        M->SetEyeLocation(GetActorTransform().TransformPosition(EyeLocalCm));

        M->RegisterComponent();
        Mirrors.Add(M);

        // The housing: BEHIND the glass (local -X), so the driver never sees
        // it and the mirror's own capture clips it with everything else
        // between its reflected eye and the glass.
        if (UStaticMeshComponent* Case = MakeMesh(this, M, kCube, Housing[si], TEXT("Housing")))
        {
            Case->SetRelativeLocation(FVector(-3.25, 0.0, 0.0));
            Case->SetRelativeScale3D(FVector(0.055, (S.W + 3.0) / 100.0, (S.H + 3.0) / 100.0));
        }
    }
    // The interior mirror's stem, from its housing forward to where the
    // windscreen would be glued: behind the glass as the driver sees it.
    if (UStaticMeshComponent* Stem = MakeMesh(this, Body, kCube, Housing[0], TEXT("MirrorStem")))
    {
        Stem->SetRelativeLocation(Local(-0.675, 0.0, 1.6425));
        Stem->SetRelativeScale3D(FVector(0.03, 0.03, 0.045));
    }

    // -ChronoMirrorProbe: every capture sees only its OWN markers. The rear
    // views overlap -- the interior mirror sees what the door mirrors see --
    // and a marker found in the wrong target would be measured against the
    // wrong prediction. No-op when the probe is off.
    for (UChronoMirrorComponent* Mine : Mirrors)
    {
        for (UChronoMirrorComponent* Other : Mirrors)
        {
            if (Mine && Other && Mine != Other) { Mine->HideProbesFrom(Other); }
        }
    }

    // Name the mirror a -ChronoLookMirror view is aimed at, so the screenshot
    // can be matched to the right glass by mirror/probe.py.
    int32 LookAt = -1;
    if (FParse::Value(FCommandLine::Get(), TEXT("ChronoLookMirror="), LookAt) &&
        Mirrors.IsValidIndex(LookAt) && Mirrors[LookAt])
    {
        UE_LOG(LogChronoVehicle, Log, TEXT("MirrorLook index=%d mirror=%s"),
               LookAt, *Mirrors[LookAt]->GetName());
    }
}

void AChronoVehicleActor::BuildCamera()
{
    Camera = NewObject<UCameraComponent>(this, TEXT("Camera"));
    Camera->SetupAttachment(RootComponent);

    // FIXED EXPOSURE.
    //
    // Auto-exposure metered on the car -- the only brightly lit thing in an
    // otherwise dark scene -- and crushed everything else to black. The first
    // frames showed a blown-out white chassis against a course that was
    // present and correctly placed but invisible. A driving test does not want
    // adaptive exposure anyway: the scene brightness is fixed, and a view that
    // dims when you look at the sky is a distraction, not realism.
    Camera->PostProcessSettings.bOverride_AutoExposureMethod = true;
    Camera->PostProcessSettings.AutoExposureMethod = AEM_Manual;
    Camera->PostProcessSettings.bOverride_AutoExposureBias = true;
    Camera->PostProcessSettings.AutoExposureBias = 10.0f;
    Camera->PostProcessSettings.bOverride_BloomIntensity = true;
    Camera->PostProcessSettings.BloomIntensity = 0.0f;
    Camera->bUseAttachParentBound = true;

    Camera->RegisterComponent();

    UE_LOG(LogChronoVehicle, Log,
           TEXT("Vehicle built: shell of %d parts, steering wheel, %d wheels, %d mirrors."),
           Body ? Body->GetNumChildrenComponents() : 0, Wheels.Num(), Mirrors.Num());
}

/* Widest edge of a mirror's glass, in centimetres. Used only to frame the
 * -ChronoLookMirror diagnostic view. */
float AChronoVehicleActor::MirrorsHalfSpan(int32 Index) const
{
    if (!Mirrors.IsValidIndex(Index) || !Mirrors[Index]) { return 25.0f; }
    return FMath::Max(Mirrors[Index]->MirrorWidth, Mirrors[Index]->MirrorHeight);
}

/* The prediction a -ChronoLookWheel screenshot is measured against. The rim
 * must stand at input x SteeringMaxDeg, clockwise from the top as the driver
 * sees it: + is right in the device convention, and full lock is the
 * pinion's maximum. The three points are the COLUMN's, which does not turn,
 * so they give the probe the unturned top and the driver's right on screen
 * whatever the camera does. Called from the -ChronoShot timer, like the
 * mirrors' probe, and for the same reason it refreshes the camera first. */
void AChronoVehicleActor::LogWheelExpectation(APlayerController* PC) const
{
    if (!SteeringColumn || !PC) { return; }
    if (PC->PlayerCameraManager) { PC->PlayerCameraManager->UpdateCamera(0.0f); }
    int32 ViewX = 0, ViewY = 0;
    PC->GetViewportSize(ViewX, ViewY);

    const FTransform& X = SteeringColumn->GetComponentTransform();
    const FVector C0 = X.GetLocation();
    const double R = kSteeringRimR * 100.0;
    FVector2D Centre(-1.0, -1.0), Twelve(-1.0, -1.0), Three(-1.0, -1.0);
    const bool bOnScreen =
        PC->ProjectWorldLocationToScreen(C0, Centre, false) &&
        PC->ProjectWorldLocationToScreen(C0 + X.TransformVectorNoScale(FVector::UpVector) * R,
                                         Twelve, false) &&
        PC->ProjectWorldLocationToScreen(C0 + X.TransformVectorNoScale(FVector::RightVector) * R,
                                         Three, false);
    UE_LOG(LogChronoVehicle, Log,
           TEXT("WheelProbe steer=%.5f full_lock_deg=%.2f expect_deg=%.3f centre=%.2f,%.2f "
                "twelve=%.2f,%.2f three=%.2f,%.2f onscreen=%d viewport=%d,%d"),
           RimSteer, SteeringMaxDeg, RimSteer * SteeringMaxDeg, Centre.X, Centre.Y,
           Twelve.X, Twelve.Y, Three.X, Three.Y, bOnScreen ? 1 : 0, ViewX, ViewY);
}

/* THE GLANCE (task 17). From the right-hand seat the left door mirror is 55
 * degrees left of straight ahead, past the 45 degree half-FOV, and both
 * parking spaces are on the left: the mirror a parking driver needs most
 * was a mirror they could not see. A glance turns the head to the glass
 * and back. The eye stays put -- each mirror is exact for that one point,
 * so what the glass shows does not change, only where it is in the view.
 * The turn is a slerp from straight ahead to the glass, by the eased
 * glance, with the head's up kept on the car's (no roll). */
FRotator AChronoVehicleActor::GlanceRotation(double Glance, const FVector& EyeLocal) const
{
    const double E = drv_glance_ease(Glance);
    const int32 Index = E < 0.0 ? kMirrorLeft : kMirrorRight;
    if (E == 0.0 || !Mirrors.IsValidIndex(Index) || !Mirrors[Index])
    {
        return FRotator::ZeroRotator;
    }
    const FVector Dir = (Mirrors[Index]->GetRelativeLocation() - EyeLocal).GetSafeNormal();
    const FQuat Look = FRotationMatrix::MakeFromXZ(Dir, FVector::UpVector).ToQuat();
    return FQuat::Slerp(FQuat::Identity, Look, FMath::Abs(E)).Rotator();
}

void AChronoVehicleActor::LogGlance() const
{
    const double E = drv_glance_ease(ViewGlance);
    const int32 Index = E < 0.0 ? kMirrorLeft : kMirrorRight;
    UE_LOG(LogChronoVehicle, Log, TEXT("GlanceProbe glance=%.4f eased=%.4f toward=%s"),
           ViewGlance, E,
           E == 0.0 ? TEXT("ahead")
                    : (Mirrors.IsValidIndex(Index) && Mirrors[Index]
                           ? *Mirrors[Index]->GetName() : TEXT("none")));
}

void AChronoVehicleActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    UChronoWorldSubsystem* Sub = GetWorld()
        ? GetWorld()->GetSubsystem<UChronoWorldSubsystem>() : nullptr;
    if (!Sub) { return; }

    const FChronoInterpResult& F = Sub->GetFrameState();
    if (!F.bValid || F.Poses.Num() < 5)
    {
        if (!bWarnedNoState)
        {
            bWarnedNoState = true;
            UE_LOG(LogChronoVehicle, Warning,
                   TEXT("No interpolated state yet (%d poses). The physics thread "
                        "may have failed to start -- check the log above."),
                   F.Poses.Num());
        }
        return;
    }
    bWarnedNoState = false;

    // Slot 0 is the chassis reference frame; 1..4 are the spindles, FL FR RL
    // RR. Already converted to Unreal axes and centimetres by the
    // interpolator, so nothing is converted again here.
    SetActorLocationAndRotation(F.Poses[0].Location, F.Poses[0].Rotation);

    for (int32 i = 0; i < Wheels.Num() && i + 1 < F.Poses.Num(); ++i)
    {
        if (Wheels[i])
        {
            Wheels[i]->SetWorldLocationAndRotation(F.Poses[i + 1].Location,
                                                   F.Poses[i + 1].Rotation);
        }
    }

    // The rim turns with the input as commanded, in the device convention
    // (+ = clockwise = right): a positive roll about the column, clockwise as
    // the driver sees it (BuildSteeringWheel).
    if (SteeringWheel)
    {
        RimSteer = FMath::Clamp(Sub->GetPhysicsStats().Steering, -1.0, 1.0);
        SteeringWheel->SetRelativeRotation(FRotator(0.0, 0.0, RimSteer * SteeringMaxDeg));
    }

    // The mirrors need the driver's eye in WORLD space, and they need it every
    // frame because the car moves. A mirror is exact for exactly one eye
    // position; this is that position.
    const FVector EyeLocal(DriverEyeChrono.X * 100.0,
                           -DriverEyeChrono.Y * 100.0,
                           DriverEyeChrono.Z * 100.0);
    const FVector EyeWorld = GetActorTransform().TransformPosition(EyeLocal);
    for (UChronoMirrorComponent* M : Mirrors)
    {
        if (M) { M->SetEyeLocation(EyeWorld); }
    }

    if (Camera)
    {
        if (bChaseCamera)
        {
            // High and well back, angled down. This is a DIAGNOSTIC vantage,
            // not a driving view: at 9 m and 3.5 m up the course beyond the car
            // occupied about ten pixels, which is correct perspective and
            // useless for confirming the layout is right.
            Camera->SetRelativeLocation(FVector(-1600.0f, 0.0f, 1100.0f));
            Camera->SetRelativeRotation(FRotator(-22.0f, 0.0f, 0.0f));
        }
        else
        {
            Camera->SetRelativeLocation(EyeLocal);

            // -ChronoLookMirror=<0|1|2> aims the eye AT a mirror instead of
            // straight ahead. A mirror is deliberately outside the forward
            // field of view -- that is what makes it a mirror rather than a
            // window -- so confirming the reflection needs the head turned,
            // and an automated screenshot cannot turn its head by itself.
            //
            // The body stays VISIBLE (task 9). Until the cockpit existed this
            // view hid the chassis: the eye sat inside a solid box, and a
            // mirror reflecting a closed box looks exactly like a broken one.
            // Now the glass is seen from the seat through the car's own
            // windows, so a pass also shows the driver can see it.
            int32 Which = -1;
            if (FParse::Value(FCommandLine::Get(), TEXT("ChronoLookMirror="), Which) &&
                Mirrors.IsValidIndex(Which) && Mirrors[Which])
            {
                const FVector Target = Mirrors[Which]->GetComponentLocation();
                Camera->SetWorldRotation((Target - EyeWorld).Rotation());

                // FRAME THE GLASS, do not just point at it.
                //
                // A fixed 60 degree FOV put a 25 x 6 cm mirror 48 cm away in
                // the middle of a view that was otherwise entirely sky -- and
                // the sky renders black, so the screenshot was 99.99% black
                // whatever the mirror did. Measured on that frame: 921,599 of
                // 921,600 pixels were pure black. A diagnostic that cannot
                // show its subject is not a diagnostic.
                //
                // Widest half-extent, doubled for margin, against the true
                // eye-to-glass distance.
                const double Dist = FMath::Max(
                    1.0, (double)FVector::Dist(Target, EyeWorld));
                const double Half = 0.5 * FMath::Max(MirrorsHalfSpan(Which), 1.0f);
                const double Fov = 2.0 * FMath::RadiansToDegrees(
                    FMath::Atan(2.0 * Half / Dist));
                Camera->SetFieldOfView((float)FMath::Clamp(Fov, 10.0, 120.0));
            }
            else if (SteeringColumn && FParse::Param(FCommandLine::Get(), TEXT("ChronoLookWheel")))
            {
                // -ChronoLookWheel: down the column from the driver's side, in
                // the column's own frame -- the rim square to the view, its
                // unturned top at the top of the frame, the car's right on
                // the right. So the stripe's angle in the image is the rim's
                // angle as the driver sees it, and the forward view, which
                // shows only the rim's top, cannot measure it. The rim's
                // radius is a third of the half-width.
                const FVector Axis = SteeringColumn->GetForwardVector();
                Camera->SetWorldLocationAndRotation(
                    SteeringColumn->GetComponentLocation() - Axis * (kLookWheelDist * 100.0),
                    SteeringColumn->GetComponentRotation());
                Camera->SetFieldOfView((float)(2.0 * FMath::RadiansToDegrees(
                    FMath::Atan(3.0 * kSteeringRimR / kLookWheelDist))));
            }
            else
            {
                ViewGlance = Sub->GetPhysicsStats().Glance;
                Camera->SetRelativeRotation(GlanceRotation(ViewGlance, EyeLocal));
            }
        }
    }
}
