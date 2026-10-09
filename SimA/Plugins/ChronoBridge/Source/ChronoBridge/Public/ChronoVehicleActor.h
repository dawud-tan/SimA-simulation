// ============================================================================
//  ChronoVehicleActor.h — the car you see, driven by the car Chrono simulates.
//
//  This actor owns NO physics. It reads interpolated poses from
//  UChronoWorldSubsystem and writes them onto meshes. That separation is the
//  whole Milestone 3 argument arriving at the renderer: the simulation runs at
//  1 kHz on its own thread and the visuals are a 60 Hz sampling of it, two
//  clocks joined by interpolation and nothing else.
//
//  It is deliberately built from engine primitives. A real Innova mesh is
//  content this project does not have, and waiting for one would have blocked
//  the mirrors, the referee's visual confirmation, and any human driving the
//  course. Until task 9 it was a solid box on four cylinders; it is now a
//  shell a driver can sit in -- windows, a dashboard, a steering wheel that
//  turns with the input -- still built from cubes and cylinders.
//
//  WHAT IS READ, NOT COPIED (task 9):
//    - the footprint (length, width, overhangs, wheelbase) from the course's
//      CAR record, the outline the referee scores: a body drawn from other
//      numbers could visibly miss a cone the referee counts as struck;
//    - the driver's seat from innova_Chassis.json's "Driver Position";
//    - the steering wheel's travel from innova_Steering.json's pinion
//      "Maximum Angle (deg)".
//  All three are written by the generators; the shell's own stations
//  (pillars, beltline, dashboard) are [EST] and live in the .cpp.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "ChronoVehicleActor.generated.h"

class UStaticMeshComponent;
class UCameraComponent;
class APlayerController;
class UChronoMirrorComponent;

UCLASS()
class CHRONOBRIDGE_API AChronoVehicleActor : public AActor
{
    GENERATED_BODY()

public:
    AChronoVehicleActor();

    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;

    /** Driver eye position in the CHASSIS REFERENCE frame, metres, Chrono
     *  axes. The reference frame is at the FRONT AXLE, at wheel-centre height
     *  (gen_innova.py puts the front hardpoints at x=0, z=0), so a driver
     *  sitting behind it has a negative x. Getting this wrong moves the
     *  mirrors' virtual eye and silently changes what they show.
     *
     *  READ at BeginPlay from innova_Chassis.json's "Driver Position", which
     *  gen_innova.py writes (task 9): one seat, set in one place. This value
     *  is used only if that file cannot be read, and the log says so -- which
     *  is why it is visible here but not editable.
     *
     *  RIGHT-HAND DRIVE: Indonesia drives on the left, so the driver sits on
     *  the right -- NEGATIVE y, because Chrono's Y points LEFT. The mirrors
     *  are aimed by the bisector law from this point, so the side it is on
     *  decides every glass angle. */
    UPROPERTY(VisibleAnywhere, Category = "Chrono|Vehicle")
    FVector DriverEyeChrono = FVector(-1.35, -0.38, 1.0336);

    /** Chase camera instead of the driver's eye: the whole car against the
     *  course, for checking where it is rather than driving it. */
    UPROPERTY(EditAnywhere, Category = "Chrono|Vehicle")
    bool bChaseCamera = false;

    /** Logs where the steering wheel must show its stripe, for a
     *  -ChronoLookWheel screenshot taken this frame: the input, the rim angle
     *  it commands, and the rim's centre, 12 and 3 o'clock on screen.
     *  cockpit/probe.py finds the stripe and compares. */
    void LogWheelExpectation(APlayerController* PC) const;

    /** Logs the glance at the screenshot (task 17): its value, eased, and
     *  the mirror it turns to. cockpit/probe.py reads it with the HUD's
     *  "HUD over mirrors" line, which says where that glass landed. */
    void LogGlance() const;

private:
    void LoadCarData();
    void BuildShell();
    void BuildSteeringWheel();
    void BuildWheels();
    void BuildMirrors();
    void BuildCamera();
    /** The head's rotation for a glance, in this actor's frame: toward the
     *  door mirror's glass, by the eased glance (task 17). */
    FRotator GlanceRotation(double Glance, const FVector& EyeLocal) const;

    /** Parent of every body panel. */
    UPROPERTY(Transient) TObjectPtr<USceneComponent> Body;
    /** One per spindle, FL FR RL RR; the tyre, rim and spoke hang off it. */
    UPROPERTY(Transient) TArray<TObjectPtr<USceneComponent>> Wheels;
    /** The column's frame, fixed to the body; the wheel turns inside it. */
    UPROPERTY(Transient) TObjectPtr<USceneComponent> SteeringColumn;
    /** Rolls about the column with the steering input. */
    UPROPERTY(Transient) TObjectPtr<USceneComponent> SteeringWheel;
    /** The input the rim was last turned by, device convention. */
    double RimSteer = 0.0;
    /** The glance the view was last turned by, -1 left .. +1 right. */
    double ViewGlance = 0.0;
    UPROPERTY(Transient) TObjectPtr<UCameraComponent> Camera;
    UPROPERTY(Transient) TArray<TObjectPtr<UChronoMirrorComponent>> Mirrors;

    /** The footprint, metres: the course's CAR record once LoadCarData has
     *  run. These defaults are gen_innova.py's, for a missing course. */
    double FrontOverhang = 0.880;
    double RearOverhang  = 1.105;
    double Wheelbase     = 2.750;
    double Width         = 1.830;
    /** Steering wheel angle at full lock, degrees: the pinion's maximum. */
    double SteeringMaxDeg = 612.5;

    /** Widest edge of mirror `Index`'s glass in cm; frames the
     *  -ChronoLookMirror diagnostic view so the glass fills the frame. */
    float MirrorsHalfSpan(int32 Index) const;

    bool bWarnedNoState = false;
};
