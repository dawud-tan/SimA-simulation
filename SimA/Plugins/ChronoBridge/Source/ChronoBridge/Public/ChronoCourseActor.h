// ============================================================================
//  ChronoCourseActor.h — the SIM A course, built from the same files the
//  physics and the referee read.
//
//  WHY BUILD IT IN CODE INSTEAD OF PLACING IT IN A LEVEL:
//  The course is not scenery. Its cone positions ARE the referee's cone
//  positions and its ramp IS the terrain patch Chrono collides against. If the
//  visuals were authored separately, they would drift, and the failure mode is
//  the worst kind: a car that visibly misses a cone the referee scores as
//  struck, with nothing in either system that is wrong on its own.
//
//  So this actor reads simA_referee.txt and simA_terrain.txt -- the exact
//  files fed to ref_load_course and Chs_CreateVehicleEx -- and builds meshes
//  from them. One source, three consumers.
//
//  It uses ENGINE BASIC SHAPES (/Engine/BasicShapes/Cone, Cube), so it needs
//  no authored asset to exist. That matters today because SimA/Content is
//  empty; it stays reasonable afterwards because a driving-test cone really is
//  a cone and a painted boundary really is a thin box.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "ChronoCourseActor.generated.h"

class UStaticMeshComponent;

UCLASS()
class CHRONOBRIDGE_API AChronoCourseActor : public AActor
{
    GENERATED_BODY()

public:
    AChronoCourseActor();

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;

    /** Rebuilds from the data files. Safe to call repeatedly; clears first. */
    UFUNCTION(CallInEditor, Category = "Chrono|Course")
    void RebuildCourse();

    /** Draw the ground plane. Off if the level already has terrain art. */
    UPROPERTY(EditAnywhere, Category = "Chrono|Course")
    bool bBuildGround = true;

    /** Painted-line width in centimetres. */
    UPROPERTY(EditAnywhere, Category = "Chrono|Course")
    float LineWidth = 10.0f;

private:
    UStaticMeshComponent* AddShape(const TCHAR* MeshPath, const FVector& Location,
                                   const FRotator& Rotation, const FVector& Scale,
                                   const FLinearColor& Colour, const TCHAR* Tag);
    void ClearBuilt();

    UPROPERTY(Transient)
    TArray<TObjectPtr<UStaticMeshComponent>> Built;
};
