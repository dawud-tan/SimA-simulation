// ============================================================================
//  ChronoGameMode.h — assembles the scene in code.
//
//  WHY A GAME MODE AND NOT A PLACED LEVEL:
//  SimA/Content is empty, and the parts that would normally live in a level --
//  where the cones are, how long the ramp is, where the car starts -- are
//  already defined by course/gen_course.py and consumed by the physics and the
//  referee. Authoring them a second time as placed actors would create a
//  second source of truth for the same numbers.
//
//  So the level contributes nothing but a place to stand, and this spawns the
//  course and the car into it. scripts/20-make-level.py can bake the result
//  into a real .umap afterwards for editor refinement; nothing here depends on
//  that having been done.
// ============================================================================
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"

#include "ChronoGameMode.generated.h"

UCLASS()
class CHRONOBRIDGE_API AChronoGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AChronoGameMode();
    virtual void StartPlay() override;

    /** Spawn the procedural course. Turn off once a real level supplies it. */
    UPROPERTY(EditAnywhere, Category = "Chrono")
    bool bSpawnCourse = true;

    UPROPERTY(EditAnywhere, Category = "Chrono")
    bool bSpawnVehicle = true;

    /** Spawn a sun and sky. Off if the level already has lighting. */
    UPROPERTY(EditAnywhere, Category = "Chrono")
    bool bSpawnLighting = true;
};
