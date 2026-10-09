# ============================================================================
#  20-make-level.py — creates /Game/Maps/SimACourse as a real, editable level.
#
#  Run it either way:
#
#    headless (one-off, slow first time -- the initial asset scan dominates):
#      /WIN_D/protek/UnrealEngine/Engine/Binaries/Linux/UnrealEditor \
#          /WIN_D/protek/SimA-simulation/SimA/SimA.uproject \
#          -run=pythonscript -script="/WIN_D/protek/SimA-simulation/scripts/20-make-level.py"
#
#    or from the editor's Output Log > Cmd > Python:
#      exec(open('/WIN_D/protek/SimA-simulation/scripts/20-make-level.py').read())
#
#  WHY THIS EXISTS WHEN AChronoGameMode ALREADY SPAWNS EVERYTHING:
#  It does not add capability -- the sim runs without it. What it adds is a
#  .umap you can OPEN, so mirror placement, the driver's eye position and
#  lighting become things you drag rather than constants you edit and rebuild.
#  Those are exactly the values that are [EST] and want a human eye.
#
#  What it deliberately does NOT bake is cone and ramp placement. Those come
#  from course/gen_course.py and are read at runtime by the physics, the
#  referee AND the course actor. Freezing them into a level would create a
#  second source of truth, and the failure mode is a car that visibly misses a
#  cone the referee scores as struck.
# ============================================================================

import unreal

MAP_PATH = "/Game/Maps/SimACourse"


def log(msg):
    print("[make-level] %s" % msg)
    unreal.log("[make-level] %s" % msg)


def get_level_subsystem():
    """LevelEditorSubsystem is the UE5 path; EditorLevelLibrary is the older
    one and is deprecated but still present. Try the modern one first rather
    than assuming either."""
    try:
        return unreal.get_editor_subsystem(unreal.LevelEditorSubsystem)
    except Exception as e:
        log("LevelEditorSubsystem unavailable (%s); falling back." % e)
        return None


def main():
    les = get_level_subsystem()

    # --- create the level ---------------------------------------------------
    created = False
    if les:
        try:
            created = les.new_level(MAP_PATH)
        except Exception as e:
            log("new_level failed: %s" % e)
    if not created:
        try:
            created = unreal.EditorLevelLibrary.new_level(MAP_PATH)
        except Exception as e:
            log("EditorLevelLibrary.new_level failed: %s" % e)
    if not created:
        log("could not create %s -- nothing was written." % MAP_PATH)
        return
    log("created %s" % MAP_PATH)

    # --- place the actors ---------------------------------------------------
    try:
        eas = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
        spawn = eas.spawn_actor_from_class
    except Exception:
        spawn = unreal.EditorLevelLibrary.spawn_actor_from_class

    origin = unreal.Vector(0.0, 0.0, 0.0)

    # The course actor rebuilds itself in OnConstruction from course/*.txt, so
    # what is saved here is the ACTOR, not the geometry. Regenerating the
    # course and reopening the level picks up the new numbers automatically.
    course = spawn(unreal.ChronoCourseActor, origin, unreal.Rotator(0, 0, 0))
    if course:
        course.set_actor_label("SimA_Course")
        log("placed ChronoCourseActor")

    car = spawn(unreal.ChronoVehicleActor, origin, unreal.Rotator(0, 0, 0))
    if car:
        car.set_actor_label("SimA_Vehicle")
        log("placed ChronoVehicleActor -- mirror placement and DriverEyeChrono "
            "are on this actor and are the [EST] values worth refining")

    # Lighting, placed rather than spawned by the GameMode so it can be tuned.
    sun = spawn(unreal.DirectionalLight, unreal.Vector(0, 0, 1000),
                unreal.Rotator(-55.0, 35.0, 0.0))
    if sun:
        sun.set_actor_label("SimA_Sun")
    sky = spawn(unreal.SkyLight, unreal.Vector(0, 0, 1200),
                unreal.Rotator(0, 0, 0))
    if sky:
        sky.set_actor_label("SimA_SkyLight")

    # A player start at the course origin. The car is positioned every frame
    # from the physics stream, so this only decides where the camera begins.
    ps = spawn(unreal.PlayerStart, unreal.Vector(-800.0, 0.0, 300.0),
               unreal.Rotator(0, 0, 0))
    if ps:
        ps.set_actor_label("SimA_PlayerStart")

    # --- save ---------------------------------------------------------------
    saved = False
    if les:
        try:
            saved = les.save_current_level()
        except Exception as e:
            log("save_current_level failed: %s" % e)
    if not saved:
        try:
            saved = unreal.EditorLevelLibrary.save_current_level()
        except Exception as e:
            log("EditorLevelLibrary.save_current_level failed: %s" % e)

    log("saved: %s" % saved)
    if saved:
        log("Now point EditorStartupMap and GameDefaultMap in "
            "SimA/Config/DefaultEngine.ini at %s" % MAP_PATH)
        log("and set bSpawnCourse/bSpawnVehicle/bSpawnLighting = false on "
            "AChronoGameMode so the level's copies are used instead.")


main()
