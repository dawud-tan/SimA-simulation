/* ===========================================================================
 *  course_terrain.h — loads simA_terrain.txt into ChsTerrainPatch records.
 *
 *  PURE C. Same arrangement as driver_source and wheel_device: the logic lives
 *  here, in the plugin, and a host-side harness compiles this same file.
 *
 *  WHY NOT JSON: gen_course.py already writes simA_course.json for the
 *  renderer and the referee, which want zones, gates and cones. This path
 *  wants six numbers a line, on the code path that builds the Chrono world,
 *  and a JSON parser there would be a dependency bought for nothing. The two
 *  files are written by the same generator run, so they cannot drift.
 *
 *  This header is PRIVATE: nothing outside the module needs it. The physics
 *  thread loads into a local array in Init() and hands the pointer straight to
 *  Chs_CreateVehicleEx, which copies. Nothing is retained.
 * ======================================================================== */
#ifndef COURSE_TERRAIN_H
#define COURSE_TERRAIN_H

#include <stddef.h>
#include <stdint.h>

#include "chrono_shim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Reads up to `max` patches from `path`.
 *
 *  Returns the number loaded, or -1 with a message in `err` naming the
 *  offending LINE. A malformed terrain file that loaded silently as "no
 *  patches" would give a car falling through the world, which is a long way
 *  from the actual mistake. */
int32_t course_load_terrain(const char* path, ChsTerrainPatch* out, int32_t max,
                            char* err, size_t errsz);

#ifdef __cplusplus
}
#endif

#endif /* COURSE_TERRAIN_H */
