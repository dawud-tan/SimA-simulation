/* course_terrain.cpp — see course_terrain.h. Written as C, compiled as C++ by
 * the UE module and as C by the host harness. */

#include "course_terrain.h"

#include <stdio.h>
#include <string.h>

int32_t course_load_terrain(const char* path, ChsTerrainPatch* out, int32_t max,
                            char* err, size_t errsz)
{
    if (err && errsz) err[0] = '\0';
    if (!path || !out || max <= 0) {
        if (err && errsz) snprintf(err, errsz, "bad arguments");
        return -1;
    }

    FILE* f = fopen(path, "r");
    if (!f) {
        if (err && errsz) snprintf(err, errsz, "cannot open '%s'", path);
        return -1;
    }

    int32_t n = 0;
    int lineno = 0;
    char line[512];

    while (fgets(line, sizeof line, f)) {
        ++lineno;
        char* hash = strchr(line, '#');
        if (hash) *hash = '\0';

        ChsTerrainPatch p;
        memset(&p, 0, sizeof p);   /* padding deterministic: this gets copied */

        const int got = sscanf(line, "%lf %lf %lf %lf %lf %lf %lf",
                               &p.center_x, &p.center_y, &p.center_z,
                               &p.length, &p.width, &p.pitch_rad, &p.friction);
        if (got <= 0) continue;                 /* blank or comment-only */
        if (got != 7) {
            if (err && errsz)
                snprintf(err, errsz,
                         "%s:%d: expected 7 fields (center_x center_y center_z "
                         "length width pitch_rad friction), got %d",
                         path, lineno, got);
            fclose(f);
            return -1;
        }
        /* A zero-area patch is a typo, not a valid degenerate case, and Chrono
         * would build a zero-extent box for it and report nothing. */
        if (p.length <= 0.0 || p.width <= 0.0) {
            if (err && errsz)
                snprintf(err, errsz, "%s:%d: length and width must be positive "
                                     "(got %.4f x %.4f)", path, lineno,
                         p.length, p.width);
            fclose(f);
            return -1;
        }
        if (n >= max) {
            if (err && errsz)
                snprintf(err, errsz, "%s:%d: more than %d patches", path, lineno, max);
            fclose(f);
            return -1;
        }
        out[n++] = p;
    }
    fclose(f);

    if (n == 0) {
        if (err && errsz) snprintf(err, errsz, "%s: no patches", path);
        return -1;
    }
    return n;
}
