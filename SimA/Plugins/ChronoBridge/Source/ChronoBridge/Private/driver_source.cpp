/* ===========================================================================
 *  driver_source.cpp — keyboard integrator and script evaluator.
 *
 *  Written as C, compiled as C++ because that is how a UE module compiles a
 *  .cpp in Private/. The header's extern "C" is what lets the host test
 *  harness (a C file) link against this same object -- one copy of the logic,
 *  tested and shipped, exactly as wheel_device.cpp is.
 * ======================================================================== */

#include "driver_source.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- small helpers ----------------------------------------------------- */

static double drv_clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Move `v` toward `target` by at most `rate * dt`. Returning the target
 * exactly when it is within reach is what stops the axis dithering around
 * zero forever at 1 kHz -- an overshoot of one step's worth, re-corrected the
 * next step, would show up in the FFB damping term as a 1 kHz buzz. */
static double drv_approach(double v, double target, double rate, double dt)
{
    const double step = rate * dt;
    if (v < target) { v += step; if (v > target) v = target; }
    else            { v -= step; if (v < target) v = target; }
    return v;
}

/* =========================================================================
 *  KEYBOARD
 * ====================================================================== */

void drv_keyboard_init(DrvKeyboard* kb)
{
    if (!kb) return;
    memset(kb, 0, sizeof(*kb));
    kb->out.gear = 0;      /* neutral: the shim expresses this as clutch-out */
}

double drv_glance_step(double glance, int32_t dir, double dt)
{
    const double target = dir < 0 ? -1.0 : (dir > 0 ? 1.0 : 0.0);
    return drv_clamp(drv_approach(glance, target, DRV_RATE_GLANCE, dt), -1.0, 1.0);
}

double drv_glance_ease(double glance)
{
    const double u = drv_clamp(fabs(glance), 0.0, 1.0);
    const double e = u * u * (3.0 - 2.0 * u);
    return glance < 0.0 ? -e : e;
}

void drv_keyboard_step(DrvKeyboard* kb, uint32_t keys, double dt)
{
    if (!kb || dt <= 0.0) return;

    /* The fine modifier scales every rate, not just steering. Feathering the
     * throttle against the clutch is the whole hill-start exercise, and a
     * throttle that can only be moved in quarter-second sweeps cannot be
     * feathered at all. */
    const double k = (keys & DRV_KEY_FINE) ? DRV_FINE_SCALE : 1.0;

    /* ---- steering -----------------------------------------------------
     * NO AUTOMATIC RETURN TO CENTRE, and that is deliberate.
     *
     * Arcade driving games self-centre because their exercises are all
     * high-speed cornering, where letting go and having the rack unwind is
     * what a real car does under its own aligning torque. Every SIM A exercise
     * is the opposite: parallel parking, bay parking and the slalom are slow
     * manoeuvres in which a fixed lock is HELD for several seconds while the
     * car creeps. A rack that unwinds the moment the key is released makes
     * those literally impossible to perform, because the input is a key and a
     * key cannot be held at 40% of full lock.
     *
     * So the rack holds position, like a real one being held by real hands,
     * and DRV_KEY_CENTER is the explicit "straighten up" the driver asks for.
     */
    const int want_left  = (keys & DRV_KEY_LEFT)  != 0;
    const int want_right = (keys & DRV_KEY_RIGHT) != 0;

    if (keys & DRV_KEY_CENTER)
    {
        kb->out.steer = drv_approach(kb->out.steer, 0.0, DRV_RATE_CENTER * k, dt);
    }
    else if (want_left != want_right)
    {
        /* Device convention: +1 is clockwise, i.e. the driver's right. */
        const double target = want_right ? 1.0 : -1.0;
        kb->out.steer = drv_approach(kb->out.steer, target, DRV_RATE_STEER * k, dt);
    }
    /* both or neither held: hold the current lock. */

    kb->out.steer = drv_clamp(kb->out.steer, -1.0, 1.0);

    /* ---- pedals -------------------------------------------------------
     * Each is a first-order approach to 1 or 0 with an asymmetric rate. The
     * asymmetry is not cosmetic: a return spring lifts a pedal faster than a
     * leg pushes it, and on the throttle that difference is what makes
     * lift-off behave the way a driver expects. */
    kb->out.throttle = drv_approach(
        kb->out.throttle, (keys & DRV_KEY_THROTTLE) ? 1.0 : 0.0,
        ((keys & DRV_KEY_THROTTLE) ? DRV_RATE_THROTTLE_UP : DRV_RATE_THROTTLE_DN) * k, dt);

    kb->out.brake = drv_approach(
        kb->out.brake, (keys & DRV_KEY_BRAKE) ? 1.0 : 0.0,
        ((keys & DRV_KEY_BRAKE) ? DRV_RATE_BRAKE_UP : DRV_RATE_BRAKE_DN) * k, dt);

    /* Clutch: 1 = pedal down = disengaged, matching both the pedal axis and
     * ChManualTransmissionShafts (verified in M5, see chrono_shim.h). Pressing
     * is fast, releasing is the slow direction, and the slow direction is the
     * one that decides whether a hill start succeeds. */
    kb->out.clutch = drv_approach(
        kb->out.clutch, (keys & DRV_KEY_CLUTCH) ? 1.0 : 0.0,
        ((keys & DRV_KEY_CLUTCH) ? DRV_RATE_CLUTCH_UP : DRV_RATE_CLUTCH_DN) * k, dt);

    kb->out.throttle = drv_clamp(kb->out.throttle, 0.0, 1.0);
    kb->out.brake    = drv_clamp(kb->out.brake,    0.0, 1.0);
    kb->out.clutch   = drv_clamp(kb->out.clutch,   0.0, 1.0);

    /* ---- glance (task 17): held to look, released to look back --------
     * Unlike the rack, the head DOES return by itself: a mirror check is a
     * glance, and a driver who has to press a key to look ahead again has
     * been made to stare at the mirror. Both keys, or neither: ahead. */
    const int glance_l = (keys & DRV_KEY_GLANCE_L) != 0;
    const int glance_r = (keys & DRV_KEY_GLANCE_R) != 0;
    kb->out.glance = drv_glance_step(kb->out.glance, glance_r - glance_l, dt);

    /* ---- gear, on the RISING EDGE only --------------------------------
     * Level-triggered gear selection would re-issue SetGear every step for as
     * long as the key is held. The shim already guards against that (it caches
     * box_gear and only calls SetGear on a change), but relying on a
     * downstream guard for an upstream mistake is how the M5 "gearbox
     * initialises in 1st, not neutral" bug survived as long as it did. Edge
     * detection belongs where the intent is interpreted. */
    if (!kb->primed)
    {
        /* First step: adopt whatever is held without firing an edge, so a key
         * already down when the world spawns does not silently select a gear
         * the driver never pressed during this session. */
        kb->primed = 1;
    }
    else
    {
        const uint32_t rising = keys & ~kb->prev_keys;
        if      (rising & DRV_KEY_GEAR_N) kb->out.gear =  0;
        else if (rising & DRV_KEY_GEAR_R) kb->out.gear = -1;
        else if (rising & DRV_KEY_GEAR_1) kb->out.gear =  1;
        else if (rising & DRV_KEY_GEAR_2) kb->out.gear =  2;
        else if (rising & DRV_KEY_GEAR_3) kb->out.gear =  3;
        else if (rising & DRV_KEY_GEAR_4) kb->out.gear =  4;
        else if (rising & DRV_KEY_GEAR_5) kb->out.gear =  5;
    }
    kb->prev_keys = keys;
}

/* =========================================================================
 *  LANE KEEPING
 * ====================================================================== */

double drv_lane_keep(const DrvLaneKeep* lk, double lateral_err, double heading_err)
{
    if (!lk) return 0.0;
    /* Both terms push the same way: a car to the LEFT of the line, or pointing
     * left of it, needs right (positive, clockwise) steering. The heading term
     * is what stops the lateral term from oscillating -- position-only
     * feedback on a system with this much lag hunts. */
    double cmd = lk->k_lateral * lateral_err + lk->k_heading * heading_err;
    const double lim = lk->limit > 0.0 ? lk->limit : 1.0;
    return drv_clamp(cmd, -lim, lim);
}

/* =========================================================================
 *  SCRIPT
 * ====================================================================== */

int32_t drv_script_eval(const DrvScript* s, double t, DrvInput* out)
{
    if (!s || !out || !s->kf || s->n <= 0)
    {
        if (out) memset(out, 0, sizeof(*out));
        return 1;
    }

    /* Before the first keyframe: hold it. A script whose first keyframe is at
     * t = 0.5 is describing half a second of "whatever the first line says",
     * not half a second of undefined state. */
    if (t <= s->kf[0].t)
    {
        out->steer    = s->kf[0].steer;
        out->throttle = s->kf[0].throttle;
        out->brake    = s->kf[0].brake;
        out->clutch   = s->kf[0].clutch;
        out->gear     = s->kf[0].gear;
        out->glance   = s->kf[0].glance;
        return 0;
    }

    /* Past the last: hold the final values and report ended. See the header --
     * zeroing here would change the trajectory the referee is measuring. */
    const DrvKeyframe* last = &s->kf[s->n - 1];
    if (t >= last->t)
    {
        out->steer    = last->steer;
        out->throttle = last->throttle;
        out->brake    = last->brake;
        out->clutch   = last->clutch;
        out->gear     = last->gear;
        out->glance   = last->glance;
        return 1;
    }

    /* Linear scan. Scripts are tens of lines and this runs inside a 1 kHz loop
     * whose budget is 1000 us against a 180 us vehicle step -- a hundred
     * double comparisons is around 100 ns and does not merit an index cache
     * that would have to be mutable and would then need its own thread-safety
     * argument. */
    int32_t i = 0;
    while (i + 1 < s->n && s->kf[i + 1].t <= t) { ++i; }

    const DrvKeyframe* a = &s->kf[i];
    const DrvKeyframe* b = &s->kf[i + 1];
    const double span = b->t - a->t;
    const double u = (span > 1e-12) ? (t - a->t) / span : 0.0;

    out->steer    = a->steer    + (b->steer    - a->steer)    * u;
    out->throttle = a->throttle + (b->throttle - a->throttle) * u;
    out->brake    = a->brake    + (b->brake    - a->brake)    * u;
    out->clutch   = a->clutch   + (b->clutch   - a->clutch)   * u;
    out->gear     = a->gear;    /* discrete: hold, never blend */
    out->glance   = a->glance   + (b->glance   - a->glance)   * u;
    return 0;
}

/* --- built-in scripts --------------------------------------------------
 * These exist so the input path is testable before any course geometry, level
 * or mesh exists -- which today is all of them. Coordinates are inputs, not
 * positions, so nothing here depends on the course. */

/* Sits still in neutral with the brake on. The trivial case, and the one that
 * would catch a regression of the M5 neutral bug (SetGear(0) sets the gearbox
 * ratio to 1e20, MINRES stops converging, and the car sinks through the
 * ground) arriving through the driver path rather than the shim API. */
static const DrvKeyframe kf_idle[] = {
    /*  t    steer thr  brk  clu  gear glance */
    {  0.0,  0.0,  0.0, 1.0, 0.0,  0, 0.0 },
    {  5.0,  0.0,  0.0, 1.0, 0.0,  0, 0.0 },
};

/* Standing start in 1st: clutch down, then a 1.5 s release against a quarter
 * throttle. This is the shortest script that proves the whole chain -- input
 * source, ABI wall, transmission, tyres -- actually moves the car. */
static const DrvKeyframe kf_pullaway[] = {
    {  0.0,  0.0,  0.00, 0.0, 1.0,  1, 0.0 },
    {  0.5,  0.0,  0.00, 0.0, 1.0,  1, 0.0 },
    {  1.0,  0.0,  0.25, 0.0, 1.0,  1, 0.0 },
    {  2.5,  0.0,  0.35, 0.0, 0.0,  1, 0.0 },
    { 12.0,  0.0,  0.35, 0.0, 0.0,  1, 0.0 },
};

/* Pull away, then a triangle-wave steering input: +/-0.35 with a 3 s period,
 * four cycles. Linear interpolation of these keyframes IS a triangle wave --
 * that is a deliberate, exactly reproducible input, not an approximation of a
 * sine that would silently depend on how the interpolator rounds.
 *
 * 0.35 of full lock at the speed a 0.35 throttle reaches in 1st is well inside
 * the friction circle. That is on purpose: this script is here to exercise the
 * input path, not to probe the tyre limit -- the M5 sweep already showed the
 * quasi-static measurement stops meaning anything past steer ~0.70. */
static const DrvKeyframe kf_slalom[] = {
    {  0.0,  0.00, 0.00, 0.0, 1.0,  1, 0.0 },
    {  0.5,  0.00, 0.00, 0.0, 1.0,  1, 0.0 },
    {  1.0,  0.00, 0.25, 0.0, 1.0,  1, 0.0 },
    {  2.5,  0.00, 0.30, 0.0, 0.0,  1, 0.0 },
    {  6.0,  0.00, 0.30, 0.0, 0.0,  1, 0.0 },
    {  6.75, 0.35, 0.30, 0.0, 0.0,  1, 0.0 },
    {  7.5,  0.00, 0.30, 0.0, 0.0,  1, 0.0 },
    {  8.25,-0.35, 0.30, 0.0, 0.0,  1, 0.0 },
    {  9.0,  0.00, 0.30, 0.0, 0.0,  1, 0.0 },
    {  9.75, 0.35, 0.30, 0.0, 0.0,  1, 0.0 },
    { 10.5,  0.00, 0.30, 0.0, 0.0,  1, 0.0 },
    { 11.25,-0.35, 0.30, 0.0, 0.0,  1, 0.0 },
    { 12.0,  0.00, 0.30, 0.0, 0.0,  1, 0.0 },
};

/* Shift 1->2 under power, crossing neutral the way an H-pattern shifter does.
 * The M5 note is explicit that every real gear change passes through neutral,
 * and that neutral is the path which used to drop the car through the floor. A
 * script that never leaves 1st would never test it. */
static const DrvKeyframe kf_upshift[] = {
    {  0.0,  0.0,  0.00, 0.0, 1.0,  1, 0.0 },
    {  0.5,  0.0,  0.00, 0.0, 1.0,  1, 0.0 },
    {  1.0,  0.0,  0.30, 0.0, 1.0,  1, 0.0 },
    {  2.5,  0.0,  0.60, 0.0, 0.0,  1, 0.0 },
    {  6.0,  0.0,  0.60, 0.0, 0.0,  1, 0.0 },
    {  6.1,  0.0,  0.00, 0.0, 1.0,  0, 0.0 },   /* clutch in, through neutral     */
    {  6.4,  0.0,  0.00, 0.0, 1.0,  0, 0.0 },
    {  6.5,  0.0,  0.00, 0.0, 1.0,  2, 0.0 },   /* 2nd selected, clutch still in  */
    {  7.2,  0.0,  0.45, 0.0, 0.0,  2, 0.0 },   /* engaged, back on the throttle  */
    { 14.0,  0.0,  0.45, 0.0, 0.0,  2, 0.0 },
};

struct DrvBuiltin { const char* name; const DrvKeyframe* kf; int32_t n; };

static const struct DrvBuiltin g_builtins[] = {
    { "idle",     kf_idle,     (int32_t)(sizeof kf_idle     / sizeof kf_idle[0])     },
    { "pullaway", kf_pullaway, (int32_t)(sizeof kf_pullaway / sizeof kf_pullaway[0]) },
    { "slalom",   kf_slalom,   (int32_t)(sizeof kf_slalom   / sizeof kf_slalom[0])   },
    { "upshift",  kf_upshift,  (int32_t)(sizeof kf_upshift  / sizeof kf_upshift[0])  },
};
static const int g_builtin_count = (int)(sizeof g_builtins / sizeof g_builtins[0]);

int32_t drv_script_builtin(const char* name, DrvScript* out)
{
    if (!name || !out) return -1;
    for (int i = 0; i < g_builtin_count; ++i)
    {
        if (strcmp(name, g_builtins[i].name) != 0) continue;
        out->kf       = g_builtins[i].kf;
        out->n        = g_builtins[i].n;
        out->duration = g_builtins[i].kf[g_builtins[i].n - 1].t;
        out->owns_kf  = 0;      /* static storage: nothing to free */
        return 0;
    }
    return -1;
}

const char* const* drv_script_builtin_names(void)
{
    /* Rebuilt once, on first call. Static so the caller can hold it. */
    static const char* names[sizeof g_builtins / sizeof g_builtins[0] + 1];
    for (int i = 0; i < g_builtin_count; ++i) { names[i] = g_builtins[i].name; }
    names[g_builtin_count] = NULL;
    return names;
}

/* --- file loader ------------------------------------------------------- */

#define DRV_MAX_KEYFRAMES 4096

int32_t drv_script_load(DrvScript* s, const char* path, char* err, size_t errsz)
{
    if (err && errsz) err[0] = '\0';
    if (!s || !path)
    {
        if (err && errsz) snprintf(err, errsz, "null argument");
        return -1;
    }
    memset(s, 0, sizeof(*s));

    FILE* f = fopen(path, "r");
    if (!f)
    {
        if (err && errsz) snprintf(err, errsz, "cannot open '%s'", path);
        return -1;
    }

    DrvKeyframe* kf = NULL;
    int32_t n = 0, cap = 0;
    char line[256];
    int lineno = 0;

    while (fgets(line, sizeof line, f))
    {
        ++lineno;

        /* Strip comments and decide whether anything is left. */
        char* hash = strchr(line, '#');
        if (hash) *hash = '\0';

        DrvKeyframe k;
        int gear = 0, used = 0;
        const int got = sscanf(line, "%lf %lf %lf %lf %lf %d%n",
                               &k.t, &k.steer, &k.throttle, &k.brake,
                               &k.clutch, &gear, &used);
        if (got <= 0) continue;              /* blank or comment-only line */
        /* The optional 7th field, then nothing: a stray 8th number was once
         * a silently ignored 7th, and is now an error with a line number. So
         * is a gear with a fraction, whose ".5" would now read as a glance. */
        int extra = got == 6 && line[used] != '\0' && !isspace((unsigned char)line[used]);
        int more = 0;
        k.glance = 0.0;
        if (got == 6 && !extra && sscanf(line + used, "%lf%n", &k.glance, &more) == 1)
        {
            used += more;
            double junk;
            extra = sscanf(line + used, "%lf", &junk) == 1;
        }
        if (got != 6 || extra)
        {
            if (err && errsz)
                snprintf(err, errsz,
                         "%s:%d: expected 6 or 7 fields "
                         "(t steer throttle brake clutch gear [glance]), got %s",
                         path, lineno, extra ? "more" : "fewer");
            goto fail;
        }
        k.gear = (int32_t)gear;

        /* Non-decreasing time is what makes the linear scan in eval correct.
         * Reject it here, with a line number, rather than silently evaluating
         * a table that has a segment of negative span. */
        if (n > 0 && k.t < kf[n - 1].t)
        {
            if (err && errsz)
                snprintf(err, errsz, "%s:%d: t=%.3f goes backwards (previous %.3f)",
                         path, lineno, k.t, kf[n - 1].t);
            goto fail;
        }

        /* Clamp to the ranges the shim will clamp to anyway, but do it HERE so
         * a typo is visible in a dumped script rather than silently absorbed
         * three layers down. */
        k.steer    = drv_clamp(k.steer,    -1.0, 1.0);
        k.throttle = drv_clamp(k.throttle,  0.0, 1.0);
        k.brake    = drv_clamp(k.brake,     0.0, 1.0);
        k.clutch   = drv_clamp(k.clutch,    0.0, 1.0);
        k.glance   = drv_clamp(k.glance,   -1.0, 1.0);

        if (n == cap)
        {
            if (cap >= DRV_MAX_KEYFRAMES)
            {
                if (err && errsz)
                    snprintf(err, errsz, "%s:%d: more than %d keyframes",
                             path, lineno, DRV_MAX_KEYFRAMES);
                goto fail;
            }
            int32_t ncap = cap ? cap * 2 : 32;
            if (ncap > DRV_MAX_KEYFRAMES) ncap = DRV_MAX_KEYFRAMES;
            DrvKeyframe* grown = (DrvKeyframe*)realloc(kf, (size_t)ncap * sizeof *kf);
            if (!grown)
            {
                if (err && errsz) snprintf(err, errsz, "out of memory");
                goto fail;
            }
            kf = grown;
            cap = ncap;
        }
        kf[n++] = k;
    }
    fclose(f);

    if (n == 0)
    {
        if (err && errsz) snprintf(err, errsz, "%s: no keyframes", path);
        free(kf);
        return -1;
    }

    s->kf       = kf;
    s->n        = n;
    s->duration = kf[n - 1].t;
    s->owns_kf  = 1;
    return 0;

fail:
    fclose(f);
    free(kf);
    memset(s, 0, sizeof(*s));
    return -1;
}

void drv_script_free(DrvScript* s)
{
    if (!s) return;
    /* Only free what this module allocated. A built-in points at static
     * storage and freeing it would be a wild pointer -- the same class of
     * mistake the ABI wall's rule 3 exists to prevent, one layer down. */
    if (s->owns_kf && s->kf) { free((void*)s->kf); }
    memset(s, 0, sizeof(*s));
}
