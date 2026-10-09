/* ===========================================================================
 *  test_driver.c — offline self-test for the keyboard integrator and the
 *  script evaluator.
 *
 *  Needs no hardware, no container, no Chrono and no Unreal. It compiles the
 *  SAME driver_source.cpp the plugin ships, so what passes here is what runs.
 *
 *  The properties asserted below are the ones the rest of Milestone 6 will
 *  depend on: that a key press produces a CONTINUOUS axis, that a script is a
 *  pure function of time, and that gear is discrete. Every referee rule to
 *  come is a predicate over a trajectory, and a trajectory is only worth
 *  asserting about if its input is reproducible.
 * ======================================================================== */

#define _GNU_SOURCE
#include "driver_source.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static int g_pass = 0, g_fail = 0;

static void ok(int cond, const char* fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    if (cond) { ++g_pass; printf("  ok   "); }
    else      { ++g_fail; printf("  FAIL "); }
    vprintf(fmt, ap);
    printf("\n");
    va_end(ap);
}

static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* Run the keyboard integrator for `secs` at 1 kHz with a fixed key mask. */
static void hold(DrvKeyboard* kb, uint32_t keys, double secs)
{
    const double dt = 0.001;
    const int n = (int)(secs / dt + 0.5);
    for (int i = 0; i < n; ++i) { drv_keyboard_step(kb, keys, dt); }
}

/* =========================================================================
 *  KEYBOARD
 * ====================================================================== */

static void test_keyboard(void)
{
    printf("keyboard integrator\n");

    /* --- rate is what the header claims ------------------------------- */
    DrvKeyboard kb;
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_RIGHT, 0.5);
    /* 1.2 units/s for 0.5 s. Tolerance is one step's worth of rate. */
    ok(near(kb.out.steer, 0.6, 0.002),
       "steer right 0.5 s -> %.4f (expect 0.600 = 1.2/s x 0.5 s)", kb.out.steer);

    /* Device convention: +1 is clockwise, the driver's right. */
    ok(kb.out.steer > 0.0, "right key gives POSITIVE steer (device convention)");

    /* --- it clamps, and does not creep past lock ----------------------- */
    hold(&kb, DRV_KEY_RIGHT, 2.0);
    ok(near(kb.out.steer, 1.0, 1e-12), "steer clamps at +1.0 (%.6f)", kb.out.steer);

    /* --- THE DESIGN DECISION: the rack HOLDS when nothing is pressed ---
     * If this ever starts self-centring, parallel parking becomes impossible
     * to perform from a keyboard: a key cannot be held at 40% of lock. */
    hold(&kb, 0u, 1.0);
    ok(near(kb.out.steer, 1.0, 1e-12),
       "steer HOLDS with no key held (%.6f) -- no auto-centre", kb.out.steer);

    /* --- explicit centring, at its own rate ---------------------------- */
    hold(&kb, DRV_KEY_CENTER, 0.25);
    ok(near(kb.out.steer, 0.5, 0.003),
       "centre 0.25 s from lock -> %.4f (expect 0.500 = 1 - 2.0/s x 0.25 s)",
       kb.out.steer);
    hold(&kb, DRV_KEY_CENTER, 1.0);
    ok(near(kb.out.steer, 0.0, 1e-12),
       "centre settles exactly on 0 without dithering (%.9f)", kb.out.steer);

    /* --- left is the mirror image -------------------------------------- */
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_LEFT, 0.5);
    ok(near(kb.out.steer, -0.6, 0.002), "steer left 0.5 s -> %.4f", kb.out.steer);

    /* --- both keys cancel, they do not fight --------------------------- */
    const double before = kb.out.steer;
    hold(&kb, DRV_KEY_LEFT | DRV_KEY_RIGHT, 0.5);
    ok(near(kb.out.steer, before, 1e-12),
       "left+right together hold position (%.4f)", kb.out.steer);

    /* --- fine modifier ------------------------------------------------- */
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_RIGHT | DRV_KEY_FINE, 0.5);
    ok(near(kb.out.steer, 0.6 * DRV_FINE_SCALE, 0.002),
       "fine modifier scales steer rate to %.2fx -> %.4f",
       DRV_FINE_SCALE, kb.out.steer);

    /* --- pedals: asymmetric by design ---------------------------------- */
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_THROTTLE, 0.125);
    const double thr_up = kb.out.throttle;          /* 4.0/s x 0.125 = 0.5 */
    hold(&kb, 0u, 0.03125);
    const double thr_after = kb.out.throttle;       /* 8.0/s x 0.03125 = 0.25 */
    ok(near(thr_up, 0.5, 0.005), "throttle rises at 4.0/s -> %.4f", thr_up);
    ok(near(thr_after, 0.25, 0.01),
       "throttle falls at 8.0/s (faster than it rises) -> %.4f", thr_after);

    /* --- clutch polarity: down = 1 = DISENGAGED ------------------------
     * Getting this backwards makes the hill start trivially easy instead of
     * the clutch-control exercise it exists to be. Verified against
     * ChManualTransmissionShafts in M5; asserted here so it stays that way. */
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_CLUTCH, 0.5);
    ok(near(kb.out.clutch, 1.0, 1e-12),
       "clutch key held -> %.3f (1 = pedal DOWN = disengaged)", kb.out.clutch);
    hold(&kb, 0u, 0.5);
    ok(near(kb.out.clutch, 0.0, 1e-12),
       "clutch released -> %.3f (0 = engaged) in 0.5 s at 2.0/s", kb.out.clutch);

    /* --- gear is edge triggered ---------------------------------------- */
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_GEAR_3, 0.010);
    ok(kb.out.gear == 0,
       "gear key already held at spawn does not fire (gear=%d)", kb.out.gear);

    hold(&kb, 0u, 0.010);
    hold(&kb, DRV_KEY_GEAR_3, 0.010);
    ok(kb.out.gear == 3, "press 3 -> gear %d", kb.out.gear);

    hold(&kb, DRV_KEY_GEAR_3, 0.100);
    ok(kb.out.gear == 3, "holding 3 does not re-fire (gear %d)", kb.out.gear);

    hold(&kb, 0u, 0.010);
    hold(&kb, DRV_KEY_GEAR_R, 0.010);
    ok(kb.out.gear == -1, "press R -> gear %d (reverse)", kb.out.gear);

    hold(&kb, 0u, 0.010);
    hold(&kb, DRV_KEY_GEAR_N, 0.010);
    ok(kb.out.gear == 0, "press 0 -> gear %d (neutral)", kb.out.gear);

    /* --- CONTINUITY: the entire reason this integrator exists -----------
     * Drive a pseudo-random key sequence that changes every 16.7 ms -- i.e.
     * at exactly the rate a 60 Hz game thread would republish -- and assert
     * that no axis ever moves more in one 1 ms step than its fastest rate
     * allows. If keys were sampled as axis values instead of integrated,
     * this is where a 0 -> 1 jump would show up. */
    drv_keyboard_init(&kb);
    const uint32_t pool[] = {
        0u, DRV_KEY_LEFT, DRV_KEY_RIGHT, DRV_KEY_THROTTLE, DRV_KEY_BRAKE,
        DRV_KEY_CLUTCH, DRV_KEY_CENTER, DRV_KEY_RIGHT | DRV_KEY_THROTTLE,
        DRV_KEY_LEFT | DRV_KEY_FINE, DRV_KEY_THROTTLE | DRV_KEY_CLUTCH,
    };
    const double dt = 0.001;
    const double max_rate = DRV_RATE_THROTTLE_DN;   /* the fastest of them all */
    const double bound = max_rate * dt + 1e-12;
    double worst = 0.0;
    uint32_t seed = 12345u;
    uint32_t keys = 0u;
    DrvInput prev = kb.out;
    for (int i = 0; i < 60000; ++i)              /* 60 s at 1 kHz */
    {
        if (i % 17 == 0)                          /* ~59 Hz republish */
        {
            seed = seed * 1103515245u + 12345u;
            keys = pool[(seed >> 16) % (sizeof pool / sizeof pool[0])];
        }
        drv_keyboard_step(&kb, keys, dt);
        const double d[4] = {
            fabs(kb.out.steer    - prev.steer),
            fabs(kb.out.throttle - prev.throttle),
            fabs(kb.out.brake    - prev.brake),
            fabs(kb.out.clutch   - prev.clutch),
        };
        for (int a = 0; a < 4; ++a) { if (d[a] > worst) worst = d[a]; }
        prev = kb.out;
    }
    ok(worst <= bound,
       "60 s of 60 Hz key churn: worst per-step axis jump %.6f <= %.6f",
       worst, bound);

    /* And the axes stayed in range the whole time. */
    ok(kb.out.steer >= -1.0 && kb.out.steer <= 1.0 &&
       kb.out.throttle >= 0.0 && kb.out.throttle <= 1.0 &&
       kb.out.brake >= 0.0 && kb.out.brake <= 1.0 &&
       kb.out.clutch >= 0.0 && kb.out.clutch <= 1.0,
       "all axes remained in range after the churn");
}

/* =========================================================================
 *  SCRIPT
 * ====================================================================== */

static void test_script(void)
{
    printf("\nscript evaluator\n");

    DrvScript s;
    ok(drv_script_builtin("pullaway", &s) == 0, "built-in 'pullaway' resolves");
    ok(s.n == 5 && near(s.duration, 12.0, 1e-12),
       "pullaway: %d keyframes, %.1f s", s.n, s.duration);

    DrvInput in;

    /* Before the first keyframe: hold it, do not invent state.
     * NOTE the shape of these three: the eval runs on its own line and the
     * result is latched into a local before ok() is called. Putting the call
     * inside ok()'s condition and reading `in` in the varargs is an
     * unsequenced read -- the message printed a stale value while the
     * assertion itself passed, which is the worst possible failure mode for a
     * test: green, and unreadable. */
    int rc = drv_script_eval(&s, -1.0, &in);
    ok(rc == 0 && near(in.clutch, 1.0, 1e-12),
       "t<0 holds the first keyframe (clutch %.2f)", in.clutch);

    /* Midpoint of the 1.0 -> 2.5 s segment: throttle 0.25 -> 0.35,
     * clutch 1.0 -> 0.0. Halfway is 0.30 and 0.50. */
    drv_script_eval(&s, 1.75, &in);
    ok(near(in.throttle, 0.30, 1e-9) && near(in.clutch, 0.5, 1e-9),
       "linear interpolation at t=1.75: throttle %.3f clutch %.3f",
       in.throttle, in.clutch);

    /* Past the end: hold, and SAY it ended. Zeroing here would change the
     * trajectory a referee rule is being asked about. */
    rc = drv_script_eval(&s, 99.0, &in);
    ok(rc == 1 && near(in.throttle, 0.35, 1e-12),
       "past the end returns 1 and HOLDS the last values (throttle %.2f)",
       in.throttle);

    /* --- gear is discrete ---------------------------------------------- */
    drv_script_builtin("upshift", &s);
    int seen_blend = 0, seen_1 = 0, seen_0 = 0, seen_2 = 0;
    for (int i = 0; i <= 14000; ++i)
    {
        drv_script_eval(&s, i * 0.001, &in);
        if (in.gear == 1) seen_1 = 1;
        else if (in.gear == 0) seen_0 = 1;
        else if (in.gear == 2) seen_2 = 1;
        else seen_blend = 1;
    }
    ok(!seen_blend && seen_1 && seen_0 && seen_2,
       "upshift: gear only ever 1, 0, 2 -- never interpolated");

    /* --- the property everything else rests on: PURE FUNCTION OF TIME --- */
    drv_script_builtin("slalom", &s);
    unsigned char h1[sizeof(DrvInput)], h2[sizeof(DrvInput)];
    int identical = 1;
    for (int pass = 0; pass < 2; ++pass)
    {
        DrvInput acc; memset(&acc, 0, sizeof acc);
        for (int i = 0; i <= 12000; ++i)
        {
            DrvInput v;
            drv_script_eval(&s, i * 0.001, &v);
            /* Fold every sample in so a single differing step is caught. */
            acc.steer    = acc.steer    * 0.5 + v.steer;
            acc.throttle = acc.throttle * 0.5 + v.throttle;
            acc.brake    = acc.brake    * 0.5 + v.brake;
            acc.clutch   = acc.clutch   * 0.5 + v.clutch;
            acc.gear    += v.gear;
        }
        memcpy(pass ? h2 : h1, &acc, sizeof acc);
    }
    identical = (memcmp(h1, h2, sizeof h1) == 0);
    ok(identical, "12000 samples fold BIT-IDENTICALLY across two passes");

    /* --- and the input it produces is physically issuable ---------------
     * Max steering slope in the slalom table is 0.35 units over 0.75 s =
     * 0.467 units/s, or 283 deg/s at the rim. A human hand does that easily.
     * Asserting it here is what stops a future edit adding a keyframe pair
     * that commands a step change into the tyre model. */
    double worst_rate = 0.0;
    DrvInput prev; drv_script_eval(&s, 0.0, &prev);
    for (int i = 1; i <= 12000; ++i)
    {
        drv_script_eval(&s, i * 0.001, &in);
        const double r = fabs(in.steer - prev.steer) / 0.001;
        if (r > worst_rate) worst_rate = r;
        prev = in;
    }
    ok(worst_rate < 2.0,
       "slalom steering rate peaks at %.3f units/s (%.0f deg/s at the rim)",
       worst_rate, worst_rate * 605.0);
}

/* =========================================================================
 *  LANE KEEPING
 * ====================================================================== */

static void test_lanekeep(void)
{
    printf("\nlane keeping\n");

    const DrvLaneKeep lk = DRV_LANEKEEP_DEFAULT;

    ok(near(drv_lane_keep(&lk, 0.0, 0.0), 0.0, 1e-12),
       "on the line, pointing along it -> zero correction");

    /* Device convention: + is clockwise, the driver's RIGHT. A car that has
     * drifted to +Y (its left) must steer right to come back. Getting this
     * backwards makes the controller drive the car off the road faster, which
     * is exactly what the drift already does unaided. */
    ok(drv_lane_keep(&lk, +1.0, 0.0) > 0.0,
       "drifted LEFT (+y) -> positive (rightward) correction");
    ok(drv_lane_keep(&lk, -1.0, 0.0) < 0.0,
       "drifted RIGHT (-y) -> negative (leftward) correction");
    ok(drv_lane_keep(&lk, 0.0, +0.1) > 0.0,
       "pointing LEFT -> positive (rightward) correction");

    ok(near(drv_lane_keep(&lk, 1.0, 0.0), 0.25, 1e-12),
       "lateral gain is k_lateral (%.3f)", drv_lane_keep(&lk, 1.0, 0.0));

    ok(near(drv_lane_keep(&lk, 100.0, 0.0), lk.limit, 1e-12),
       "clamps to +limit however large the error");
    ok(near(drv_lane_keep(&lk, -100.0, 0.0), -lk.limit, 1e-12),
       "clamps to -limit");

    /* The property the reference runs actually need: a closed loop over this
     * controller must SETTLE, not hunt. Simulated with a kinematic bicycle at
     * 8 m/s, which is the speed the scripted runs reach. */
    {
        const double L = 2.75, v = 8.0, dt = 0.001;
        double y = 2.0, psi = 0.0;          /* start 2 m off line */
        double worst_after_settle = 0.0;
        for (int i = 0; i < 20000; ++i) {   /* 20 s */
            const double cmd = drv_lane_keep(&lk, y, psi);
            /* device -> road-wheel angle, positive-left, at ~34.6 deg lock */
            const double delta = -cmd * (34.6 * 3.14159265358979 / 180.0);
            psi += v * tan(delta) / L * dt;
            y   += v * sin(psi) * dt;
            if (i > 8000 && fabs(y) > worst_after_settle) worst_after_settle = fabs(y);
        }
        ok(worst_after_settle < 0.05,
           "closed loop settles from 2.0 m off line: |y| < %.4f m after 8 s",
           worst_after_settle);
    }
}

/* =========================================================================
 *  LOADER
 * ====================================================================== */

static void test_loader(void)
{
    printf("\nscript loader\n");

    char path[] = "/tmp/drv_test_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) { ok(0, "mkstemp"); return; }
    FILE* f = fdopen(fd, "w");
    fprintf(f,
            "# a comment line\n"
            "\n"
            "0.0  0.0  0.0  0.0  1.0  1   # trailing comment\n"
            "1.0  0.5  0.4  0.0  0.0  1\n"
            "2.0 -0.5  0.4  0.0  0.0  2\n");
    fclose(f);

    DrvScript s;
    char err[256];
    int rc = drv_script_load(&s, path, err, sizeof err);
    ok(rc == 0, "loads a well-formed file (%s)", err[0] ? err : "no error");
    ok(s.n == 3 && s.owns_kf == 1, "3 keyframes, owns its table (n=%d)", s.n);

    DrvInput in;
    drv_script_eval(&s, 0.5, &in);
    ok(near(in.steer, 0.25, 1e-9), "loaded table interpolates (steer %.4f)", in.steer);
    ok(in.gear == 1, "gear holds the earlier keyframe's value (%d)", in.gear);

    drv_script_free(&s);
    ok(s.kf == NULL && s.n == 0, "free zeroes the struct");

    /* Freeing a BUILT-IN must not touch static storage. */
    drv_script_builtin("idle", &s);
    drv_script_free(&s);
    ok(s.kf == NULL, "freeing a built-in is safe (owns_kf was 0)");

    /* --- malformed input is REJECTED WITH A LINE NUMBER -----------------
     * Chrono's JSON readers segfault on a missing key and name nothing. That
     * cost real time in M4. This loader does the opposite on purpose. */
    char bad[] = "/tmp/drv_bad_XXXXXX";
    fd = mkstemp(bad);
    f = fdopen(fd, "w");
    fprintf(f, "0.0 0 0 0 1 1\n"
               "2.0 0 0 0 0 1\n"
               "1.0 0 0 0 0 1\n");     /* time goes backwards on line 3 */
    fclose(f);
    err[0] = '\0';
    rc = drv_script_load(&s, bad, err, sizeof err);
    ok(rc == -1 && strstr(err, ":3:") != NULL,
       "backwards time rejected, naming the line: %s", err);

    char bad2[] = "/tmp/drv_bad2_XXXXXX";
    fd = mkstemp(bad2);
    f = fdopen(fd, "w");
    fprintf(f, "0.0 0 0 0 1\n");        /* only 5 fields */
    fclose(f);
    err[0] = '\0';
    rc = drv_script_load(&s, bad2, err, sizeof err);
    ok(rc == -1 && strstr(err, ":1:") != NULL,
       "short line rejected, naming the line: %s", err);

    err[0] = '\0';
    rc = drv_script_load(&s, "/nonexistent/script.txt", err, sizeof err);
    ok(rc == -1 && err[0], "missing file rejected: %s", err);

    remove(path); remove(bad); remove(bad2);
}

/* =========================================================================
 *  GLANCE (task 17)
 * ====================================================================== */

static void test_glance(void)
{
    printf("\nglance\n");

    /* --- held key: the head turns at the stated rate, and only it ------- */
    DrvKeyboard kb;
    drv_keyboard_init(&kb);
    hold(&kb, DRV_KEY_GLANCE_L, 0.1);
    ok(near(kb.out.glance, -0.4, 1e-9), "Z held 0.1 s: glance %.4f (rate 4/s: -0.4)", kb.out.glance);
    hold(&kb, DRV_KEY_GLANCE_L, 0.2);
    ok(kb.out.glance == -1.0, "held 0.3 s: exactly -1, clamped (%.4f)", kb.out.glance);
    ok(kb.out.steer == 0.0 && kb.out.throttle == 0.0 && kb.out.brake == 0.0 &&
       kb.out.clutch == 0.0 && kb.out.gear == 0,
       "glancing moves no control");

    /* --- released: it comes back, at the same rate --------------------- */
    double worst = 0.0, prev = kb.out.glance;
    for (int i = 0; i < 300; ++i)
    {
        drv_keyboard_step(&kb, 0, 0.001);
        worst = fmax(worst, fabs(kb.out.glance - prev));
        prev = kb.out.glance;
    }
    ok(kb.out.glance == 0.0, "released: back to 0 within 0.3 s (%.4f)", kb.out.glance);
    ok(worst <= DRV_RATE_GLANCE * 0.001 + 1e-12,
       "no step larger than rate x dt (%.5f per ms): a turn, never a cut", worst);

    /* --- right, both, and the fine modifier ------------------------------ */
    hold(&kb, DRV_KEY_GLANCE_R | DRV_KEY_FINE, 0.25);
    ok(near(kb.out.glance, 1.0, 1e-9), "C with the fine modifier: full right in 0.25 s (%.4f)",
       kb.out.glance);
    hold(&kb, DRV_KEY_GLANCE_L | DRV_KEY_GLANCE_R, 0.3);
    ok(kb.out.glance == 0.0, "both held: looks ahead (%.4f)", kb.out.glance);

    /* --- the wheel's D-pad path is the same step ------------------------ */
    double g = 0.0;
    for (int i = 0; i < 125; ++i) { g = drv_glance_step(g, -1, 0.001); }
    ok(near(g, -0.5, 1e-9), "drv_glance_step(-1) for 0.125 s: %.4f", g);
    for (int i = 0; i < 500; ++i) { g = drv_glance_step(g, +1, 0.001); }
    ok(g == 1.0, "drv_glance_step(+1) settles at exactly +1 (%.4f)", g);

    /* --- the ease: ends fixed, odd, monotonic, flat at both ends -------- */
    ok(drv_glance_ease(0.0) == 0.0 && drv_glance_ease(1.0) == 1.0 &&
       drv_glance_ease(-1.0) == -1.0 && near(drv_glance_ease(0.5), 0.5, 1e-12),
       "ease: 0, +-1 and 0.5 fixed");
    ok(drv_glance_ease(2.0) == 1.0 && drv_glance_ease(-3.0) == -1.0, "ease clamps past +-1");
    int mono = 1, odd = 1;
    for (int i = 0; i < 1000; ++i)
    {
        const double a = i / 1000.0, b = (i + 1) / 1000.0;
        if (drv_glance_ease(b) < drv_glance_ease(a)) mono = 0;
        if (drv_glance_ease(-a) != -drv_glance_ease(a)) odd = 0;
    }
    ok(mono && odd, "ease is monotonic and odd");
    const double s0 = drv_glance_ease(0.01) / 0.01;
    const double s1 = (1.0 - drv_glance_ease(0.99)) / 0.01;
    ok(s0 < 0.05 && s1 < 0.05, "ease starts and stops flat (slope %.4f, %.4f; linear is 1)", s0, s1);

    /* --- scripts: the optional 7th column ------------------------------- */
    char path[] = "/tmp/drv_glance_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) { ok(0, "mkstemp"); return; }
    FILE* f = fdopen(fd, "w");
    fprintf(f, "0.0  0.0 0.0 1.0 1.0 0\n"         /* six fields: glance 0 */
               "1.0  0.0 0.0 1.0 1.0 0  0.0\n"
               "1.5  0.0 0.0 1.0 1.0 0 -1.0  # look left\n"
               "9.0  0.0 0.0 1.0 1.0 0 -1.0\n");
    fclose(f);
    DrvScript s;
    char err[256] = "";
    const int rc = drv_script_load(&s, path, err, sizeof err);
    ok(rc == 0 && s.n == 4, "six- and seven-field lines mix (%s)", err[0] ? err : "loaded");
    DrvInput in;
    drv_script_eval(&s, 0.5, &in);
    ok(in.glance == 0.0, "a six-field keyframe looks ahead (%.4f)", in.glance);
    drv_script_eval(&s, 1.25, &in);
    ok(near(in.glance, -0.5, 1e-12), "glance interpolates like a pedal (%.4f at t=1.25)", in.glance);
    drv_script_eval(&s, 20.0, &in);
    ok(in.glance == -1.0, "and holds past the end (%.4f)", in.glance);
    drv_script_free(&s);

    drv_script_builtin("pullaway", &s);
    double gmax = 0.0;
    for (double t = 0.0; t < s.duration + 1.0; t += 0.01)
    {
        drv_script_eval(&s, t, &in);
        gmax = fmax(gmax, fabs(in.glance));
    }
    ok(gmax == 0.0, "a built-in never glances");

    char bad[] = "/tmp/drv_glance_bad_XXXXXX";
    fd = mkstemp(bad);
    f = fdopen(fd, "w");
    fprintf(f, "0.0 0 0 0 1 1 0.0\n"
               "1.0 0 0 0 1 1 0.0 7\n");          /* eight fields */
    fclose(f);
    err[0] = '\0';
    ok(drv_script_load(&s, bad, err, sizeof err) == -1 && strstr(err, ":2:") != NULL,
       "an eighth field is rejected, naming the line: %s", err);
    f = fopen(bad, "w");
    fprintf(f, "0.0 0 0 0 1 1\n"
               "1.0 0 0 0 1 1.5\n");             /* gear 1.5: not gear 1, glance .5 */
    fclose(f);
    err[0] = '\0';
    ok(drv_script_load(&s, bad, err, sizeof err) == -1 && strstr(err, ":2:") != NULL,
       "a fractional gear is rejected, not read as a glance: %s", err);
    remove(path); remove(bad);
}

/* =========================================================================
 *  COST
 * ====================================================================== */

static double now_us(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec * 1e-3;
}

static void test_cost(void)
{
    printf("\ncost per physics step\n");

    DrvKeyboard kb; drv_keyboard_init(&kb);
    DrvScript s;    drv_script_builtin("slalom", &s);
    DrvInput in;

    const int N = 200000;
    volatile double sink = 0.0;

    double t0 = now_us();
    for (int i = 0; i < N; ++i) { drv_keyboard_step(&kb, DRV_KEY_RIGHT, 0.001); }
    double kb_us = (now_us() - t0) / N;
    sink += kb.out.steer;

    t0 = now_us();
    for (int i = 0; i < N; ++i) { drv_script_eval(&s, (i % 12000) * 0.001, &in); }
    double sc_us = (now_us() - t0) / N;
    sink += in.steer;

    printf("  keyboard step : %.4f us\n", kb_us);
    printf("  script eval   : %.4f us  (%d keyframes, linear scan)\n", sc_us, s.n);
    /* The physics thread has a 1000 us period and spends 180 us on the
     * vehicle. Anything here above a microsecond would deserve an argument;
     * these are two orders below that. */
    ok(kb_us < 1.0 && sc_us < 1.0,
       "both under 1 us -- negligible against a 180 us vehicle step");
    (void)sink;
}

/* =========================================================================
 *  dump: print a script as the physics thread would see it
 * ====================================================================== */

static int do_dump(const char* name, double hz)
{
    DrvScript s;
    char err[256];
    if (drv_script_builtin(name, &s) != 0 &&
        drv_script_load(&s, name, err, sizeof err) != 0)
    {
        fprintf(stderr, "no built-in and cannot load '%s': %s\n", name, err);
        fprintf(stderr, "built-ins:");
        for (const char* const* p = drv_script_builtin_names(); *p; ++p)
            fprintf(stderr, " %s", *p);
        fprintf(stderr, "\n");
        return 1;
    }

    printf("# %s: %d keyframes, %.2f s, sampled at %.0f Hz\n", name, s.n, s.duration, hz);
    printf("#    t     steer  throttle  brake  clutch  gear  glance  state\n");
    const double dt = 1.0 / hz;
    for (double t = 0.0; t <= s.duration + dt * 0.5; t += dt)
    {
        DrvInput in;
        const int ended = drv_script_eval(&s, t, &in);
        printf("%7.3f  %7.4f  %7.4f  %6.4f  %6.4f  %4d  %6.3f  %s\n",
               t, in.steer, in.throttle, in.brake, in.clutch, in.gear, in.glance,
               ended ? "ended" : "run");
    }
    drv_script_free(&s);
    return 0;
}

int main(int argc, char** argv)
{
    const char* mode = argc > 1 ? argv[1] : "selftest";

    if (!strcmp(mode, "dump"))
        return do_dump(argc > 2 ? argv[2] : "slalom",
                       argc > 3 ? atof(argv[3]) : 20.0);

    if (!strcmp(mode, "list"))
    {
        for (const char* const* p = drv_script_builtin_names(); *p; ++p)
            printf("%s\n", *p);
        return 0;
    }

    if (strcmp(mode, "selftest") != 0)
    {
        fprintf(stderr, "usage: test_driver selftest | dump <name|file> [hz] | list\n");
        return 2;
    }

    test_keyboard();
    test_script();
    test_lanekeep();
    test_loader();
    test_glance();
    test_cost();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
