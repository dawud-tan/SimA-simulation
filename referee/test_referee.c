/* ===========================================================================
 *  test_referee.c — offline self-test for the SIM A rule engine.
 *
 *  No hardware, no container, no Chrono, no Unreal. Compiles the SAME
 *  referee.cpp the plugin ships.
 *
 *  Every rule here is a predicate over a trajectory, so each test synthesises
 *  a trajectory and asserts the verdict. That is cheap precisely because the
 *  referee does not need physics: it needs a pose per step, and a pose per
 *  step can be written down.
 * ======================================================================== */

#define _GNU_SOURCE
#include "referee.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int g_pass = 0, g_fail = 0;

static void ok(int cond, const char* fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    if (cond) { ++g_pass; printf("  ok   "); } else { ++g_fail; printf("  FAIL "); }
    vprintf(fmt, ap); printf("\n"); va_end(ap);
}
static int near(double a, double b, double tol) { return fabs(a - b) <= tol; }

/* A minimal course built in memory, so the geometry tests do not depend on
 * whatever gen_course.py currently emits. */
static void tiny_course(RefCourse* c)
{
    memset(c, 0, sizeof(*c));
    c->car_length = 4.735; c->car_width = 1.830;
    c->front_overhang = 0.880; c->rear_overhang = 1.105; c->wheelbase = 2.750;
    c->r_roll = 0.3331;
}

static RefVehicleState vs(double t, uint64_t step, double x, double y, double yaw)
{
    RefVehicleState v;
    memset(&v, 0, sizeof v);
    v.t = t; v.step = step; v.x = x; v.y = y; v.yaw = yaw;
    v.engine_rpm = 1500.0; v.gear = 1; v.clutch = 0.0;
    return v;
}

/* ref_update takes a pointer, and C has no address-of for an rvalue, so the
 * sample needs a name. Wrapping it keeps the trajectory loops readable. */
static void feed(RefState* s, double t, uint64_t step, double x, double y, double yaw)
{
    const RefVehicleState v = vs(t, step, x, y, yaw);
    ref_update(s, &v);
}

static int count_of(const RefState* s, int32_t type)
{
    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(s, ev, REF_MAX_EVENTS);
    int c = 0;
    for (int32_t i = 0; i < n; ++i) if (ev[i].type == type) ++c;
    return c;
}

/* The last event of `type`, or NULL. */
static const RefEvent* last_of(const RefState* s, int32_t type)
{
    const RefEvent* found = NULL;
    for (int32_t i = 0; i < s->n_events; ++i)
        if (s->events[i].type == type) found = &s->events[i];
    return found;
}

/* ------------------------------------------------------------------ */

static void test_geometry(void)
{
    printf("body geometry\n");

    RefCourse c; tiny_course(&c);
    double k[4][2];
    ref_body_corners(&c, 0.0, 0.0, 0.0, k);

    /* The reference frame is at the FRONT AXLE, so the nose is only
     * front_overhang ahead of the origin and the tail is a whole wheelbase
     * plus the rear overhang behind it. An earlier version used the
     * body-centre convention and put the box 2.75 m too far forward. */
    ok(near(k[0][0], 0.880, 1e-9), "nose at +front_overhang (%.4f)", k[0][0]);
    ok(near(k[2][0], -3.855, 1e-9),
       "tail at -(wheelbase+rear_overhang) (%.4f)", k[2][0]);
    ok(near(k[0][0] - k[2][0], c.car_length, 1e-9),
       "box length equals car length (%.4f)", k[0][0] - k[2][0]);
    ok(near(k[0][1], 0.915, 1e-9) && near(k[1][1], -0.915, 1e-9),
       "half width both sides");

    /* Rotated 90 deg: the nose should point along +Y. */
    ref_body_corners(&c, 0.0, 0.0, M_PI / 2.0, k);
    ok(near(k[0][0], -0.915, 1e-9) && near(k[0][1], 0.880, 1e-9),
       "yaw=90deg puts the nose on +Y (%.3f, %.3f)", k[0][0], k[0][1]);

    ok(near(ref_point_segment_distance(0, 1, -1, 0, 1, 0), 1.0, 1e-12),
       "point-segment distance, perpendicular");
    ok(near(ref_point_segment_distance(3, 0, -1, 0, 1, 0), 2.0, 1e-12),
       "point-segment distance clamps past the end");
    ok(ref_segments_intersect(-1, 0, 1, 0, 0, -1, 0, 1) == 1, "crossing segments");
    ok(ref_segments_intersect(-1, 0, 1, 0, 0, 1, 0, 2) == 0, "non-crossing segments");
    ok(ref_segments_intersect(-1, 0, 1, 0, -1, 1, 1, 1) == 0, "parallel segments");
}

static void test_cones(void)
{
    printf("\ncone strikes\n");

    RefCourse c; tiny_course(&c);
    c.n_cones = 2;
    c.cones[0] = (RefCone){ 10.0,  0.0, 0.15, 0, 1 };   /* dead ahead   */
    c.cones[1] = (RefCone){ 20.0,  5.0, 0.15, 1, 1 };   /* well off line */

    RefState s; ref_init(&s, &c);
    /* Drive straight down y=0 from x=0 to x=40. The nose is at x+0.88, so the
     * body reaches cone 0 well before the reference point does. */
    for (int i = 0; i <= 4000; ++i)
        feed(&s, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);

    ok(count_of(&s, REF_CONE_STRIKE) == 1,
       "hit the cone on the line, missed the one 5 m off it (%d strikes)",
       count_of(&s, REF_CONE_STRIKE));

    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(&s, ev, REF_MAX_EVENTS);
    int32_t hit_id = -1; double hit_t = -1.0;
    for (int32_t i = 0; i < n; ++i)
        if (ev[i].type == REF_CONE_STRIKE) { hit_id = ev[i].object_id; hit_t = ev[i].t; }
    ok(hit_id == 0, "the struck cone is id 0 (got %d)", hit_id);
    /* Contact when the nose reaches 10 - 0.15 = 9.85, i.e. reference point at
     * 8.97, i.e. t = 0.897 s at 10 m/s. */
    ok(near(hit_t, 0.897, 0.005),
       "struck at t=%.3f s, when the NOSE reached it, not the origin", hit_t);

    /* LATCHING: a cone must fire once, not once per millisecond while the car
     * slides past it. Without this a single cone produces hundreds of events. */
    ok(count_of(&s, REF_CONE_STRIKE) == 1, "one event per cone, not one per step");
}

static void test_lines(void)
{
    printf("\nboundary lines\n");

    RefCourse c; tiny_course(&c);
    c.n_lines = 1;
    c.lines[0] = (RefLine){ 10.0, -5.0, 10.0, 5.0, 0, 0 };  /* across the path */

    RefState s; ref_init(&s, &c);
    for (int i = 0; i <= 3000; ++i)
        feed(&s, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);
    ok(count_of(&s, REF_LINE_CROSS) == 1, "crossed the line once (%d)",
       count_of(&s, REF_LINE_CROSS));

    /* A line the car never reaches must not fire. */
    RefCourse c2; tiny_course(&c2);
    c2.n_lines = 1;
    c2.lines[0] = (RefLine){ 10.0, 5.0, 20.0, 5.0, 0, 0 };  /* parallel, 5 m off */
    RefState s2; ref_init(&s2, &c2);
    for (int i = 0; i <= 3000; ++i)
        feed(&s2, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);
    ok(count_of(&s2, REF_LINE_CROSS) == 0, "a line 5 m to the side does not fire");
}

static void test_gates(void)
{
    printf("\ngates\n");

    RefCourse c; tiny_course(&c);
    c.n_gates = 3;
    c.gates[0] = (RefGate){ 10.0, -3.0, 10.0, 3.0, 0, 0 };
    c.gates[1] = (RefGate){ 20.0, -3.0, 20.0, 3.0, 1, 1 };
    c.gates[2] = (RefGate){ 30.0, -3.0, 30.0, 3.0, 2, 2 };

    RefState s; ref_init(&s, &c);
    for (int i = 0; i <= 4000; ++i)
        feed(&s, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);

    ok(count_of(&s, REF_GATE_PASS) == 3, "passed all three in order (%d)",
       count_of(&s, REF_GATE_PASS));
    ok(count_of(&s, REF_GATE_SKIP) == 0, "no out-of-order gates");
    ok(s.next_gate == 3, "sequence counter reached 3 (%d)", s.next_gate);

    /* A gate whose SEGMENT the car misses must not fire, even though the car
     * does cross the infinite line the segment lies on. This is the failure a
     * bare sign-of-side test would produce. */
    RefCourse c2; tiny_course(&c2);
    c2.n_gates = 1;
    c2.gates[0] = (RefGate){ 10.0, 20.0, 10.0, 25.0, 0, 0 };  /* far off to +Y */
    RefState s2; ref_init(&s2, &c2);
    for (int i = 0; i <= 3000; ++i)
        feed(&s2, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);
    ok(count_of(&s2, REF_GATE_PASS) == 0,
       "crossing the gate's LINE but not its SEGMENT does not count");

    /* Out of order: start past gate 0 so gate 1 is met first. */
    RefState s3; ref_init(&s3, &c);
    for (int i = 0; i <= 2000; ++i)
        feed(&s3, i * 0.001, (uint64_t)i, 15.0 + i * 0.01, 0.0, 0.0);
    ok(count_of(&s3, REF_GATE_SKIP) >= 1, "a gate met out of sequence is reported");
}

static void test_stall(void)
{
    printf("\nstall\n");

    RefCourse c; tiny_course(&c);
    RefState s; ref_init(&s, &c);

    /* In gear, clutch engaged, stopped, rpm on the floor. */
    for (int i = 0; i <= 3000; ++i) {
        RefVehicleState v = vs(i * 0.001, (uint64_t)i, 0.0, 0.0, 0.0);
        v.engine_rpm = 100.0; v.gear = 1; v.clutch = 0.0; v.speed = 0.0;
        ref_update(&s, &v);
    }
    ok(count_of(&s, REF_STALL) == 1, "stalled once (%d)", count_of(&s, REF_STALL));

    /* A launch DIPS through low rpm briefly. That must not be a stall, or
     * every hill start fails on the way up. */
    RefState s2; ref_init(&s2, &c);
    for (int i = 0; i <= 3000; ++i) {
        RefVehicleState v = vs(i * 0.001, (uint64_t)i, 0.0, 0.0, 0.0);
        v.engine_rpm = (i > 500 && i < 900) ? 300.0 : 1200.0;   /* 0.4 s dip */
        ref_update(&s2, &v);
    }
    ok(count_of(&s2, REF_STALL) == 0,
       "a 0.4 s dip below the floor is a launch, not a stall");

    /* THE REGRESSION THAT MATTERS. Measured on the real model: every clutch
     * engagement drags the engine to 330-380 rpm and holds it there for over a
     * second, on runs that then pull away cleanly. Without the speed conjunct
     * this fires on every launch. */
    RefState s4; ref_init(&s4, &c);
    for (int i = 0; i <= 6000; ++i) {
        RefVehicleState v = vs(i * 0.001, (uint64_t)i, 0.0, 0.0, 0.0);
        v.engine_rpm = 360.0;              /* bogged, for six whole seconds */
        v.gear = 1; v.clutch = 0.0;
        v.speed = 0.5 + i * 0.001;          /* but accelerating away */
        ref_update(&s4, &v);
    }
    ok(count_of(&s4, REF_STALL) == 0,
       "low rpm while ACCELERATING is a bogged launch, not a stall");

    /* Neutral is expressed by the shim as clutch = 1.0, so a stopped engine
     * with the clutch down is a parked car, not a stall. */
    RefState s3; ref_init(&s3, &c);
    for (int i = 0; i <= 3000; ++i) {
        RefVehicleState v = vs(i * 0.001, (uint64_t)i, 0.0, 0.0, 0.0);
        v.engine_rpm = 0.0; v.gear = 0; v.clutch = 1.0;
        ref_update(&s3, &v);
    }
    ok(count_of(&s3, REF_STALL) == 0, "engine off in neutral is not a stall");
}

static void test_sample_gap(void)
{
    printf("\nsample continuity\n");

    RefCourse c; tiny_course(&c);
    RefState s; ref_init(&s, &c);
    for (int i = 0; i <= 100; ++i)
        feed(&s, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);
    ok(count_of(&s, REF_SAMPLE_GAP) == 0, "contiguous samples: no gap reported");

    /* Skip 20 steps, as a game thread that let the ring wrap would. */
    feed(&s, 0.121, 121, 1.21, 0.0, 0.0);
    ok(count_of(&s, REF_SAMPLE_GAP) == 1,
       "a 20-step hole IS reported -- the verdict is not silently scored over");

    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(&s, ev, REF_MAX_EVENTS);
    double missed = -1;
    for (int32_t i = 0; i < n; ++i)
        if (ev[i].type == REF_SAMPLE_GAP) missed = ev[i].detail;
    ok(near(missed, 20.0, 1e-9), "and it says how many were missed (%.0f)", missed);
}

/* ------------------------------------------------------------------ */
/* PARKING                                                            */
/* ------------------------------------------------------------------ */

/* The real bay's shape at a convenient place: 2.5 x 5.0 m, mouth on y = 4,
 * reversed into, so a parked car faces -Y. Built in memory, bypassing the
 * loader that would normalise it, so it is wound counter-clockwise by hand;
 * test_zone_loader checks the normalisation separately. */
static void tiny_bay(RefCourse* c)
{
    tiny_course(c);
    RefZone* z = &c->zones[0];
    const double px[4] = { 10.0, 12.5, 12.5, 10.0 };
    const double py[4] = {  4.0,  4.0,  9.0,  9.0 };
    for (int i = 0; i < 4; ++i) { z->x[i] = px[i]; z->y[i] = py[i]; }
    z->n = 4; z->id = 7; z->exercise = 3; z->heading = -M_PI / 2.0;
    c->n_zones = 1;
}

/* Front-axle reference pose of a car centred in tiny_bay, square to it:
 * nose toward the mouth, 0.1325 m of the 0.265 m spare depth at each end. */
static const double BAY_X = 11.25, BAY_Y = 4.0 + 0.1325 + 0.880, BAY_YAW = -M_PI / 2.0;

/* The front-axle reference that puts the BODY CENTRE at the bay centre for
 * any yaw. Rotating about the reference point instead would swing the tail
 * (L + rear overhang) sin(yaw) sideways, and a test meant to isolate the
 * angle would also be testing a corner that left the bay. */
static void centred(double yaw, double* x, double* y)
{
    const double back = (2.750 + 1.105 - 0.880) / 2.0;   /* centre -> front axle */
    *x = 11.25 + back * cos(yaw);
    *y = 6.50 + back * sin(yaw);
}

static void feed_v(RefState* s, double t, uint64_t step,
                   double x, double y, double yaw, double speed)
{
    RefVehicleState v = vs(t, step, x, y, yaw);
    v.speed = speed;
    v.wheel_speed = speed;          /* rolling: the wheels carry the car */
    ref_update(s, &v);
}

/* Drives in along the bay axis at 0.5 m/s, stops at (x, y, yaw), then stays
 * stopped for `hold` s. Returns the step counter so callers can continue. */
static uint64_t park_at(RefState* s, uint64_t step, double x, double y, double yaw,
                        double hold)
{
    const double dt = 0.001;
    for (int i = 200; i > 0; --i, ++step)       /* 0.2 s of approach, reversing */
        feed_v(s, step * dt, step, x, y - i * 0.0005, yaw, -0.5);
    const long n = (long)(hold / dt);
    for (long i = 0; i <= n; ++i, ++step)
        feed_v(s, step * dt, step, x, y, yaw, 0.0);
    return step;
}

static void test_zone_geometry(void)
{
    printf("\nparking zones: geometry\n");

    RefCourse c; tiny_bay(&c);
    const RefZone* z = &c.zones[0];
    ok(ref_zone_contains(z, 11.25, 6.5) && !ref_zone_contains(z, 11.25, 3.9) &&
       !ref_zone_contains(z, 12.6, 6.5), "zone contains its interior and nothing else");

    const double clear = ref_zone_clearance(&c, z, BAY_X, BAY_Y, BAY_YAW);
    ok(near(clear, 0.1325, 1e-9),
       "a car centred in a 5.0 m bay has (5.0 - 4.735)/2 = 0.1325 m clearance (%.6f)", clear);
    ok(near(ref_zone_clearance(&c, z, BAY_X - 0.2, BAY_Y, BAY_YAW), 0.1325, 1e-9),
       "0.2 m sideways still binds on depth, not width (0.335 - 0.2 > 0.1325)");
    ok(near(ref_zone_clearance(&c, z, BAY_X, BAY_Y - 0.3, BAY_YAW), 0.1325 - 0.3, 1e-9),
       "0.3 m short: the nose is 0.1675 m over the mouth (%.4f)",
       ref_zone_clearance(&c, z, BAY_X, BAY_Y - 0.3, BAY_YAW));
}

static void test_zone_loader(void)
{
    printf("\nparking zones: loader\n");

    char path[] = "/tmp/ref_zone_XXXXXX";
    const int fd = mkstemp(path);
    FILE* f = fdopen(fd, "w");
    fprintf(f, "CAR 4.735 1.83 0.88 1.105 2.75 0.3331\n");
    /* CLOCKWISE on purpose: the loader must flip it, or every clearance
     * would come out with the wrong sign. */
    fprintf(f, "ZONE 3 2 0.0 4  0 0  0 2  5 2  5 0   # clockwise\n");
    fclose(f);
    RefCourse c; char err[256] = {0};
    ok(ref_load_course(&c, path, err, sizeof err) == 0 && c.n_zones == 1,
       "ZONE record loads (%s)", err[0] ? err : "ok");
    ok(ref_zone_contains(&c.zones[0], 2.5, 1.0) && ref_zone_contains(&c.zones[0], 0.01, 0.01),
       "a clockwise zone is normalised: its interior is inside");
    ok(c.zones[0].id == 3 && c.zones[0].exercise == 2 && c.zones[0].heading == 0.0,
       "id, exercise and heading read");

    struct { const char* rec; const char* why; } bad[] = {
        { "ZONE 0 2 0.0 4  0 0  5 0  5 2\n",          "fewer vertices than n says" },
        { "ZONE 0 2 0.0 3  0 0  5 0  5 2  0 2\n",     "more numbers than n says" },
        { "ZONE 0 2 0.0 4  0 0  5 0  1 1  0 5\n",     "a non-convex polygon" },
        { "ZONE 0 2 0.0 3  0 0  1 1  2 2\n",          "a degenerate (collinear) polygon" },
        { "ZONE 0 2 0.0 9  0 0 1 0 2 0 3 0 4 1 3 2 2 2 1 2 0 1\n", "more vertices than REF_MAX_ZONE_VERTS" },
    };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
        f = fopen(path, "w");
        fprintf(f, "CAR 4.735 1.83 0.88 1.105 2.75 0.3331\n%s", bad[i].rec);
        fclose(f);
        err[0] = '\0';
        ok(ref_load_course(&c, path, err, sizeof err) != 0 && strstr(err, ":2:"),
           "refused: %s (%s)", bad[i].why, err);
    }
    unlink(path);
}

static void test_parking(void)
{
    printf("\nparking: the rule\n");
    RefCourse c; tiny_bay(&c);
    RefState s;

    /* 1. A clean park: exactly one PARK_OK, for this zone, stamped at the STOP. */
    ref_init(&s, &c);
    park_at(&s, 0, BAY_X, BAY_Y, BAY_YAW, 1.5);
    const RefEvent* e = last_of(&s, REF_PARK_OK);
    ok(count_of(&s, REF_PARK_OK) == 1 && count_of(&s, REF_PARK_OUTSIDE) == 0 &&
       count_of(&s, REF_PARK_ANGLE) == 0, "centred and square: exactly one PARK_OK");
    ok(e && e->object_id == 7 && near(e->detail, 0.1325, 1e-9),
       "reports the zone id and the worst-corner clearance (%.4f)", e ? e->detail : -1);
    ok(e && near(e->t, 0.200, 1e-9),
       "stamped with the moment of stopping, not the end of the dwell (t=%.3f)", e ? e->t : -1);

    /* 2. The dwell: a pause shorter than REF_PARK_DWELL is not a park. */
    ref_init(&s, &c);
    uint64_t k = park_at(&s, 0, BAY_X, BAY_Y, BAY_YAW, 0.6);
    for (int i = 0; i < 300; ++i, ++k)          /* moves off again */
        feed_v(&s, k * 0.001, k, BAY_X, BAY_Y - i * 0.0005, BAY_YAW, 0.5);
    ok(count_of(&s, REF_PARK_OK) + count_of(&s, REF_PARK_OUTSIDE) == 0,
       "a 0.6 s pause to change gear is not judged");

    /* 3. THE DESIGN NOTE: judge the pose AT the stop. After stopping cleanly
     *    the car creeps (below the move threshold, as an unbraked
     *    force-element tyre does) 0.4 m toward the mouth -- where it would be
     *    OUTSIDE. The verdict must still be the stop pose's. */
    ref_init(&s, &c);
    k = park_at(&s, 0, BAY_X, BAY_Y, BAY_YAW, 0.0);
    for (int i = 1; i <= 2000; ++i, ++k)
        feed_v(&s, k * 0.001, k, BAY_X, BAY_Y - i * 0.0002, BAY_YAW, -0.10);
    e = last_of(&s, REF_PARK_OK);
    ok(count_of(&s, REF_PARK_OK) == 1 && count_of(&s, REF_PARK_OUTSIDE) == 0 &&
       e && near(e->detail, 0.1325, 1e-9),
       "a car that drifts 0.4 m after stopping is judged where it STOPPED (%.4f)",
       e ? e->detail : -1);

    /* 4. Short: the nose 0.1675 m over the mouth. */
    ref_init(&s, &c);
    park_at(&s, 0, BAY_X, BAY_Y - 0.3, BAY_YAW, 1.5);
    e = last_of(&s, REF_PARK_OUTSIDE);
    ok(count_of(&s, REF_PARK_OK) == 0 && e && near(e->detail, -0.1675, 1e-9),
       "0.3 m short: PARK_OUTSIDE, detail -0.1675 m (%.4f)", e ? e->detail : 0);

    /* 5. On the paint: inside the zone, but within REF_PARK_CLEARANCE. */
    ref_init(&s, &c);
    park_at(&s, 0, BAY_X, BAY_Y - 0.1, BAY_YAW, 1.5);
    e = last_of(&s, REF_PARK_OUTSIDE);
    ok(count_of(&s, REF_PARK_OK) == 0 && e && near(e->detail, 0.0325, 1e-9),
       "a corner 3.25 cm from the line is ON the paint: PARK_OUTSIDE (%.4f)",
       e ? e->detail : 0);

    /* 6. Angle just over tolerance, body still inside: ONLY PARK_ANGLE. 5.5
     *    degrees in a 2.5 x 5.0 bay still fits: (5.0 - 4.888)/2 = 0.056 m. */
    ref_init(&s, &c);
    double ax, ay;
    centred(BAY_YAW + 5.5 * M_PI / 180.0, &ax, &ay);
    park_at(&s, 0, ax, ay, BAY_YAW + 5.5 * M_PI / 180.0, 1.5);
    e = last_of(&s, REF_PARK_ANGLE);
    ok(count_of(&s, REF_PARK_ANGLE) == 1 && count_of(&s, REF_PARK_OUTSIDE) == 0 &&
       count_of(&s, REF_PARK_OK) == 0 && e && near(e->detail, 5.5, 1e-6),
       "5.5 deg off, fully inside: PARK_ANGLE alone, detail +5.5 deg (%.3f)",
       e ? e->detail : 0);

    /* 7. Nose-in: the space is right, the manoeuvre is not. */
    ref_init(&s, &c);
    park_at(&s, 0, BAY_X, 9.0 - 0.1325 - 0.880, M_PI / 2.0, 1.5);
    e = last_of(&s, REF_PARK_ANGLE);
    ok(count_of(&s, REF_PARK_OK) == 0 && e && near(fabs(e->detail), 180.0, 1e-6),
       "driven in NOSE first: PARK_ANGLE at 180 deg -- the bay is reversed into");

    /* 8. A stop outside every zone is nobody's business. */
    ref_init(&s, &c);
    park_at(&s, 0, 30.0, 0.0, 0.0, 1.5);
    ok(count_of(&s, REF_PARK_OK) + count_of(&s, REF_PARK_OUTSIDE) +
       count_of(&s, REF_PARK_ANGLE) == 0, "stopping on the road judges nothing");

    /* 9. A stop BESIDE the zone, overlapping it but centred outside: not a
     *    park either -- a car waiting in the aisle is not in the space. */
    ref_init(&s, &c);
    park_at(&s, 0, 14.5, 5.0, 0.0, 1.5);
    ok(count_of(&s, REF_PARK_OUTSIDE) == 0, "overlapping the zone with the centre outside: not judged");

    /* 10. The car never moved: sitting at the spawn is not parking. */
    ref_init(&s, &c);
    for (int i = 0; i < 3000; ++i) feed_v(&s, i * 0.001, (uint64_t)i, BAY_X, BAY_Y, BAY_YAW, 0.0);
    ok(count_of(&s, REF_PARK_OK) == 0, "a car that never moved is never judged");

    /* 11. Two attempts, two verdicts: out short, then in properly. */
    ref_init(&s, &c);
    k = park_at(&s, 0, BAY_X, BAY_Y - 0.3, BAY_YAW, 1.5);
    park_at(&s, k, BAY_X, BAY_Y, BAY_YAW, 1.5);
    ok(count_of(&s, REF_PARK_OUTSIDE) == 1 && count_of(&s, REF_PARK_OK) == 1,
       "a second attempt is judged again: one OUTSIDE, then one OK");
}

/* ------------------------------------------------------------------ */
/* THE VERDICT                                                        */
/* ------------------------------------------------------------------ */

/* Three exercises, one of each kind, along +X:
 *   1 GATED  entry gate 0 (x=10), exit gate 1 (x=20), one cone at (15, 2)
 *   2 PARK   entry gate 2 (x=30), zone 0 = a tiny_bay-shaped bay at x 40..42.5
 *   3 HILL   entry gate 3 (x=60), progress gate 4 (x=70), exit gate 5 (x=80) */
static void verdict_course(RefCourse* c)
{
    tiny_course(c);
    const RefGate g[6] = {
        { 10.0, -3.0, 10.0, 3.0, 0, 0, 1 }, { 20.0, -3.0, 20.0, 3.0, 1, 1, 1 },
        { 30.0, -3.0, 30.0, 3.0, 2, 2, 2 },
        { 60.0, -3.0, 60.0, 3.0, 3, 3, 3 }, { 70.0, -3.0, 70.0, 3.0, 4, 4, 3 },
        { 80.0, -3.0, 80.0, 3.0, 5, 5, 3 },
    };
    memcpy(c->gates, g, sizeof g); c->n_gates = 6;
    c->cones[0] = (RefCone){ 15.0, 2.0, 0.15, 0, 1 }; c->n_cones = 1;
    RefZone* z = &c->zones[0];
    const double px[4] = { 40.0, 42.5, 42.5, 40.0 }, py[4] = { 4.0, 4.0, 9.0, 9.0 };
    for (int i = 0; i < 4; ++i) { z->x[i] = px[i]; z->y[i] = py[i]; }
    z->n = 4; z->id = 0; z->exercise = 2; z->heading = -M_PI / 2.0;
    c->n_zones = 1;
    const RefExercise x[3] = {
        { 1, REF_EXK_GATED, 0, 1, -1 }, { 2, REF_EXK_PARK, 2, -1, 0 },
        { 3, REF_EXK_HILL, 3, 5, -1 },
    };
    memcpy(c->exercises, x, sizeof x); c->n_exercises = 3;
}

/* The park pose centred in verdict_course's bay. */
static const double VB_X = 41.25, VB_Y = 4.0 + 0.1325 + 0.880;

static uint64_t drive_x(RefState* s, uint64_t step, double x0, double x1, double y, double v)
{
    for (double x = x0; x < x1; x += v * 0.001, ++step)
        feed_v(s, step * 0.001, step, x, y, 0.0, v);
    return step;
}

static uint64_t hold_still(RefState* s, uint64_t step, double x, double y, double yaw, double secs)
{
    for (long i = 0; i < (long)(secs / 0.001); ++i, ++step)
        feed_v(s, step * 0.001, step, x, y, yaw, 0.0);
    return step;
}

static uint64_t stall_at(RefState* s, uint64_t step, double x, double y, double secs)
{
    for (long i = 0; i < (long)(secs / 0.001); ++i, ++step) {
        RefVehicleState v = vs(step * 0.001, step, x, y, 0.0);
        v.speed = 0.2; v.engine_rpm = 250.0; v.gear = 1; v.clutch = 0.0;
        ref_update(s, &v);
    }
    return step;
}

/* The clean course, as segments, so the tests can break exactly one part.
 * `park` 0 = no parking (drive straight past), otherwise the number of short
 * stops before the good one; `hill_stop` whether it stops on the hill. */
static uint64_t run_slalom(RefState* s, uint64_t k, int hit_cone)
{
    k = drive_x(s, k, 0.0, 12.0, 0.0, 5.0);
    k = drive_x(s, k, 12.0, 18.0, hit_cone ? 1.2 : 0.0, 5.0);
    return drive_x(s, k, 18.0, 38.0, 0.0, 5.0);
}

static uint64_t run_park(RefState* s, uint64_t k, int short_stops, int good)
{
    for (int i = 0; i < short_stops; ++i)
        k = park_at(s, k, VB_X, VB_Y - 0.3, -M_PI / 2.0, 1.5);   /* nose out: OUTSIDE */
    if (good) k = park_at(s, k, VB_X, VB_Y, -M_PI / 2.0, 1.5);
    return k;
}

static uint64_t run_hill(RefState* s, uint64_t k, int stop)
{
    k = drive_x(s, k, 45.0, 72.0, 0.0, 5.0);
    if (stop) k = hold_still(s, k, 72.0, 0.0, 0.0, 1.5);
    return drive_x(s, k, 72.0, 85.0, 0.0, 5.0);
}

static void clean_run(RefState* s)
{
    uint64_t k = run_slalom(s, 0, 0);
    k = run_park(s, k, 0, 1);
    run_hill(s, k, 1);
}

static void test_verdict(void)
{
    printf("\nthe verdict\n");
    RefCourse c; verdict_course(&c);
    RefState s; RefVerdict v;

    /* 1. Clean: every exercise passes, in order, and the test passes. */
    ref_init(&s, &c);
    clean_run(&s);
    ref_verdict(&s, &v);
    ok(v.result == REF_RESULT_PASS && v.passed == 3 && v.failed == 0 && v.unfinished == 0,
       "a clean run PASSES: %d/%d exercises (%s)", v.passed, v.total, ref_result_name(v.result));
    ok(count_of(&s, REF_EX_START) == 3 && count_of(&s, REF_EX_PASS) == 3 &&
       count_of(&s, REF_EX_FAIL) == 0, "three starts, three passes, no fails");
    ok(s.running == -1, "nothing left running at the end");

    /* 2. One cone fails ITS exercise and only that one. */
    ref_init(&s, &c);
    uint64_t k = run_slalom(&s, 0, 1);
    k = run_park(&s, k, 0, 1);
    run_hill(&s, k, 1);
    ref_verdict(&s, &v);
    ok(s.ex[0].state == REF_EX_FAILED && s.ex[0].fail_reason == REF_FAIL_CONE &&
       s.ex[1].state == REF_EX_PASSED && s.ex[2].state == REF_EX_PASSED &&
       v.result == REF_RESULT_FAIL && v.failed == 1 && v.passed == 2,
       "a struck cone fails the slalom alone; the test FAILS 2/3");

    /* 3. Parking attempts: two short stops then a good one still passes, with
     *    the attempts counted; three short stops fail it. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 2, 1);
    ok(s.ex[1].state == REF_EX_PASSED && s.ex[1].park_attempts == 2,
       "two failed attempts then PARK_OK: passed, 2 attempts recorded (%d)",
       s.ex[1].park_attempts);
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, REF_PARK_MAX_ATTEMPTS, 0);
    ok(s.ex[1].state == REF_EX_FAILED && s.ex[1].fail_reason == REF_FAIL_PARK_ATTEMPTS,
       "%d failed attempts: FAILED (%s)", REF_PARK_MAX_ATTEMPTS,
       ref_fail_reason_name(s.ex[1].fail_reason));

    /* 4. One stop that fails BOTH OUTSIDE and ANGLE is one attempt, not two. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = park_at(&s, k, VB_X, VB_Y - 0.3, -M_PI / 2.0 + 0.2, 1.5);
    ok(count_of(&s, REF_PARK_OUTSIDE) == 1 && count_of(&s, REF_PARK_ANGLE) == 1 &&
       s.ex[1].park_attempts == 1, "OUTSIDE and ANGLE from one stop count once");

    /* 5. The hill: through the exit without a stop fails it; with one passes. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 0, 1);
    run_hill(&s, k, 0);
    ok(s.ex[2].state == REF_EX_FAILED && s.ex[2].fail_reason == REF_FAIL_NO_STOP,
       "hill without a stop: FAILED (%s)", ref_fail_reason_name(s.ex[2].fail_reason));

    /* 6. Abandoned: entering the hill before parking fails the park. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    run_hill(&s, k, 1);
    ok(s.ex[1].state == REF_EX_FAILED && s.ex[1].fail_reason == REF_FAIL_ABANDONED &&
       s.ex[2].state == REF_EX_PASSED, "leaving the bay unparked for the hill: park "
       "ABANDONED, hill still judged on its own");

    /* 7. A missed gate is ONE fault, charged to its own exercise -- not the
     *    cascade the sequence used to produce. Start past gate 0. */
    ref_init(&s, &c);
    k = drive_x(&s, 0, 11.0, 38.0, 0.0, 5.0);
    k = run_park(&s, k, 0, 1);
    run_hill(&s, k, 1);
    ref_verdict(&s, &v);
    ok(count_of(&s, REF_GATE_MISS) == 1 && count_of(&s, REF_GATE_SKIP) == 1,
       "one gate bypassed: one GATE_MISS, one GATE_SKIP (%d, %d)",
       count_of(&s, REF_GATE_MISS), count_of(&s, REF_GATE_SKIP));
    ok(s.ex[0].state == REF_EX_FAILED && s.ex[0].fail_reason == REF_FAIL_GATE &&
       s.ex[1].state == REF_EX_PASSED && s.ex[2].state == REF_EX_PASSED,
       "the slalom fails for the missed gate; the gates after it are in sequence "
       "again and the later exercises pass");

    /* 8. Stalls: inside an exercise it fails that exercise; between exercises
     *    it is a general fault that fails the test. */
    ref_init(&s, &c);
    k = drive_x(&s, 0, 0.0, 32.0, 0.0, 5.0);
    k = stall_at(&s, k, 32.0, 0.0, 1.0);
    ok(s.ex[1].state == REF_EX_FAILED && s.ex[1].fail_reason == REF_FAIL_STALL,
       "a stall while parking fails the park (%s)", ref_fail_reason_name(s.ex[1].fail_reason));
    ref_init(&s, &c);
    k = drive_x(&s, 0, 0.0, 22.0, 0.0, 5.0);            /* slalom passed, nothing running */
    k = stall_at(&s, k, 22.0, 0.0, 1.0);
    ref_verdict(&s, &v);
    ok(s.running == -1 && v.general_faults == 1 && v.result == REF_RESULT_FAIL,
       "a stall between exercises is a general fault: the test FAILS");

    /* 9. Incomplete: nothing failed, but the run stopped in the middle. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    ref_verdict(&s, &v);
    ok(v.result == REF_RESULT_INCOMPLETE && v.passed == 1 && v.unfinished == 2,
       "stopping mid-test: INCOMPLETE, 1 passed, 2 unfinished");

    /* 10. Missed physics samples mark the verdict suspect -- without hiding it. */
    ref_init(&s, &c);
    k = drive_x(&s, 0, 0.0, 5.0, 0.0, 5.0);
    feed_v(&s, (k + 50) * 0.001, k + 50, 5.3, 0.0, 0.0, 5.0);
    ref_verdict(&s, &v);
    ok(v.suspect == 1, "a sample gap marks the verdict suspect");

    /* 11. A FULL event log must not change the result. Fill it with sample
     *     gaps first -- every other step, parked far off the course -- then
     *     drive the clean course with every one of its events dropped. */
    ref_init(&s, &c);
    k = 0;
    for (int i = 0; i < REF_MAX_EVENTS + 20; ++i, k += 2)
        feed_v(&s, k * 0.001, k, -50.0, 0.0, 0.0, 0.0);
    k = run_slalom(&s, k, 0);
    k = run_park(&s, k, 0, 1);
    run_hill(&s, k, 1);
    ref_verdict(&s, &v);
    ok(s.n_events == REF_MAX_EVENTS && count_of(&s, REF_EX_PASS) == 0 &&
       v.result == REF_RESULT_PASS && v.passed == 3 && v.suspect == 1,
       "log full, every exercise event dropped (%d): verdict still PASS 3/3, "
       "flagged suspect for the gaps", s.dropped_events);

    /* 12. Deterministic: the same trajectory twice gives a byte-identical
     *     verdict and per-exercise record. */
    RefState a, b; RefVerdict va, vb;
    ref_init(&a, &c); clean_run(&a); ref_verdict(&a, &va);
    ref_init(&b, &c); clean_run(&b); ref_verdict(&b, &vb);
    ok(memcmp(&va, &vb, sizeof va) == 0 && memcmp(a.ex, b.ex, sizeof a.ex) == 0,
       "two identical runs: byte-identical verdicts");

    /* 13. One gate that is both an exit and the next entry: the running
     *     exercise must FINISH before the next starts, or it would be failed
     *     as abandoned an instant before it passed. */
    RefCourse cs; tiny_course(&cs);
    cs.gates[0] = (RefGate){ 10.0, -3.0, 10.0, 3.0, 0, 0, 1 };
    cs.gates[1] = (RefGate){ 20.0, -3.0, 20.0, 3.0, 1, 1, 1 };   /* exit 1 AND entry 2 */
    cs.gates[2] = (RefGate){ 30.0, -3.0, 30.0, 3.0, 2, 2, 2 };
    cs.n_gates = 3;
    cs.exercises[0] = (RefExercise){ 1, REF_EXK_GATED, 0, 1, -1 };
    cs.exercises[1] = (RefExercise){ 2, REF_EXK_GATED, 1, 2, -1 };
    cs.n_exercises = 2;
    ref_init(&s, &cs);
    drive_x(&s, 0, 0.0, 35.0, 0.0, 5.0);
    ref_verdict(&s, &v);
    ok(s.ex[0].state == REF_EX_PASSED && s.ex[1].state == REF_EX_PASSED &&
       v.result == REF_RESULT_PASS,
       "a gate shared as exit and entry: the first passes, then the second starts");

    /* 14. A park counts only for an exercise ENTERED through its gate. A car
     *     that reaches the bay without crossing gate 2 parks perfectly -- the
     *     PARK_OK is reported -- but the exercise was never started. */
    ref_init(&s, &c);
    k = drive_x(&s, 0, 0.0, 25.0, 0.0, 5.0);            /* slalom done, gate 2 not crossed */
    /* Round the gate, not through it: a straight jump from (25,0) into the
     * bay crosses x = 30 at y = 1.5, inside the gate -- and the referee, whose
     * gate test is the travelled SEGMENT, rightly counts that as entering. */
    feed_v(&s, k * 0.001, k, 25.0, 10.0, 0.0, 5.0); ++k;
    feed_v(&s, k * 0.001, k, 41.25, 10.0, 0.0, 5.0); ++k;
    park_at(&s, k, VB_X, VB_Y, -M_PI / 2.0, 1.5);
    ok(count_of(&s, REF_PARK_OK) == 1 && s.ex[1].state == REF_EX_ARMED,
       "a perfect park without entering the exercise: PARK_OK reported, "
       "exercise still ARMED");
}

/* ------------------------------------------------------------------ */
/* THE HILL START: ROLLBACK (task 7)                                  */
/* ------------------------------------------------------------------ */

/* On the hill at (72, 0), facing +X up the slope: approach, stop, then travel
 * `back` metres BACKWARDS at 0.3 m/s (speed negative), hold, drive off. The
 * referee sees only the horizontal pose and the signed speed; the distance it
 * judges is the speed integrated, so the pose follows the speed here. */
static uint64_t hill_with_rollback(RefState* s, uint64_t k, double back)
{
    k = drive_x(s, k, 45.0, 72.0, 0.0, 5.0);
    k = hold_still(s, k, 72.0, 0.0, 0.0, 0.3);            /* stopped ...        */
    double x = 72.0;
    for (double d = 0.0; d < back; d += 0.3 * 0.001, ++k) {
        x -= 0.3 * 0.001;
        feed_v(s, k * 0.001, k, x, 0.0, 0.0, -0.3);        /* ... rolls back     */
    }
    k = hold_still(s, k, x, 0.0, 0.0, 1.5);                /* held: the dwell    */
    return drive_x(s, k, x, 85.0, 0.0, 5.0);
}

static void test_rollback(void)
{
    printf("\nthe hill start: rollback\n");
    RefCourse c; verdict_course(&c);
    RefState s;

    /* 1. A clean hill start: stopped, held, drove off. No event, nothing
     *    recorded against it. */
    ref_init(&s, &c);
    uint64_t k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 0, 1);
    run_hill(&s, k, 1);
    ok(count_of(&s, REF_ROLLBACK) == 0 && s.ex[2].state == REF_EX_PASSED &&
       s.ex[2].rollback > -1e-9,
       "a clean hill start: no ROLLBACK, hill PASSED, rollback recorded %.4f m",
       s.ex[2].rollback);

    /* 2. Rolled back 0.10 m after stopping: ROLLBACK fires as the floor is
     *    crossed, once, and fails the hill -- the regulation allows none. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 0, 1);
    hill_with_rollback(&s, k, 0.10);
    const RefEvent* e = last_of(&s, REF_ROLLBACK);
    ok(count_of(&s, REF_ROLLBACK) == 1 && e && e->detail <= -REF_ROLLBACK_FLOOR &&
       e->detail > -REF_ROLLBACK_FLOOR - 0.001,
       "rolled back 0.10 m: ONE ROLLBACK, fired as it crossed the floor (%.4f m)",
       e ? e->detail : 0.0);
    ok(s.ex[2].state == REF_EX_FAILED && s.ex[2].fail_reason == REF_FAIL_ROLLBACK &&
       near(s.ex[2].rollback, -0.10, 0.002),
       "... the hill FAILED (%s), worst rollback recorded %.4f m",
       ref_fail_reason_name(s.ex[2].fail_reason), s.ex[2].rollback);

    /* 3. Under the floor: 0.02 m is the model's own noise -- tyre pre-slide
     *    is 1-4 mm -- not a rollback. Recorded, not faulted. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 0, 1);
    hill_with_rollback(&s, k, 0.02);
    ok(count_of(&s, REF_ROLLBACK) == 0 && s.ex[2].state == REF_EX_PASSED &&
       near(s.ex[2].rollback, -0.02, 0.002),
       "0.02 m back, under the %.2f m floor: no event, hill PASSED, recorded %.4f m",
       REF_ROLLBACK_FLOOR, s.ex[2].rollback);

    /* 4. Reversing is the manoeuvre in a parking exercise, not a rollback:
     *    only a HILL exercise watches. run_park reverses into the bay and
     *    stops, several times. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 2, 1);
    ok(count_of(&s, REF_ROLLBACK) == 0 && s.ex[1].rollback > -1e-9,
       "reversing into the bay after stops: no ROLLBACK outside the hill");

    /* 5. Travel is measured from the STOP, along the road: a car that stopped,
     *    drove off forward 0.3 m and only then rolled back 0.10 m has rolled
     *    back from its NEW stop -- it must pass through zero speed to reverse,
     *    which is a stop -- and is caught. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 0, 1);
    k = drive_x(&s, k, 45.0, 72.0, 0.0, 5.0);
    k = hold_still(&s, k, 72.0, 0.0, 0.0, 1.5);
    k = drive_x(&s, k, 72.0, 72.3, 0.0, 0.3);              /* crept forward      */
    k = hold_still(&s, k, 72.3, 0.0, 0.0, 0.2);            /* stops again        */
    double x = 72.3;
    for (double d = 0.0; d < 0.10; d += 0.3 * 0.001, ++k) {
        x -= 0.3 * 0.001;
        feed_v(&s, k * 0.001, k, x, 0.0, 0.0, -0.3);
    }
    k = hold_still(&s, k, x, 0.0, 0.0, 1.5);
    drive_x(&s, k, x, 85.0, 0.0, 5.0);
    ok(count_of(&s, REF_ROLLBACK) == 1 && s.ex[2].fail_reason == REF_FAIL_ROLLBACK,
       "crept forward, stopped, then rolled back 0.10 m: caught from the new stop");

    /* 6. THE SETTLE IS NOT A ROLLBACK. Braked firmly to a stop on 15 degrees
     *    the BODY settles back 6-8 cm on the tyres' compliance with the
     *    wheels locked (`rollback`, task 7). The body moves; the wheels do
     *    not; the car has not rolled. */
    ref_init(&s, &c);
    k = run_slalom(&s, 0, 0);
    k = run_park(&s, k, 0, 1);
    k = drive_x(&s, k, 45.0, 72.0, 0.0, 5.0);
    k = hold_still(&s, k, 72.0, 0.0, 0.0, 0.1);
    x = 72.0;
    for (double d = 0.0; d < 0.08; d += 0.3 * 0.001, ++k) {
        x -= 0.3 * 0.001;
        RefVehicleState v = vs(k * 0.001, k, x, 0.0, 0.0);
        v.speed = -0.3;                                    /* the body ... */
        v.wheel_speed = 0.0;                               /* ... not the wheels */
        ref_update(&s, &v);
    }
    k = hold_still(&s, k, x, 0.0, 0.0, 1.5);
    drive_x(&s, k, x, 85.0, 0.0, 5.0);
    ok(count_of(&s, REF_ROLLBACK) == 0 && s.ex[2].state == REF_EX_PASSED,
       "the body settles back 0.08 m, wheels locked: no ROLLBACK, hill PASSED");

    /* 7. ref_wheel_speed: spin about the BODY's lateral axis, less the body's
     *    own rotation about it. v = 1.2 m/s, r = 0.3331. */
    const double r = 0.3331, om = 1.2 / r;
    {   const double q[4] = { 1, 0, 0, 0 }, wb[3] = { 0, 0, 0 };
        const double ws[2][3] = { { 0, om, 0 }, { 0, om, 0 } };
        ok(near(ref_wheel_speed(q, wb, ws, 2, r), 1.2, 1e-9),
           "wheel speed, level, heading +X: %.6f m/s", ref_wheel_speed(q, wb, ws, 2, r)); }
    {   const double h = M_PI / 4.0;                       /* yaw 90 deg: heading +Y */
        const double q[4] = { cos(h), 0, 0, sin(h) }, wb[3] = { 0, 0, 0 };
        const double ws[2][3] = { { -om, 0, 0 }, { -om, 0, 0 } };   /* lateral axis is -X */
        ok(near(ref_wheel_speed(q, wb, ws, 2, r), 1.2, 1e-9),
           "wheel speed, heading +Y: %.6f m/s (the lateral axis turns with the car)",
           ref_wheel_speed(q, wb, ws, 2, r)); }
    {   const double h = -15.0 * M_PI / 360.0;             /* pitched 15 deg nose up */
        const double q[4] = { cos(h), 0, sin(h), 0 }, wb[3] = { 0, 0.2, 0 };
        const double ws[2][3] = { { 0, 0.2, 0 }, { 0, 0.2, 0 } };   /* locked, body pitching */
        ok(near(ref_wheel_speed(q, wb, ws, 2, r), 0.0, 1e-9),
           "locked wheels on a pitching body: %.6f m/s (the pitch is not rolling)",
           ref_wheel_speed(q, wb, ws, 2, r)); }
    {   /* yaw 30 deg then pitch -15: lateral axis (-sin 30, cos 30, 0) */
        const double a = M_PI / 12.0, b = -15.0 * M_PI / 360.0;
        const double q[4] = { cos(a) * cos(b), -sin(a) * sin(b), cos(a) * sin(b), sin(a) * cos(b) };
        const double wb[3] = { 0, 0, 0 };
        const double ws[2][3] = { { -0.5 * om, 0.8660254037844386 * om, 0 },
                                  { -0.5 * om, 0.8660254037844386 * om, 0 } };
        ok(near(ref_wheel_speed(q, wb, ws, 2, r), 1.2, 1e-9),
           "yawed 30 deg AND pitched 15 deg: %.6f m/s", ref_wheel_speed(q, wb, ws, 2, r)); }
}

static void test_exercise_loader(void)
{
    printf("\nexercises: loader\n");
    char path[] = "/tmp/ref_ex_XXXXXX";
    const int fd = mkstemp(path);
    FILE* f = fdopen(fd, "w"); fclose(f);
    const char* head =
        "CAR 4.735 1.83 0.88 1.105 2.75 0.3331\n"
        "GATE 0 0 10 -3 10 3 1\nGATE 1 1 20 -3 20 3 1\n"
        "ZONE 0 2 0.0 4 0 0 5 0 5 2 0 2\n";
    struct { const char* rec; const char* why; } bad[] = {
        { "EXERCISE 1 0 9 1 -1\n", "entry gate does not exist" },
        { "EXERCISE 1 0 0 9 -1\n", "exit gate does not exist" },
        { "EXERCISE 1 1 0 -1 7\n", "zone does not exist" },
        { "EXERCISE 1 5 0 1 -1\n", "unknown kind" },
        { "EXERCISE 1 0 0 1 -1\nEXERCISE 1 0 0 1 -1\n", "duplicate id" },
    };
    RefCourse c; char err[256];
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; ++i) {
        f = fopen(path, "w"); fprintf(f, "%s%s", head, bad[i].rec); fclose(f);
        err[0] = '\0';
        ok(ref_load_course(&c, path, err, sizeof err) != 0 && strstr(err, bad[i].why),
           "refused: %s (%s)", bad[i].why, err);
    }
    f = fopen(path, "w");
    fprintf(f, "%sEXERCISE 1 0 0 1 -1\nEXERCISE 2 1 1 -1 0\n", head); fclose(f);
    ok(ref_load_course(&c, path, err, sizeof err) == 0 && c.n_exercises == 2 &&
       c.lines[0].exercise == 0 && c.gates[1].exercise == 1,
       "valid EXERCISE records load, and GATE carries its exercise");
    unlink(path);
}

static void test_real_course(void)
{
    printf("\nthe generated course\n");

    RefCourse c;
    char err[256] = {0};
    const int rc = ref_load_course(&c, "../course/simA_referee.txt", err, sizeof err);
    if (rc != 0) {
        ok(0, "load ../course/simA_referee.txt: %s", err);
        return;
    }
    ok(1, "loaded: %d cones, %d lines, %d gates",
       c.n_cones, c.n_lines, c.n_gates);
    ok(c.wheelbase > 2.0 && c.car_length > 4.0,
       "CAR record present (wheelbase %.3f, length %.3f)", c.wheelbase, c.car_length);
    ok(near(c.r_roll, 0.3331, 0.002), "CAR carries the rolling radius (%.4f m)", c.r_roll);
    {   /* A five-field CAR is a course generated before task 7: refused, not
         * guessed -- a missing radius would silently zero every wheel travel. */
        char tmpl[] = "/tmp/ref_car_XXXXXX";
        const int fd = mkstemp(tmpl);
        FILE* f = fdopen(fd, "w");
        fprintf(f, "CAR 4.735 1.83 0.88 1.105 2.75\n");
        fclose(f);
        RefCourse old; char err[256] = {0};
        const int rc = ref_load_course(&old, tmpl, err, sizeof err);
        unlink(tmpl);
        ok(rc != 0, "a five-field CAR (no rolling radius) is refused (%s)", err);
    }

    /* Drive the centreline at 10 m/s and see what the course says. The slalom
     * cones sit ON the centreline, so a car that drives straight through them
     * must be scored as hitting them -- that is the sanity check that the
     * course and the rules agree about where things are. */
    RefState s; ref_init(&s, &c);
    for (int i = 0; i <= 6000; ++i)
        feed(&s, i * 0.001, (uint64_t)i, i * 0.01, 0.0, 0.0);
    printf("       driving the centreline to x=60: %d cone strikes, %d gates\n",
           count_of(&s, REF_CONE_STRIKE), count_of(&s, REF_GATE_PASS));
    ok(count_of(&s, REF_CONE_STRIKE) >= 5,
       "a car driven straight through the slalom hits the cones");

    /* The parking spaces, as gen_course.py emits them. */
    ok(c.n_zones == 2, "two parking zones (%d)", c.n_zones);
    for (int32_t i = 0; i < c.n_zones; ++i) {
        const RefZone* z = &c.zones[i];
        double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
        for (int32_t k = 0; k < z->n; ++k) {
            x0 = fmin(x0, z->x[k]); x1 = fmax(x1, z->x[k]);
            y0 = fmin(y0, z->y[k]); y1 = fmax(y1, z->y[k]);
        }
        /* Centre a car in it at its heading: the rear-axle-to-body offset is
         * what makes "centred" non-trivial with a front-axle reference. */
        const double h = z->heading;
        const double back = (c.wheelbase + c.rear_overhang - c.front_overhang) / 2.0;
        const double xr = (x0 + x1) / 2.0 + back * cos(h);
        const double yr = (y0 + y1) / 2.0 + back * sin(h);
        RefState ps; ref_init(&ps, &c);
        uint64_t k = 0;
        for (int j = 200; j > 0; --j, ++k)
            feed_v(&ps, k * 0.001, k, xr - j * 0.0005 * cos(h), yr - j * 0.0005 * sin(h), h, 0.5);
        for (int j = 0; j <= 1500; ++j, ++k)
            feed_v(&ps, k * 0.001, k, xr, yr, h, 0.0);
        const RefEvent* e = last_of(&ps, REF_PARK_OK);
        ok(e && e->object_id == z->id && count_of(&ps, REF_LINE_CROSS) == 0,
           "zone %d (exercise %d, heading %+.1f deg): a centred car is PARK_OK, "
           "clearance %.3f m, no line touched", z->id, z->exercise,
           h * 180.0 / M_PI, e ? e->detail : -1.0);
    }

    /* The test's exercises, as gen_course.py defines them: one of each kind
     * where it matters, and entered in driving order. */
    const int kinds[4] = { REF_EXK_GATED, REF_EXK_PARK, REF_EXK_PARK, REF_EXK_HILL };
    int shape = c.n_exercises == 4, prev_seq = -1;
    for (int32_t i = 0; i < c.n_exercises && shape; ++i) {
        const RefExercise* x = &c.exercises[i];
        int seq = -1;
        for (int32_t g = 0; g < c.n_gates; ++g) if (c.gates[g].id == x->entry_gate) seq = c.gates[g].seq;
        shape = x->id == i + 1 && x->kind == kinds[i] && seq > prev_seq;
        prev_seq = seq;
    }
    ok(shape, "four exercises -- slalom (gated), parallel and bay (park), hill -- "
              "entered in driving order");
}

int main(void)
{
    test_geometry();
    test_cones();
    test_lines();
    test_gates();
    test_stall();
    test_sample_gap();
    test_zone_geometry();
    test_zone_loader();
    test_parking();
    test_verdict();
    test_rollback();
    test_exercise_loader();
    test_real_course();
    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
