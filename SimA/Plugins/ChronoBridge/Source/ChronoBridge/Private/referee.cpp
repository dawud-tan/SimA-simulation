/* referee.cpp — see referee.h. Written as C, compiled as C++ by the UE module
 * and as C by the host harness. */

#include "referee.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* --- geometry ---------------------------------------------------------- */

void ref_body_corners(const RefCourse* c, double x, double y, double yaw,
                      double out[4][2])
{
    /* The pose in snapshot slot 0 is the chassis REFERENCE frame, and for this
     * vehicle that frame is at the FRONT AXLE: gen_innova.py places the front
     * suspension hardpoints at x = 0 and the rear axle at x = -wheelbase. So
     * the body runs from +front_overhang forward of the origin to
     * -(wheelbase + rear_overhang) behind it.
     *
     * Getting this wrong is not subtle in its effect but is very quiet in the
     * code: an earlier version used the body-centre convention and placed the
     * box 2.75 m too far forward, which would have scored cone strikes the car
     * never made and missed the ones it did. */
    const double fx =  c->front_overhang;
    const double bx = -(c->wheelbase + c->rear_overhang);
    const double hw =  c->car_width * 0.5;

    const double local[4][2] = {
        { fx,  hw }, { fx, -hw }, { bx, -hw }, { bx,  hw },
    };
    const double cs = cos(yaw), sn = sin(yaw);
    for (int i = 0; i < 4; ++i) {
        out[i][0] = x + local[i][0] * cs - local[i][1] * sn;
        out[i][1] = y + local[i][0] * sn + local[i][1] * cs;
    }
}

double ref_point_segment_distance(double px, double py,
                                  double x0, double y0, double x1, double y1)
{
    const double dx = x1 - x0, dy = y1 - y0;
    const double len2 = dx * dx + dy * dy;
    if (len2 < 1e-18) return hypot(px - x0, py - y0);   /* degenerate segment */
    double t = ((px - x0) * dx + (py - y0) * dy) / len2;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return hypot(px - (x0 + t * dx), py - (y0 + t * dy));
}

static double cross2(double ax, double ay, double bx, double by)
{
    return ax * by - ay * bx;
}

int32_t ref_segments_intersect(double ax0, double ay0, double ax1, double ay1,
                               double bx0, double by0, double bx1, double by1)
{
    /* Standard orientation test. Collinear-overlap is deliberately NOT treated
     * as an intersection: a car driving exactly along a kerb line is not
     * crossing it, and the floating-point neighbourhood of that case is one
     * where a rule should stay quiet rather than fire at random. */
    const double d1 = cross2(bx1 - bx0, by1 - by0, ax0 - bx0, ay0 - by0);
    const double d2 = cross2(bx1 - bx0, by1 - by0, ax1 - bx0, ay1 - by0);
    const double d3 = cross2(ax1 - ax0, ay1 - ay0, bx0 - ax0, by0 - ay0);
    const double d4 = cross2(ax1 - ax0, ay1 - ay0, bx1 - ax0, by1 - ay0);
    return ((d1 > 0.0) != (d2 > 0.0)) && ((d3 > 0.0) != (d4 > 0.0));
}

/* Signed distance of (px, py) from zone edge i, positive on the INSIDE. The
 * loader normalised every zone to counter-clockwise, so the inside of each
 * edge is on its left and the sign needs no per-zone bookkeeping. */
static double zone_edge_distance(const RefZone* z, int32_t i, double px, double py)
{
    const int32_t j = (i + 1) % z->n;
    const double ex = z->x[j] - z->x[i], ey = z->y[j] - z->y[i];
    return cross2(ex, ey, px - z->x[i], py - z->y[i]) / hypot(ex, ey);
}

int32_t ref_zone_contains(const RefZone* z, double px, double py)
{
    if (!z || z->n < 3) return 0;
    for (int32_t i = 0; i < z->n; ++i)
        if (zone_edge_distance(z, i, px, py) < 0.0) return 0;
    return 1;
}

static double zone_clearance_of_corners(const RefZone* z, const double corner[4][2])
{
    double worst = 1e30;
    for (int k = 0; k < 4; ++k)
        for (int32_t i = 0; i < z->n; ++i) {
            const double d = zone_edge_distance(z, i, corner[k][0], corner[k][1]);
            if (d < worst) worst = d;
        }
    return worst;
}

double ref_zone_clearance(const RefCourse* c, const RefZone* z,
                          double x, double y, double yaw)
{
    if (!c || !z || z->n < 3) return -1e30;
    double corner[4][2];
    ref_body_corners(c, x, y, yaw, corner);
    return zone_clearance_of_corners(z, corner);
}

/* Normalises a zone to counter-clockwise and checks it is convex. Returns 0,
 * or -1 for a degenerate or non-convex polygon -- for which one signed
 * distance per edge would NOT be a clearance, and the rule would be quietly
 * wrong rather than loudly refused. */
static int zone_normalise(RefZone* z)
{
    double area2 = 0.0;
    for (int32_t i = 0; i < z->n; ++i) {
        const int32_t j = (i + 1) % z->n;
        area2 += z->x[i] * z->y[j] - z->x[j] * z->y[i];
    }
    if (fabs(area2) < 1e-9) return -1;
    if (area2 < 0.0) {
        for (int32_t i = 0, j = z->n - 1; i < j; ++i, --j) {
            const double tx = z->x[i], ty = z->y[i];
            z->x[i] = z->x[j]; z->y[i] = z->y[j];
            z->x[j] = tx;      z->y[j] = ty;
        }
    }
    for (int32_t i = 0; i < z->n; ++i) {
        const int32_t j = (i + 1) % z->n, k = (i + 2) % z->n;
        if (cross2(z->x[j] - z->x[i], z->y[j] - z->y[i],
                   z->x[k] - z->x[j], z->y[k] - z->y[j]) < -1e-12)
            return -1;                          /* a right turn: not convex */
    }
    return 0;
}

/* --- events ------------------------------------------------------------ */

static void verdict_consume(RefState* s, const RefEvent* e);

static void emit(RefState* s, double t, int32_t type, int32_t id, double detail)
{
    RefEvent e;
    e.t = t; e.type = type; e.object_id = id; e.detail = detail;
    if (s->n_events < REF_MAX_EVENTS) s->events[s->n_events++] = e;
    else ++s->dropped_events;
    /* AFTER the capacity check, unconditionally: the verdict must see every
     * event, stored or not. A full log changes what is kept, never a result. */
    verdict_consume(s, &e);
}

/* --- the verdict ------------------------------------------------------- */

static const RefGate* gate_by_id(const RefCourse* c, int32_t id)
{
    for (int32_t i = 0; i < c->n_gates; ++i) if (c->gates[i].id == id) return &c->gates[i];
    return NULL;
}

static const RefGate* gate_by_seq(const RefCourse* c, int32_t seq)
{
    for (int32_t i = 0; i < c->n_gates; ++i) if (c->gates[i].seq == seq) return &c->gates[i];
    return NULL;
}

static int32_t exercise_index(const RefCourse* c, int32_t exercise_id)
{
    for (int32_t i = 0; i < c->n_exercises; ++i)
        if (c->exercises[i].id == exercise_id) return i;
    return -1;
}

/* An exercise's place in the test: its entry gate's position in the gate
 * sequence, which is the order the course is driven in. */
static int32_t exercise_order(const RefCourse* c, int32_t i)
{
    const RefGate* g = gate_by_id(c, c->exercises[i].entry_gate);
    return g ? g->seq : 0;
}

static void ex_end(RefState* s, int32_t i, int32_t state, int32_t reason, double t)
{
    RefExerciseResult* r = &s->ex[i];
    r->state = state;
    r->fail_reason = reason;
    r->t_end = t;
    if (s->running == i) s->running = -1;
    if (state == REF_EX_PASSED)
        emit(s, t, REF_EX_PASS, s->course->exercises[i].id, t - r->t_start);
    else
        emit(s, t, REF_EX_FAIL, s->course->exercises[i].id, (double)reason);
}

/* The first fault fails an exercise that has not already passed. Late faults
 * are still counted -- a result should not hide that the car went back and
 * hit something -- but they do not un-pass it. */
static void fault(RefState* s, int32_t exercise_id, int32_t reason, double t)
{
    const int32_t i = exercise_index(s->course, exercise_id);
    if (i < 0) { ++s->general_faults; return; }
    ++s->ex[i].faults;
    if (s->ex[i].state == REF_EX_ARMED || s->ex[i].state == REF_EX_RUNNING)
        ex_end(s, i, REF_EX_FAILED, reason, t);
}

static void ex_start(RefState* s, int32_t i, double t)
{
    const RefCourse* c = s->course;
    /* A running exercise that has not passed was abandoned; any exercise
     * EARLIER in the test that never started was skipped. Either way the car
     * has moved on, and leaving them armed would report an unfinished test
     * rather than a failed one. */
    if (s->running >= 0 && s->running != i)
        ex_end(s, s->running, REF_EX_FAILED, REF_FAIL_ABANDONED, t);
    for (int32_t j = 0; j < c->n_exercises; ++j)
        if (j != i && s->ex[j].state == REF_EX_ARMED &&
            exercise_order(c, j) < exercise_order(c, i))
            ex_end(s, j, REF_EX_FAILED, REF_FAIL_SKIPPED, t);

    if (s->ex[i].state != REF_EX_ARMED) return;     /* entry gates latch anyway */
    s->ex[i].state = REF_EX_RUNNING;
    s->ex[i].t_start = t;
    s->running = i;
    emit(s, t, REF_EX_START, c->exercises[i].id, 0.0);
}

static void verdict_consume(RefState* s, const RefEvent* e)
{
    const RefCourse* c = s->course;
    switch (e->type) {
    case REF_GATE_PASS:
    case REF_GATE_SKIP: {
        /* EXIT FIRST: were one gate both an exercise's exit and the next
         * one's entry, starting the next first would fail this one as
         * abandoned an instant before it finished. */
        const int32_t r = s->running;
        if (r >= 0 && c->exercises[r].exit_gate == e->object_id) {
            if (c->exercises[r].kind == REF_EXK_HILL && !s->ex[r].stopped)
                ex_end(s, r, REF_EX_FAILED, REF_FAIL_NO_STOP, e->t);
            else
                ex_end(s, r, REF_EX_PASSED, REF_FAIL_NONE, e->t);
        }
        /* Crossing an entry starts its exercise even out of sequence: the car
         * is at that exercise now, and whatever it bypassed was already
         * charged, to ITS exercise, as a GATE_MISS. */
        for (int32_t i = 0; i < c->n_exercises; ++i)
            if (c->exercises[i].entry_gate == e->object_id) ex_start(s, i, e->t);
        break;
    }
    case REF_GATE_MISS: {
        const RefGate* g = gate_by_id(c, e->object_id);
        if (g) fault(s, g->exercise, REF_FAIL_GATE, e->t);
        break;
    }
    case REF_CONE_STRIKE:
        for (int32_t i = 0; i < c->n_cones; ++i)
            if (c->cones[i].id == e->object_id) fault(s, c->cones[i].exercise, REF_FAIL_CONE, e->t);
        break;
    case REF_LINE_CROSS:
        for (int32_t i = 0; i < c->n_lines; ++i)
            if (c->lines[i].id == e->object_id) fault(s, c->lines[i].exercise, REF_FAIL_LINE, e->t);
        break;
    case REF_STALL:
        if (s->running >= 0) fault(s, c->exercises[s->running].id, REF_FAIL_STALL, e->t);
        else ++s->general_faults;
        break;
    case REF_ROLLBACK:          /* object_id: the HILL exercise it stopped in */
        fault(s, e->object_id, REF_FAIL_ROLLBACK, e->t);
        break;
    case REF_SAMPLE_GAP:
        ++s->sample_gaps;
        break;
    case REF_PARK_OK:
    case REF_PARK_OUTSIDE:
    case REF_PARK_ANGLE:
        for (int32_t i = 0; i < c->n_exercises; ++i) {
            if (c->exercises[i].kind != REF_EXK_PARK || c->exercises[i].zone != e->object_id)
                continue;
            RefExerciseResult* r = &s->ex[i];
            /* Only a park made while the exercise is RUNNING counts: it has to
             * have been entered through its gate, like every other exercise. */
            if (r->state != REF_EX_RUNNING) continue;
            if (e->type == REF_PARK_OK) {
                ex_end(s, i, REF_EX_PASSED, REF_FAIL_NONE, e->t);
            } else if (e->t != r->last_attempt_t) {   /* OUTSIDE and ANGLE on one
                                                        stop are one attempt */
                r->last_attempt_t = e->t;
                if (++r->park_attempts >= REF_PARK_MAX_ATTEMPTS)
                    ex_end(s, i, REF_EX_FAILED, REF_FAIL_PARK_ATTEMPTS, e->t);
            }
        }
        break;
    default:
        break;                  /* REF_EX_* are the verdict's own: no feedback */
    }
}

void ref_verdict(const RefState* s, RefVerdict* out)
{
    if (!out) return;
    memset(out, 0, sizeof *out);
    if (!s || !s->course) return;
    const int32_t n = s->course->n_exercises;
    out->total = n;
    for (int32_t i = 0; i < n; ++i) {
        if (s->ex[i].state == REF_EX_PASSED)      ++out->passed;
        else if (s->ex[i].state == REF_EX_FAILED) ++out->failed;
        else                                      ++out->unfinished;
    }
    out->general_faults = s->general_faults;
    out->suspect = s->sample_gaps > 0;
    if (out->failed > 0 || out->general_faults > 0) out->result = REF_RESULT_FAIL;
    else if (out->passed == n && n > 0)             out->result = REF_RESULT_PASS;
    else                                            out->result = REF_RESULT_INCOMPLETE;
}

const char* ref_fail_reason_name(int32_t reason)
{
    switch (reason) {
    case REF_FAIL_NONE:          return "-";
    case REF_FAIL_CONE:          return "cone";
    case REF_FAIL_LINE:          return "line";
    case REF_FAIL_GATE:          return "missed gate";
    case REF_FAIL_STALL:         return "stall";
    case REF_FAIL_PARK_ATTEMPTS: return "park attempts";
    case REF_FAIL_NO_STOP:       return "no stop on the hill";
    case REF_FAIL_ABANDONED:     return "abandoned";
    case REF_FAIL_SKIPPED:       return "skipped";
    case REF_FAIL_ROLLBACK:      return "rollback";
    default:                     return "?";
    }
}

const char* ref_exercise_state_name(int32_t state)
{
    switch (state) {
    case REF_EX_ARMED:   return "ARMED";
    case REF_EX_RUNNING: return "RUNNING";
    case REF_EX_PASSED:  return "PASSED";
    case REF_EX_FAILED:  return "FAILED";
    default:             return "?";
    }
}

const char* ref_result_name(int32_t result)
{
    switch (result) {
    case REF_RESULT_PASS:       return "PASS";
    case REF_RESULT_FAIL:       return "FAIL";
    case REF_RESULT_INCOMPLETE: return "INCOMPLETE";
    default:                    return "?";
    }
}

const char* ref_event_name(int32_t type)
{
    switch (type) {
    case REF_CONE_STRIKE: return "CONE_STRIKE";
    case REF_LINE_CROSS:  return "LINE_CROSS";
    case REF_GATE_PASS:   return "GATE_PASS";
    case REF_GATE_SKIP:   return "GATE_SKIP";
    case REF_STALL:       return "STALL";
    case REF_SAMPLE_GAP:  return "SAMPLE_GAP";
    case REF_PARK_OK:      return "PARK_OK";
    case REF_PARK_OUTSIDE: return "PARK_OUTSIDE";
    case REF_PARK_ANGLE:   return "PARK_ANGLE";
    case REF_GATE_MISS:    return "GATE_MISS";
    case REF_EX_START:     return "EX_START";
    case REF_EX_PASS:      return "EX_PASS";
    case REF_EX_FAIL:      return "EX_FAIL";
    case REF_ROLLBACK:     return "ROLLBACK";
    default:              return "?";
    }
}

/* --- lifecycle --------------------------------------------------------- */

void ref_init(RefState* s, const RefCourse* c)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->course = c;
    s->next_gate = 0;
    s->low_rpm_since = -1.0;
    s->running = -1;
    for (int32_t i = 0; i < REF_MAX_EXERCISES; ++i) s->ex[i].last_attempt_t = -1.0;
}

/* Judges the pose recorded when the car stopped, in every zone whose interior
 * holds the body centre. Each failed criterion emits its own event, stamped
 * with the STOP time -- the moment judged -- not the end of the dwell.
 *
 * The same confirmed stop is the hill start's hold: a stop that lasted the
 * dwell while a HILL exercise is running is the stop the exercise asks for. */
static void judge_park(RefState* s, const RefCourse* c)
{
    if (s->running >= 0 && c->exercises[s->running].kind == REF_EXK_HILL)
        s->ex[s->running].stopped = 1;

    double corner[4][2];
    ref_body_corners(c, s->park_x, s->park_y, s->park_yaw, corner);
    const double cx = 0.25 * (corner[0][0] + corner[1][0] + corner[2][0] + corner[3][0]);
    const double cy = 0.25 * (corner[0][1] + corner[1][1] + corner[2][1] + corner[3][1]);

    for (int32_t i = 0; i < c->n_zones; ++i) {
        const RefZone* z = &c->zones[i];
        if (!ref_zone_contains(z, cx, cy)) continue;

        const double clear = zone_clearance_of_corners(z, corner);
        const double dyaw = atan2(sin(s->park_yaw - z->heading),
                                  cos(s->park_yaw - z->heading));
        const int inside = clear >= REF_PARK_CLEARANCE;
        const int square = fabs(dyaw) <= REF_PARK_YAW_TOL;

        if (inside && square) emit(s, s->park_t, REF_PARK_OK, z->id, clear);
        if (!inside)          emit(s, s->park_t, REF_PARK_OUTSIDE, z->id, clear);
        if (!square)          emit(s, s->park_t, REF_PARK_ANGLE, z->id,
                                   dyaw * 180.0 / 3.14159265358979323846);
    }
}

void ref_update(RefState* s, const RefVehicleState* v)
{
    if (!s || !v || !s->course) return;
    const RefCourse* c = s->course;

    /* --- sample continuity ---------------------------------------------
     * The referee's guarantees hold only if it sees EVERY physics sample. A
     * gap means the consumer fell behind -- on the game thread, that the ring
     * wrapped before the frame drained it. Report it rather than score a
     * trajectory with holes and present the result as if it were sound. */
    if (s->primed && v->step > s->last_step + 1) {
        emit(s, v->t, REF_SAMPLE_GAP, -1, (double)(v->step - s->last_step - 1));
    }
    const int had_prev = s->primed;
    s->last_step = v->step;
    s->primed = 1;

    double corner[4][2];
    ref_body_corners(c, v->x, v->y, v->yaw, corner);

    /* --- cones ---------------------------------------------------------
     * A cone is struck when the body OUTLINE comes within its radius. Testing
     * the outline rather than the interior is right: a cone the car has driven
     * completely over has still been struck, and it was struck at the moment
     * the bodywork reached it. */
    for (int32_t i = 0; i < c->n_cones; ++i) {
        if (s->cone_hit[i]) continue;
        const RefCone* k = &c->cones[i];
        double dmin = 1e30;
        for (int e = 0; e < 4; ++e) {
            const int f = (e + 1) & 3;
            const double d = ref_point_segment_distance(
                k->x, k->y, corner[e][0], corner[e][1], corner[f][0], corner[f][1]);
            if (d < dmin) dmin = d;
        }
        if (dmin <= k->radius) {
            s->cone_hit[i] = 1;
            emit(s, v->t, REF_CONE_STRIKE, k->id, k->radius - dmin);
        }
    }

    /* --- boundary lines ------------------------------------------------- */
    for (int32_t i = 0; i < c->n_lines; ++i) {
        if (s->line_hit[i]) continue;
        const RefLine* l = &c->lines[i];
        int crossed = 0;
        for (int e = 0; e < 4 && !crossed; ++e) {
            const int f = (e + 1) & 3;
            crossed = ref_segments_intersect(
                corner[e][0], corner[e][1], corner[f][0], corner[f][1],
                l->x0, l->y0, l->x1, l->y1);
        }
        if (crossed) {
            s->line_hit[i] = 1;
            emit(s, v->t, REF_LINE_CROSS, l->id, 0.0);
        }
    }

    /* --- gates ----------------------------------------------------------
     * Tracked on the REFERENCE POINT, not the body outline: a gate is a
     * progress marker, and a car whose nose has crossed but whose mass has not
     * has not been through it yet.
     *
     * A gate is passed when the SEGMENT TRAVELLED since the previous sample
     * intersects the gate segment. That tests "changed sides" and "within the
     * gate's extent" in one operation. A bare sign-of-side test would fire on
     * the infinite line and score a gate the car drove nowhere near, and a
     * proximity test would miss a gate crossed quickly between two samples. */
    if (had_prev) {
        for (int32_t i = 0; i < c->n_gates; ++i) {
            if (s->gate_done[i]) continue;
            const RefGate* g = &c->gates[i];
            if (!ref_segments_intersect(s->prev_x, s->prev_y, v->x, v->y,
                                        g->x0, g->y0, g->x1, g->y1))
                continue;
            s->gate_done[i] = 1;
            if (g->seq == s->next_gate) {
                ++s->next_gate;
                emit(s, v->t, REF_GATE_PASS, g->id, (double)g->seq);
            } else {
                /* Out of order. If it is AHEAD of the sequence, the gates in
                 * between were left behind: each is a GATE_MISS, charged to
                 * its own exercise, and the sequence RESYNCHRONISES past this
                 * gate. It used to stay put, and one missed slalom gate then
                 * made every later gate on the course "out of sequence" -- a
                 * single mistake scored as a cascade. */
                if (g->seq > s->next_gate) {
                    for (int32_t k = s->next_gate; k < g->seq; ++k) {
                        const RefGate* m = gate_by_seq(c, k);
                        if (m && !s->gate_done[m - c->gates]) {
                            s->gate_done[m - c->gates] = 1;
                            emit(s, v->t, REF_GATE_MISS, m->id, (double)k);
                        }
                    }
                    s->next_gate = g->seq + 1;
                }
                /* Always worth recording: the car did not follow the route. */
                emit(s, v->t, REF_GATE_SKIP, g->id, (double)g->seq);
            }
        }
    }
    s->prev_x = v->x;
    s->prev_y = v->y;

    /* --- stall ----------------------------------------------------------
     * Only meaningful with drive connected: in neutral, or with the clutch
     * down, a low engine speed is a stopped car, not a stalled one. The shim
     * expresses neutral as clutch = 1.0 (see chrono_shim.cpp), so the clutch
     * test covers both. */
    const int connected = (v->gear != 0) && (v->clutch < 0.5);
    const int stopped   = fabs(v->speed) < REF_STALL_MAX_SPEED;
    if (connected && stopped && v->engine_rpm < REF_STALL_RPM) {
        if (s->low_rpm_since < 0.0) s->low_rpm_since = v->t;
        if (!s->stalled && (v->t - s->low_rpm_since) >= REF_STALL_SECONDS) {
            s->stalled = 1;
            emit(s, v->t, REF_STALL, -1, v->engine_rpm);
        }
    } else {
        s->low_rpm_since = -1.0;
        s->stalled = 0;
    }

    /* --- parking ---------------------------------------------------------
     * Moving -> stopped records the pose; staying stopped for REF_PARK_DWELL
     * judges THAT pose; moving again first discards it as a pause. The
     * referee starts in "stopped, nothing pending", so a car sitting at its
     * spawn is never judged: it has to have moved to have parked. */
    const double speed = fabs(v->speed);
    const int hill_running = s->running >= 0 && c->exercises[s->running].kind == REF_EXK_HILL;
    if (s->park_moving) {
        if (speed < REF_PARK_STOP_SPEED) {
            s->park_moving  = 0;
            s->park_pending = 1;
            s->park_t = v->t;
            s->park_x = v->x; s->park_y = v->y; s->park_yaw = v->yaw;
            if (hill_running) {             /* rollback is watched from HERE */
                s->roll_watch = 1;
                s->roll_ex = s->running;
                s->roll_fired = 0;
                s->roll_d = 0.0;
                s->roll_t = v->t;
            }
        }
    } else if (speed > REF_PARK_MOVE_SPEED) {
        s->park_moving  = 1;
        s->park_pending = 0;
    } else if (s->park_pending && v->t - s->park_t >= REF_PARK_DWELL) {
        s->park_pending = 0;
        judge_park(s, c);
    }

    /* --- rollback (task 7) -------------------------------------------------
     * Distance along the road since the stop, from the signed speed. The
     * worst is kept in the exercise's result, reached the floor or not -- and
     * kept on after the rollback has failed the exercise, which ends it: the
     * whole excursion is the measurement. Watching ends when the car has
     * driven off, or another exercise has begun. */
    if (s->roll_watch) {
        if (s->running >= 0 && s->running != s->roll_ex) {
            s->roll_watch = 0;
        } else {
            s->roll_d += v->wheel_speed * (v->t - s->roll_t);
            s->roll_t = v->t;
            RefExerciseResult* r = &s->ex[s->roll_ex];
            if (s->roll_d < r->rollback) r->rollback = s->roll_d;
            if (!s->roll_fired && s->roll_d < -REF_ROLLBACK_FLOOR) {
                s->roll_fired = 1;
                emit(s, v->t, REF_ROLLBACK, c->exercises[s->roll_ex].id, s->roll_d);
            }
            if (s->roll_d > REF_ROLLBACK_CLEAR) s->roll_watch = 0;   /* drove off */
        }
    }
}

double ref_wheel_speed(const double q[4], const double w_body_local[3],
                       const double (*w_spindle_world)[3], int n, double r_roll)
{
    if (n <= 0) return 0.0;
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    /* The body's lateral axis in the world: the second column of R(q). */
    const double ly[3] = { 2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z),
                           2.0 * (y * z + w * x) };
    double spin = 0.0;
    for (int i = 0; i < n; ++i)
        spin += w_spindle_world[i][0] * ly[0] + w_spindle_world[i][1] * ly[1]
              + w_spindle_world[i][2] * ly[2];
    return (spin / n - w_body_local[1]) * r_roll;
}

/* --- course loading ---------------------------------------------------- */

int32_t ref_load_course(RefCourse* c, const char* path, char* err, size_t errsz)
{
    if (err && errsz) err[0] = '\0';
    if (!c || !path) {
        if (err && errsz) snprintf(err, errsz, "bad arguments");
        return -1;
    }
    memset(c, 0, sizeof(*c));

    FILE* f = fopen(path, "r");
    if (!f) {
        if (err && errsz) snprintf(err, errsz, "cannot open '%s'", path);
        return -1;
    }

    char line[512];
    int lineno = 0;
    int have_car = 0;

    while (fgets(line, sizeof line, f)) {
        ++lineno;
        char* hash = strchr(line, '#');
        if (hash) *hash = '\0';

        char kw[16] = {0};
        if (sscanf(line, "%15s", kw) != 1) continue;    /* blank/comment */

        if (!strcmp(kw, "START")) {
            if (sscanf(line, "%*s %lf %lf %lf",
                       &c->start_x, &c->start_y, &c->start_yaw) != 3) goto bad;
        } else if (!strcmp(kw, "CAR")) {
            /* Six fields since task 7: the rolling radius. A five-field CAR is
             * a course generated before it -- regenerate, do not guess. */
            if (sscanf(line, "%*s %lf %lf %lf %lf %lf %lf",
                       &c->car_length, &c->car_width, &c->front_overhang,
                       &c->rear_overhang, &c->wheelbase, &c->r_roll) != 6
                || c->r_roll <= 0.0) goto bad;
            have_car = 1;
        } else if (!strcmp(kw, "CONE")) {
            if (c->n_cones >= REF_MAX_CONES) goto full;
            RefCone* k = &c->cones[c->n_cones];
            if (sscanf(line, "%*s %d %lf %lf %lf %d",
                       &k->id, &k->x, &k->y, &k->radius, &k->exercise) != 5) goto bad;
            ++c->n_cones;
        } else if (!strcmp(kw, "LINE")) {
            if (c->n_lines >= REF_MAX_LINES) goto full;
            RefLine* l = &c->lines[c->n_lines];
            if (sscanf(line, "%*s %d %d %lf %lf %lf %lf %d",
                       &l->id, &l->kind, &l->x0, &l->y0, &l->x1, &l->y1,
                       &l->exercise) != 7) goto bad;
            ++c->n_lines;
        } else if (!strcmp(kw, "GATE")) {
            if (c->n_gates >= REF_MAX_GATES) goto full;
            RefGate* g = &c->gates[c->n_gates];
            if (sscanf(line, "%*s %d %d %lf %lf %lf %lf %d",
                       &g->id, &g->seq, &g->x0, &g->y0, &g->x1, &g->y1,
                       &g->exercise) != 7) goto bad;
            ++c->n_gates;
        } else if (!strcmp(kw, "EXERCISE")) {
            if (c->n_exercises >= REF_MAX_EXERCISES) goto full;
            RefExercise* x = &c->exercises[c->n_exercises];
            if (sscanf(line, "%*s %d %d %d %d %d", &x->id, &x->kind,
                       &x->entry_gate, &x->exit_gate, &x->zone) != 5) goto bad;
            ++c->n_exercises;
        } else if (!strcmp(kw, "ZONE")) {
            /* ZONE id exercise heading n x0 y0 ... x(n-1) y(n-1): a variable
             * vertex list, so the fixed part is scanned and the rest walked. */
            if (c->n_zones >= REF_MAX_ZONES) goto full;
            RefZone* z = &c->zones[c->n_zones];
            int used = 0;
            if (sscanf(line, "%*s %d %d %lf %d%n",
                       &z->id, &z->exercise, &z->heading, &z->n, &used) != 4) goto bad;
            if (z->n < 3 || z->n > REF_MAX_ZONE_VERTS) goto bad;
            char* q = line + used;
            for (int32_t k = 0; k < z->n; ++k) {
                char* e = NULL;
                z->x[k] = strtod(q, &e); if (e == q) goto bad; q = e;
                z->y[k] = strtod(q, &e); if (e == q) goto bad; q = e;
            }
            while (*q == ' ' || *q == '\t' || *q == '\r' || *q == '\n') ++q;
            if (*q) goto bad;                    /* more numbers than n said */
            if (zone_normalise(z) != 0) {
                if (err && errsz)
                    snprintf(err, errsz, "%s:%d: ZONE %d is degenerate or not "
                             "convex", path, lineno, z->id);
                fclose(f);
                return -1;
            }
            ++c->n_zones;
        } else {
            if (err && errsz)
                snprintf(err, errsz, "%s:%d: unknown keyword '%s'", path, lineno, kw);
            fclose(f);
            return -1;
        }
        continue;
    bad:
        if (err && errsz)
            snprintf(err, errsz, "%s:%d: malformed %s record", path, lineno, kw);
        fclose(f);
        return -1;
    full:
        if (err && errsz)
            snprintf(err, errsz, "%s:%d: too many %s records", path, lineno, kw);
        fclose(f);
        return -1;
    }
    fclose(f);

    /* The CAR record is what places the body box. Without it every geometric
     * rule would silently test a zero-size car and never fire. */
    if (!have_car) {
        if (err && errsz) snprintf(err, errsz, "%s: no CAR record", path);
        return -1;
    }

    /* Every reference an exercise makes must resolve. A dangling entry gate
     * would not fail loudly later -- the exercise would simply never start,
     * and the test would report INCOMPLETE for a reason nobody could see. */
    for (int32_t i = 0; i < c->n_exercises; ++i) {
        const RefExercise* x = &c->exercises[i];
        const char* why = NULL;
        int zone_ok = 0;
        for (int32_t k = 0; k < c->n_zones; ++k) zone_ok |= c->zones[k].id == x->zone;
        if (x->kind < REF_EXK_GATED || x->kind > REF_EXK_HILL) why = "unknown kind";
        else if (!gate_by_id(c, x->entry_gate))               why = "entry gate does not exist";
        else if (x->kind != REF_EXK_PARK && !gate_by_id(c, x->exit_gate))
                                                               why = "exit gate does not exist";
        else if (x->kind == REF_EXK_PARK && !zone_ok)          why = "zone does not exist";
        for (int32_t j = 0; j < i && !why; ++j)
            if (c->exercises[j].id == x->id)                   why = "duplicate id";
        if (why) {
            if (err && errsz)
                snprintf(err, errsz, "%s: EXERCISE %d: %s", path, x->id, why);
            return -1;
        }
    }
    return 0;
}

int32_t ref_get_events(const RefState* s, RefEvent* out, int32_t max)
{
    if (!s || !out || max <= 0) return 0;
    int32_t n = s->n_events < max ? s->n_events : max;
    memcpy(out, s->events, (size_t)n * sizeof(RefEvent));
    return n;
}
