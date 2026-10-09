/* ===========================================================================
 *  referee.h — the SIM A rules, as predicates over a trajectory.
 *
 *  PURE C11. No Unreal, no Chrono. Same arrangement as driver_source and
 *  wheel_device: one copy, living in the plugin, exercised by a host harness
 *  that runs in a second.
 *
 *  WHY THE REFEREE IS NOT A PHYSICS FEATURE:
 *  Cones could have been rigid bodies in the Chrono world, knocked over on
 *  contact. They are not, for three reasons.
 *    1. ChsSnapshot carries 16 bodies and the vehicle already uses 5. A full
 *       course has more cones than the remaining 11, so they could not all
 *       cross the ABI wall anyway.
 *    2. Growing the snapshot to fit them would grow the 64-slot ring in step.
 *       At 64 bodies that is ~428 KB, published every millisecond, evicting
 *       the physics thread's working set on a machine with 256 KB of L2 per
 *       core. The ring is deliberately small.
 *    3. A 0.5 kg cone against a 1800 kg car changes the trajectory by nothing
 *       measurable. Simulating it buys no fidelity in the thing being scored.
 *  So Chrono simulates the car and the referee is exact geometry over its
 *  pose. A struck cone falling over is a renderer's problem.
 *
 *  WHY IT CONSUMES EVERY SAMPLE, NOT THE RENDERED POSE:
 *  The interpolated pose the renderer draws is a function of frame timing, so
 *  a verdict derived from it would depend on the framerate -- two runs of the
 *  same input could score differently. The referee is driven from the physics
 *  stream instead: ref_update() is called once per PHYSICS sample, at 1 kHz.
 *  On the game thread that means draining the ring rather than sampling it,
 *  and ref_update reports a gap if samples were missed rather than silently
 *  scoring a trajectory with holes in it.
 *
 *  COORDINATES: Chrono ISO 8855 -- X forward, Y LEFT, Z up, metres. The frame
 *  the physics publishes and gen_course.py emits, so nothing converts.
 * ======================================================================== */
#ifndef REFEREE_H
#define REFEREE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define REF_MAX_CONES       64
#define REF_MAX_LINES       32
#define REF_MAX_GATES       32
#define REF_MAX_ZONES       8
#define REF_MAX_ZONE_VERTS  8
#define REF_MAX_EXERCISES   8
#define REF_MAX_EVENTS      256

/* --- course ------------------------------------------------------------ */

/* `exercise` on every object is what attributes a fault to an exercise of the
 * test. It is the LAST field of RefLine and RefGate so that positional
 * initialisers written before it existed still compile, zero meaning "none". */
typedef struct { double x, y, radius; int32_t id, exercise; }          RefCone;
typedef struct { double x0, y0, x1, y1; int32_t id, kind, exercise; }  RefLine;
typedef struct { double x0, y0, x1, y1; int32_t id, seq, exercise; }   RefGate;

/* How an exercise is PASSED. How it FAILS is common to all three: the first
 * fault attributed to it while it is armed or running. */
typedef enum {
    REF_EXK_GATED = 0,   /* by crossing its exit gate                         */
    REF_EXK_PARK  = 1,   /* by a PARK_OK in its zone                          */
    REF_EXK_HILL  = 2,   /* by crossing its exit gate AFTER a confirmed stop
                            on the way -- the hold is the exercise. Rollback
                            is not judged yet: task `rollback-rule`.         */
} RefExerciseKind;

/* One exercise of the test. Its ORDER in the test is its entry gate's seq. */
typedef struct {
    int32_t id;          /* the exercise id cones, lines, gates, zones carry */
    int32_t kind;        /* RefExerciseKind                                  */
    int32_t entry_gate;  /* gate id whose crossing starts it                 */
    int32_t exit_gate;   /* gate id that ends it (GATED, HILL), else -1      */
    int32_t zone;        /* zone id it is judged in (PARK), else -1          */
} RefExercise;

/* A parking space: a CONVEX polygon and the heading a correctly parked car has
 * in it. The loader rejects a non-convex polygon and normalises the winding to
 * counter-clockwise, so every edge has the inside on its left -- which is what
 * lets a corner's clearance be one signed distance per edge. */
typedef struct {
    double  x[REF_MAX_ZONE_VERTS], y[REF_MAX_ZONE_VERTS];
    int32_t n;
    int32_t id, exercise;
    double  heading;    /* radians: the yaw of a correctly parked car. The bay
                           is reversed INTO, so a car that drove in nose first
                           is 180 degrees out, not "on the axis".            */
} RefZone;

typedef struct {
    double  start_x, start_y, start_yaw;
    /* Body box, in the CHASSIS REFERENCE FRAME -- which is what
     * Chs_GetSnapshot publishes in slot 0, not the centre of mass. */
    double  car_length, car_width, front_overhang, rear_overhang;
    /* Needed because the reference frame is at the FRONT AXLE, not at the
     * centre of the body -- gen_innova.py puts the front hardpoints at x=0 and
     * the rear axle at x=-wheelbase. The box therefore runs from
     * +front_overhang to -(wheelbase + rear_overhang). */
    double  wheelbase;
    /* The tyres' rolling radius, m: wheel spin x this is how far a wheel has
     * rolled. The rollback rule (task 7) counts the WHEELS rolling back. */
    double  r_roll;

    RefCone cones[REF_MAX_CONES]; int32_t n_cones;
    RefLine lines[REF_MAX_LINES]; int32_t n_lines;
    RefGate gates[REF_MAX_GATES]; int32_t n_gates;
    RefZone zones[REF_MAX_ZONES]; int32_t n_zones;
    RefExercise exercises[REF_MAX_EXERCISES]; int32_t n_exercises;
} RefCourse;

/** The wheels' rolling speed for RefVehicleState.wheel_speed, from a
 *  Chs_GetSnapshot frame without including the shim: the chassis rotation
 *  (w, x, y, z) and its angular velocity in its OWN frame (slot 0), and the
 *  spindles' angular velocities in the WORLD frame (slots 1-4 -- Chrono's
 *  GetSpindleAngVel is documented as global). Each spindle's spin is its
 *  angular velocity about the body's lateral axis, less the body's own
 *  rotation about that axis, so the body pitching does not read as rolling.
 *  The mean of `n` spindles, times `r_roll`; + = rolling forward (ISO 8855,
 *  +Y left: a wheel rolling forward spins positively about +Y). */
double  ref_wheel_speed(const double q[4], const double w_body_local[3],
                        const double (*w_spindle_world)[3], int n, double r_roll);

/** Loads a course emitted by gen_course.py. Returns 0, or -1 with a message
 *  in `err` naming the offending line. */
int32_t ref_load_course(RefCourse* c, const char* path, char* err, size_t errsz);

/* --- what the referee is shown each step ------------------------------- */

typedef struct {
    double  t;          /* simulation time, seconds                         */
    uint64_t step;      /* physics step index; gaps are detected            */
    double  x, y, yaw;  /* chassis REFERENCE frame pose                     */
    double  speed;      /* signed along the body X axis                     */
    /* How fast the WHEELS roll the car, m/s, + forward: the driven wheels'
     * spin RELATIVE TO THE BODY x r_roll -- see ref_wheel_speed. Not the
     * same thing as `speed`: a braked car on a slope settles back several
     * centimetres on its tyres' compliance with its wheels locked (task 7),
     * and the regulation's fault is the car ROLLING back. */
    double  wheel_speed;
    double  engine_rpm;
    int32_t gear;
    double  clutch;     /* 1 = disengaged                                   */
} RefVehicleState;

/* --- events ------------------------------------------------------------ */

typedef enum {
    REF_CONE_STRIKE = 0,  /* body overlapped a cone                         */
    REF_LINE_CROSS  = 1,  /* body crossed a boundary line                   */
    REF_GATE_PASS   = 2,  /* the next gate in sequence was passed           */
    REF_GATE_SKIP   = 3,  /* a gate was passed out of order                 */
    REF_STALL       = 4,  /* engine died with the clutch engaged in gear    */
    REF_SAMPLE_GAP  = 5,  /* physics samples were missed -- verdict suspect */
    /* A finished park, judged at the moment the car stopped in a zone. Every
     * failed criterion gets its own event, so a stop can fail twice. */
    REF_PARK_OK      = 6, /* parked: detail = worst-corner clearance, m     */
    REF_PARK_OUTSIDE = 7, /* a corner out of the zone or on its line:
                             detail = that clearance, m; negative = over it  */
    REF_PARK_ANGLE   = 8, /* not square to the space: detail = heading
                             error, DEGREES, signed                          */
    /* A gate left behind uncrossed, found when a LATER gate is crossed:
     * detail = its seq. Without it one missed gate made every later gate
     * "out of sequence" for the rest of the run. */
    REF_GATE_MISS    = 9,
    /* The verdict's own transitions. object = exercise id. */
    REF_EX_START     = 10,
    REF_EX_PASS      = 11, /* detail = seconds the exercise took            */
    REF_EX_FAIL      = 12, /* detail = RefFailReason                        */
    /* Task 7. After stopping inside a HILL exercise the car travelled
     * backwards along the road by more than REF_ROLLBACK_FLOOR. object_id =
     * that exercise's id; detail = the distance when it fired, m, negative.
     * Once per stop. */
    REF_ROLLBACK     = 13,
} RefEventType;

/* Why an exercise failed. */
typedef enum {
    REF_FAIL_NONE          = 0,
    REF_FAIL_CONE          = 1, /* struck one of its cones                    */
    REF_FAIL_LINE          = 2, /* crossed one of its lines                   */
    REF_FAIL_GATE          = 3, /* left one of its gates uncrossed            */
    REF_FAIL_STALL         = 4, /* stalled while it was running               */
    REF_FAIL_PARK_ATTEMPTS = 5, /* REF_PARK_MAX_ATTEMPTS stops, no PARK_OK    */
    REF_FAIL_NO_STOP       = 6, /* hill: reached the exit without a stop      */
    REF_FAIL_ABANDONED     = 7, /* the next exercise began before it passed   */
    REF_FAIL_SKIPPED       = 8, /* a LATER exercise began before it did       */
    REF_FAIL_ROLLBACK      = 9, /* hill: rolled back after a stop             */
} RefFailReason;

typedef enum {
    REF_EX_ARMED   = 0,
    REF_EX_RUNNING = 1,
    REF_EX_PASSED  = 2,
    REF_EX_FAILED  = 3,
} RefExerciseState;

/* One exercise's progress, kept in the same order as course->exercises. */
typedef struct {
    int32_t state;          /* RefExerciseState                              */
    int32_t fail_reason;    /* RefFailReason, when FAILED                    */
    int32_t faults;         /* every fault attributed to it, even late ones  */
    int32_t park_attempts;  /* judged stops in its zone that were not OK     */
    int32_t stopped;        /* HILL: a confirmed stop while it was running   */
    double  rollback;       /* HILL: worst travel BACK along the road after
                               a stop, m, <= 0 -- recorded whether or not it
                               reached the floor, so a clean start can be
                               measured against it                          */
    double  t_start, t_end;
    double  last_attempt_t; /* a stop failing twice is one attempt, not two  */
} RefExerciseResult;

typedef enum {
    REF_RESULT_INCOMPLETE = 0,  /* nothing failed, but not everything passed */
    REF_RESULT_PASS       = 1,
    REF_RESULT_FAIL       = 2,
} RefResult;

typedef struct {
    int32_t result;          /* RefResult                                    */
    int32_t passed, failed, unfinished, total;
    int32_t general_faults;  /* faults outside every exercise                */
    int32_t suspect;         /* physics samples were missed: the verdict was
                                scored on a trajectory with holes in it      */
} RefVerdict;

typedef struct {
    double   t;
    int32_t  type;        /* RefEventType                                   */
    int32_t  object_id;   /* cone / line / gate id, or -1                   */
    double   detail;      /* penetration depth, rpm, missed-sample count... */
} RefEvent;

/* --- state ------------------------------------------------------------- */

typedef struct {
    const RefCourse* course;

    /* Latched per object so one contact does not fire once per millisecond.
     * A cone knocked at 1 kHz would otherwise produce 300 identical events
     * while the car slid past it. */
    uint8_t  cone_hit[REF_MAX_CONES];
    uint8_t  line_hit[REF_MAX_LINES];
    uint8_t  gate_done[REF_MAX_GATES];

    int32_t  next_gate;          /* sequence position                        */
    uint64_t last_step;
    int      primed;
    /* Previous reference-point position. A gate is crossed when the SEGMENT
     * travelled between two samples intersects the gate segment -- which tests
     * "changed sides" and "within the gate" in one operation, and cannot fire
     * on the infinite line the way a bare sign test can. */
    double   prev_x, prev_y;

    /* Stall detection needs hysteresis in TIME, not just a threshold: engine
     * speed dips through the bite point on every launch. */
    double   low_rpm_since;
    int      stalled;

    /* Parking. A stop is judged only once it has LASTED REF_PARK_DWELL, but
     * the pose judged is the one recorded at the moment of stopping. */
    int      park_moving;        /* speed hysteresis state                    */
    int      park_pending;       /* a stop awaiting its dwell                 */
    double   park_t, park_x, park_y, park_yaw;

    /* Rollback (task 7): after a stop inside a HILL exercise, the signed
     * speed integrated over time -- distance ALONG THE ROAD, the slope
     * included, since `speed` is along the body's own X axis. */
    int      roll_watch;         /* watching since that stop                  */
    int32_t  roll_ex;            /* index of the HILL exercise it stopped in  */
    int      roll_fired;         /* REF_ROLLBACK already emitted for it       */
    double   roll_d;             /* m, + forward, since the stop              */
    double   roll_t;             /* time of the last sample integrated        */

    /* The verdict. Fed from emit() BEFORE the event buffer's capacity check,
     * so it sees every event whether or not the log had room for it: a full
     * log must never change a result. */
    RefExerciseResult ex[REF_MAX_EXERCISES];
    int32_t  running;            /* index into course->exercises, or -1       */
    int32_t  general_faults;
    int32_t  sample_gaps;

    RefEvent events[REF_MAX_EVENTS];
    int32_t  n_events;
    int32_t  dropped_events;
} RefState;

/** Idle-speed floor. gen_innova.py sets IDLE_RPM = 800 [EST]; this is half.
 *
 *  [EST], AND NOT YET CALIBRATED -- see the speed conjunct below for why. */
#define REF_STALL_RPM        400.0
/** How long the condition must hold. A launch dips through the bite point; a
 *  stall does not recover. */
#define REF_STALL_SECONDS    0.75
/** A stalled engine cannot be driving the car, so a stalled car is stopped.
 *
 *  THIS CONJUNCT IS LOAD-BEARING, and it was added because the rule without it
 *  produced a FALSE POSITIVE ON EVERY LAUNCH. Measured on this model: engaging
 *  the clutch drags the engine to 330-380 rpm and holds it there for well over
 *  0.75 s before it recovers -- on runs that then pull away cleanly to 6.4 m/s.
 *  Raising the time window or lowering the rpm floor would not separate the two
 *  cases, because a run that genuinely bogs sits in the SAME 330-380 rpm band.
 *  rpm alone does not discriminate on this vehicle.
 *
 *  Speed does. A car doing 6 m/s has not stalled whatever the tachometer says.
 *
 *  The underlying oddity -- that a 2.4 diesel with 343 N.m from 1400 rpm is
 *  dragged below a third of its idle speed by a 30% throttle launch, and that
 *  the engine model has no idle governor to recover from it -- is a VEHICLE
 *  question, not a referee one. It is recorded in CLAUDE.md. Do not tune this
 *  threshold to compensate for it.
 *
 *  RESOLVED ON THE VEHICLE SIDE (tasks 13 and 8): the clutch was welded shut,
 *  so every launch started from 0 rpm, and there was no governor. Now the
 *  whole course never takes the engine under 654 rpm while the clutch carries
 *  torque, and an engine dragged under 300 rpm really dies. The conjunct stays because it is still
 *  right -- in 1st, 400 rpm is 0.8 m/s, so a car moving faster than this has
 *  an engine turning -- and test_stall's "bogged launch" case pins it. */
#define REF_STALL_MAX_SPEED  0.5
/** Half-width of the band around a gate line that counts as crossing it. */
#define REF_GATE_TOLERANCE   0.05

/* --- parking -------------------------------------------------------------
 *
 *  JUDGED AT THE MOMENT THE CAR STOPS, NOT AFTER IT SETTLES. A force-element
 *  tyre cannot hold a static force (CLAUDE.md, "the car creeps backwards"), so
 *  a pose sampled after a wait has drifted by an amount that belongs to the
 *  tyre model, not the driver. The stop must still LAST, or every pause to
 *  change gear mid-manoeuvre would be scored as a finished park -- so the
 *  judgement waits REF_PARK_DWELL, and then judges the pose it recorded when
 *  the car first stopped.
 *
 *  A stop is judged in a zone only if the BODY CENTRE is inside it: the car
 *  is in that space, as opposed to passing it or waiting beside it.
 *
 *  All four values are [EST]. The regulation's own tolerances are unverified
 *  (see the [REG?] note in gen_course.py), and these are placeholders a
 *  primary source should replace -- task `regulation-dimensions`. */

/** Below this the car counts as stopped, m/s. Well under the 0.17 m/s an
 *  unbraked force-element tyre creeps at, so a creeping car is NOT stopped;
 *  the brake-conditioned parking hold holds a braked one to ~0.001 m/s. */
#define REF_PARK_STOP_SPEED  0.05
/** Above this it counts as moving again, m/s. Hysteresis, so a car hovering
 *  near the threshold does not flicker between stopped and moving. */
#define REF_PARK_MOVE_SPEED  0.15
/** How long a stop must last to be a finished park rather than a pause, s. */
#define REF_PARK_DWELL       1.0
/** Every body corner must be at least this far inside every zone edge, m.
 *  Half a 10 cm painted line: a corner on the paint is on the line. */
#define REF_PARK_CLEARANCE   0.05
/** Largest heading error that still counts as square to the space, rad. */
#define REF_PARK_YAW_TOL     (5.0 * 3.14159265358979323846 / 180.0)

/* --- the verdict ------------------------------------------------------------
 *
 *  POLICY, and [REG?] like the course dimensions: the first fault fails the
 *  exercise it belongs to (a touched cone or line, a stall, a missed gate),
 *  which is how the SIM A test is commonly described but has not been checked
 *  against a primary source. The test passes only if every exercise passes and
 *  nothing faulted between them. */

/** Judged stops in a parking zone, none of them PARK_OK, before the exercise
 *  fails. [REG?] Each stop that lasts the dwell is one attempt. */
#define REF_PARK_MAX_ATTEMPTS  3

/* --- the hill start: rollback (task 7) ----------------------------------
 *
 *  THE REGULATION ALLOWS NONE. Perkapolri 9/2012, Lampiran A 3.d(2): stopped
 *  on the ascent, pulling away "tidak ada reaksi kendaraan mundur"; 3.d(7)
 *  fails "mundur pada saat berhenti ditanjakan". So this is not a tolerance
 *  but a DETECTION FLOOR: the least backward travel that is unambiguously
 *  the car rolling, as opposed to the model's own noise. Measured on the
 *  course's 15 degrees (task 7, `rollback` and the course runs), as WHEEL
 *  travel: a car braked to a stop from 0.5-2 m/s and held, 1.2-1.3 cm (its
 *  locked wheels still turn a little against the body -- probably the rear
 *  axle tube pitching on its links as the body settles; not separated); the
 *  pilot's clutch start, 0.8 cm; a bad start -- the brake released at half
 *  the holding capacity, and half a second to react -- 12 cm.
 *
 *  Measured as the WHEELS' travel from the moment of stopping -- the
 *  wheel_speed integrated -- and only inside a HILL exercise: reversing is
 *  the manoeuvre in the parking exercises. Not the body's travel: stopped
 *  firmly on 15 degrees with the wheels LOCKED, the body settles back 5.8 to
 *  7.6 cm on the tyres' bristle compliance (`rollback`), which a 5 cm floor
 *  on body travel would fail as a rollback. Watching ends when the car has
 *  rolled REF_ROLLBACK_CLEAR forward of where it stopped. */
#define REF_ROLLBACK_FLOOR   0.05
#define REF_ROLLBACK_CLEAR   0.50

void    ref_init(RefState* s, const RefCourse* c);
/** Feed exactly one PHYSICS sample. Safe to call at 1 kHz. */
void    ref_update(RefState* s, const RefVehicleState* v);
/** Copies out up to `max` events; returns how many were written. */
int32_t ref_get_events(const RefState* s, RefEvent* out, int32_t max);
/** Human-readable name, for logs and the harness. */
const char* ref_event_name(int32_t type);
/** Aggregates the exercises into one result. Pure: callable at any time, and
 *  two identical trajectories give byte-identical verdicts. */
void    ref_verdict(const RefState* s, RefVerdict* out);
const char* ref_fail_reason_name(int32_t reason);
const char* ref_exercise_state_name(int32_t state);
const char* ref_result_name(int32_t result);

/* --- geometry, exposed because it is what the tests actually pin down --- */

/** Fills `out` with the four body corners, world frame, counter-clockwise. */
void   ref_body_corners(const RefCourse* c, double x, double y, double yaw,
                        double out[4][2]);
/** Shortest distance from point (px,py) to segment (x0,y0)-(x1,y1). */
double ref_point_segment_distance(double px, double py,
                                  double x0, double y0, double x1, double y1);
/** 1 if the two segments properly intersect. */
int32_t ref_segments_intersect(double ax0, double ay0, double ax1, double ay1,
                               double bx0, double by0, double bx1, double by1);
/** 1 if (px, py) is inside zone `z` (as loaded: convex, counter-clockwise). */
int32_t ref_zone_contains(const RefZone* z, double px, double py);
/** How far the car at this pose is INSIDE zone `z`: the smallest distance
 *  from any body corner to any zone edge, positive inside. Negative means a
 *  corner is over an edge. For a convex zone this is exact for the whole body:
 *  a rectangle is inside a convex polygon iff its four corners are. */
double  ref_zone_clearance(const RefCourse* c, const RefZone* z,
                           double x, double y, double yaw);

#ifdef __cplusplus
}
#endif

#endif /* REFEREE_H */
