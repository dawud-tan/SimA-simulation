/* ===========================================================================
 *  driver_source.h — the two input sources that are not the wheel.
 *
 *  PURE C11. No Unreal, no Chrono, no Eigen, no libstdc++. Same reasoning as
 *  wheel_device.h: this is arithmetic over doubles, so there is nothing here
 *  that an ABI wall would protect anyone from. Keeping it free of UE types is
 *  not about safety -- it is so the whole thing can be exercised by a host
 *  compiler in under a second, instead of by launching an editor that takes
 *  twenty minutes to reach its first Tick.
 *
 *  WHY THIS ONE IS IN Public/ WHEN wheel_device.h IS NOT:
 *  ChronoWheelInput.h keeps the device opaque -- it holds a `struct
 *  WheelDevice*` and never needs the definition. FChronoPhysicsThread holds a
 *  DrvKeyboard and a DrvScript BY VALUE, and C++ needs a complete type for
 *  that, so the definition has to reach a public header. Exposing it is safe
 *  for exactly the reason ChronoBridge.Build.cs gives for putting
 *  chrono_shim.h on the public include path: there is nothing in this file
 *  whose layout depends on -march or on which C++ runtime is in use. It is
 *  doubles and fixed-width integers. What must never become reachable from a
 *  dependent module is a CHRONO header, and this is not one.
 *
 *  WHY TWO SOURCES AND NOT ONE:
 *
 *  KEYBOARD exists because there is no wheel and will not be one until every
 *  milestone is done. It is for a human, and a human is not reproducible.
 *
 *  SCRIPT exists for the opposite reason. Every referee rule Milestone 6 adds
 *  -- cone strike, boundary crossing, stall, hill-start rollback -- is a
 *  predicate over the vehicle's trajectory. To regression-test a predicate you
 *  need the trajectory to be the SAME trajectory every run. That is a property
 *  of the input, not of the rule, and it is why the script is keyed to
 *  simulation time rather than published through a seqlock like the wheel is.
 *
 *  THE DETERMINISM ARGUMENT, SPELLED OUT:
 *  The wheel thread and the physics thread each keep their own 1 kHz deadline
 *  clock. They are not phase-locked and never will be -- hybrid spin gets
 *  wakeup jitter to 0.1 us mean but 28.5 us max, and 28.5 us of relative drift
 *  is enough that on some steps physics reads a value the wheel thread has
 *  already replaced, and on others it reads the same value twice. For a driver
 *  that is invisible. For a test it is fatal: a changed verdict could be the
 *  rule change you are testing, or it could be the scheduler. A script
 *  evaluated at t = step_index * dt on the physics thread has neither problem.
 *  Same table, same t, same inputs, every run, at any framerate, under any
 *  scheduling policy.
 *
 *  SIGN CONVENTION: `steer` here is the DEVICE convention, +1 = clockwise =
 *  driver's right, matching WheelState::steer. The single negation into
 *  Chrono's positive-left convention still happens in ChronoPhysicsThread.cpp
 *  and nowhere else.
 * ======================================================================== */
#ifndef DRIVER_SOURCE_H
#define DRIVER_SOURCE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- What every source produces ---------------------------------------- */

typedef struct {
    double  steer;      /* -1 .. +1, clockwise positive (device convention) */
    double  throttle;   /*  0 .. 1                                          */
    double  brake;      /*  0 .. 1                                          */
    double  clutch;     /*  0 .. 1, 1 = pedal down = disengaged             */
    int32_t gear;       /* -1 reverse, 0 neutral, 1..5 forward              */
    double  glance;     /* -1 left door mirror, 0 ahead, +1 right (task 17).
                           The HEAD, not the car: it never reaches Chrono.  */
} DrvInput;

/* =========================================================================
 *  KEYBOARD
 *
 *  The game thread publishes WHICH KEYS ARE HELD. It does not publish axis
 *  values, and this is the whole design.
 *
 *  A key is a boolean sampled once per rendered frame -- 16.7 ms apart, and
 *  irregularly so. Turning that into an axis on the game thread would produce
 *  a steering signal that steps between 0 and 1 at frame boundaries. Fed to
 *  the tyres at 1 kHz that is a steering RATE of 605 deg per millisecond, and
 *  the FFB damping term in the shim (which differentiates m_steering) would
 *  see a rate ~16000x anything a human hand can produce.
 *
 *  So the intent crosses at 60 Hz and the INTEGRATION happens at 1 kHz, on the
 *  physics thread. The axis is then continuous by construction, and its slope
 *  is a number chosen from how fast a person can actually move a limb.
 * ====================================================================== */

/* Held-key bitmask. Level-triggered for the analog channels, edge-triggered
 * for gear selection -- drv_keyboard_step does the edge detection, so the
 * publisher stays a dumb mirror of the keyboard and all the logic stays in
 * this file where a host compiler can reach it. */
#define DRV_KEY_LEFT      (1u <<  0)
#define DRV_KEY_RIGHT     (1u <<  1)
#define DRV_KEY_THROTTLE  (1u <<  2)
#define DRV_KEY_BRAKE     (1u <<  3)
#define DRV_KEY_CLUTCH    (1u <<  4)
#define DRV_KEY_CENTER    (1u <<  5)  /* return the rack to straight ahead   */
#define DRV_KEY_FINE      (1u <<  6)  /* modifier: quarter rate on every axis */
#define DRV_KEY_GEAR_N    (1u <<  7)
#define DRV_KEY_GEAR_R    (1u <<  8)
#define DRV_KEY_GEAR_1    (1u <<  9)
#define DRV_KEY_GEAR_2    (1u << 10)
#define DRV_KEY_GEAR_3    (1u << 11)
#define DRV_KEY_GEAR_4    (1u << 12)
#define DRV_KEY_GEAR_5    (1u << 13)
#define DRV_KEY_GLANCE_L  (1u << 14)  /* look at the left door mirror, held  */
#define DRV_KEY_GLANCE_R  (1u << 15)  /* look at the right door mirror, held */

/* Rates, in axis-units per second. Every one of these is [EST]: they describe
 * how fast a limb moves, and nothing in the vehicle model constrains them.
 *
 * STEER 1.2 /s is derived, not guessed. Full lock is 34.6 deg of road wheel
 * times the 17.5 steering ratio = 605 deg of rim, which is steer = 1.0. A
 * brisk hand-over-hand shuffle is about two turns per second, 720 deg/s, so
 * 720/605 = 1.19 axis-units per second. Centre to full lock in 0.83 s.
 *
 * The pedals release faster than they press because a return spring is doing
 * the work in one direction and a leg in the other. */
#define DRV_RATE_STEER        1.2
#define DRV_RATE_CENTER       2.0   /* only while DRV_KEY_CENTER is held     */
#define DRV_RATE_THROTTLE_UP  4.0   /* 0 -> 1 in 0.25 s                      */
#define DRV_RATE_THROTTLE_DN  8.0
#define DRV_RATE_BRAKE_UP     5.0   /* brakes get stamped, not squeezed      */
#define DRV_RATE_BRAKE_DN     8.0
#define DRV_RATE_CLUTCH_UP    6.0   /* dipping the clutch is fast            */

/* THE ONE THAT MATTERS, AND THE ONE THAT CANNOT BE VALIDATED HERE.
 *
 * Clutch ENGAGEMENT rate is what makes a hill start hard. 2.0 /s is a 0.5 s
 * full-travel release, which is ordinary driving; with DRV_KEY_FINE held it
 * becomes 0.5 /s, a two-second release that lets the bite point be searched
 * for. Whether that is difficult in the RIGHT WAY is a question about pedal
 * ergonomics and it needs a real clutch pedal to answer -- CLAUDE.md is
 * explicit that hill-start difficulty must not be tuned blind. This number is
 * therefore chosen to be usable, not to be correct, and it is left alone. */
#define DRV_RATE_CLUTCH_DN    2.0

/* GLANCE (task 17). Held to look, released to look back: a head turn, not a
 * setting. 4 /s is 0.25 s from ahead to a mirror, [EST] from a driver's
 * mirror check; through drv_glance_ease that peaks at 1.5x the mean angular
 * speed, ~330 deg/s for the left mirror's 55 deg, inside what a head does.
 * The fine modifier does not slow it: it is for limbs on controls. */
#define DRV_RATE_GLANCE       4.0

/* Fine-modifier multiplier. A quarter, not a half: the useful range for
 * parallel parking is small steering corrections held for seconds, and half
 * rate is still too coarse to place the rack where you want it. */
#define DRV_FINE_SCALE        0.25

typedef struct {
    DrvInput out;        /* the integrated axes; read this after each step   */
    uint32_t prev_keys;  /* for gear edge detection                          */
    int      primed;     /* 0 until the first step, so the first frame's
                            already-held gear key does not fire an edge      */
} DrvKeyboard;

/** Zeroes the integrator and selects neutral. */
void drv_keyboard_init(DrvKeyboard* kb);

/** One step of the glance axis toward `dir` (-1 left, 0 ahead, +1 right) at
 *  DRV_RATE_GLANCE. The keyboard and the wheel's D-pad both use it: each is
 *  a held direction, so each moves the head the same way. */
double drv_glance_step(double glance, int32_t dir, double dt);

/** Eases a glance for the camera: 0 -> 0, +-1 -> +-1, odd, monotonic, and
 *  flat at both ends (smoothstep on |glance|), so the head starts and stops
 *  without a jolt whatever the source -- a ramped key or a script's linear
 *  segment alike. */
double drv_glance_ease(double glance);

/** Advances every axis by one physics step. Call at the STEP rate (1 kHz),
 *  not at the rate keys arrive -- that is the entire point. `keys` is simply
 *  the newest bitmask; repeating a stale one for a few steps is correct
 *  behaviour and produces continued motion, exactly as a held key should. */
void drv_keyboard_step(DrvKeyboard* kb, uint32_t keys, double dt);

/* =========================================================================
 *  SCRIPT
 *
 *  A keyframe table, evaluated at t = step_index * dt.
 *
 *  WHY THE ANALOG CHANNELS INTERPOLATE AND GEAR DOES NOT:
 *  Interpolation here is not smoothing for looks. A step change in steering
 *  between two 1 ms samples is a physically impossible input -- it commands an
 *  infinite steering rate, drives an unbounded value into the shim's damping
 *  term, and slams a lateral slip transient into TMeasy that no hand could
 *  produce. A script full of steps would be testing the tyre model's response
 *  to nonsense. Linear ramps describe what limbs actually do.
 *
 *  Gear is the exception because gear is genuinely discrete: there is no
 *  physical state between 2nd and 3rd, and Chrono's SetGear takes an int. It
 *  holds the previous keyframe's value until the next keyframe is reached.
 * ====================================================================== */

typedef struct {
    double  t;          /* seconds from script start, must be non-decreasing */
    double  steer, throttle, brake, clutch;
    int32_t gear;
    double  glance;     /* optional 7th column; interpolates like the pedals */
} DrvKeyframe;

typedef struct {
    const DrvKeyframe* kf;
    int32_t            n;
    double             duration;   /* kf[n-1].t                             */
    int32_t            owns_kf;    /* 1 if drv_script_free must free kf      */
} DrvScript;

/** Evaluates the script at time `t`. Returns 0 while running, 1 once t is past
 *  the last keyframe -- at which point `out` holds that last keyframe's values
 *  unchanged. Holding rather than zeroing matters: a script that ends with the
 *  car rolling should leave it rolling, not slam every pedal to zero and
 *  change the trajectory the referee is about to be asked about. */
int32_t drv_script_eval(const DrvScript* s, double t, DrvInput* out);

/** Compiled-in scripts, so the input path can be exercised before any course
 *  or asset exists. Returns 0 on success, -1 if the name is unknown.
 *  Never allocates; `owns_kf` comes back 0. */
int32_t drv_script_builtin(const char* name, DrvScript* out);

/** Names of the built-ins, NULL-terminated. For usage messages. */
const char* const* drv_script_builtin_names(void);

/** Loads a keyframe table from a text file. One keyframe per line:
 *
 *      # comment
 *      t  steer throttle brake clutch gear [glance]
 *      0.0   0.0  0.00   0.0   1.0   1
 *      2.0   0.0  0.35   0.0   0.0   1   -1.0
 *
 *  `glance` (task 17) is optional per line and 0 when absent, so every
 *  script written before it loads unchanged.
 *
 *  Plain whitespace-separated doubles: no JSON, because this file is read by
 *  the physics thread's Init() and by a C test harness, and neither should
 *  need a parser dependency to run a regression.
 *
 *  Returns 0 on success. On failure returns -1 and writes a message naming the
 *  offending LINE NUMBER into `err` -- the Chrono JSON readers' habit of
 *  segfaulting on a malformed file is exactly the experience not to repeat. */
int32_t drv_script_load(DrvScript* s, const char* path, char* err, size_t errsz);

/* =========================================================================
 *  LANE KEEPING
 *
 *  The residual zero-steer drift is REAL PHYSICS, not a defect: drive torque
 *  reacts as a roll moment on the chassis, roll gives roll steer, and a
 *  solid-axle RWD car curves under power. It is ~0.3 m over 30 m at ~1590 m
 *  radius and it is not going away.
 *
 *  That is fine for a human, who steers. It is not fine for an open-loop
 *  scripted run: over 122 m the offset reaches 4.7 m, which is how the terrain
 *  test first "proved" the car fell through a hill it had actually driven off
 *  the side of. A referee regression built on open-loop scripts would measure
 *  the drift rather than the rule under test.
 *
 *  So reference runs close the loop. This is a proportional controller on
 *  lateral offset and heading -- the minimum that holds a lane.
 *
 *  DETERMINISM IS PRESERVED. A script is a pure function of TIME; this is a
 *  pure function of STATE. The state comes from a simulation that is already
 *  bit-reproducible (verified: 12,500 steps, in-process and cross-process), so
 *  closing the loop over it stays bit-reproducible too. Nothing here reads a
 *  clock.
 *
 *  SIGN: returns a steering command in the DEVICE convention, + = clockwise =
 *  the driver's right = toward -Y. A car that has drifted to +Y (its left) has
 *  a positive lateral error and needs a positive, i.e. rightward, correction.
 * ====================================================================== */

typedef struct {
    double k_lateral;   /* steer per metre of lateral error                  */
    double k_heading;   /* steer per radian of heading error                 */
    double limit;       /* clamp on the output, absolute                     */
} DrvLaneKeep;

/** Sensible defaults for the Innova at test speeds. Tuned to settle without
 *  overshoot rather than to be quick; a reference run is not a lap record. */
#define DRV_LANEKEEP_DEFAULT { 0.25, 2.0, 0.4 }

/** @param lateral_err  y_actual - y_target, metres (+ = car is to the LEFT)
 *  @param heading_err  yaw_actual - yaw_target, radians (+ = nose to the LEFT)
 *  @return steering in device convention, clamped to +-limit. */
double drv_lane_keep(const DrvLaneKeep* lk, double lateral_err, double heading_err);

/** Frees a loaded script. Safe on a built-in (does nothing) and on a zeroed
 *  struct. The allocator that malloc'd the table is the one that frees it. */
void drv_script_free(DrvScript* s);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* DRIVER_SOURCE_H */
