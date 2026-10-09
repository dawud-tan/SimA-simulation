/* ===========================================================================
 *  test_shim — exercises the Milestone 5 vehicle entry points.
 *
 *  Deliberately compiled as C, not C++, and linked against nothing but
 *  libchrono_shim.so. That is itself the strongest available test of the ABI
 *  wall: a C translation unit has no libstdc++, no libc++, no exceptions and
 *  no Eigen, so if this program can build a full Chrono vehicle, step it, and
 *  read forces back, then nothing C++ is leaking across the boundary.
 *
 *    assemble  build the vehicle, settle it, report static state
 *    sweep     accelerate, then ramp steering -- shows the self-aligning
 *              torque rising, peaking, and collapsing as the tyres saturate
 *    mzlow     the same signal at PARKING speed, sweeping the m_frblend_begin
 *              gate below which ChTMeasyTire computes no Mz at all
 *    clutch    hill-start-style clutch modulation from rest
 *    diff      is the rear differential open? (task 13: under MINRES, no)
 *    perf      step-cost distribution with the vehicle behind the wall
 *    determ    THE MILESTONE 6 PRECONDITION: run a scripted driver twice and
 *              prove the trajectory is bit-identical. Every referee rule is a
 *              predicate over a trajectory, so a rule change is only
 *              distinguishable from noise if the trajectory is reproducible.
 *    judge     script -> physics -> referee -> verdict; `judge park_*` runs a
 *              parking manoeuvre, `judge course*` the whole test (task 4)
 * ======================================================================== */
#include "chrono_shim.h"
#include "driver_source.h"
#include "course_terrain.h"
#include "referee.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static double now_us(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return t.tv_sec * 1e6 + t.tv_nsec / 1e3;
}

static int cmp_d(const void* a, const void* b) {
    double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}

/* The step. 1 ms is the 1 kHz loop's; SHIM_DT overrides it for a step-size
 * experiment only (task 6: is a steady flat-ground push physics, or the
 * integrator's?). Everything else in this file is measured at 1 ms. */
static double DT = 1e-3;

/* FNV-1a over raw bytes. Used to fingerprint a step's body states.
 *
 * A hash rather than a stored trajectory because we want the FIRST diverging
 * step, not a diff of 12000 poses: divergence in a coupled solve is
 * exponential, so by the end everything differs and the interesting number is
 * where it started. */
static uint64_t fnv1a(const void* p, size_t n, uint64_t h) {
    const unsigned char* b = (const unsigned char*)p;
    for (size_t i = 0; i < n; ++i) { h ^= b[i]; h *= 1099511628211ULL; }
    return h;
}
#define FNV_SEED 1469598103934665603ULL

/* Hashes ONLY the body states.
 *
 * ChsSnapshot also carries last_step_us, which is a wall-clock measurement and
 * therefore differs on every run by construction. Hashing the whole struct
 * would make this test fail 100% of the time for a reason that has nothing to
 * do with the simulation. sim_time and step_index are deterministic but add
 * nothing -- they are the loop counter. */
static uint64_t hash_bodies(const ChsSnapshot* s) {
    return fnv1a(s->bodies, sizeof(ChsBodyState) * (size_t)s->body_count, FNV_SEED);
}

static ChsWorld make(const char* path) {
    printf("abi %d (header %d), chrono ", Chs_GetAbiVersion(), CHRONO_SHIM_ABI_VERSION);
    char v[64]; Chs_GetChronoVersion(v, sizeof v); printf("%s\n", v);
    if (Chs_GetAbiVersion() != CHRONO_SHIM_ABI_VERSION) {
        printf("ABI MISMATCH -- rebuild the shim\n"); return NULL;
    }
    ChsWorld w = Chs_CreateVehicle(path, 0.45);
    if (!w) { printf("Chs_CreateVehicle failed\n"); return NULL; }
    return w;
}

/* Drives with fixed inputs for `secs`, returning the last feedback. */
static void run_(ChsWorld w, ChsDriverInput in, double secs, ChsVehicleFeedback* fb) {
    Chs_SetDriverInput(w, in);
    const long n = (long)(secs / DT);
    for (long i = 0; i < n; ++i) Chs_Step(w, DT);
    Chs_GetFeedback(w, fb);
}

/* Resolve a script by built-in name, literal path, or bare name in the staged
 * scripts directory -- so `drift coast` works as well as `drift pullaway`. */
static int resolve_script(const char* name, DrvScript* sc, char* err, size_t errsz) {
    if (drv_script_builtin(name, sc) == 0) return 0;
    if (drv_script_load(sc, name, err, errsz) == 0) return 0;
    char buf[512];
    snprintf(buf, sizeof buf, "/work/driver/scripts/%s.txt", name);
    if (drv_script_load(sc, buf, err, errsz) == 0) return 0;
    snprintf(buf, sizeof buf, "/work/driver/scripts/%s", name);
    if (drv_script_load(sc, buf, err, errsz) == 0) return 0;
    return -1;
}


static int do_assemble(const char* path) {
    ChsWorld w = make(path); if (!w) return 1;
    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};      /* clutch down, neutral */
    ChsVehicleFeedback fb;
    run_(w, in, 1.0, &fb);                          /* settle under gravity */

    ChsSnapshot s; int n = Chs_GetSnapshot(w, &s);
    printf("\n=== VEHICLE THROUGH THE SHIM ===\n");
    printf("  bodies published : %d (chassis + 4 spindles)\n", n);
    printf("  chassis z        : %.4f m\n", s.bodies[0].position.z);
    printf("  front axle load  : %.1f N (FL) + %.1f N (FR) = %.1f N\n",
           fb.tire_fz_n[0], fb.tire_fz_n[1], fb.tire_fz_n[0] + fb.tire_fz_n[1]);
    printf("  front grounded   : 0x%x (bit0=FL bit1=FR)\n", fb.front_grounded);
    printf("  engine           : %.0f rpm, gear %d\n", fb.engine_rpm, fb.gear);
    printf("  column torque    : %+.4f Nm  (straight ahead: expect ~0)\n",
           fb.column_torque_nm);
    printf("  rim torque       : %+.4f Nm\n", fb.steer_torque_nm);
    /* The blend range is a shim default rather than a caller argument, so the
     * basic report is where it has to be visible -- otherwise the one number
     * that decides whether the steering works below 1 m/s is invisible in
     * every run that does not go looking for it. */
    double g0 = -1, g1 = -1, gb = -1, ge = -1;
    if (Chs_GetTireStandstill(w, &g0, &g1, &gb, &ge) == 0)
        printf("  tyre blend range : %.2f .. %.2f m/s (Chrono stock 1.0 .. 3.0),"
               " Dahl sigma %.0f / %.0f\n", gb, ge, g0, g1);
    Chs_DestroyWorld(w);
    return 0;
}

static int do_sweep(const char* path, double blend_begin, double blend_end) {
    ChsWorld w = make(path); if (!w) return 1;
    /* No damping for the sweep: it is a quasi-static measurement and the rate
     * term would just add a constant offset proportional to the ramp speed. */
    Chs_SetFfbTuning(w, 0.7, 0.0, 1.0);

    /* THE M5 BASELINE REGRESSION for mz-below-1ms. Raising blend_end to damp
     * the low-speed ripple leaves the Dahl bristle model contributing at
     * speeds this sweep runs at, and the Dahl model is pure Coulomb -- it has
     * no slip curve, so it cannot produce the peak-then-fall this measurement
     * exists to show. If the trail collapse disappears here, the blend_end
     * that bought the smoothness is not adoptable however good it looked at
     * 1.8 m/s. 0 leaves the shim default; pass 1.0 and 3.0 for stock. */
    if (blend_begin > 0.0) {
        Chs_SetTireStandstill(w, 0.0, 0.0, blend_begin, blend_end);
        double gb = -1, ge = -1;
        Chs_GetTireStandstill(w, NULL, NULL, &gb, &ge);
        printf("  [blend range applied: %.2f .. %.2f m/s]\n", gb, ge);
    }

    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
    ChsVehicleFeedback fb;
    run_(w, in, 0.5, &fb);

    printf("\n--- accelerating in 1st (clutch engaged, 80%% throttle) ---\n");
    in.gear = 1; in.clutch = 0.0; in.throttle = 0.8;
    Chs_SetDriverInput(w, in);
    for (long i = 0; i < 12000; ++i) {
        Chs_Step(w, DT);
        if ((i % 2000) == 0) {
            Chs_GetFeedback(w, &fb);
            printf("    t=%4.1fs  v=%5.2f m/s  %5.0f rpm\n",
                   i * DT, fb.speed_mps, fb.engine_rpm);
        }
        Chs_GetFeedback(w, &fb);
        if (fb.speed_mps > 8.0) break;
    }
    printf("    reached %.2f m/s (%.0f km/h)\n", fb.speed_mps, fb.speed_mps * 3.6);

    printf("\n=== STEERING RAMP: self-aligning torque vs steer ===\n");
    printf("  This is the cue force feedback exists to deliver. Mz must RISE,\n");
    printf("  PEAK, then FALL while the wheel is still being turned further --\n");
    printf("  pneumatic trail collapsing as the rear of the contact patch\n");
    printf("  starts to slide. That is the wheel going light before the front\n");
    printf("  washes out.\n\n");
    printf("  %-7s %-9s %-10s %-10s %-11s %-9s %-6s %-9s %s\n",
           "steer", "road(deg)", "Mz_FL(Nm)", "Mz_FR(Nm)", "column(Nm)", "rim(Nm)",
           "v(m/s)", "ay_kin(g)", "ay(g)");

    in.throttle = 0.25;
    /* Two lateral accelerations. ay_kin = v^2 tan(delta)/L is what the car
     * WOULD pull if it followed the steered path without slip -- a command,
     * not a measurement. ay = v * yaw rate is what it actually pulls. Until
     * task 14 only ay_kin existed, and "past grip" was ay_kin > 0.85 g: a
     * statement about the steering wheel, not the tyres. With the CG put at
     * its real height the car grips later, and the flag landed on top of the
     * torque peak and hid it. Past grip is now MEASURED: the car's own
     * lateral acceleration has stopped rising while the steer keeps going.
     * WHEELBASE is gen_innova.py's 2.75 m. */
    const double WHEELBASE = 2.75, G = 9.81;
    double peak_col = 0.0, peak_at = 0.0, ay_max = 0.0, ay_max_at = 0.0, col_at_ay_max = 0.0;
    int    first_past_limit = -1;

    for (int k = 0; k <= 20; ++k) {
        const double steer = k / 20.0;
        in.steering = steer;                     /* positive = LEFT */
        run_(w, in, 0.25, &fb);
        ChsSnapshot snap;
        Chs_GetSnapshot(w, &snap);               /* chassis omega is in the LOCAL frame */

        const double delta = fb.road_steer_rad[0];
        const double ay_kin = fb.speed_mps * fb.speed_mps * tan(fabs(delta)) / WHEELBASE;
        const double ay = fabs(fb.speed_mps * snap.bodies[0].angular_velocity.z);
        if (ay > ay_max) { ay_max = ay; ay_max_at = steer; col_at_ay_max = fabs(fb.column_torque_nm); }
        const int past = first_past_limit >= 0 || (k > 0 && ay < 0.98 * ay_max);
        if (past && first_past_limit < 0) first_past_limit = k;

        const double col = fabs(fb.column_torque_nm);
        /* Peak of the PRE-LIMIT branch only. Scanning the whole sweep would
         * just find the largest numerical excursion after the car lets go. */
        if (!past && col > peak_col) { peak_col = col; peak_at = steer; }
        printf("  %-7.2f %-9.2f %-10.1f %-10.1f %-11.3f %-9.3f %-6.2f %-9.2f %.2f%s\n",
               steer, delta * 180.0 / M_PI,
               fb.tire_mz_nm[0], fb.tire_mz_nm[1],
               fb.column_torque_nm, fb.steer_torque_nm, fb.speed_mps,
               ay_kin / G, ay / G, past ? "   <-- past grip" : "");
    }
    printf("\n  column torque peaks at steer=%.2f, |%.3f| Nm at the rim ratio.\n",
           peak_at, peak_col);
    printf("  the car's lateral acceleration peaks at steer=%.2f, %.2f g.\n",
           ay_max_at, ay_max / G);
    if (first_past_limit >= 0)
        printf("  car past its grip (ay falling while steer rises) from steer=%.2f.\n",
               first_past_limit / 20.0);
    /* The cue is a torque that PEAKS AND FALLS WHILE THE CAR IS STILL GAINING
     * GRIP: the driver feels the wheel go light before the car stops turning.
     * (Until task 14 this asked for a 40% fall before a kinematic "limit",
     * which is a statement about the steering wheel, not the tyres.) */
    const double fall = peak_col > 0.0 ? 1.0 - col_at_ay_max / peak_col : 0.0;
    if (peak_at < ay_max_at && fall >= 0.10)
        printf("  TRAIL COLLAPSE PRESENT: the torque peaks at steer %.2f and has fallen\n"
               "  %.0f%% by the time the car's grip peaks at %.2f. That is the wheel going\n"
               "  light before the front washes out -- the cue the whole force-feedback\n"
               "  chain exists to deliver.\n", peak_at, 100.0 * fall, ay_max_at);
    else
        printf("  NO CUE: the torque does not fall before the car's grip peaks\n"
               "  (peak at %.2f, grip at %.2f, %.0f%% fallen).\n",
               peak_at, ay_max_at, 100.0 * fall);
    Chs_DestroyWorld(w);
    return 0;
}

/* ===========================================================================
 *  mzlow -- steering feel at PARKING speed.  (backlog task 1: mz-below-1ms)
 *
 *  ChTMeasyTire::Advance, line 254 of the stock source:
 *
 *      double Mz = 0;
 *      if (m_data.vel.x() >= m_frblend_begin) {
 *          Mz = AlignmentTorque(Fy);
 *      }
 *
 *  m_frblend_begin is 1.0 m/s. Every SIM A parking manoeuvre -- both parking
 *  exercises and the tight end of the slalom -- runs UNDER that, so the
 *  aligning moment is identically zero and the wheel goes dead in exactly the
 *  regime the test lives in. `sweep` never showed this because it measures at
 *  8 m/s, eight times above the gate.
 *
 *  The same constant also opens the Fx/Fy blend into the Dahl standstill
 *  model (line 214), so moving it is NOT free -- it changes where the bristle
 *  model applies, which is the hill start and the creep. That is why this
 *  mode reads the applied value back and why hill/creep take a blend argument.
 *
 *  Two things are measured here and they are different questions:
 *
 *    A. WHERE THE DEAD ZONE ENDS. A fixed parking lock is held while a P
 *       controller walks the car up through the low-speed band. Mz is 0 below
 *       the gate and a step appears at it. Lowering the gate moves the step
 *       down; it does not remove it, and the size of that step is a torque
 *       discontinuity the driver feels as a click.
 *
 *    B. HOW MUCH TORQUE IS ACTUALLY THERE once the gate is out of the way.
 *       A force-element tyre generates Mz from lateral SLIP, and a slow turn
 *       is nearly kinematic -- slip angles go to zero as speed does. So the
 *       recoverable torque at 0.3 m/s is small no matter where the gate sits.
 *       Chrono's own note ("no parking slip calculations, no dynamic parking
 *       torque") is the reason: real parking effort is mostly tyre scrub
 *       against the ground, which this model does not have. Measure it rather
 *       than assume it.
 * ======================================================================== */

typedef struct {
    double col_nm;      /* column torque, mean over the measurement window   */
    double rim_nm;      /* after ratio + assist                              */
    double mz_fl, mz_fr;
    double v;           /* speed actually settled at                         */
    double road_deg;    /* FL road-wheel angle                               */
    /* A mean alone cannot answer the question this task asks. "Restore
     * steering FEEL" fails just as badly if the torque is present but
     * chattering: a constant-force channel reproduces the ripple, and a wheel
     * that buzzes at 3 Nm is worse than one that is dead. p2p is the swing
     * over the window; max_jump is the largest change between two consecutive
     * 1 kHz samples, which is what the 250 Hz FFB transmit path turns into a
     * step at the rim. */
    double col_p2p;
    double col_max_jump;
    /* THE CONFOUND. The speed here is held by a proportional controller, and a
     * controller hunting between throttle and brake would put a real
     * oscillation into the tyre -- which would then be reported as tyre
     * chatter. These two say whose ripple it is: if the pedals and the speed
     * are steady and the torque is not, the ripple is the tyre's. */
    double thr_p2p;
    double v_p2p;
} MzPoint;

/* ON THE CLUTCH BELOW IDLE SPEED (task 8). The engine has a governor now,
 * so in 1st or reverse with the pedal up the car idles along at ~1.6 m/s.
 * A driver goes slower on the CLUTCH: throttle off, the governor holding
 * idle, the pedal near the bite so the clutch slips and passes only the
 * torque the crawl needs. It passes (1 - pedal) x 600 N m while slipping, so
 * the crawl lives in the last few percent of travel: ~0.7% of capacity on
 * the flat, ~7% to hold 12%. A linear pedal map is [EST], and its feel is
 * the real pedal's question (task 12), not this loop's.
 *
 * (Before the governor the loop drove on the throttle with the pedal up at
 * every speed, and every launch dragged the engine from ~0 rpm; a throttle
 * floor, PILOT_TH_LAUNCH 0.45 for the first second, kept that from being
 * scored a STALL. On the clutch the engine never leaves idle, and the floor
 * is gone.) */
#define PILOT_IDLE_RPM  780.0  /* governed idle under a crawl's load (783 free) */
#define PILOT_C_KP      0.30   /* clutch capacity per m/s of speed error        */
#define PILOT_C_KI      0.60   /* ... per m of integrated error                 */
#define PILOT_C_IMAX    0.40   /* integrator ceiling, capacity / KI             */

static double pilot_v_idle(int gear)
{
    const double ratio = (gear < 0 ? 4.220 : 4.313) * 3.909;   /* gen_innova.py */
    return PILOT_IDLE_RPM * 2.0 * M_PI / 60.0 / ratio * 0.3264;
}

/* The speed loop mz_point and park_turn share: on the clutch below idle
 * speed, on the throttle above it (task 8). A negative target is driven in
 * reverse. Returns the net pedal (clutch capacity or throttle, minus brake). */
typedef struct { double target; int on_clutch; double integ; } SpeedLoop;

static SpeedLoop speed_loop(double v_target)
{
    const int gear = v_target < 0.0 ? -1 : 1;
    SpeedLoop L = { v_target, fabs(v_target) < 0.85 * pilot_v_idle(gear), 0.0 };
    return L;
}

static double speed_loop_step(SpeedLoop* L, double speed, ChsDriverInput* in)
{
    const double dir = L->target < 0.0 ? -1.0 : 1.0;
    const double e = dir * (L->target - speed);       /* > 0: too slow */
    in->gear = dir < 0.0 ? -1 : 1;
    if (L->on_clutch) {
        L->integ = fmax(-0.05, fmin(PILOT_C_IMAX, L->integ + e * DT));
        const double c = fmax(0.0, fmin(1.0, PILOT_C_KP * e + PILOT_C_KI * L->integ));
        in->clutch = 1.0 - c;
        in->throttle = 0.0;
    } else {
        in->clutch = 0.0;
        /* Proportional, with a small standing throttle so the car does not
         * park itself at zero error against rolling resistance. */
        in->throttle = e > 0.0 ? fmin(0.55, 1.2 * e + 0.06) : 0.0;
    }
    /* Brake is capped WELL BELOW 0.8 on purpose: that is Chs_SetParkingHold's
     * kBrakeOn threshold, and the parking hold applies a chassis force that
     * would sit directly on top of the quantity being measured. (Off by
     * default since task 15; the cap keeps this independent of that.) */
    in->braking = e < -0.02 ? fmin(0.35, -1.5 * e) : 0.0;
    /* Net pedal: brake counts as negative throttle, so a controller
     * flipping between the two shows up as one large swing rather than two
     * small ones. On the clutch, the clutch's capacity is the pedal. */
    return (L->on_clutch ? 1.0 - in->clutch : in->throttle) - in->braking;
}

/* Holds `v_target` against a fixed steering lock, then averages the torque
 * over the last second. Steady state, not a transient: the FFB damping term
 * differentiates steering and an accelerating car would put a speed ramp into
 * the tyre's slip as well, so a sampled ramp would confound both. */
static MzPoint mz_point(ChsWorld w, double steer, double v_target) {
    MzPoint P; memset(&P, 0, sizeof P);
    ChsDriverInput in = {0, 0, 0, 0.0, 1, 0};      /* 1st, clutch engaged */
    ChsVehicleFeedback fb;
    in.steering = steer;

    const double SETTLE = 4.0, WINDOW = 1.0;
    const long n_settle = (long)(SETTLE / DT), n_win = (long)(WINDOW / DT);
    long samples = 0;
    double col_min = 0.0, col_max = 0.0, col_prev = 0.0;
    double thr_min = 0.0, thr_max = 0.0, v_min = 0.0, v_max = 0.0;
    /* Below idle speed in 1st, on the CLUTCH (task 8): the governor holds the
     * engine at idle, so the throttle cannot take the car under ~1.6 m/s (1.2
     * at this lock, the tyres scrubbing). Until task 8 fixed the idle, the
     * throttle loop crawled at any speed; after it, every probe under 1.2
     * m/s silently ran at 1.16. Same law as the pilot's. */
    SpeedLoop L = speed_loop(v_target);

    for (long i = 0; i < n_settle + n_win; ++i) {
        Chs_GetFeedback(w, &fb);
        const double net = speed_loop_step(&L, fb.speed_mps, &in);
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);

        if (i >= n_settle) {
            Chs_GetFeedback(w, &fb);
            P.col_nm += fb.column_torque_nm;
            P.rim_nm += fb.steer_torque_nm;
            P.mz_fl  += fb.tire_mz_nm[0];
            P.mz_fr  += fb.tire_mz_nm[1];
            P.v      += fb.speed_mps;
            P.road_deg += fb.road_steer_rad[0] * 180.0 / M_PI;
            const double c = fb.column_torque_nm;
            if (samples == 0) {
                col_min = col_max = col_prev = c;
                thr_min = thr_max = net;
                v_min = v_max = fb.speed_mps;
            }
            if (net < thr_min) thr_min = net;
            if (net > thr_max) thr_max = net;
            if (fb.speed_mps < v_min) v_min = fb.speed_mps;
            if (fb.speed_mps > v_max) v_max = fb.speed_mps;
            if (c < col_min) col_min = c;
            if (c > col_max) col_max = c;
            const double jump = fabs(c - col_prev);
            if (jump > P.col_max_jump) P.col_max_jump = jump;
            col_prev = c;
            ++samples;
        }
    }
    if (samples > 0) {
        P.col_nm /= samples; P.rim_nm /= samples;
        P.mz_fl  /= samples; P.mz_fr  /= samples;
        P.v      /= samples; P.road_deg /= samples;
        P.col_p2p = col_max - col_min;
        P.thr_p2p = thr_max - thr_min;
        P.v_p2p   = v_max - v_min;
    }
    return P;
}

/* Speeds to probe, ascending so one world can walk up through them without
 * ever having to brake hard. 0.15 is below every candidate gate; 2.0 is above
 * the stock one, so the stock row still has a non-zero entry to compare to. */
/* 3.0 and 5.0 are here because of blend_end, not because of Mz. Raising
 * blend_end to damp the low-speed ripple leaves the Dahl bristle model
 * contributing right across the manoeuvring range, and `sweep` only covers
 * 7-8 m/s. Without these two rows the mid band would be adopted against an
 * absence of measurement rather than a measurement. */
static const double kMzSpeeds[] = { 0.15, 0.30, 0.50, 0.80, 1.20, 2.00, 3.00, 5.00 };
#define N_MZ_SPEEDS ((int)(sizeof kMzSpeeds / sizeof kMzSpeeds[0]))

/* The parking lock. Not full lock: `sweep` showed the tyres let go past
 * steer ~0.70 at speed, and while that limit is far away at 0.5 m/s, a value
 * inside the measured-good band keeps this table comparable with that one. */
static const double kParkSteer = 0.60;

/* blend_end is a SECOND, independent knob and the sweep above holds it at
 * Chrono's stock 3.0. The one constant m_frblend_begin does two jobs, but the
 * PAIR does three: begin gates Mz, and the begin..end width sets how much of
 * the Dahl bristle force -- and with it sigma1's viscous damping -- survives
 * at any given speed. So the ripple that lowering begin lets through can in
 * principle be bought back by RAISING end, without re-gating Mz. Whether it
 * actually is, is a measurement. */
/* ===========================================================================
 *  The parking torque (task 16) -- turning the wheel, not holding it.
 *
 *  The table below holds a steady lock, and a steady lock is the one case a
 *  twist model leaves alone once the car rolls: the twist relaxes and only
 *  TMeasy's Mz is left. What the parking torque is FOR is the wheel being
 *  TURNED: at a standstill, and while crawling. So each row holds a speed,
 *  turns the wheel 0 -> kParkSteer at kParkRate, holds it, then turns it
 *  back, once with CHS_PARKING_TORQUE=0 (the wheel before task 16, same
 *  code) and once with it on.
 *
 *    turning  column torque, mean over the second half of the turn
 *    held     ... over 0.4-0.5 s after the wheel stops
 *    steady   ... over 4-5 s after it stops: what the table below measures
 *    back     ... over the second half of the turn back to 0
 *    left@0   ... over 0.4-0.5 s after reaching 0: twist left in the patch
 * ======================================================================== */
#define STEER_RATIO 17.5      /* the shim's kSteerRatio */
static const double kParkRate = 0.30;   /* steer units/s: ~10.6 deg/s of road
                                           wheel, ~184 deg/s at the rim */

typedef struct { double v, turning, held, steady, back, left0, park_turn; } TurnPoint;

/* Mean of the column torque over the steps [from, to) of a phase. */
typedef struct { double sum; long n; } Mean;
static void mean_add(Mean* m, double x) { m->sum += x; ++m->n; }
static double mean_of(const Mean* m) { return m->n ? m->sum / m->n : 0.0; }

static TurnPoint park_turn(const char* path, double v_target, int park_on,
                           double blend_begin, double blend_end)
{
    TurnPoint T; memset(&T, 0, sizeof T);
    /* Off or on for THIS world only; the shim reads it at creation. */
    const char* was = getenv("CHS_PARKING_TORQUE");
    char saved[16] = "";
    if (was) snprintf(saved, sizeof saved, "%s", was);
    if (park_on) unsetenv("CHS_PARKING_TORQUE"); else setenv("CHS_PARKING_TORQUE", "0", 1);
    ChsWorld w = Chs_CreateVehicle(path, 0.45);
    if (was) setenv("CHS_PARKING_TORQUE", saved, 1); else unsetenv("CHS_PARKING_TORQUE");
    if (!w) { printf("  Chs_CreateVehicle failed\n"); return T; }
    Chs_SetFfbTuning(w, 0.7, 0.0, 1.0);        /* no rate damping: same as below */
    if (blend_begin > 0.0) Chs_SetTireStandstill(w, 0.0, 0.0, blend_begin, blend_end);

    ChsDriverInput in = {0, 0, 0, 1.0, 1, 0};
    ChsVehicleFeedback fb;
    run_(w, in, 0.5, &fb);                     /* settle on the ground */

    /* Standstill: clutch down, brake on. Otherwise the speed loop. */
    SpeedLoop L = speed_loop(v_target);
    const int parked = v_target == 0.0;
    const long n_turn = (long)(kParkSteer / kParkRate / DT);
    const long n_settle = (long)(4.0 / DT), n_hold = (long)(5.0 / DT), n_back = (long)(0.5 / DT);
    Mean m_turn = {0}, m_held = {0}, m_steady = {0}, m_back = {0}, m_left = {0}, m_park = {0}, m_v = {0};

    /* phases: settle at 0, turn out, hold, turn back, hold at 0 */
    const long p1 = n_settle, p2 = p1 + n_turn, p3 = p2 + n_hold, p4 = p3 + n_turn, p5 = p4 + n_back;
    for (long i = 0; i < p5; ++i) {
        Chs_GetFeedback(w, &fb);
        if (parked) { in.clutch = 1.0; in.throttle = 0.0; in.braking = 0.5; in.gear = 1; }
        else speed_loop_step(&L, fb.speed_mps, &in);
        if (i < p1)      in.steering = 0.0;
        else if (i < p2) in.steering = kParkRate * (i - p1 + 1) * DT;
        else if (i < p3) in.steering = kParkSteer;
        else if (i < p4) in.steering = kParkSteer - kParkRate * (i - p3 + 1) * DT;
        else             in.steering = 0.0;
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);
        Chs_GetFeedback(w, &fb);
        const double c = fb.column_torque_nm;
        const double park = c - (fb.tire_mz_nm[0] + fb.tire_mz_nm[1]) / STEER_RATIO;
        if (i >= p1 + n_turn / 2 && i < p2) { mean_add(&m_turn, c); mean_add(&m_park, park); mean_add(&m_v, fb.speed_mps); }
        if (i >= p2 + 400 && i < p2 + 500)  mean_add(&m_held, c);
        if (i >= p2 + 4000 && i < p3)       mean_add(&m_steady, c);
        if (i >= p3 + n_turn / 2 && i < p4) mean_add(&m_back, c);
        if (i >= p4 + 400 && i < p5)        mean_add(&m_left, c);
    }
    T.v = mean_of(&m_v);
    T.turning = mean_of(&m_turn); T.held = mean_of(&m_held); T.steady = mean_of(&m_steady);
    T.back = mean_of(&m_back); T.left0 = mean_of(&m_left); T.park_turn = mean_of(&m_park);
    Chs_DestroyWorld(w);
    return T;
}

/* Returns the number of failed checks. */
static int parking_rows(const char* path, double blend_begin, double blend_end)
{
    static const double kV[] = { 0.0, 0.05, 0.15, 0.30, 0.80, -0.30 };
    const int nv = (int)(sizeof kV / sizeof kV[0]);
    TurnPoint off[6], on[6];

    printf("\n=== PARKING TORQUE: turning the wheel (task 16) ===\n");
    printf("  Each row: hold the speed, turn 0 -> %.2f at %.2f/s (%.1f s), hold 5 s,\n",
           kParkSteer, kParkRate, kParkSteer / kParkRate);
    printf("  turn back, hold 0.5 s. Column torque in N m (rim = x0.3 with the\n");
    printf("  0.7 assist). 'park' is the parking share while turning. Negative\n");
    printf("  speed = reverse, the way both parking exercises are driven.\n\n");
    printf("  %-7s %-5s %-8s %-8s %-8s %-8s %-8s %-8s %s\n",
           "v(m/s)", "park", "turning", "held", "steady", "back", "left@0", "park", "v held");
    for (int i = 0; i < nv; ++i) {
        off[i] = park_turn(path, kV[i], 0, blend_begin, blend_end);
        on[i]  = park_turn(path, kV[i], 1, blend_begin, blend_end);
        for (int k = 0; k < 2; ++k) {
            const TurnPoint* T = k ? &on[i] : &off[i];
            printf("  %-7.2f %-5s %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %-8.3f %.3f\n",
                   kV[i], k ? "on" : "off", T->turning, T->held, T->steady,
                   T->back, T->left0, T->park_turn, T->v);
        }
        fflush(stdout);
    }

    int fails = 0;
    #define CHECK(cond, ...) do { int ok_ = (cond); printf("  [%s] ", ok_ ? "PASS" : "FAIL"); \
                                  printf(__VA_ARGS__); printf("\n"); fails += !ok_; } while (0)
    printf("\n");
    /* Standstill: TMeasy alone gives nothing; the model resists the turn
     * (steer LEFT is positive, so resisting is negative), holds it when the
     * wheel stops, and reverses with the turn. */
    CHECK(fabs(off[0].turning) < 0.5,
          "standstill, before: turning the wheel is free (%.3f N m)", off[0].turning);
    CHECK(on[0].turning < -10.0,
          "standstill, after: it resists the turn (%.2f N m at the column)", on[0].turning);
    CHECK(on[0].held < 0.9 * on[0].turning,
          "standstill: held when the wheel stops (%.2f, %.0f%% of turning)",
          on[0].held, 100.0 * on[0].held / on[0].turning);
    CHECK(on[0].back > 10.0,
          "standstill: turning back reverses it (%.2f N m)", on[0].back);
    /* Rolling: the share falls as the car rolls faster, and a steady lock
     * gives back TMeasy's own value. The brief: within 10% by 0.80 m/s. */
    int monotone = 1;
    for (int i = 1; i < 5; ++i)
        monotone &= fabs(on[i].park_turn) < fabs(on[i - 1].park_turn);
    CHECK(monotone, "the share while turning falls with speed: %.2f %.2f %.2f %.2f %.2f",
          on[0].park_turn, on[1].park_turn, on[2].park_turn, on[3].park_turn, on[4].park_turn);
    for (int i = 3; i < 5; ++i)
        CHECK(fabs(on[i].steady - off[i].steady) <= 0.10 * fabs(off[i].steady),
              "steady lock at %.2f m/s: %.3f vs %.3f before (within 10%%)",
              kV[i], on[i].steady, off[i].steady);
    #undef CHECK
    printf("  %s\n", fails ? "PARKING TORQUE CHECKS FAILED" : "PARKING TORQUE: ALL CHECKS PASSED");
    return fails;
}

static int do_mzlow(const char* path, double blend_begin, double blend_end) {
    const int park_fails = parking_rows(path, blend_begin, blend_end);

    printf("\n=== ALIGNING MOMENT BELOW 1 m/s (parking speed) ===\n");
    printf("  Stock ChTMeasyTire computes Mz only when vel.x >= m_frblend_begin\n");
    printf("  (1.0 m/s). Steering lock held at %.2f throughout.\n", kParkSteer);
    printf("  blend_end = %.2f m/s (Chrono stock: 3.0).\n", blend_end);
    printf("  Shim default is %.2f .. %.2f; pass 1.0 3.0 to measure stock.\n",
           CHS_TIRE_BLEND_BEGIN, CHS_TIRE_BLEND_END);
    printf("  Torques are means over a 1 s window at a settled speed.\n\n");

    const double sweep[] = { 1.0, 0.70, 0.50, 0.30, 0.20, 0.10 };
    const int nsweep = (int)(sizeof sweep / sizeof sweep[0]);

    /* One explicit value measures just that value; no argument sweeps. */
    const double one[] = { blend_begin };
    const double* vals = blend_begin > 0.0 ? one : sweep;
    const int nvals = blend_begin > 0.0 ? 1 : nsweep;

    printf("  %-9s", "blend_b");
    for (int s = 0; s < N_MZ_SPEEDS; ++s) printf(" %8.2f", kMzSpeeds[s]);
    printf("   %s\n", "applied");
    printf("  %-9s", "(m/s)");
    for (int s = 0; s < N_MZ_SPEEDS; ++s) printf(" %8s", "col Nm");
    printf("\n");

    /* Retained so the detailed per-tyre table below reports the last row. */
    MzPoint last[N_MZ_SPEEDS];
    double  last_blend = 0.0;
    memset(last, 0, sizeof last);

    for (int b = 0; b < nvals; ++b) {
        ChsWorld w = Chs_CreateVehicle(path, 0.45);
        if (!w) { printf("  Chs_CreateVehicle failed\n"); return 1; }

        /* Quasi-static, so no rate damping -- same reason as `sweep`. */
        Chs_SetFfbTuning(w, 0.7, 0.0, 1.0);

        double gb = -1.0, ge = -1.0, g0 = -1.0, g1 = -1.0;
        if (vals[b] > 0.0) {
            /* sigma0/sigma1 <= 0 means "leave stock" -- this task moves the
             * blend range only. blend_end stays at Chrono's 3.0. */
            if (Chs_SetTireStandstill(w, 0.0, 0.0, vals[b], blend_end) != 0) {
                printf("  Chs_SetTireStandstill failed\n");
                Chs_DestroyWorld(w); return 1;
            }
        }
        /* READ IT BACK. Chs_SetTireStandstill silently skips any tyre that is
         * not a ChTMeasyTire, so a failed cast would make every row identical
         * -- which is indistinguishable from a genuine null result. */
        Chs_GetTireStandstill(w, &g0, &g1, &gb, &ge);

        ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
        ChsVehicleFeedback fb;
        run_(w, in, 0.5, &fb);                    /* settle on the ground */

        printf("  %-9.2f", vals[b]);
        for (int s = 0; s < N_MZ_SPEEDS; ++s) {
            const MzPoint P = mz_point(w, kParkSteer, kMzSpeeds[s]);
            printf(" %8.3f", P.col_nm);
            fflush(stdout);
            last[s] = P;
        }
        printf("   %.2f\n", gb);
        last_blend = gb;
        Chs_DestroyWorld(w);
    }

    printf("\n  Per-tyre detail for blend_begin = %.2f (the last row above):\n",
           last_blend);
    printf("  %-8s %-9s %-10s %-10s %-11s %-9s %-9s %-9s %-8s %-8s %s\n",
           "v(m/s)", "road(deg)", "Mz_FL(Nm)", "Mz_FR(Nm)", "column(Nm)",
           "rim(Nm)", "col p2p", "max jump", "ped p2p", "v p2p", "note");
    for (int s = 0; s < N_MZ_SPEEDS; ++s) {
        const int gated = kMzSpeeds[s] < last_blend;
        printf("  %-8.2f %-9.2f %-10.3f %-10.3f %-11.3f %-9.3f %-9.3f %-9.4f %-8.3f %-8.4f %s\n",
               last[s].v, last[s].road_deg, last[s].mz_fl, last[s].mz_fr,
               last[s].col_nm, last[s].rim_nm, last[s].col_p2p,
               last[s].col_max_jump, last[s].thr_p2p, last[s].v_p2p,
               gated ? "below the gate: Mz forced to 0" : "");
    }
    printf("\n  col p2p is the swing over the 1 s window and max jump the\n");
    printf("  largest step between two 1 kHz samples. A mean of 3.5 Nm with a\n");
    printf("  p2p of 3.5 Nm is not steering feel, it is a buzz.\n");
    printf("  ped p2p and v p2p are the CONTROL ripple over the same window:\n");
    printf("  a torque swing next to a steady pedal and a steady speed is the\n");
    printf("  tyre's; one next to a hunting pedal is this harness's.\n");

    /* The step at the gate is the artefact, not the magnitude: Mz goes from
     * exactly 0 to AlignmentTorque(Fy) in one step, and a discontinuity in a
     * constant-force channel is felt as a click at the rim. */
    int below = -1, above = -1;
    for (int s = 0; s < N_MZ_SPEEDS; ++s) {
        if (kMzSpeeds[s] <  last_blend) below = s;
        if (kMzSpeeds[s] >= last_blend && above < 0) above = s;
    }
    if (below >= 0 && above >= 0) {
        printf("\n  Step across the gate: %.3f -> %.3f Nm at the column\n",
               last[below].col_nm, last[above].col_nm);
        printf("  between %.2f and %.2f m/s.\n",
               kMzSpeeds[below], kMzSpeeds[above]);
    } else if (below < 0) {
        printf("\n  No probed speed sits below the gate: the dead zone is\n");
        printf("  under %.2f m/s and out of the manoeuvring band.\n",
               kMzSpeeds[0]);
    }
    return park_fails ? 1 : 0;
}

static int do_clutch(const char* path) {
    ChsWorld w = make(path); if (!w) return 1;
    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
    ChsVehicleFeedback fb;
    run_(w, in, 0.5, &fb);

    printf("\n=== CLUTCH POLARITY + BITE ===\n");
    printf("  Chrono: SetModulation(1 - m_clutch), so 1.0 = pedal DOWN =\n");
    printf("  disengaged. If that were backwards the car would launch with the\n");
    printf("  pedal on the floor, and the hill start would be unfailable.\n\n");
    printf("  %-8s %-8s %-9s %-9s %s\n", "clutch", "throttle", "v(m/s)", "rpm", "verdict");

    const double c[] = {1.0, 0.9, 0.7, 0.5, 0.3, 0.0};
    double v_down = 0.0, rpm_down = 0.0, v_up = 0.0;
    for (unsigned i = 0; i < sizeof c / sizeof c[0]; ++i) {
        Chs_DestroyWorld(w);
        w = Chs_CreateVehicle(path, 0.45);
        ChsDriverInput s = {0, 0, 0, 1.0, 0, 0};
        run_(w, s, 0.5, &fb);
        s.gear = 1; s.clutch = c[i]; s.throttle = 0.5;
        run_(w, s, 2.0, &fb);
        printf("  %-8.2f %-8.2f %-9.3f %-9.0f %s\n", c[i], 0.5, fb.speed_mps, fb.engine_rpm,
               fb.speed_mps > 0.3 ? "moving" : "held");
        if (i == 0) { v_down = fb.speed_mps; rpm_down = fb.engine_rpm; }
        if (c[i] == 0.0) v_up = fb.speed_mps;
    }
    Chs_DestroyWorld(w);
    /* ASSERTED since task 13. This mode printed "moving" on every row for
     * months -- the clutch welded shut under MINRES -- and nothing failed,
     * because the verdict was only ever printed. Pedal down must hold the car
     * AND leave the engine free to rev; pedal up must drive it. */
    const int works = v_down <= 0.3 && rpm_down > 1500.0 && v_up > 0.3;
    printf("\n  %s\n", works
           ? "CLUTCH WORKS: pedal down holds the car and frees the engine; pedal up drives"
           : "CLUTCH WELDED SHUT: the pedal changes nothing (see CLAUDE.md, task 13)");
    return works ? 0 : 1;
}

/* Regression test for the 1e20 neutral-ratio trap. An H-pattern shifter passes
 * through neutral on every change, so this path is exercised constantly.
 *
 * Since task 10 neutral IS passed to the transmission (under PSOR), so this
 * also has to show that a true neutral is a neutral while MOVING: the engine
 * revs free of the wheels with the pedal UP, the car coasts, and a gear taken
 * afterwards drives again. CHS_NEUTRAL=hold runs the pre-task-10 workaround
 * through the same checks. */
static int do_neutral(const char* path) {
    ChsWorld w = make(path); if (!w) return 1;
    ChsVehicleFeedback fb;
    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
    run_(w, in, 1.0, &fb);

    ChsSnapshot s;
    printf("\n=== NEUTRAL STABILITY (Chrono sets a 1e20 gearbox ratio) ===\n");
    printf("  %-22s %-10s %-10s %s\n", "requested", "chassis z", "gear", "verdict");

    const int seq[] = { 0, 1, 0, 2, 0, -1, 0 };
    const char* nm[] = { "neutral", "1st", "neutral", "2nd", "neutral", "reverse", "neutral" };
    int bad = 0;
    for (unsigned i = 0; i < sizeof seq / sizeof seq[0]; ++i) {
        in.gear = seq[i];
        in.clutch = (seq[i] == 0) ? 1.0 : 0.2;
        in.throttle = 0.1;
        run_(w, in, 0.5, &fb);
        Chs_GetSnapshot(w, &s);
        const int ok = s.bodies[0].position.z > 0.0 && s.bodies[0].position.z < 1.0;
        if (!ok) ++bad;
        printf("  %-22s %-10.4f %-10d %s\n", nm[i], s.bodies[0].position.z, fb.gear,
               ok ? "on the ground" : "FELL THROUGH");
    }
    Chs_DestroyWorld(w);

    /* ---- rolling ------------------------------------------------------- */
    printf("\n  rolling: 1st to ~4 m/s, NEUTRAL with the pedal UP and 40%% throttle,\n"
           "  then 2nd. In neutral the engine must rev free of the wheels while\n"
           "  the car coasts; afterwards 2nd must drive it again.\n");
    printf("  %-34s %-8s %-8s %-8s %s\n", "phase", "v(m/s)", "rpm", "z(m)", "");
    w = make(path); if (!w) return 1;
    ChsDriverInput r = {0, 0, 0, 1.0, 0, 0};
    run_(w, r, 1.0, &fb);
    r.gear = 1; r.clutch = 0.0; r.throttle = 0.5;
    for (int k = 0; k < 8000 && fb.speed_mps < 4.0; ++k) run_(w, r, 0.001, &fb);
    const double v_in = fb.speed_mps;
    Chs_GetSnapshot(w, &s);
    printf("  %-34s %-8.2f %-8.0f %-8.4f\n", "1st, 50% throttle", fb.speed_mps,
           fb.engine_rpm, s.bodies[0].position.z);

    r.clutch = 1.0; r.throttle = 0.0; run_(w, r, 0.2, &fb);          /* pedal down */
    r.gear = 0;                        run_(w, r, 0.2, &fb);          /* neutral    */
    r.clutch = 0.0;                    run_(w, r, 0.3, &fb);          /* pedal UP   */
    const double v_n0 = fb.speed_mps;
    r.throttle = 0.4;                  run_(w, r, 1.5, &fb);          /* blip       */
    const double v_n1 = fb.speed_mps, rpm_n = fb.engine_rpm;
    Chs_GetSnapshot(w, &s);
    const double z_n = s.bodies[0].position.z;
    /* In 2nd at v_n1 the engine would turn this fast; free, it must exceed it. */
    const double rpm_geared_2nd = v_n1 / 0.3264 * (2.330 * 3.909) * 60.0 / (2.0 * M_PI);
    printf("  %-34s %-8.2f %-8.0f %-8.4f %s\n", "neutral, pedal UP, 40% throttle",
           v_n1, rpm_n, z_n, "");

    r.throttle = 0.0; r.clutch = 1.0; run_(w, r, 0.5, &fb);          /* rpm falls  */
    r.gear = 2;                       run_(w, r, 0.2, &fb);
    const double v_2a = fb.speed_mps;
    r.throttle = 0.5;
    for (int k = 0; k < 600; ++k) { r.clutch = 1.0 - k / 600.0; run_(w, r, 0.001, &fb); }
    run_(w, r, 2.0, &fb);
    const double v_2b = fb.speed_mps;
    Chs_GetSnapshot(w, &s);
    printf("  %-34s %-8.2f %-8.0f %-8.4f\n", "2nd, clutch in over 0.6 s, 2 s", v_2b,
           fb.engine_rpm, s.bodies[0].position.z);
    Chs_DestroyWorld(w);

    const int on_ground = z_n > 0.2 && z_n < 0.5 && s.bodies[0].position.z > 0.2 &&
                          s.bodies[0].position.z < 0.5;
    const int free_rev  = rpm_n > 1.3 * rpm_geared_2nd && rpm_n > 2500.0;
    const int coasts    = v_n1 <= v_n0 + 0.02 && v_n1 > 0.5 * v_in;
    const int drives    = v_2b > v_2a + 0.5;
    printf("\n  in neutral: engine %.0f rpm, %.0f if geared in 2nd -> %s\n", rpm_n,
           rpm_geared_2nd, free_rev ? "free" : "NOT FREE");
    printf("  in neutral: %.3f -> %.3f m/s over 1.5 s of throttle -> %s\n", v_n0, v_n1,
           coasts ? "coasting" : "NOT COASTING");
    printf("  2nd: %.2f -> %.2f m/s -> %s\n", v_2a, v_2b, drives ? "drives" : "DOES NOT DRIVE");
    if (!on_ground) ++bad;
    if (!free_rev)  ++bad;
    if (!coasts)    ++bad;
    if (!drives)    ++bad;
    printf("\n  %s\n", bad ? "NEUTRAL HANDLING BROKEN" :
           "PASSED: on the ground through every shift; neutral is a neutral while moving.");
    return bad ? 1 : 0;
}

static int do_perf(const char* path, int steps) {
    ChsWorld w = make(path); if (!w) return 1;
    ChsDriverInput in = {0.1, 0.3, 0, 0.0, 1, 0};
    Chs_SetDriverInput(w, in);
    for (int i = 0; i < 500; ++i) Chs_Step(w, DT);      /* warm up */

    double* v = malloc((size_t)steps * sizeof(double));
    ChsVehicleFeedback fb;
    ChsSnapshot snap;
    double sum_full = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t0 = now_us();
        v[i] = Chs_Step(w, DT);
        Chs_GetFeedback(w, &fb);
        Chs_GetSnapshot(w, &snap);
        sum_full += now_us() - t0;
    }
    qsort(v, (size_t)steps, sizeof(double), cmp_d);
    double sum = 0; for (int i = 0; i < steps; ++i) sum += v[i];

    printf("\n=== STEP COST, VEHICLE BEHIND THE SHIM (%d steps, 1 ms) ===\n", steps);
    printf("  Chs_Step only     mean %7.1f  p50 %7.1f  p99 %7.1f  max %7.1f us\n",
           sum / steps, v[steps / 2], v[(int)(steps * 0.99)], v[steps - 1]);
    printf("  + feedback + snapshot: mean %7.1f us  <- what the 1 kHz thread pays\n",
           sum_full / steps);
    printf("  budget 1000 us -> %.1f%% used\n", 100.0 * sum_full / steps / 1000.0);
    free(v);
    Chs_DestroyWorld(w);
    return 0;
}

/* ===========================================================================
 *  DETERMINISM
 *
 *  Milestone 6.1 proved the SCRIPT is a pure function of time. That is not the
 *  same claim as "the trajectory repeats": between the script and the
 *  trajectory sits a 240-DOF multibody solve, four TMeasy tyres and MINRES.
 *  Every referee rule in 6.3 -- cone strike, boundary crossing, stall,
 *  hill-start rollback -- is a predicate over that trajectory, so if the
 *  trajectory wanders then a changed verdict tells you nothing about the rule
 *  you edited.
 *
 *  There is good reason to EXPECT determinism (single thread, fixed dt, MINRES
 *  with a fixed iteration cap, no wall-clock anywhere in the integration) and
 *  it was assumed for the whole of M5. Assumed is not measured.
 *
 *  Two failure modes are worth naming, because they need different tests:
 *
 *  1. IN-PROCESS. Solver state surviving across Chs_DestroyWorld, an
 *     accumulator not reset, a static cache. Caught by running two passes in
 *     one process and comparing.
 *
 *  2. ACROSS PROCESSES. This is the subtle one. If anything inside Chrono
 *     iterates a container ordered by POINTER VALUE -- a std::set of raw
 *     pointers, an unordered_map hashing addresses -- then ASLR changes the
 *     iteration order between runs, which changes the summation order of a
 *     floating-point reduction, which changes the last bits of the result. The
 *     simulation would be perfectly repeatable within one process and quietly
 *     irreproducible between runs, which is the worst possible shape for a
 *     regression suite. Two passes in one process CANNOT see this. The shell
 *     wrapper runs the binary twice and compares the printed digest.
 * ======================================================================== */
static int do_determinism(const char* path, const char* script, int passes) {
    if (passes < 2) passes = 2;
    if (passes > 3) passes = 3;

    DrvScript sc;
    char err[256] = {0};
    if (resolve_script(script, &sc, err, sizeof err) != 0) {
        printf("cannot resolve script '%s': %s\nbuilt-ins:", script, err);
        for (const char* const* n = drv_script_builtin_names(); *n; ++n) printf(" %s", *n);
        printf("\n");
        return 1;
    }

    ChsWorld probe = make(path); if (!probe) { drv_script_free(&sc); return 1; }
    Chs_DestroyWorld(probe);

    /* Half a second past the last keyframe, so the tail (where the script
     * holds its final values) is covered too. */
    const long nsteps = (long)((sc.duration + 0.5) / DT);

    printf("\n=== TRAJECTORY DETERMINISM ===\n");
    printf("  script '%s': %d keyframes, %.2f s -> %ld steps, %d passes\n",
           script, sc.n, sc.duration, nsteps, passes);

    uint64_t* h[3] = {NULL, NULL, NULL};
    ChsSnapshot fin[3];
    ChsVehicleFeedback fbf[3];

    for (int p = 0; p < passes; ++p) {
        ChsWorld w = Chs_CreateVehicle(path, 0.45);
        if (!w) { printf("  pass %d: Chs_CreateVehicle failed\n", p); goto fail; }
        h[p] = (uint64_t*)malloc((size_t)nsteps * sizeof(uint64_t));
        if (!h[p]) { Chs_DestroyWorld(w); goto fail; }

        for (long i = 0; i < nsteps; ++i) {
            DrvInput d;
            /* The SAME time base the physics thread uses: index * dt,
             * multiplied rather than accumulated. */
            drv_script_eval(&sc, (double)i * DT, &d);

            ChsDriverInput in;
            /* The same single sign flip ChronoPhysicsThread.cpp applies:
             * sources are device convention (+ = clockwise), Chrono's
             * m_steering is positive-LEFT. */
            in.steering = -d.steer;
            in.throttle =  d.throttle;
            in.braking  =  d.brake;
            in.clutch   =  d.clutch;
            in.gear     =  d.gear;
            in._pad     =  0;
            Chs_SetDriverInput(w, in);
            Chs_Step(w, DT);

            ChsSnapshot snap;
            Chs_GetSnapshot(w, &snap);
            h[p][i] = hash_bodies(&snap);
        }
        Chs_GetSnapshot(w, &fin[p]);
        Chs_GetFeedback(w, &fbf[p]);
        Chs_DestroyWorld(w);
        printf("  pass %d complete: chassis at (%.6f, %.6f, %.6f), %.3f m/s\n",
               p, fin[p].bodies[0].position.x, fin[p].bodies[0].position.y,
               fin[p].bodies[0].position.z, fbf[p].speed_mps);
    }

    /* Where did they first differ? */
    long first = -1;
    for (long i = 0; i < nsteps && first < 0; ++i)
        for (int p = 1; p < passes; ++p)
            if (h[p][i] != h[0][i]) { first = i; break; }

    printf("\n");
    if (first < 0) {
        printf("  IN-PROCESS: bit-identical for all %ld steps, all %d passes.\n",
               nsteps, passes);
    } else {
        printf("  IN-PROCESS: DIVERGED at step %ld (t=%.3f s).\n", first, first * DT);
        printf("  Exact final positions, pass 0 vs 1:\n");
        for (int b = 0; b < fin[0].body_count; ++b) {
            printf("    body %d  x %.17g / %.17g\n", b,
                   fin[0].bodies[b].position.x, fin[1].bodies[b].position.x);
        }
    }

    /* The digest the shell wrapper compares across two separate processes.
     * Printed unconditionally and in a fixed format so a grep is enough. */
    const uint64_t digest = hash_bodies(&fin[0]);
    printf("\n  final-state digest (compare across processes):\n");
    printf("DIGEST %016llx\n", (unsigned long long)digest);
    printf("  chassis %.17g %.17g %.17g\n",
           fin[0].bodies[0].position.x, fin[0].bodies[0].position.y,
           fin[0].bodies[0].position.z);

    for (int p = 0; p < passes; ++p) free(h[p]);
    drv_script_free(&sc);
    return first < 0 ? 0 : 1;

fail:
    for (int p = 0; p < passes; ++p) free(h[p]);
    drv_script_free(&sc);
    return 1;
}

/* ===========================================================================
 *  DRIFT — why does the car curve with zero steering input?
 *
 *  Measured by the determinism run: +0.55 m of lateral travel over 30.8 m and
 *  +8.84 m over 107.4 m, with `steering` held at exactly 0.0 throughout.
 *  Lateral growth is parabolic in distance, so it is a constant curvature of
 *  roughly 700 m radius -- a sustained lateral force, not a settle transient.
 *  A pure heading offset would give a straight line and a LINEAR y(x).
 *
 *  This mode instruments the mechanism rather than the symptom. Everything it
 *  needs is already across the ABI: road_steer_rad[2] and tire_fz_n[2] are in
 *  ChsVehicleFeedback, and the chassis quaternion is in the snapshot. No ABI
 *  change is needed to answer the question.
 *
 *  What each column distinguishes:
 *    dFL, dFR    the two front road-wheel angles. Static toe is +-0.1 deg and
 *                MIRRORED, so symmetric toe shows as dFL = -dFR and a net of
 *                zero. A non-zero NET is a real steer input the driver never
 *                asked for.
 *    net         (dFL + dFR)/2. This is the number that curves the car.
 *    roll        chassis roll angle. A RWD solid axle reacts drive torque as a
 *                ROLL moment on the body. Roll puts one front wheel into bump
 *                and the other into droop, and if the bump-steer curve is not
 *                odd about ride height the two toe changes do not cancel --
 *                which is a net steer that scales with torque. That is the
 *                leading hypothesis and this column tests it.
 *    dFz         front load asymmetry, the other observable consequence of
 *                roll.
 * ======================================================================== */
/* The referee's wheel speed (task 7) from a snapshot, through the referee's
 * own ref_wheel_speed: slot 0 is the chassis (angular velocity in its own
 * frame), slots 3 and 4 the REAR spindles -- driven, unsteered -- with
 * angular velocities in the world frame. */
static double snap_wheel_speed(const ChsSnapshot* s, double r_roll)
{
    const ChsQuat* q = &s->bodies[0].rotation;
    const double qq[4] = { q->w, q->x, q->y, q->z };
    const ChsVec3* b = &s->bodies[0].angular_velocity;
    const double wb[3] = { b->x, b->y, b->z };
    const double ws[2][3] = {
        { s->bodies[3].angular_velocity.x, s->bodies[3].angular_velocity.y, s->bodies[3].angular_velocity.z },
        { s->bodies[4].angular_velocity.x, s->bodies[4].angular_velocity.y, s->bodies[4].angular_velocity.z },
    };
    return ref_wheel_speed(qq, wb, ws, 2, r_roll);
}

static void euler_zyx(const ChsQuat* q, double* roll, double* pitch, double* yaw) {
    const double w = q->w, x = q->x, y = q->y, z = q->z;
    *roll  = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    double sp = 2.0 * (w * y - z * x);
    if (sp >  1.0) sp =  1.0;
    if (sp < -1.0) sp = -1.0;
    *pitch = asin(sp);
    *yaw   = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

static int do_drift(const char* path, const char* script) {
    DrvScript sc;
    char err[256] = {0};
    if (resolve_script(script, &sc, err, sizeof err) != 0) {
        printf("cannot resolve script '%s': %s\nbuilt-ins:", script, err);
        for (const char* const* n = drv_script_builtin_names(); *n; ++n) printf(" %s", *n);
        printf("\n");
        return 1;
    }
    ChsWorld w = make(path); if (!w) { drv_script_free(&sc); return 1; }

    /* Damping off: it differentiates m_steering, which is identically zero
     * here, so it contributes nothing -- but leaving it on would invite the
     * question. */
    Chs_SetFfbTuning(w, 0.7, 0.0, 1.0);

    const long nsteps = (long)((sc.duration + 0.5) / DT);
    const double RAD = 180.0 / M_PI;

    printf("\n=== ZERO-STEER DRIFT: where does the curvature come from? ===\n");
    printf("  script '%s', steering forced to EXACTLY 0.0 for every step.\n\n",
           script);
    printf("  %-6s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-9s %-8s %s\n",
           "t", "x", "y", "yaw", "roll", "dFL", "dFR", "net", "dFz", "v", "gear");
    printf("  %-6s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-9s %-8s %s\n",
           "(s)", "(m)", "(m)", "(deg)", "(deg)", "(deg)", "(deg)", "(deg)",
           "(N)", "(m/s)", "");

    ChsVehicleFeedback fb;
    ChsSnapshot snap;
    for (long i = 0; i < nsteps; ++i) {
        DrvInput d;
        drv_script_eval(&sc, (double)i * DT, &d);
        ChsDriverInput in;
        in.steering = 0.0;              /* THE POINT: no steering, ever. */
        in.throttle = d.throttle;
        in.braking  = d.brake;
        in.clutch   = d.clutch;
        in.gear     = d.gear;
        in._pad     = 0;
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);

        if (i % 1000 == 0 || i == nsteps - 1) {
            Chs_GetFeedback(w, &fb);
            Chs_GetSnapshot(w, &snap);
            double roll, pitch, yaw;
            euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
            const double dfl = fb.road_steer_rad[0] * RAD;
            const double dfr = fb.road_steer_rad[1] * RAD;
            printf("  %-6.1f %-8.3f %-8.4f %-8.4f %-8.4f %-8.4f %-8.4f %-8.4f "
                   "%-9.1f %-8.2f %d\n",
                   i * DT, snap.bodies[0].position.x, snap.bodies[0].position.y,
                   yaw * RAD, roll * RAD, dfl, dfr, 0.5 * (dfl + dfr),
                   fb.tire_fz_n[0] - fb.tire_fz_n[1], fb.speed_mps, fb.gear);
        }
    }

    printf("\n  READ IT LIKE THIS:\n");
    printf("    net ~ 0 and dFL = -dFR  -> toe is symmetric, look elsewhere.\n");
    printf("    net grows with roll     -> roll steer driven by torque reaction.\n");
    printf("    net non-zero at t=0     -> a static asymmetry in the geometry.\n");
    printf("    dFz non-zero standing   -> the body is not level to begin with.\n");
    Chs_DestroyWorld(w);
    drv_script_free(&sc);
    return 0;
}

/* ===========================================================================
 *  TERRAIN — does the hill exist, and can the car climb it?
 *
 *  Two separate claims, and the first is the one that fails silently. A ramp
 *  whose patch length used the horizontal run instead of the inclined length,
 *  or whose pitch had the wrong sign, still LOADS and still produces a drivable
 *  world -- just the wrong one. So this probes the terrain height directly
 *  before putting a car on it.
 * ======================================================================== */
static int do_terrain(const char* path, const char* terrain_file) {
    ChsTerrainPatch patches[32];
    char err[256] = {0};
    const int32_t n = course_load_terrain(terrain_file, patches,
                                          (int32_t)(sizeof patches / sizeof patches[0]),
                                          err, sizeof err);
    if (n <= 0) { printf("terrain load failed: %s\n", err); return 1; }

    printf("abi %d (header %d)\n", Chs_GetAbiVersion(), CHRONO_SHIM_ABI_VERSION);
    if (Chs_GetAbiVersion() != CHRONO_SHIM_ABI_VERSION) {
        printf("ABI MISMATCH -- rebuild the shim\n"); return 1;
    }

    printf("\n=== TERRAIN PATCHES (%d) ===\n", n);
    printf("  %-10s %-10s %-10s %-10s %-9s %s\n",
           "center_x", "center_z", "length", "width", "pitch", "friction");
    for (int32_t i = 0; i < n; ++i)
        printf("  %-10.3f %-10.3f %-10.4f %-10.2f %-9.4f %.2f\n",
               patches[i].center_x, patches[i].center_z, patches[i].length,
               patches[i].width, patches[i].pitch_rad * 180.0 / M_PI,
               patches[i].friction);

    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, patches, n);
    if (!w) { printf("Chs_CreateVehicleEx failed\n"); return 1; }

    /* --- claim 1: the surface has the shape the generator intended -------
     * Expected heights come from each patch's OWN numbers under the header's
     * conventions -- (center_x, center_z) on the driving surface, `length`
     * along the incline, a NEGATIVE pitch climbing -- so this checks exactly
     * the arithmetic the header warns about, for whatever hill the generator
     * builds. (It hard-coded the 12% hill until task 11 made it 15 degrees,
     * with a bridge and a descent.) */
    printf("\n=== TERRAIN HEIGHT PROFILE ===\n");
    printf("  %-9s %-10s %-10s %s\n", "x (m)", "z (m)", "expected", "");
    int bad = 0;
    double x_toe = 1e9, x_foot = -1e9, z_top = 0.0;
    for (int32_t i = 0; i < n; ++i)
        if (fabs(patches[i].pitch_rad) > 1e-6) {
            const double half = 0.5 * patches[i].length * cos(patches[i].pitch_rad);
            x_toe  = fmin(x_toe,  patches[i].center_x - half);
            x_foot = fmax(x_foot, patches[i].center_x + half);
        }
    for (double x = x_toe - 4.0; x <= x_foot + 4.0; x += 1.0) {
        double want = 0.0;
        for (int32_t i = 0; i < n; ++i) {
            const double half = 0.5 * patches[i].length * cos(patches[i].pitch_rad);
            if (x >= patches[i].center_x - half && x <= patches[i].center_x + half)
                want = patches[i].center_z - (x - patches[i].center_x) * tan(patches[i].pitch_rad);
        }
        const double z = Chs_GetTerrainHeight(w, x, 0.0);
        z_top = fmax(z_top, z);
        const int ok = fabs(z - want) < 0.05;
        if (!ok) ++bad;
        printf("  %-9.2f %-10.4f %-10.4f %s\n", x, z, want, ok ? "" : "  <-- MISMATCH");
    }
    printf("\n  %s (top %.3f m, toe x=%.2f, foot x=%.2f)\n",
           bad ? "PROFILE WRONG -- check pitch sign and inclined length"
               : "PROFILE CORRECT: every patch where its own numbers put it", z_top, x_toe, x_foot);

    /* --- claim 2: a car put on it climbs --------------------------------
     * ONE world, reused. An earlier version destroyed the world and respawned
     * at the foot of the ramp, computing the spawn height with
     * Chs_GetTerrainHeight(w, ...) AFTER Chs_DestroyWorld(w) -- a
     * use-after-free that left the second world with no usable terrain and
     * produced a convincing-looking "the car fell through the hill" result.
     * The terrain was fine; the test was not.
     *
     * Driving from the spawn at x=0 is better anyway: it crosses the patch
     * seam at x=104.1, which is exactly where badly tiled terrain trips a car. */
    printf("\n=== CAN THE CAR CLIMB IT? ===\n");
    printf("  Driving from the spawn at x=0 to the ramp at x=%.1f, so the\n"
           "  patch seams are exercised rather than stepped over.\n\n", x_toe);

    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
    ChsVehicleFeedback fb;
    ChsSnapshot snap;
    run_(w, in, 0.5, &fb);
    /* Held at ~3 m/s in 1st. It was 85% throttle, which reached a 15 degree
     * crest at ~9 m/s and flew it: a crest test, not a climb test. */
    in.gear = 1; in.clutch = 0.0; in.throttle = 0.0;
    Chs_SetDriverInput(w, in);

    printf("  %-7s %-10s %-8s %-10s %-10s %-9s %s\n",
           "t(s)", "x", "y", "chassis z", "terrain z", "v(m/s)", "above road");
    /* A LANE-KEEPING CONTROLLER, and it is not padding.
     *
     * The first version of this test drove with steering pinned at 0 and
     * reported "DID NOT CLIMB". The terrain was correct; the car had simply
     * driven off the SIDE of the ramp. With the residual zero-steer drift
     * (~1590 m radius) the lateral offset by x=122 m is y = x^2/2R = 4.7 m,
     * and the ramp is 8 m wide -- half-width 4 m. The approach patch is 40 m
     * wide, which is why nothing went wrong before x=104.
     *
     * That is CLAUDE.md's warning landing: a test built on a car that does not
     * track straight measures the drift, not the thing under test. A driver
     * steers; so does this. Proportional on lateral offset and heading, which
     * is the minimum that holds a lane. */
    /* Stop at the crest. The generator gives 20 m of flat beyond the ramp and
     * then the world ends -- driving past that is a car falling off the edge of
     * the terrain, which is expected, uninteresting, and drowns the result. */
    double best_road_z = 0.0, best_ride = 0.0, best_x = 0.0;
    double yaw_prev = 0.0;
    for (long i = 0; i < 90000; ++i) {                 /* 90 s at ~3 m/s */
        Chs_GetSnapshot(w, &snap);
        double roll, pitch, yaw;
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
        /* steering is POSITIVE-LEFT, so a positive y needs negative steer. */
        double cmd = -(0.25 * snap.bodies[0].position.y + 2.0 * yaw);
        if (cmd >  0.4) cmd =  0.4;
        if (cmd < -0.4) cmd = -0.4;
        in.steering = cmd;
        Chs_GetFeedback(w, &fb);
        {   const double e = 3.0 - fb.speed_mps;
            in.throttle = e > 0.0 ? fmin(0.9, 0.2 + 0.8 * e) : 0.0;
            in.braking  = e < -0.3 ? fmin(0.4, -0.5 * e) : 0.0; }
        Chs_SetDriverInput(w, in);
        yaw_prev = yaw;
        (void)yaw_prev;

        Chs_Step(w, DT);

        /* Track the highest road the car was genuinely SUPPORTED on. Terrain
         * height alone is not evidence: the car could be falling through it.
         * A plausible ride height is what makes it evidence. */
        {
            Chs_GetSnapshot(w, &snap);
            const double tz_now = Chs_GetTerrainHeight(w, snap.bodies[0].position.x,
                                                       snap.bodies[0].position.y);
            const double ride = snap.bodies[0].position.z - tz_now;
            if (tz_now > best_road_z && ride > 0.10 && ride < 0.50) {
                best_road_z = tz_now; best_ride = ride;
                best_x = snap.bodies[0].position.x;
            }
            if (snap.bodies[0].position.x > x_foot + 6.0) { break; }
        }

        if (i % 4000 == 0) {
            Chs_GetSnapshot(w, &snap);
            Chs_GetFeedback(w, &fb);
            const double tx = snap.bodies[0].position.x;
            const double tz = Chs_GetTerrainHeight(w, tx, snap.bodies[0].position.y);
            printf("  %-7.1f %-10.3f %-8.3f %-10.4f %-10.4f %-9.2f %.4f\n",
                   i * DT, tx, snap.bodies[0].position.y,
                   snap.bodies[0].position.z, tz, fb.speed_mps,
                   snap.bodies[0].position.z - tz);
        }
    }
    Chs_GetSnapshot(w, &snap);
    const double end_x = snap.bodies[0].position.x;
    const double end_ride = snap.bodies[0].position.z
                          - Chs_GetTerrainHeight(w, end_x, snap.bodies[0].position.y);
    printf("\n  highest road supported on: z=%.3f m at x=%.2f, ride height %.3f m\n",
           best_road_z, best_x, best_ride);
    printf("  hill top is %.3f m; ended at x=%.2f, ride height %.3f m\n", z_top, end_x, end_ride);
    const int climbed = best_road_z > z_top - 0.10 && end_x > x_foot
                        && end_ride > 0.10 && end_ride < 0.50;
    printf("  %s\n", climbed
           ? "CLIMBED AND CAME DOWN: supported at a plausible ride height at the top\n"
             "  and at the far foot, across every patch seam on the way."
           : "DID NOT CROSS THE HILL -- check the pitch sign and the inclined length.");
    if (bad || !climbed) { Chs_DestroyWorld(w); return 1; }
    Chs_DestroyWorld(w);
    return bad ? 1 : 0;
}

/* ===========================================================================
 *  CREEP — why does a stationary car move on flat ground?
 *
 *  Found by the coast diagnostic: with the clutch disengaged, no brakes and
 *  flat terrain, the car settled to a steady -0.17 m/s and travelled 1.9 m
 *  BACKWARDS over 11 s. Nothing on level ground should do that.
 *
 *  This is not cosmetic. The hill-start rollback rule is a displacement
 *  threshold (0.30 m) on exactly this quantity, so a spurious 0.17 m/s creep
 *  fails a perfectly held car in under two seconds. The rule cannot be written
 *  until this is understood.
 *
 *  The discriminating question is whether it creeps FROM REST. A car that only
 *  drifts after having been moving points at a zero-crossing artefact in the
 *  tyre model; a car that accelerates from a standstill is being pushed by a
 *  constant force that does not know about speed at all.
 * ======================================================================== */
static int do_creep(const char* path, double blend_begin, double blend_end) {
    /* `settle` is the seconds allowed before measurement, and `spawn_z` the
     * height the vehicle is created at. Both are here because the driveline
     * cases came back BYTE-IDENTICAL to each other, which rules the driveline
     * out entirely and points at the spawn: the car is created at z=0.45 and
     * settles to about 0.26, so it DROPS 0.19 m before anything is measured. */
    /* `idle` marks the cases the engine drives (task 8). 1: in 1st with the
     * pedal up the governor holds idle and a diesel idles forward, at the
     * idle speed through 1st -- before the governor that row crept BACKWARDS
     * at 0.18 m/s, the engine's losses driving it. 2: the same on the
     * brakes, which STALLS the engine: the flywheel's momentum lurches the
     * car a little, then it must be still. */
    struct { const char* name; int gear; double clutch, brake, throttle;
             double settle, spawn_z; int idle; }
    cases[] = {
        { "neutral, settle 1 s",      0, 1.0, 0.0, 0.0,  1.0, 0.45, 0 },
        { "1st, clutch OUT (open)",   1, 1.0, 0.0, 0.0,  1.0, 0.45, 0 },
        { "1st, clutch IN (engaged)", 1, 0.0, 0.0, 0.0,  1.0, 0.45, 1 },
        { "neutral, brake 0.5",       0, 1.0, 0.5, 0.0,  1.0, 0.45, 0 },
        { "neutral, settle 10 s",     0, 1.0, 0.0, 0.0, 10.0, 0.45, 0 },
        { "neutral, settle 30 s",     0, 1.0, 0.0, 0.0, 30.0, 0.45, 0 },
        { "neutral, spawn at 0.26",   0, 1.0, 0.0, 0.0,  1.0, 0.26, 0 },
        { "neutral, 0.26 + 10 s",     0, 1.0, 0.0, 0.0, 10.0, 0.26, 0 },
        /* Does the brake HOLD it? This is the actionable question: if full
         * brake pins the car, the hill-start rule can require the brake during
         * the held phase and the creep never enters the measurement. */
        { "neutral, brake 1.0",       0, 1.0, 1.0, 0.0,  1.0, 0.45, 0 },
        /* Since task 8 the engine STALLS here, as a real one does. */
        { "1st engaged, brake 1.0",   1, 0.0, 1.0, 0.0,  1.0, 0.45, 2 },
    };
    /* The idle-in-gear speed: gen_innova.py's IDLE_RPM less the governor's
     * droop (783 rpm measured free), through 1st and the final drive. */
    const double v_idle_1st = 783.0 * 2.0 * M_PI / 60.0 / (4.313 * 3.909) * 0.33645 * 0.97;
    const int ncase = (int)(sizeof cases / sizeof cases[0]);

    printf("\n=== CREEP ON FLAT GROUND, STARTING FROM REST ===\n");
    printf("  Terrain is one flat patch at z=0. Steering is 0 and no case applies\n");
    printf("  throttle. Every case must stand still except 1st with the pedal up,\n");
    printf("  which must IDLE FORWARD at %.2f m/s (the governed idle through 1st).\n\n",
           v_idle_1st);
    printf("  %-26s %-9s %-9s %-9s %-9s %-9s %s\n",
           "case", "dx@5s", "dx@20s", "v@20s", "omega_y", "w*R", "verdict");

    int bad = 0;
    for (int c = 0; c < ncase; ++c) {
        ChsWorld w = Chs_CreateVehicle(path, cases[c].spawn_z);
        if (!w) { printf("  create failed\n"); return 1; }
        /* mz-below-1ms regression hook. m_frblend_begin also opens the Dahl
         * standstill blend, so a value chosen for steering feel has to be
         * re-checked HERE -- creep is the flat-ground behaviour of exactly
         * that model. 0 leaves the SHIM's default (CHS_TIRE_BLEND_BEGIN /
         * _END), which is no longer Chrono's 1.0 / 3.0 -- pass 1.0 and 3.0
         * explicitly to measure stock. */
        if (blend_begin > 0.0)
            Chs_SetTireStandstill(w, 0.0, 0.0, blend_begin, blend_end);

        ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
        ChsVehicleFeedback fb;
        ChsSnapshot snap;
        /* Settle first, so what follows is creep and not the drop transient --
         * assuming the settle is long enough, which is exactly what the extra
         * cases below are testing. */
        run_(w, in, cases[c].settle, &fb);

        in.gear = cases[c].gear;
        in.clutch = cases[c].clutch;
        in.braking = cases[c].brake;
        in.throttle = cases[c].throttle;
        Chs_SetDriverInput(w, in);

        double x5 = 0.0;
        Chs_GetSnapshot(w, &snap);
        const double x0 = snap.bodies[0].position.x;
        for (long i = 0; i < 20000; ++i) {
            Chs_Step(w, DT);
            if (i == 5000) { Chs_GetSnapshot(w, &snap); x5 = snap.bodies[0].position.x - x0; }
        }
        Chs_GetSnapshot(w, &snap);
        Chs_GetFeedback(w, &fb);
        const double x20 = snap.bodies[0].position.x - x0;
        /* GetSpeed() is a magnitude; take the signed value from the chassis. */
        const double vx = snap.bodies[0].linear_velocity.x;

        /* THE DISCRIMINATOR. Snapshot slots 1..4 are the spindles (FL FR RL RR).
         * Their angular_velocity is in WORLD axes (Chrono's GetSpindleAngVel;
         * an earlier comment here said local): with the car heading +X on flat
         * ground its Y is the spin axis all the same.
         *
         * Rolling without slipping means v = +omega_y * R_loaded -- ISO 8855,
         * +Y left: a wheel rolling forward spins positively about +Y. (This
         * printed -omega*R, and "-w*R matches v" was read off columns that
         * matched in MAGNITUDE with opposite signs; task 7 found it.) omega ~ 0
         * with v /= 0 means the car is SLIDING on locked wheels. The two have
         * nothing in common as bugs. */
        double wy = 0.0;
        for (int b = 1; b <= 4; ++b) wy += snap.bodies[b].angular_velocity.y;
        wy *= 0.25;
        /* 0.33645 unloaded x 0.97 static deflection. NOT 0.203 -- that is the RIM
         * radius from innova_Wheel.json, and using it here understated the
         * rolling speed by 40% and made the wheels look inconsistent with the
         * chassis when they were in fact free-rolling with it. */
        const double R_LOADED = 0.33645 * 0.97;
        const double v_if_rolling = wy * R_LOADED;

        const int ok = cases[c].idle == 1 ? fabs(vx - v_idle_1st) < 0.15 * v_idle_1st
                     : cases[c].idle == 2 ? fabs(x20 - x5) < 0.02 && fabs(vx) < 0.02
                     : fabs(x20) < 0.05 && fabs(vx) < 0.02;
        if (!ok) ++bad;
        printf("  %-26s %-9.4f %-9.4f %-9.4f %-9.4f %-9.4f %s\n",
               cases[c].name, x5, x20, vx, wy, v_if_rolling,
               cases[c].idle == 1 ? (ok ? "idles forward" : "WRONG IDLE CREEP")
             : cases[c].idle == 2 ? (ok ? "stalls, held" : "NOT HELD")
             : (ok ? "still" : "MOVING"));
        Chs_DestroyWorld(w);
    }

    printf("\n  READ IT LIKE THIS:\n");
    printf("    w*R matches v   -> the wheels are ROLLING; something turns them.\n");
    printf("    omega ~ 0, v /= 0 -> the car is SLIDING on effectively locked wheels.\n");
    printf("\n  %s\n", bad ? "CREEP PRESENT."
                            : "NO CREEP from rest: the car stays put in every configuration.");
    return bad ? 1 : 0;
}

/* ===========================================================================
 *  PARK — the parking exercises, driven closed-loop and judged end to end.
 *
 *  Task 3's end-to-end check. The referee's parking rule is host-tested
 *  (referee/run.sh selftest); this proves it on a REAL trajectory: Chrono's
 *  tyres and brakes, the parking hold, the stop detector's thresholds against
 *  real speeds, and the zone test against real poses.
 *
 *  WHY A PILOT, NOT A KEYFRAME SCRIPT. A script is open loop and a parking
 *  manoeuvre is a PATH. An open-loop steering script tuned to this car would
 *  stop parking the moment any dynamics changed, and the run would then be
 *  reporting the script, not the rule. So the path is planned with the same
 *  kinematic bicycle gen_course.py uses to prove the exercise feasible, and
 *  followed with pure pursuit (either direction) and the proportional speed
 *  loop mzlow already proved at these speeds; the car is then held on the
 *  brake so the referee sees a genuine stop.
 *
 *  WHY IT STARTS AT THE MANOEUVRE, NOT AT THE COURSE START. The spawn is fixed
 *  at the origin (Chs_CreateVehicleEx takes a height, not a pose), and driving
 *  90 m of course to reach the bay would test the approach, not the park. The
 *  ground there is flat and uniform, so a manoeuvre's physics does not depend
 *  on WHERE it happens: the referee is shown the car through a rigid transform
 *  that puts the settled spawn pose on the manoeuvre's start pose. The course
 *  -- lines, cones, zones -- is the real one, and every rule runs.
 *
 *  The plans are derived from the course's own ZONE records, so they move
 *  with the course, and were checked against every line and cone before any
 *  of this was written (parking task notes in CLAUDE.md).
 * ======================================================================== */
/* 1 cm per point, so 32768 points is 327 m of path. It was 8192 (82 m)
 * until task 11 lengthened the slalom to the regulation's 9 cones: leg A grew
 * to ~95 m, plan_extend stopped appending AT THE LIMIT, SILENTLY, and the car
 * "arrived" at a truncated end inside the slalom's run-out -- so the next leg
 * began 15 m from its own path. A full plan now fails the run instead. */
#define PARK_PLAN_MAX 32768
typedef struct {
    double x[PARK_PLAN_MAX], y[PARK_PLAN_MAX], h[PARK_PLAN_MAX], s[PARK_PLAN_MAX];
    int    n;
    int    dir;            /* +1 forward, -1 reverse, for the whole plan     */
    int    overflow;       /* set if plan_extend ran out of points           */
} ParkPlan;

static ParkPlan g_plan;    /* 1 MB: not on the stack */

/* [SPEC] measured full-lock road angle, Ackermann average
 * (vehicle/run.sh ackermann; gen_course.py DELTA_MAX_DEG). 35.36 since task 14
 * put the car at its design ride height and refitted PINION_CAL. */
static const double PARK_DELTA_MAX = 35.36 * M_PI / 180.0;

/* Kinematic bicycle about the REAR axle, driven FORWARD from the plan's last
 * point along curvature `kappa` (1/m, + = left) for `length` metres. */
static void plan_extend(ParkPlan* P, double kappa, double length)
{
    const int steps = (int)ceil(length / 0.01);
    const double ds = length / steps;
    if (P->n + steps > PARK_PLAN_MAX) P->overflow = 1;
    for (int i = 0; i < steps && P->n < PARK_PLAN_MAX; ++i) {
        const int k = P->n - 1;
        const double hm = P->h[k] + kappa * ds * 0.5;
        P->x[P->n] = P->x[k] + ds * cos(hm);
        P->y[P->n] = P->y[k] + ds * sin(hm);
        P->h[P->n] = P->h[k] + kappa * ds;
        ++P->n;
    }
}

/* Arc length along the plan, and the direction it is driven in. Called once a
 * plan is complete; pursue() reads both. */
static void plan_finish(ParkPlan* P, int dir)
{
    if (P->overflow)
        printf("  PLAN OVERFLOW: more than %d points (%.0f m); the plan is truncated\n",
               PARK_PLAN_MAX, PARK_PLAN_MAX * 0.01);
    P->s[0] = 0.0;
    for (int i = 1; i < P->n; ++i)
        P->s[i] = P->s[i - 1] + hypot(P->x[i] - P->x[i - 1], P->y[i] - P->y[i - 1]);
    P->dir = dir;
}

/* The plans are built by DRIVING OUT of the space from the parked pose -- the
 * geometry gen_course.py integrates -- and then run backwards, which for a
 * kinematic bicycle is exactly the reverse manoeuvre with the same steering. */
static void plan_run_backwards(ParkPlan* P)
{
    for (int i = 0, j = P->n - 1; i < j; ++i, --j) {
        double t;
        t = P->x[i]; P->x[i] = P->x[j]; P->x[j] = t;
        t = P->y[i]; P->y[i] = P->y[j]; P->y[j] = t;
        t = P->h[i]; P->h[i] = P->h[j]; P->h[j] = t;
    }
    plan_finish(P, -1);
}

/* --- primitives for the full test's route (do_course) -------------------- */

static void plan_start(ParkPlan* P, double x, double y, double h)
{
    P->n = 1; P->overflow = 0; P->x[0] = x; P->y[0] = y; P->h[0] = h;
}

/* `src` driven the other way: its poses in reverse order. For a parking plan
 * that is the drive OUT of the space, forward, along the path it reversed in
 * on. The copy is open: extend it, then plan_finish() it. */
static void plan_copy_reversed(ParkPlan* dst, const ParkPlan* src)
{
    dst->n = src->n;
    for (int i = 0; i < src->n; ++i) {
        dst->x[i] = src->x[src->n - 1 - i];
        dst->y[i] = src->y[src->n - 1 - i];
        dst->h[i] = src->h[src->n - 1 - i];
    }
}

/* Straight on, heading ~0, to rear-axle x. */
static void plan_straight_to(ParkPlan* P, double x)
{
    const double len = x - P->x[P->n - 1];
    if (len > 0.0) plan_extend(P, 0.0, len);
}

/* A lane change of `dy` at heading 0: two equal arcs of radius R, turning
 * toward dy first, ending at heading 0 again. It is the parallel plan's
 * geometry, and the path of least peak curvature for a given run and offset.
 * Returns the distance it covers along x. */
static double plan_shift_dx(double dy, double R)
{
    return 2.0 * R * sin(acos(1.0 - fabs(dy) / (2.0 * R)));
}

static void plan_shift(ParkPlan* P, double dy, double R)
{
    if (fabs(dy) < 1e-9) return;
    const double th = acos(1.0 - fabs(dy) / (2.0 * R));
    const double k  = copysign(1.0 / R, dy);
    plan_extend(P, +k, R * th);
    plan_extend(P, -k, R * th);
}

/* The same, ending at a given point: the radius is whatever makes the two
 * arcs cover exactly (dx, dy), R = (dx^2 + dy^2) / (4|dy|). */
static void plan_shift_to(ParkPlan* P, double x, double y)
{
    const double dx = x - P->x[P->n - 1], dy = y - P->y[P->n - 1];
    if (fabs(dy) < 1e-9) { plan_straight_to(P, x); return; }
    plan_shift(P, dy, (dx * dx + dy * dy) / (4.0 * fabs(dy)));
}

/* Pure pursuit about the rear axle, in the plan's direction of travel.
 * Returns the ROAD-WHEEL angle (+ = left); updates the progress index and the
 * distance still to go. Reversing, the travel direction is yaw + pi and the
 * path curvature is -tan(delta)/L, hence the dir factor. */
static double pursue(const ParkPlan* P, int* idx, double rx, double ry, double yaw,
                     double ld, double wheelbase, double* remaining, double* deviation)
{
    int i = *idx;
    double best = hypot(P->x[i] - rx, P->y[i] - ry);
    for (int k = i + 1; k < P->n && k < i + 400; ++k) {   /* forward only */
        const double d = hypot(P->x[k] - rx, P->y[k] - ry);
        if (d < best) { best = d; i = k; }
    }
    *idx = i;
    *deviation = best;
    *remaining = P->s[P->n - 1] - P->s[i];

    int j = i;
    while (j < P->n - 1 && P->s[j] - P->s[i] < ld) ++j;
    double tx = P->x[j], ty = P->y[j];
    if (P->s[j] - P->s[i] < ld) {        /* past the end: extend the last heading */
        const double extra = ld - (P->s[P->n - 1] - P->s[i]);
        const double th = P->h[P->n - 1] + (P->dir < 0 ? M_PI : 0.0);
        tx = P->x[P->n - 1] + extra * cos(th);
        ty = P->y[P->n - 1] + extra * sin(th);
    }
    const double travel = yaw + (P->dir < 0 ? M_PI : 0.0);
    const double alpha = atan2(ty - ry, tx - rx) - travel;
    const double kappa = 2.0 * sin(alpha) / fmax(hypot(tx - rx, ty - ry), 1e-3);
    return atan(P->dir * kappa * wheelbase);
}

/* Plans `name` against the loaded course. Returns the target zone, or NULL. */
static const RefZone* park_plan(const RefCourse* c, const char* name, ParkPlan* P)
{
    const int bay = strstr(name, "bay") != NULL;
    const RefZone* z = NULL;
    for (int32_t i = 0; i < c->n_zones; ++i)
        if (c->zones[i].exercise == (bay ? 3 : 2)) z = &c->zones[i];
    if (!z) { printf("no %s zone in the course\n", bay ? "bay" : "parallel"); return NULL; }

    double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
    for (int32_t k = 0; k < z->n; ++k) {
        x0 = fmin(x0, z->x[k]); x1 = fmax(x1, z->x[k]);
        y0 = fmin(y0, z->y[k]); y1 = fmax(y1, z->y[k]);
    }
    const double L = c->wheelbase;
    P->n = 1;
    P->overflow = 0;

    if (bay) {
        /* Reversed in, nose out (-Y). Centred in both directions: the spare
         * depth is only (5.0 - 4.735) = 0.265 m, so every centimetre counts.
         * 85% lock leaves pure pursuit room to correct on the arc; y_start 0.6
         * sits mid-way in the window the plan check found clean (0.4 .. 0.6
         * cleared everything; the bay edges by 0.23 m, the aisle by 0.53+). */
        if (fabs(z->heading + M_PI / 2.0) > 1e-3) { printf("bay heading is not -90 deg\n"); return NULL; }
        const double R = L / tan(0.85 * PARK_DELTA_MAX), y_start = 0.6;
        const double gap = (y1 - y0 - c->car_length) / 2.0;
        P->x[0] = (x0 + x1) / 2.0;
        P->y[0] = y0 + gap + c->front_overhang + L;          /* rear axle */
        P->h[0] = -M_PI / 2.0;
        const double straight = P->y[0] - R - y_start;
        plan_extend(P, 0.0, straight);
        plan_extend(P, 1.0 / R, R * M_PI / 2.0);
        printf("  plan    : reverse a 90.0 deg arc at R %.3f m (85%% lock), then %.3f m "
               "straight\n", R, straight);
    } else {
        /* Two arcs, reversing: left lock then right lock. The slot is TIGHT
         * for this car (gen_course.py: 0.56 m spare at full lock), so 95%
         * lock and a start 0.8 m right of centre, beside the parked cars --
         * the plan check found that clearing the front car's corner cone by
         * 0.11 to 0.17 m. Planned 0.15 m from the car behind, MEASURED: at
         * 0.10 the tracking error left 5.6 cm (a pass by 6 mm); 0.15 gives
         * 10.6 cm at the stop. */
        if (fabs(z->heading) > 1e-3) { printf("parallel heading is not 0 deg\n"); return NULL; }
        /* The start is placed off the space's OUTER edge -- the parked cars'
         * corners -- not off the road's centreline: -0.8 was measured with
         * that edge at y = 0.57, i.e. 1.37 m clear of it, and when task 11
         * widened the space to the regulation's 1.5 x car width the edge moved
         * to 0.255 and a fixed -0.8 put the car's flank on a cone. */
        const double R = L / tan(0.95 * PARK_DELTA_MAX), y_start = y0 - 1.37;
        P->x[0] = x0 + 0.15 + c->rear_overhang;
        P->y[0] = (y0 + y1) / 2.0;
        P->h[0] = 0.0;
        const double th = acos(1.0 - (P->y[0] - y_start) / (2.0 * R));
        plan_extend(P, -1.0 / R, R * th);
        plan_extend(P, +1.0 / R, R * th);
        printf("  plan    : reverse two %.1f deg arcs at R %.3f m (95%% lock), left "
               "then right\n", th * 180.0 / M_PI, R);
    }
    plan_run_backwards(P);

    /* The deliberately SHORT variant stops with the NOSE 0.30 m out past the
     * bay mouth: the negative case, which must be judged PARK_OUTSIDE. It
     * stopped a fixed 0.30 m early until task 11 gave the bay the
     * regulation's 1.5 x car length -- 1.18 m spare at each end -- and a
     * 0.30 m early stop was then a good park. So: short by the spare depth
     * as well. */
    if (strstr(name, "short")) {
        const double spare = bay ? (y1 - y0 - c->car_length) / 2.0 : 0.0;
        const double end = P->s[P->n - 1] - (spare + 0.30);
        while (P->n > 2 && P->s[P->n - 1] > end) --P->n;
    }
    return z;
}

/* ===========================================================================
 *  THE PILOT — one leg of driving: follow a plan, stop at its end, hold.
 *
 *  Shared by the parking runs (one leg each) and the full test (six legs), so
 *  the full test parks with exactly the controller task 3 measured. Every leg
 *  ENDS the way task 3's did: arrive at PILOT_V_PARK, brake onto the end with
 *  the clutch in, hold. The faster legs only add a cruise before that and a
 *  deceleration down to it; a parking leg (v_cruise = PILOT_V_PARK, no slow
 *  section) reduces to task 3's loop operation for operation.
 * ======================================================================== */
#define PILOT_V_PARK   0.6   /* m/s, a parking crawl. (Chosen above
                                REF_STALL_MAX_SPEED while the engine bogged at
                                a crawl; since task 8's governor it idles at
                                any crawl, but task 3's parks were measured
                                here.)                                        */
#define PILOT_A_ACC    2.0   /* m/s^2 target ramp. A brisk launch: past 0.5 m/s
                                inside the stall rule's 0.75 s window         */
#define PILOT_A_DEC    0.8   /* m/s^2 from cruise down to PILOT_V_PARK        */
#define PILOT_D_FINAL  0.3   /* m driven at PILOT_V_PARK before the stop      */
#define PILOT_LD_TIME  0.6   /* s: lookahead grows with speed above the floor */
/* Below idle speed the pilot drives ON THE CLUTCH: PILOT_C_* and
 * pilot_v_idle() are defined above mz_point, which crawls the same way. */

/* The hill start, on the clutch: brake held, throttle off, lift the pedal
 * until the clutch carries the car, release the brake, and hand over to the
 * clutch loop where the foot already is. How far to lift is what a driver
 * FEELS -- the car taking the load -- so it is computed from the slope the
 * car is standing on: gravity along it through 1st and the final drive,
 * against the clutch's 600 N m, times a margin. 15 deg (task 11) needs 15% of
 * capacity; 12% needed 7%, and a fixed 10% bite, right for 12%, would have
 * rolled the car back on 15 deg. [EST] -- how HARD a hill start should be is
 * clutch ergonomics, which CLAUDE.md forbids tuning without a real pedal.
 * These only have to be a competent start. */
#define HILL_C_MARGIN   1.3    /* bite = this x the capacity that holds the car */
#define HILL_C_MIN      0.05   /* ... and never less than this                  */
#define HILL_C_RATE     0.20   /* capacity per second while lifting to it       */

/* `course_rollback` sets the margin BELOW 1 -- the brake released before the
 * clutch can hold the car -- and gives the pilot a driver's reaction time:
 * the foot stays where it was for g_hill_react before the loop corrects. The
 * hill-start mistake the rollback rule exists to catch (task 7). Without the
 * delay the loop catches the car within milliseconds, faster than a person:
 * measured, half the bite alone rolled back only 3.6 cm. */
static double g_hill_c_margin = HILL_C_MARGIN;
static double g_hill_react    = 0.0;

static double hill_bite(double pitch)
{
    const double c_hold = 1800.0 * 9.81 * sin(fabs(pitch)) * 0.3318
                          / (4.313 * 3.909) / 600.0;       /* gen_innova.py */
    return fmax(HILL_C_MIN, g_hill_c_margin * c_hold);
}


typedef struct {
    const char*     name;
    const ParkPlan* plan;
    double v_cruise;    /* m/s once past s_slow                              */
    double s_slow;      /* first metres at PILOT_V_PARK: leaving a space     */
    double ld;          /* pure-pursuit lookahead floor, m                   */
    double thr_max;     /* throttle ceiling                                  */
    double i_max;       /* speed-loop integrator ceiling                     */
    double hold;        /* s on the brake, clutch in, after the stop         */
    int    hill_start;  /* leave the previous leg's hold as a hill start     */
} Leg;

typedef struct {
    int    idx, stopped, launching, on_clutch;
    double drive_t, v_integ, c_integ, pedal, t_fast, t_launch;
    double pitch;       /* the chassis pitch, set by the caller each step    */
    double rem, dev, max_dev;
    double t_stop, stop_x, stop_y, stop_yaw;
} Pilot;

static void pilot_begin(Pilot* p, const Leg* g)
{
    memset(p, 0, sizeof *p);
    p->rem       = g->plan->s[g->plan->n - 1];
    p->t_stop    = -1.0;
    p->t_fast    = -1.0;
    p->launching = g->hill_start;
}

/* One control step. (cx, cy, cyaw) is the FRONT-axle pose in the course
 * frame, t the time on the caller's clock. Returns 1 once the leg is over:
 * stopped, and held for g->hold. */
static int pilot_step(Pilot* p, const Leg* g, double L, double cx, double cy,
                      double cyaw, const ChsVehicleFeedback* fb, double t,
                      ChsDriverInput* in)
{
    const ParkPlan* P = g->plan;
    const double rx = cx - L * cos(cyaw), ry = cy - L * sin(cyaw);
    const double v_along = P->dir * fb->speed_mps;     /* + = the planned way */

    if (!p->stopped) {
        /* The floor is task 3's measured 0.8 m: at 1.2 m the parallel park
         * drifted 8.6 cm and ended 4.4 deg out. It only grows above
         * 0.8 / PILOT_LD_TIME = 1.33 m/s, i.e. never while parking. */
        const double ld = fmax(g->ld, PILOT_LD_TIME * fabs(fb->speed_mps));
        const double delta = pursue(P, &p->idx, rx, ry, cyaw, ld, L, &p->rem, &p->dev);
        if (p->dev > p->max_dev) p->max_dev = p->dev;
        in->steering = fmax(-1.0, fmin(1.0, delta / PARK_DELTA_MAX));
    }

    if (!p->stopped && p->launching) {
        /* From the hold: brake on, clutch down, throttle off. Lift to the
         * bite, then release the brake and hand over to the clutch loop
         * bumplessly: its integrator starts where the foot is. */
        const double bite = hill_bite(p->pitch);
        const double c = fmin(bite, HILL_C_RATE * p->t_launch);
        in->gear     = P->dir < 0 ? -1 : 1;
        in->throttle = 0.0;
        in->clutch   = 1.0 - c;
        in->braking  = c < bite ? 1.0 : 0.0;
        p->t_launch += DT;
        if (c >= bite && p->t_launch >= bite / HILL_C_RATE + g_hill_react) {
            p->launching = 0;
            p->on_clutch = 1;
            p->c_integ   = c / PILOT_C_KI;
            p->pedal     = 1.0 - c;
            p->drive_t   = 0.0;
        }
        return 0;
    }

    if (!p->stopped) {
        /* STOP BY BRAKING ONTO THE END, clutch in: when the predicted
         * stopping distance at ~6 m/s^2 reaches what is left. There is
         * deliberately no slower final approach. Coasting clutch-in, the
         * tyres scrubbing at 95% lock stopped the car short with no way to
         * restart; crawling in gear instead drags this engine far under
         * idle, which the stall rule rightly scores. Arrive at PILOT_V_PARK,
         * then stop: about 3 cm of stopping distance. */
        if (p->drive_t > 0.5 && p->rem < 0.02 + v_along * v_along / (2.0 * 6.0)) {
            p->stopped = 1;
        } else {
            double vt = fmin(g->v_cruise, PILOT_A_ACC * p->drive_t);
            if (P->s[p->idx] < g->s_slow) {
                vt = fmin(vt, PILOT_V_PARK);            /* still in the space */
            } else if (g->s_slow > 0.0) {               /* out: ramp up again */
                if (p->t_fast < 0.0) p->t_fast = p->drive_t;
                vt = fmin(vt, PILOT_V_PARK + PILOT_A_ACC * (p->drive_t - p->t_fast));
            }
            if (g->v_cruise > PILOT_V_PARK)             /* down to the approach */
                vt = fmin(vt, sqrt(PILOT_V_PARK * PILOT_V_PARK + 2.0 * PILOT_A_DEC
                                   * fmax(0.0, p->rem - PILOT_D_FINAL)));
            const double e = vt - v_along;
            in->gear   = P->dir < 0 ? -1 : 1;
            /* Clutch below idle speed, throttle above; hysteresis so the
             * boundary does not chatter. */
            const double v_idle = pilot_v_idle(in->gear);
            if (p->on_clutch ? vt > 0.95 * v_idle : vt < 0.85 * v_idle) {
                p->on_clutch = !p->on_clutch;
                p->v_integ = 0.0;
                p->c_integ = 0.0;
            }
            if (p->on_clutch) {
                /* The speed loop drives the clutch's CAPACITY; the brake only
                 * helps when even an open clutch is too fast (downhill). */
                p->c_integ = fmax(-0.05, fmin(PILOT_C_IMAX, p->c_integ + e * DT));
                const double c = fmax(0.0, fmin(1.0, PILOT_C_KP * e + PILOT_C_KI * p->c_integ));
                p->pedal     = 1.0 - c;
                in->throttle = 0.0;
                in->braking  = e < -0.10 ? fmin(0.35, -1.5 * (e + 0.10)) : 0.0;
            } else {
                /* The integral term matters: a P-only loop settled at 0.45
                 * m/s against a 0.6 target (task 3). The pedal comes up over
                 * ~0.3 s, not in a step: the car is near the idle speed the
                 * clutch is slipping towards, so it locks without a jolt. */
                p->pedal   = fmax(0.0, p->pedal - 3.0 * DT);
                p->v_integ = fmax(-0.3, fmin(g->i_max, p->v_integ + e * DT));
                const double u = 0.06 + 1.2 * e + 0.6 * p->v_integ;
                in->throttle = u > 0.0 ? fmin(g->thr_max, u) : 0.0;
                in->braking  = e < -0.02 ? fmin(0.35, -1.5 * e) : 0.0;
            }
            in->clutch = p->pedal;
            p->drive_t += DT;
        }
    }
    if (p->stopped) {
        in->throttle = 0.0; in->braking = 1.0; in->clutch = 1.0;
        if (p->t_stop < 0.0 && fabs(fb->speed_mps) < REF_PARK_STOP_SPEED) {
            p->t_stop = t; p->stop_x = cx; p->stop_y = cy; p->stop_yaw = cyaw;
        }
        if (p->t_stop >= 0.0 && t - p->t_stop > g->hold) return 1;
    }
    return 0;
}

/* The referee's result, and a digest of it for the two-process comparison.
 * The digest covers the event log and every exercise's result field by field
 * (the structs have padding, so hashing them whole would hash garbage). */
static const char* const kExerciseKind[] = { "gated", "park", "hill" };

static void print_verdict(const RefCourse* c, const RefState* s)
{
    RefVerdict v;
    ref_verdict(s, &v);
    uint64_t h = FNV_SEED;
    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(s, ev, REF_MAX_EVENTS);
    for (int32_t i = 0; i < n; ++i) {
        h = fnv1a(&ev[i].t, sizeof ev[i].t, h);
        h = fnv1a(&ev[i].type, sizeof ev[i].type, h);
        h = fnv1a(&ev[i].object_id, sizeof ev[i].object_id, h);
        h = fnv1a(&ev[i].detail, sizeof ev[i].detail, h);
    }

    printf("\n  %-3s %-6s %-8s %-19s %6s %8s %8s\n",
           "ex", "kind", "state", "reason", "faults", "start s", "took s");
    for (int32_t i = 0; i < c->n_exercises; ++i) {
        const RefExercise* e = &c->exercises[i];
        const RefExerciseResult* r = &s->ex[i];
        const int ended = r->state == REF_EX_PASSED || r->state == REF_EX_FAILED;
        printf("  %-3d %-6s %-8s %-19s %6d", e->id,
               (e->kind >= 0 && e->kind <= 2) ? kExerciseKind[e->kind] : "?",
               ref_exercise_state_name(r->state),
               r->state == REF_EX_FAILED ? ref_fail_reason_name(r->fail_reason) : "-",
               r->faults);
        if (r->state != REF_EX_ARMED) printf(" %8.2f", r->t_start); else printf(" %8s", "-");
        if (ended && r->t_start >= 0.0) printf(" %8.2f", r->t_end - r->t_start);
        else                            printf(" %8s", "-");
        if (e->kind == REF_EXK_PARK) printf("   failed attempts %d", r->park_attempts);
        if (e->kind == REF_EXK_HILL)
            printf("   stopped %s, rollback %.4f m", r->stopped ? "yes" : "no", r->rollback);
        printf("\n");
        h = fnv1a(&r->state, sizeof r->state, h);
        h = fnv1a(&r->fail_reason, sizeof r->fail_reason, h);
        h = fnv1a(&r->faults, sizeof r->faults, h);
        h = fnv1a(&r->park_attempts, sizeof r->park_attempts, h);
        h = fnv1a(&r->stopped, sizeof r->stopped, h);
        h = fnv1a(&r->rollback, sizeof r->rollback, h);
        h = fnv1a(&r->t_start, sizeof r->t_start, h);
        h = fnv1a(&r->t_end, sizeof r->t_end, h);
    }
    h = fnv1a(&v, sizeof v, h);       /* seven int32_t: no padding */

    printf("\n  VERDICT: %s -- %d of %d passed, %d failed, %d unfinished; "
           "%d general fault(s)%s\n", ref_result_name(v.result), v.passed, v.total,
           v.failed, v.unfinished, v.general_faults,
           v.suspect ? "; SUSPECT: physics samples were missed" : "");
    printf("VERDICT_DIGEST %016llx\n", (unsigned long long)h);
}

static int do_park(const char* path, const char* name)
{
    RefCourse course;
    char err[256] = {0};
    if (ref_load_course(&course, "/work/course/simA_referee.txt", err, sizeof err) != 0) {
        printf("course load failed: %s\n", err);
        return 1;
    }
    ChsTerrainPatch patches[32];
    const int32_t np = course_load_terrain("/work/course/simA_terrain.txt", patches,
                                           32, err, sizeof err);
    if (np <= 0) { printf("terrain load failed: %s\n", err); return 1; }

    printf("\n=== JUDGED PARKING RUN: '%s' ===\n", name);
    const RefZone* zone = park_plan(&course, name, &g_plan);
    if (!zone) return 1;
    ParkPlan* P = &g_plan;
    const double L = course.wheelbase;
    const double full_len = P->s[P->n - 1];
    printf("  zone    : %d (exercise %d), heading %+.1f deg\n",
           zone->id, zone->exercise, zone->heading * 180.0 / M_PI);
    printf("  start   : rear axle (%.3f, %.3f) yaw %+.1f deg; %.2f m to go, "
           "reversing\n", P->x[0], P->y[0], P->h[0] * 180.0 / M_PI, full_len);

    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, patches, np);
    if (!w) { printf("Chs_CreateVehicleEx failed\n"); return 1; }

    ChsSnapshot snap;
    ChsVehicleFeedback fb;
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, -1, 0};    /* braked, clutch in, R */
    double roll, pitch, yaw;

    /* Settle on the brake, then fix the rigid map from physics to course. */
    const long settle = (long)(1.5 / DT);
    for (long i = 0; i < settle; ++i) { Chs_SetDriverInput(w, in); Chs_Step(w, DT); }
    Chs_GetSnapshot(w, &snap);
    euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
    const double th = P->h[0] - yaw;
    const double cs = cos(th), sn = sin(th);
    const double px0 = snap.bodies[0].position.x, py0 = snap.bodies[0].position.y;
    const double fx0 = P->x[0] + L * cos(P->h[0]), fy0 = P->y[0] + L * sin(P->h[0]);

    RefState ref;
    ref_init(&ref, &course);

    /* One leg, task 3's: cruise at PILOT_V_PARK throughout, 3 s hold. */
    const Leg leg = { name, P, PILOT_V_PARK, 0.0, 0.8, 0.55, 0.5, 3.0, 0 };
    Pilot p;
    pilot_begin(&p, &leg);
    const long n_max = (long)(60.0 / DT);

    for (long i = 0; i < n_max; ++i) {
        Chs_GetSnapshot(w, &snap);
        Chs_GetFeedback(w, &fb);
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
        const double dx = snap.bodies[0].position.x - px0;
        const double dy = snap.bodies[0].position.y - py0;
        const double cx = fx0 + cs * dx - sn * dy;          /* course frame, */
        const double cy = fy0 + sn * dx + cs * dy;          /* front axle    */
        if (pilot_step(&p, &leg, L, cx, cy, yaw + th, &fb, i * DT, &in))
            break;                                          /* dwell done    */

        if (getenv("PARK_TRACE") && i % 250 == 0)
            printf("  trace t=%6.2f v=%+.3f rem=%5.2f dev=%.3f thr=%.2f brk=%.2f clu=%.1f "
                   "rpm=%6.1f steer=%+.2f\n", (settle + i) * DT, fb.speed_mps, p.rem, p.dev,
                   in.throttle, in.braking, in.clutch, fb.engine_rpm, in.steering);
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);

        Chs_GetSnapshot(w, &snap);
        Chs_GetFeedback(w, &fb);
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
        const double qx = snap.bodies[0].position.x - px0;
        const double qy = snap.bodies[0].position.y - py0;
        RefVehicleState v;
        memset(&v, 0, sizeof v);
        v.t = (double)(settle + i) * DT;
        v.step = (uint64_t)(settle + i);
        v.x = fx0 + cs * qx - sn * qy;
        v.y = fy0 + sn * qx + cs * qy;
        v.yaw = yaw + th;
        v.speed = fb.speed_mps;
        v.wheel_speed = snap_wheel_speed(&snap, course.r_roll);
        v.engine_rpm = fb.engine_rpm;
        v.gear = fb.gear;
        v.clutch = in.clutch;
        ref_update(&ref, &v);
    }

    if (p.t_stop < 0.0) printf("\n  THE CAR NEVER STOPPED -- no park to judge\n");
    else {
        const double plan_fx = P->x[P->n - 1] + L * cos(P->h[P->n - 1]);
        const double plan_fy = P->y[P->n - 1] + L * sin(P->h[P->n - 1]);
        printf("\n  stopped : t=%.3f s, front axle (%.3f, %.3f) yaw %+.2f deg\n",
               (double)settle * DT + p.t_stop, p.stop_x, p.stop_y,
               p.stop_yaw * 180.0 / M_PI);
        printf("  vs plan : %.3f m from the planned end, %+.2f deg; path deviation "
               "max %.3f m\n", hypot(p.stop_x - plan_fx, p.stop_y - plan_fy),
               (p.stop_yaw - P->h[P->n - 1]) * 180.0 / M_PI, p.max_dev);
        printf("  in zone : clearance %+.4f m at the stop pose\n",
               ref_zone_clearance(&course, zone, p.stop_x, p.stop_y, p.stop_yaw));
    }

    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(&ref, ev, REF_MAX_EVENTS);
    printf("\n  %-8s %-14s %-8s %s\n", "t (s)", "event", "object", "detail");
    for (int32_t i = 0; i < n; ++i)
        printf("  %-8.3f %-14s %-8d %.4f\n",
               ev[i].t, ref_event_name(ev[i].type), ev[i].object_id, ev[i].detail);
    if (n == 0) printf("  (no events)\n");

    /* A parking run starts at the manoeuvre, never crossing the exercise's
     * entry gate, so the exercise never starts and the PARK_OK above is not
     * credited: INCOMPLETE is the correct verdict here, by design. The whole
     * test, where it is credited, is `judge course`. */
    print_verdict(&course, &ref);

    Chs_DestroyWorld(w);
    return 0;
}

/* ===========================================================================
 *  COURSE — the whole SIM A test, driven by the pilot and judged (task 4).
 *
 *  The verdict machine is host-tested against synthetic trajectories
 *  (referee/run.sh selftest). This is the same machine on a REAL one: the car
 *  starts at the course start and drives every exercise in order -- slalom,
 *  parallel park, bay park, hill start -- through Chrono, and the referee
 *  produces the verdict an examiner would. Nothing is teleported: unlike the
 *  parking runs, every exercise here is ENTERED through its entry gate, so
 *  this is the only run in which a PARK_OK is credited.
 *
 *  Six legs, each a plan the pilot follows in one direction to a stop:
 *
 *    A  start -> slalom -> the parallel set-up pose           forward
 *    B  into the parallel slot (task 3's plan)                reverse
 *    C  out of the slot -> the bay set-up pose                forward
 *    D  into the bay (task 3's plan)                          reverse
 *    E  out of the bay -> up the hill, stop past the line     forward
 *    F  hill start, over the crest                            forward
 *
 *  Everything is read off the course: the slalom's peaks from its gates, the
 *  parking plans from the zones, the stop from the hill's middle gate. The
 *  route was checked against every line, cone, gate and the parked cars
 *  before this was written (task 4 notes in CLAUDE.md).
 *
 *  THE SLALOM IS ARCS, NOT A SINE. The first prototype weaved the rear axle
 *  on a cosine: 92% of lock at its peaks and 0.12 m from a cone at best. The
 *  path of least peak curvature through the same peaks is a chain of equal
 *  arcs, R = (s^2/4 + A^2)/(2A) -- 6.08 m, 69% of lock -- and while the car
 *  is on the arc past a cone the turn centre lies beyond that cone, so every
 *  point of the body stays at least R - w/2 from it: 0.41 m of clearance by
 *  construction, not by tuning.
 *
 *  Variants, the negative cases the verdict must catch end to end:
 *    course         the clean test                              -> PASS
 *    course_cone    slalom weave too narrow to clear the cones  -> FAIL
 *    course_nostop  drives up and over the hill without stopping -> FAIL
 * ======================================================================== */
#define ROUTE_LEGS        6
static ParkPlan g_route[ROUTE_LEGS];   /* 6 MB: not on the stack */

#define ROUTE_WEAVE       1.5   /* m, rear axle either side of the cones     */
#define ROUTE_WEAVE_CONE  1.0   /* course_cone: body edge 0.085 m off-centre,
                                   inside a 0.15 m cone                       */
#define ROUTE_R_SHIFT     8.0   /* m, lane changes outside the slalom: ~50%
                                   of lock. 6 m left 0.30 m to the aisle edge
                                   on the way out of the bay; 8 m leaves 0.43 */
#define ROUTE_V           2.0   /* m/s between and through the exercises     */
#define ROUTE_PARKED_CAR  5.0   /* [EST] m, the car in front of the parallel
                                   slot: the lane change waits until the whole
                                   car is past it                            */
#define ROUTE_STOP_PAST   1.0   /* m, front axle past the hill's stop line   */
#define ROUTE_RUN_OUT     8.0   /* m, front axle past the crest at the end   */

static const RefGate* course_gate(const RefCourse* c, int32_t id)
{
    for (int32_t i = 0; i < c->n_gates; ++i)
        if (c->gates[i].id == id) return &c->gates[i];
    return NULL;
}

static const RefGate* course_gate_seq(const RefCourse* c, int32_t seq)
{
    for (int32_t i = 0; i < c->n_gates; ++i)
        if (c->gates[i].seq == seq) return &c->gates[i];
    return NULL;
}

static const RefExercise* course_exercise(const RefCourse* c, int32_t kind)
{
    for (int32_t i = 0; i < c->n_exercises; ++i)
        if (c->exercises[i].kind == kind) return &c->exercises[i];
    return NULL;
}

/* Builds the legs for `variant`. Returns the number of legs, or 0. */
static int route_plan(const RefCourse* c, const char* variant, Leg* legs)
{
    const double L = c->wheelbase;
    ParkPlan *A = &g_route[0], *B = &g_route[1], *C = &g_route[2],
             *D = &g_route[3], *E = &g_route[4], *F = &g_route[5];

    const RefZone* zp = park_plan(c, "park_parallel", B);
    const RefZone* zb = park_plan(c, "park_bay", D);
    const RefExercise* sl = course_exercise(c, REF_EXK_GATED);
    const RefExercise* hl = course_exercise(c, REF_EXK_HILL);
    if (!zp || !zb || !sl || !hl) { printf("course lacks an exercise the route needs\n"); return 0; }

    /* A. The slalom's peaks: one per cone gate, at its x, on its side. */
    double px[REF_MAX_GATES], py[REF_MAX_GATES];
    int npk = 0;
    const double amp = strstr(variant, "cone") ? ROUTE_WEAVE_CONE : ROUTE_WEAVE;
    for (int32_t q = 0; q < c->n_gates; ++q) {
        const RefGate* g = course_gate_seq(c, q);
        if (!g || g->exercise != sl->id || g->id == sl->entry_gate) continue;
        px[npk] = 0.5 * (g->x0 + g->x1);
        py[npk] = (g->y0 + g->y1) > 0.0 ? amp : -amp;
        ++npk;
    }
    if (npk < 2) { printf("slalom has %d cone gates\n", npk); return 0; }
    const double gap = px[1] - px[0];
    const double R_sl = (0.25 * gap * gap + amp * amp) / (2.0 * amp);
    plan_start(A, -L, 0.0, 0.0);               /* the spawn: front axle at 0 */
    plan_straight_to(A, px[0] - plan_shift_dx(py[0], R_sl));
    plan_shift(A, py[0], R_sl);
    for (int k = 1; k < npk; ++k) plan_shift_to(A, px[k], py[k]);
    plan_shift(A, B->y[0] - A->y[A->n - 1], R_sl);     /* to the parallel lane */
    plan_straight_to(A, B->x[0]);
    plan_finish(A, +1);

    /* C. Out of the slot, past the car in front, over to the bay lane. */
    double zx1 = -1e9;
    for (int32_t k = 0; k < zp->n; ++k) zx1 = fmax(zx1, zp->x[k]);
    plan_copy_reversed(C, B);
    plan_straight_to(C, zx1 + ROUTE_PARKED_CAR);
    plan_shift(C, D->y[0] - C->y[C->n - 1], ROUTE_R_SHIFT);
    plan_straight_to(C, D->x[0]);
    plan_finish(C, +1);

    /* E. Out of the bay, back to the centreline straight before the hill
     * exercise begins, and up to the stop: its middle gate is the line. */
    const RefGate* hin = course_gate(c, hl->entry_gate);
    const RefGate* hex = course_gate(c, hl->exit_gate);
    const RefGate* hst = NULL;
    for (int32_t i = 0; i < c->n_gates; ++i)
        if (c->gates[i].exercise == hl->id && c->gates[i].id != hl->entry_gate
            && c->gates[i].id != hl->exit_gate) hst = &c->gates[i];
    if (!hin || !hex || !hst) { printf("hill gates missing\n"); return 0; }
    const double yc = 0.5 * (hin->y0 + hin->y1);
    const double x_stop = 0.5 * (hst->x0 + hst->x1) + ROUTE_STOP_PAST;
    const double x_end  = 0.5 * (hex->x0 + hex->x1) + ROUTE_RUN_OUT;
    const int nostop = strstr(variant, "nostop") != NULL;
    plan_copy_reversed(E, D);
    const double dy = yc - E->y[E->n - 1];
    plan_straight_to(E, hin->x0 - 3.0 - plan_shift_dx(dy, ROUTE_R_SHIFT));
    plan_shift(E, dy, ROUTE_R_SHIFT);
    plan_straight_to(E, (nostop ? x_end : x_stop) - L);
    plan_finish(E, +1);

    /* F. The hill start, from the stop, over the crest. */
    plan_start(F, x_stop - L, yc, 0.0);
    plan_straight_to(F, x_end - L);
    plan_finish(F, +1);

    /*               name                             plan  cruise        slow            ld   thr   imax  hold  hill */
    const Leg spec[ROUTE_LEGS] = {
        { "A  start, slalom, to parallel set-up", A, ROUTE_V,      0.0,            0.8, 0.80, 1.0, 0.5, 0 },
        { "B  into the parallel slot",            B, PILOT_V_PARK, 0.0,            0.8, 0.55, 0.5, 3.0, 0 },
        { "C  out, to the bay set-up",            C, ROUTE_V,      B->s[B->n - 1], 0.8, 0.80, 1.0, 0.5, 0 },
        { "D  into the bay",                      D, PILOT_V_PARK, 0.0,            0.8, 0.55, 0.5, 3.0, 0 },
        { nostop ? "E  out, up and over the hill"
                 : "E  out, up the hill to the line", E, ROUTE_V,  D->s[D->n - 1], 0.8, 0.80, 1.0,
          nostop ? 1.5 : 2.0, 0 },
        { "F  hill start, over the crest",        F, ROUTE_V,      0.0,            0.8, 0.80, 1.0, 1.5, 1 },
    };
    for (int k = 0; k < ROUTE_LEGS; ++k) {
        legs[k] = spec[k];
        if (spec[k].plan->overflow) { printf("  leg %s: plan overflow\n", spec[k].name); return 0; }
    }
    printf("  slalom  : %d peaks, weave %.2f m, arcs R %.3f m (%.0f%% lock)\n",
           npk, amp, R_sl, 100.0 * atan(L / R_sl) / PARK_DELTA_MAX);
    return nostop ? 5 : ROUTE_LEGS;
}

static int do_course(const char* path, const char* variant)
{
    RefCourse course;
    char err[256] = {0};
    if (ref_load_course(&course, "/work/course/simA_referee.txt", err, sizeof err) != 0) {
        printf("course load failed: %s\n", err);
        return 1;
    }
    ChsTerrainPatch patches[32];
    const int32_t np = course_load_terrain("/work/course/simA_terrain.txt", patches,
                                           32, err, sizeof err);
    if (np <= 0) { printf("terrain load failed: %s\n", err); return 1; }

    printf("\n=== JUDGED TEST: '%s' -- the whole course, driven ===\n", variant);
    /* course_rollback: a hill start with the brake released at HALF the
     * clutch capacity that holds the car. It must roll back, and fail. */
    g_hill_c_margin = strstr(variant, "rollback") ? 0.5 : HILL_C_MARGIN;
    g_hill_react    = strstr(variant, "rollback") ? 0.5 : 0.0;
    Leg legs[ROUTE_LEGS];
    const int n_legs = route_plan(&course, variant, legs);
    if (n_legs == 0) return 1;
    const double L = course.wheelbase;
    for (int k = 0; k < n_legs; ++k) {
        const ParkPlan* P = legs[k].plan;
        printf("  leg %-36s %s %6.2f m  (%7.2f,%+6.2f) -> (%7.2f,%+6.2f)\n", legs[k].name,
               P->dir < 0 ? "rev" : "fwd", P->s[P->n - 1],
               P->x[0], P->y[0], P->x[P->n - 1], P->y[P->n - 1]);
    }

    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, patches, np);
    if (!w) { printf("Chs_CreateVehicleEx failed\n"); return 1; }

    ChsSnapshot snap;
    ChsVehicleFeedback fb;
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 1, 0};     /* braked, clutch in, 1st */
    double roll, pitch, yaw;

    /* Settle on the brake. The spawn IS the course start, so physics and the
     * course share a frame and nothing is transformed. The test begins when
     * the car is ready, as an examiner's would. */
    const long settle = (long)(1.5 / DT);
    for (long i = 0; i < settle; ++i) { Chs_SetDriverInput(w, in); Chs_Step(w, DT); }

    RefState ref;
    ref_init(&ref, &course);

    struct { double t_stop, x, y, yaw, max_dev; } rec[ROUTE_LEGS];
    memset(rec, 0, sizeof rec);
    int leg = 0;
    Pilot p;
    pilot_begin(&p, &legs[0]);
    const long n_max = (long)(400.0 / DT);
    double min_rpm_connected = 1e9;
    long i;

    for (i = 0; i < n_max && leg < n_legs; ++i) {
        Chs_GetSnapshot(w, &snap);
        Chs_GetFeedback(w, &fb);
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
        const double cx = snap.bodies[0].position.x, cy = snap.bodies[0].position.y;
        p.pitch = pitch;
        while (pilot_step(&p, &legs[leg], L, cx, cy, yaw, &fb, i * DT, &in)) {
            rec[leg].t_stop = (double)settle * DT + p.t_stop;
            rec[leg].x = p.stop_x; rec[leg].y = p.stop_y; rec[leg].yaw = p.stop_yaw;
            rec[leg].max_dev = p.max_dev;
            if (++leg == n_legs) break;
            pilot_begin(&p, &legs[leg]);
            p.pitch = pitch;
        }
        if (leg == n_legs) break;

        if (getenv("PARK_TRACE") && i % 250 == 0)
            printf("  trace %c t=%6.2f x=%7.2f y=%+6.2f v=%+.3f rem=%6.2f dev=%.3f thr=%.2f "
                   "brk=%.2f clu=%.2f rpm=%6.1f steer=%+.2f pitch=%+.1f\n", 'A' + leg,
                   (settle + i) * DT, cx, cy, fb.speed_mps, p.rem, p.dev, in.throttle,
                   in.braking, in.clutch, fb.engine_rpm, in.steering, pitch * 180.0 / M_PI);
        /* Any moment the clutch carries torque -- including a crawl's slip at
         * ~0.99 of pedal travel, which a "pedal < 0.5" test (the old one)
         * never sees. This is how close the drive came to a stall. */
        if (in.clutch < 0.99)
            min_rpm_connected = fmin(min_rpm_connected, fb.engine_rpm);
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);

        Chs_GetSnapshot(w, &snap);
        Chs_GetFeedback(w, &fb);
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);
        RefVehicleState v;
        memset(&v, 0, sizeof v);
        v.t = (double)(settle + i) * DT;
        v.step = (uint64_t)(settle + i);
        v.x = snap.bodies[0].position.x;
        v.y = snap.bodies[0].position.y;
        v.yaw = yaw;
        v.speed = fb.speed_mps;
        v.wheel_speed = snap_wheel_speed(&snap, course.r_roll);
        v.engine_rpm = fb.engine_rpm;
        v.gear = fb.gear;
        v.clutch = in.clutch;
        ref_update(&ref, &v);
    }

    printf("\n  %-38s %8s %9s %8s %8s\n", "leg", "stop t", "vs plan", "yaw err", "max dev");
    for (int k = 0; k < n_legs; ++k) {
        const ParkPlan* P = legs[k].plan;
        if (k >= leg) { printf("  %-38s  NOT COMPLETED\n", legs[k].name); continue; }
        const double fx = P->x[P->n - 1] + L * cos(P->h[P->n - 1]);
        const double fy = P->y[P->n - 1] + L * sin(P->h[P->n - 1]);
        printf("  %-38s %8.2f %7.3f m %+7.2f %6.3f m\n", legs[k].name, rec[k].t_stop,
               hypot(rec[k].x - fx, rec[k].y - fy),
               (rec[k].yaw - P->h[P->n - 1]) * 180.0 / M_PI, rec[k].max_dev);
    }
    printf("  run: %.1f s of test in %ld steps; lowest engine rpm while the clutch "
           "carried torque: %.0f\n", i * DT, i,
           min_rpm_connected < 1e8 ? min_rpm_connected : -1.0);

    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(&ref, ev, REF_MAX_EVENTS);
    printf("\n  %-8s %-14s %-8s %s\n", "t (s)", "event", "object", "detail");
    for (int32_t k = 0; k < n; ++k)
        printf("  %-8.3f %-14s %-8d %.4f\n",
               ev[k].t, ref_event_name(ev[k].type), ev[k].object_id, ev[k].detail);
    if (n == 0) printf("  (no events)\n");
    if (ref.dropped_events) printf("  (+%d dropped)\n", ref.dropped_events);

    print_verdict(&course, &ref);
    Chs_DestroyWorld(w);
    return 0;
}

/* ===========================================================================
 *  DIFF — is the rear differential open?
 *
 *  ShaftsDriveline2WD locks its differential with a ChShaftsClutch held at
 *  modulation 0, i.e. open -- IF the solver enforces the clutch's torque box.
 *  The pedal clutch is the same element, and `clutch` shows it rigid under
 *  MINRES. An open differential lets each rear wheel turn at its own ground
 *  speed; a locked one (a spool) forces equal spin and scrubs both rear tyres
 *  through every turn -- the regime of both parking exercises.
 *
 *  A slow, steady full-lock circle, left. Each rear wheel's spin against its
 *  centre's ground speed: open, the two ratios agree (inner/outer ~0.68 at
 *  this lock); a spool pins the spin ratio at 1.0 whatever the speeds are.
 * ======================================================================== */
static double spin_rate(const ChsBodyState* b)
{
    /* Spin axis = the spindle's local Y. GetSpindleAngVel is in the GLOBAL
     * frame (the snapshot inconsistency noted in CLAUDE.md), so project the
     * global angular velocity onto the rotated Y axis. */
    const ChsQuat* q = &b->rotation;
    const double ax = 2.0 * (q->x * q->y - q->w * q->z);
    const double ay = 1.0 - 2.0 * (q->x * q->x + q->z * q->z);
    const double az = 2.0 * (q->y * q->z + q->w * q->x);
    return b->angular_velocity.x * ax + b->angular_velocity.y * ay
         + b->angular_velocity.z * az;
}

static int do_diff(const char* path)
{
    ChsWorld w = make(path);
    if (!w) return 1;
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 1, 0};
    ChsVehicleFeedback fb;
    ChsSnapshot snap;
    run_(w, in, 1.0, &fb);                               /* settle, braked */

    in.steering = 0.9; in.braking = 0.0; in.clutch = 0.0;
    const double v_target = 1.0;
    double integ = 0.0, wl = 0, wr = 0, vl = 0, vr = 0, rpm = 0;
    long n = 0;
    for (long i = 0; i < (long)(14.0 / DT); ++i) {
        Chs_GetFeedback(w, &fb);
        const double e = v_target - fb.speed_mps;
        integ = fmax(-0.3, fmin(1.0, integ + e * DT));
        const double u = 0.06 + 1.2 * e + 0.6 * integ;
        in.throttle = u > 0.0 ? fmin(0.8, u) : 0.0;
        in.braking  = e < -0.05 ? fmin(0.3, -1.5 * e) : 0.0;
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);
        if (i * DT >= 8.0) {                             /* steady part */
            Chs_GetSnapshot(w, &snap);
            wl += fabs(spin_rate(&snap.bodies[3]));
            wr += fabs(spin_rate(&snap.bodies[4]));
            vl += hypot(snap.bodies[3].linear_velocity.x, snap.bodies[3].linear_velocity.y);
            vr += hypot(snap.bodies[4].linear_velocity.x, snap.bodies[4].linear_velocity.y);
            rpm += fb.engine_rpm;
            ++n;
        }
    }
    wl /= n; wr /= n; vl /= n; vr /= n; rpm /= n;
    const double rw = wl / wr, rv = vl / vr;
    printf("\n=== REAR DIFFERENTIAL, steady full-lock circle at %.1f m/s ===\n", v_target);
    printf("  spin   RL %.4f  RR %.4f rad/s   inner/outer %.4f\n", wl, wr, rw);
    printf("  ground RL %.4f  RR %.4f m/s     inner/outer %.4f\n", vl, vr, rv);
    printf("  implied rolling radius  RL %.4f  RR %.4f m   (engine %.0f rpm)\n",
           vl / wl, vr / wr, rpm);
    const int open = fabs(rw - 1.0) >= 0.02 && fabs(rw - rv) < 0.03;
    printf("  verdict: %s\n",
           fabs(rw - 1.0) < 0.02 ? "LOCKED -- the rear axle is a spool; both tyres scrub"
           : open ? "OPEN -- each wheel turns at its own ground speed"
           : "PARTIAL -- neither open nor locked");
    Chs_DestroyWorld(w);
    return open ? 0 : 1;          /* asserted: the Innova has an open diff */
}

/* ===========================================================================
 *  SOLVERS — which solver, and how many iterations? (task 13)
 *
 *  MINRES cannot open a clutch: a clutch is a BOXED constraint, and only a
 *  VI solver projects the box. PSOR can. What decides the switch is whether
 *  PSOR, at an affordable iteration count, still solves the rest of the car
 *  -- the double wishbones' closed loops -- as well. Three scenarios each:
 *
 *    statics  braked in neutral, 4 s: ride height, pitch, front load.
 *             SPARSE_QR solves the joints EXACTLY, so its row is the
 *             reference (it drops the clutch box too, which a braked car in
 *             neutral does not notice).
 *    turn     steady circle, steer 0.3 at 5 m/s in 1st: yaw rate, lateral
 *             acceleration, roll, column torque. The rear differential
 *             matters here, so the reference is PSOR at 1000 iterations --
 *             converged AND box-correct.
 *    launch   1st gear, clutch dumped at 50% throttle from rest, 3 s.
 *
 *  The shim is configured through its diagnostic environment variables,
 *  set here between worlds; it reads them when a world is created. Each
 *  world's convergence line (CHS_SOLVER_STATS) follows its row.
 *
 *  SINCE TASK 15 THE BRAKES ARE CLUTCHES TOO (BrakeShafts), and a linear
 *  solver welds them: under qr/minres the car cannot move, so those rows
 *  mean something only for a vehicle built with BRAKE_MODEL=simple -- which
 *  is how task 13's comparison was made.
 * ======================================================================== */
typedef struct { const char *label, *solver, *iters, *warm; int open_diff; } SolverCfg;

static void solver_env(const SolverCfg* c)
{
    setenv("CHS_SOLVER", c->solver, 1);
    if (c->iters) setenv("CHS_SOLVER_ITERS", c->iters, 1); else unsetenv("CHS_SOLVER_ITERS");
    if (c->warm)  setenv("CHS_SOLVER_WARM", c->warm, 1);   else unsetenv("CHS_SOLVER_WARM");
    if (c->open_diff) setenv("CHS_OPEN_DIFF", "1", 1);     else unsetenv("CHS_OPEN_DIFF");
    setenv("CHS_SOLVER_STATS", "1", 1);
}

static int do_solvers(const char* path)
{
    /* Round 1 (CLAUDE.md, task 13) also ran PSOR 30/100/300 and warm
     * starts: warm-started PSOR DIVERGES on this car (residual 1-3), and
     * cold PSOR 50 already matches PSOR 1000 in the turn to < 1%. */
    static const SolverCfg cfg[] = {
        { "qr (exact)",   "qr",     NULL,   NULL, 0 },
        { "qr open diff", "qr",     NULL,   NULL, 1 },
        { "minres 60",    "minres", "60",   NULL, 0 },
        { "minres open",  "minres", "60",   NULL, 1 },
        { "psor 50",      "psor",   "50",   NULL, 0 },
        { "psor 1000",    "psor",   "1000", NULL, 0 },
    };
    const int n_cfg = (int)(sizeof cfg / sizeof cfg[0]);
    ChsVehicleFeedback fb;
    ChsSnapshot s;
    double roll, pitch, yaw;

    printf("\n=== SOLVERS: statics (unbraked) / steady turn / launch ===\n");
    for (int k = 0; k < n_cfg; ++k) {
        solver_env(&cfg[k]);

        /* statics -- UNBRAKED. Braked, the parking hold (which opposes the
         * chassis' full 3-D velocity, vertical included) freezes the body
         * at the bottom of its first spring compression: round 1's braked
         * "statics" read 0.11 m ride height and 74% front load. */
        ChsWorld w = Chs_CreateVehicle(path, 0.45);
        if (!w) { printf("  %s: create failed\n", cfg[k].label); continue; }
        ChsDriverInput in = {0.0, 0.0, 0.0, 1.0, 0, 0};
        double us = 0.0; long n = 0;
        for (long i = 0; i < (long)(4.0 / DT); ++i) {
            Chs_SetDriverInput(w, in); us += Chs_Step(w, DT); ++n;
        }
        Chs_GetSnapshot(w, &s); Chs_GetFeedback(w, &fb);
        euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
        printf("\n  %-14s statics  z %.5f m  pitch %+.4f deg  front Fz %.1f N (%.2f%%)  "
               "col %+.4f N m   %.0f us/step\n", cfg[k].label, s.bodies[0].position.z,
               pitch * 180.0 / M_PI, fb.tire_fz_n[0] + fb.tire_fz_n[1],
               100.0 * (fb.tire_fz_n[0] + fb.tire_fz_n[1]) / 17658.0,
               fb.column_torque_nm, us / n);
        fflush(stdout); Chs_DestroyWorld(w); fflush(stderr);

        /* steady turn */
        w = Chs_CreateVehicle(path, 0.45);
        ChsDriverInput t = {0.0, 0.0, 1.0, 1.0, 1, 0};
        run_(w, t, 0.5, &fb);
        t.steering = 0.3; t.braking = 0.0; t.clutch = 0.0;
        double integ = 0.0, yr = 0, vv = 0, rl = 0, col = 0; long m = 0;
        for (long i = 0; i < (long)(12.0 / DT); ++i) {
            Chs_GetFeedback(w, &fb);
            const double e = 5.0 - fb.speed_mps;
            integ = fmax(-0.3, fmin(1.0, integ + e * DT));
            const double u = 0.06 + 0.8 * e + 0.4 * integ;
            t.throttle = u > 0.0 ? fmin(0.9, u) : 0.0;
            t.braking  = e < -0.2 ? fmin(0.3, -0.5 * e) : 0.0;
            Chs_SetDriverInput(w, t);
            Chs_Step(w, DT);
            if (i * DT >= 8.0) {
                Chs_GetSnapshot(w, &s); Chs_GetFeedback(w, &fb);
                euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
                yr += s.bodies[0].angular_velocity.z; vv += fb.speed_mps;
                rl += roll; col += fb.column_torque_nm; ++m;
            }
        }
        yr /= m; vv /= m; rl /= m; col /= m;
        printf("  %-14s turn     v %.3f m/s  yaw %.3f deg/s  a_lat %.3f m/s^2  "
               "roll %+.3f deg  col %+.3f N m\n", cfg[k].label, vv, yr * 180.0 / M_PI,
               vv * yr, rl * 180.0 / M_PI, col);
        fflush(stdout); Chs_DestroyWorld(w); fflush(stderr);

        /* launch */
        w = Chs_CreateVehicle(path, 0.45);
        ChsDriverInput l = {0.0, 0.0, 1.0, 1.0, 1, 0};
        run_(w, l, 1.0, &fb);
        Chs_GetSnapshot(w, &s);
        const double x0 = s.bodies[0].position.x;
        l.braking = 0.0; l.clutch = 0.0; l.throttle = 0.5;
        double rpm1 = 0.0;
        Chs_SetDriverInput(w, l);
        for (long i = 0; i < (long)(3.0 / DT); ++i) {
            Chs_Step(w, DT);
            if (i == (long)(1.0 / DT)) { Chs_GetFeedback(w, &fb); rpm1 = fb.engine_rpm; }
        }
        Chs_GetSnapshot(w, &s); Chs_GetFeedback(w, &fb);
        printf("  %-14s launch   %.3f m in 3 s, %.3f m/s, rpm %.0f at 1 s, %.0f at 3 s\n",
               cfg[k].label, s.bodies[0].position.x - x0, fb.speed_mps, rpm1, fb.engine_rpm);
        fflush(stdout); Chs_DestroyWorld(w); fflush(stderr);
    }
    unsetenv("CHS_SOLVER"); unsetenv("CHS_SOLVER_ITERS");
    unsetenv("CHS_SOLVER_WARM"); unsetenv("CHS_SOLVER_STATS");
    return 0;
}

/* ===========================================================================
 *  HOLD — a braked car at rest (task 15). ASSERTED: exits non-zero on failure.
 *
 *  The parking hold (Chs_SetParkingHold) was built to supply the static
 *  friction a force-element tyre lacks, and as first built it opposed the
 *  chassis' full 3-D velocity: while braked and slow it froze the body
 *  wherever it was, heave included. Task 15 found the reason it was needed
 *  at all: ChBrakeSimple's sticking is dead code in Chrono 10, so a braked
 *  wheel at rest CHATTERS instead of holding. Each check is a property the
 *  braked car must have, run on the shim's DEFAULT configuration; the rows
 *  marked (info) force the hold on or off to show what it changes.
 *
 *    chatter   a braked wheel at rest is STUCK, not oscillating.
 *    flat      a braked spawn settles where a free one does, and braking a
 *              settled car moves it only by the physical brake-lock effect.
 *    slope     12%, stopped on it the way a driver stops: it holds, and the
 *              drift STOPS -- a tyre's pre-slide deflection (F/sigma0 ~ 5 mm)
 *              is fine, a creep is not.
 *    grades    it holds up to the friction limit and slides beyond it: a
 *              brake that welds the car to any slope is as wrong as one
 *              that holds it on none.
 *    iters     the slope result does not depend on the solver's iteration
 *              count (the 3-D hold slid 1.26 m at PSOR 200 and held at 50).
 * ======================================================================== */
static ChsWorld hold_world(const char* path, double grade)
{
    if (grade <= 0.0) return Chs_CreateVehicle(path, 0.45);
    ChsTerrainPatch ramp;              /* do_hill's ramp: through the origin */
    memset(&ramp, 0, sizeof ramp);
    const double run = 240.0;
    ramp.length = run * sqrt(1.0 + grade * grade);
    ramp.width = 20.0;
    ramp.pitch_rad = -atan2(run * grade, run);
    ramp.friction = 0.9;
    return Chs_CreateVehicleEx(path, 0.45, &ramp, 1);
}

typedef struct { double x, z, pitch, front; } HoldState;

static HoldState hold_state(ChsWorld w)
{
    ChsSnapshot s; ChsVehicleFeedback fb; double roll, pitch, yaw;
    Chs_GetSnapshot(w, &s); Chs_GetFeedback(w, &fb);
    euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
    HoldState h = { s.bodies[0].position.x, s.bodies[0].position.z,
                    pitch * 180.0 / M_PI, fb.tire_fz_n[0] + fb.tire_fz_n[1] };
    return h;
}

/* hold: 1 on, 0 off, -1 the shim's default. */
static void hold_set(ChsWorld w, int hold) { if (hold >= 0) Chs_SetParkingHold(w, hold); }

/* Settles from the spawn for 4 s; returns the final state and fills z(t)
 * every 10 ms. */
static HoldState hold_settle(const char* path, double brake, int hold, double* zt)
{
    ChsWorld w = hold_world(path, 0.0);
    hold_set(w, hold);
    ChsDriverInput in = {0.0, 0.0, brake, 1.0, 0, 0};    /* clutch down, neutral */
    ChsSnapshot s;
    Chs_SetDriverInput(w, in);
    for (long i = 0; i < 4000; ++i) {
        Chs_Step(w, DT);
        if (i % 10 == 0) { Chs_GetSnapshot(w, &s); zt[i / 10] = s.bodies[0].position.z; }
    }
    const HoldState h = hold_state(w);
    Chs_DestroyWorld(w);
    return h;
}

/* Flat: settle FREE for 3 s, then brake for 3 s. */
static void hold_flat(const char* path, int hold, HoldState* before, HoldState* after)
{
    ChsWorld w = hold_world(path, 0.0);
    hold_set(w, hold);
    ChsDriverInput in = {0.0, 0.0, 0.0, 1.0, 0, 0};       /* clutch down, neutral */
    ChsVehicleFeedback fb;
    run_(w, in, 3.0, &fb);
    *before = hold_state(w);
    in.braking = 1.0;
    run_(w, in, 3.0, &fb);
    *after = hold_state(w);
    Chs_DestroyWorld(w);
}

/* A slope the way a driver meets one: spawn on a flat top, drive over the
 * crest of a DESCENDING ramp at walking pace, brake to a stop with the whole
 * car on the grade, then hold. (Spawning a level car straight onto a steep
 * ramp is a crash test: at 25% the rear axle drops 0.69 m and the car
 * tumbles, pitch -38 then +74 deg -- task 15's first steep-grade rows.)
 * Downhill, so any grade can be reached without climbing traction.
 * Returns the drift along the slope over the hold; `tail` gets its last 2 s
 * (a tyre's pre-slide deflection stops, a creep does not). */
static double hold_ramp(const char* path, double grade, int hold, double secs,
                        double* tail, HoldState* out)
{
    ChsTerrainPatch p[2];
    memset(p, 0, sizeof p);
    const double run = 40.0, th = atan(grade);
    p[0].center_x = -10.0; p[0].length = 30.0;                  /* top: -25..5 */
    p[0].width = 20.0; p[0].friction = 0.9;
    p[1].center_x = 5.0 + run / 2.0;                            /* ramp: 5.. */
    p[1].center_z = -grade * run / 2.0;
    p[1].length = run * sqrt(1.0 + grade * grade);
    p[1].width = 20.0; p[1].pitch_rad = th;                     /* + descends */
    p[1].friction = 0.9;
    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, p, 2);
    hold_set(w, hold);

    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 1, 0};
    ChsVehicleFeedback fb; ChsSnapshot s;
    run_(w, in, 1.5, &fb);                                      /* settle */
    in.braking = 0.0; in.clutch = 0.0;
    double t = 0.0;
    for (;;) {                                                   /* ~1 m/s */
        Chs_GetFeedback(w, &fb); Chs_GetSnapshot(w, &s);
        if (s.bodies[0].position.x > 5.0 + 2.75 + 2.5 || t > 30.0) break;
        const double e = 1.0 - fb.speed_mps;
        in.throttle = e > 0.0 ? fmin(0.4, 0.3 + 0.5 * e) : 0.0;
        in.braking  = e < 0.0 ? fmin(1.0, -1.0 * e) : 0.0;
        Chs_SetDriverInput(w, in); Chs_Step(w, DT); t += DT;
    }
    in.throttle = 0.0; in.braking = 1.0; in.clutch = 1.0;       /* stop */
    run_(w, in, 2.0, &fb);
    const HoldState a = hold_state(w);
    run_(w, in, secs - 2.0, &fb);
    const HoldState b = hold_state(w);
    run_(w, in, 2.0, &fb);
    const HoldState c = hold_state(w);
    if (tail) *tail = (c.x - b.x) / cos(th);
    if (out) *out = c;
    Chs_DestroyWorld(w);
    return (c.x - a.x) / cos(th);
}

static int do_hold(const char* path)
{
    int fails = 0;
    printf("\n=== A BRAKED CAR AT REST (task 15) ===\n");

    /* --- chatter ----------------------------------------------------------- */
    {
        ChsWorld w = hold_world(path, 0.0);
        ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 0, 0};
        ChsVehicleFeedback fb; ChsSnapshot s;
        run_(w, in, 3.0, &fb);
        int flips = 0; double wmax = 0.0, prev = 0.0;
        for (int i = 0; i < 20; ++i) {
            Chs_Step(w, DT);
            Chs_GetSnapshot(w, &s);
            const double om = spin_rate(&s.bodies[3]);          /* rear left */
            if (i > 0 && (om > 0.0) != (prev > 0.0)) ++flips;
            prev = om; wmax = fmax(wmax, fabs(om));
        }
        const int stuck = wmax < 0.01;
        printf("\n  chatter : braked rear wheel at rest, 20 steps: max spin %.3f rad/s, "
               "%d sign flips -> %s\n", wmax, flips, stuck ? "STUCK" : "CHATTERS");
        Chs_DestroyWorld(w);
        fails += !stuck;
    }

    /* --- flat --------------------------------------------------------------
     * Braking MAY move a settled car a little, and physically should: with
     * the wheels locked the contact patches carry the longitudinal forces the
     * suspension geometry makes under static load (~400 N here; a free wheel
     * rolls them away), and anti-dive/anti-squat turns that into a few mm of
     * jacking -- measured +3.6 mm, 0.08 deg, 0.6% of front load, 1 mm of
     * tyre pre-slide. What must NOT happen is the 3-D hold's freeze: a braked
     * spawn stopped 125 mm short of settled, 2.2 deg, 44% of front load. The
     * bounds sit just above the physical effect and 25x under the defect. */
    printf("\n  flat                                    dz (mm)   dpitch (deg)  dfront (N)  moved (mm)\n");
    {   /* the literal symptom: a braked spawn against a free one */
        static double zf[400], zb[400];
        const HoldState f = hold_settle(path, 0.0, -1, zf);
        const HoldState b = hold_settle(path, 1.0, -1, zb);
        const double dz = (b.z - f.z) * 1000.0, dp = b.pitch - f.pitch, df = b.front - f.front;
        const int ok = fabs(dz) < 5.0 && fabs(dp) < 0.2 && fabs(df) < 0.015 * f.front;
        printf("    %-34s %+7.2f   %+8.4f     %+8.1f        -    %s\n",
               "braked spawn vs free spawn", dz, dp, df, ok ? "settles" : "FROZEN");
        fails += !ok;
    }
    static const struct { int hold; const char* label; } flat_rows[] = {
        { -1, "settled free, then braked" }, { 1, "  ... hold ON  (info)" },
        { 0, "  ... hold OFF (info)" } };
    for (unsigned k = 0; k < 3; ++k) {
        HoldState b, a;
        hold_flat(path, flat_rows[k].hold, &b, &a);
        const double dz = (a.z - b.z) * 1000.0, dp = a.pitch - b.pitch;
        const double df = a.front - b.front, dx = (a.x - b.x) * 1000.0;
        const int ok = fabs(dz) < 5.0 && fabs(dp) < 0.2 && fabs(df) < 0.015 * b.front
                    && fabs(dx) < 5.0;
        printf("    %-34s %+7.2f   %+8.4f     %+8.1f    %+7.2f   %s\n", flat_rows[k].label,
               dz, dp, df, dx, ok ? "ok" : "MOVED");
        if (flat_rows[k].hold < 0) fails += !ok;
    }

    /* --- slope -------------------------------------------------------------
     * The course's hill is 12%. Held, and the drift has STOPPED: under 1 cm
     * in all and under 1 mm over the last 2 s. */
    HoldState st;
    double tail;
    printf("\n  12%% slope, stopped on it, held 20 s     drift     last 2 s   pitch   front Fz\n");
    const double d = hold_ramp(path, 0.12, -1, 20.0, &tail, &st);
    const int slope_ok = fabs(d) < 0.010 && fabs(tail) < 0.001;
    printf("    %-34s %+8.4f m  %+7.4f m  %+6.2f  %8.1f   %s\n", "default", d, tail,
           st.pitch, st.front, slope_ok ? "HELD" : "NOT HELD");
    fails += !slope_ok;
    for (int hold = 1; hold >= 0; --hold) {
        const double di = hold_ramp(path, 0.12, hold, 20.0, &tail, &st);
        printf("    %-34s %+8.4f m  %+7.4f m  %+6.2f  %8.1f\n",
               hold ? "hold ON  (info)" : "hold OFF (info)", di, tail, st.pitch, st.front);
    }

    /* --- grades -------------------------------------------------------------
     * Four braked wheels on mu = 0.9 hold up to tan(theta) ~ mu. 25% and 45%
     * must hold; 120% (50 deg) is past friction and must slide. */
    printf("\n  grades, stopped on the grade, held 8 s  drift\n");
    static const double grades[] = { 0.25, 0.45, 1.20 };
    for (unsigned k = 0; k < 3; ++k) {
        const double dg = hold_ramp(path, grades[k], -1, 8.0, NULL, NULL);
        const int must_hold = grades[k] < 0.8;
        const int ok = must_hold ? fabs(dg) < 0.02 : fabs(dg) > 0.5;
        printf("    %3.0f%%                                %+8.4f m   %s (%s)\n",
               grades[k] * 100.0, dg, fabs(dg) < 0.02 ? "held" : "slides",
               ok ? "as it must" : "WRONG");
        fails += !ok;
    }

    /* --- iters ------------------------------------------------------------- */
    printf("\n  12%% slope vs PSOR iterations            drift over 8 s\n");
    static const char* const its[] = { "50", "100", "200", "300", "1000" };
    int iters_ok = 1;
    for (unsigned k = 0; k < sizeof its / sizeof its[0]; ++k) {
        setenv("CHS_SOLVER", "psor", 1);
        setenv("CHS_SOLVER_ITERS", its[k], 1);
        const double di = hold_ramp(path, 0.12, -1, 8.0, NULL, NULL);
        printf("    PSOR %-5s                           %+8.4f m\n", its[k], di);
        fflush(stdout);
        iters_ok &= fabs(di) < 0.010;
    }
    unsetenv("CHS_SOLVER"); unsetenv("CHS_SOLVER_ITERS");
    printf("    -> %s\n", iters_ok ? "holds at every iteration count"
                                   : "DEPENDS ON THE ITERATION COUNT");
    fails += !iters_ok;

    printf("\n  %s\n", fails ? "HOLD CHECKS FAILED" : "ALL HOLD CHECKS PASSED");
    return fails ? 1 : 0;
}

/* HOLDTRACE <grade> <hold>: a braked spawn on a slope, traced every 50 ms --
 * in-plane speed, the four wheels' spin, pitch. Tells a brake that lets the
 * wheels turn (spins non-zero) from a tyre that lets a locked wheel slide
 * (spins zero, car moving). hold: 1 on, 0 off, -1 the shim's default; with
 * a third argument `land`, the hold is on for the first 3 s only, so the
 * landing is held and what follows is the brake alone. */
static int do_holdtrace(const char* path, double grade, int hold, int land)
{
    ChsWorld w = hold_world(path, grade);
    hold_set(w, land ? 1 : hold);
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 0, 0};
    ChsSnapshot s; double roll, pitch, yaw;
    const double th = atan(grade);
    printf("\n=== HOLDTRACE: %.0f%% grade, hold %s ===\n", grade * 100.0,
           land ? "ON for the landing, then OFF" : hold > 0 ? "ON" : hold == 0 ? "OFF" : "default");
    printf("  t(s)   x (m)      v_slope   spin FL   FR       RL       RR      pitch\n");
    Chs_SetDriverInput(w, in);
    for (long i = 0; i <= 8000; ++i) {
        if (land && i == 3000) hold_set(w, 0);
        if (i % 50 == 0 && (i < 1500 || i % 500 == 0)) {
            Chs_GetSnapshot(w, &s);
            euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
            const ChsVec3 v = s.bodies[0].linear_velocity;
            const double vs = v.x * cos(th) + v.z * sin(th);      /* along the slope */
            printf("  %5.2f  %+8.4f  %+8.4f  %+7.3f  %+7.3f  %+7.3f  %+7.3f  %+7.2f\n",
                   i * DT, s.bodies[0].position.x, vs, spin_rate(&s.bodies[1]),
                   spin_rate(&s.bodies[2]), spin_rate(&s.bodies[3]),
                   spin_rate(&s.bodies[4]), pitch * 180.0 / M_PI);
        }
        Chs_Step(w, DT);
    }
    Chs_DestroyWorld(w);
    return 0;
}

/* ===========================================================================
 *  ATTITUDE — where the suspension actually sits at rest (task 14).
 *
 *  Settles unbraked on flat ground and expresses each spindle in the CHASSIS
 *  frame. At design ride height every spindle sits at z = 0 of its
 *  suspension frame (gen_innova.py builds the hardpoints that way), front at
 *  x = 0 and rear at x = -wheelbase. A spindle above its design point is
 *  suspension in bump -- that corner of the body sits LOW.
 * ======================================================================== */
static void chassis_local(const ChsBodyState* c, ChsVec3 p, double out[3])
{
    const ChsQuat* q = &c->rotation;
    const double w = q->w, x = q->x, y = q->y, z = q->z;
    const double R[3][3] = {                          /* body -> world */
        { 1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y) },
        { 2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x) },
        { 2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y) } };
    const double d[3] = { p.x - c->position.x, p.y - c->position.y, p.z - c->position.z };
    for (int i = 0; i < 3; ++i)                       /* R^T d */
        out[i] = R[0][i] * d[0] + R[1][i] * d[1] + R[2][i] * d[2];
}

/* ===========================================================================
 *  ATTITUDE (task 14) -- does the car sit at its DESIGN ride height?
 *
 *  Every suspension frame puts the wheel centre at z = 0, so a spindle's z in
 *  the CHASSIS frame is its bump travel away from design. Settled, unbraked,
 *  in neutral, all four should be ~0, the body level, and the front axle
 *  should carry the designed 53%. Before task 14: front +84 mm, rear +289 mm,
 *  pitch -4.37 deg (nose up), front 51.2%.
 *
 *  The spawn drop is a disturbance a force-element tyre can REMEMBER: the
 *  contact patch scrubs sideways as the suspension compresses, and the Dahl
 *  bristles hold that as a lateral preload, which the wishbones turn into a
 *  jacking force. So the settle is repeated from several spawn heights, and a
 *  longer one, to separate the design ride height from the drop.
 * ======================================================================== */
typedef struct { double z, pitch, roll, bump[4], front_pct; } Attitude;

static Attitude attitude_once(const char* path, double init_z, double settle)
{
    Attitude a;
    ChsWorld w = Chs_CreateVehicle(path, init_z);
    ChsDriverInput in = {0.0, 0.0, 0.0, 1.0, 0, 0};
    ChsVehicleFeedback fb; ChsSnapshot s; double yaw;
    run_(w, in, settle, &fb);
    Chs_GetSnapshot(w, &s);
    euler_zyx(&s.bodies[0].rotation, &a.roll, &a.pitch, &yaw);
    a.pitch *= 180.0 / M_PI; a.roll *= 180.0 / M_PI;
    a.z = s.bodies[0].position.z;
    for (int i = 0; i < 4; ++i) {
        double l[3];
        chassis_local(&s.bodies[0], s.bodies[1 + i].position, l);
        a.bump[i] = l[2] * 1000.0;                     /* design z = 0, in mm */
    }
    a.front_pct = 100.0 * (fb.tire_fz_n[0] + fb.tire_fz_n[1]) / 17658.0;
    Chs_DestroyWorld(w);
    return a;
}

static int do_attitude(const char* path)
{
    static const char* const names[] = { "FL", "FR", "RL", "RR" };
    const Attitude a = attitude_once(path, 0.45, 6.0);
    printf("\n=== STATIC ATTITUDE (6 s, unbraked, neutral, spawn z 0.45) ===\n");
    printf("  chassis ref z %.4f m, pitch %+.3f deg (negative = nose UP), roll %+.3f deg\n",
           a.z, a.pitch, a.roll);
    for (int i = 0; i < 4; ++i)
        printf("  %s bump vs design %+6.1f mm\n", names[i], a.bump[i]);
    printf("  front axle load %.2f%% of 17658 N (designed 53%%)\n", a.front_pct);

    printf("\n  sensitivity to the spawn drop:\n");
    printf("  spawn z  settle   pitch     front bump  rear bump  front load\n");
    static const double spawn[][2] = { {0.45, 6.0}, {0.45, 12.0}, {0.36, 6.0}, {0.33, 6.0} };
    for (int k = 0; k < 4; ++k) {
        const Attitude b = attitude_once(path, spawn[k][0], spawn[k][1]);
        printf("  %.2f     %4.1f s  %+.3f   %+6.1f mm   %+6.1f mm  %.2f%%\n",
               spawn[k][0], spawn[k][1], b.pitch,
               0.5 * (b.bump[0] + b.bump[1]), 0.5 * (b.bump[2] + b.bump[3]), b.front_pct);
    }

    int fails = 0;
    #define ATT_CHECK(cond, ...) do { if (!(cond)) { ++fails; printf("  FAIL: " __VA_ARGS__); printf("\n"); } } while (0)
    ATT_CHECK(fabs(a.pitch) < 0.3, "pitch %+.3f deg, want |pitch| < 0.3", a.pitch);
    ATT_CHECK(fabs(a.roll) < 0.05, "roll %+.3f deg, want |roll| < 0.05", a.roll);
    for (int i = 0; i < 4; ++i)
        ATT_CHECK(fabs(a.bump[i]) < 5.0, "%s bump %+.1f mm, want |bump| < 5", names[i], a.bump[i]);
    ATT_CHECK(fabs(a.front_pct - 53.0) < 0.5, "front load %.2f%%, want 53 +- 0.5", a.front_pct);
    #undef ATT_CHECK
    printf("\n%s\n", fails ? "ATTITUDE: FAILED" : "ATTITUDE: at design ride height, level, 53% front");
    return fails ? 1 : 0;
}

/* ===========================================================================
 *  JUDGE — the whole chain, headless and deterministic.
 *
 *  script -> shim -> Chrono -> referee -> verdict, with no editor, no wheel
 *  and no human. This is what Milestone 6.1 was built for: every rule is a
 *  predicate over a trajectory, the trajectory is bit-reproducible (verified
 *  in-process and cross-process), so a changed verdict means a changed RULE
 *  and never a changed scheduler.
 *
 *  The run is CLOSED-LOOP on lane position. The car does not track straight --
 *  drive torque reacts as a roll moment, roll gives roll steer, and a
 *  solid-axle RWD car curves under power at ~1590 m radius. Open-loop, that is
 *  4.7 m of lateral error by x=122, which would score the drift instead of the
 *  driver. drv_lane_keep closes it; determinism survives because the
 *  controller is a pure function of state and the state is reproducible.
 * ======================================================================== */
static int do_judge(const char* path, const char* script) {
    /* The parking exercises need steering the lane keeper cannot give, and
     * the whole test needs all of them. */
    if (!strncmp(script, "park_", 5))  return do_park(path, script);
    if (!strncmp(script, "course", 6)) return do_course(path, script);

    RefCourse course;
    char err[256] = {0};
    if (ref_load_course(&course, "/work/course/simA_referee.txt", err, sizeof err) != 0) {
        printf("course load failed: %s\n", err);
        return 1;
    }
    ChsTerrainPatch patches[32];
    const int32_t np = course_load_terrain("/work/course/simA_terrain.txt", patches,
                                           32, err, sizeof err);
    if (np <= 0) { printf("terrain load failed: %s\n", err); return 1; }

    DrvScript sc;
    if (resolve_script(script, &sc, err, sizeof err) != 0) {
        printf("cannot resolve script '%s': %s\n", script, err);
        return 1;
    }

    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, patches, np);
    if (!w) { printf("Chs_CreateVehicleEx failed\n"); drv_script_free(&sc); return 1; }

    RefState ref;
    ref_init(&ref, &course);

    const DrvLaneKeep lk = DRV_LANEKEEP_DEFAULT;
    const long nsteps = (long)((sc.duration + 0.5) / DT);

    printf("\n=== JUDGED RUN ===\n");
    printf("  course : %d cones, %d lines, %d gates\n",
           course.n_cones, course.n_lines, course.n_gates);
    printf("  terrain: %d patches\n", np);
    printf("  script : '%s', %.2f s -> %ld steps\n\n", script, sc.duration, nsteps);

    ChsSnapshot snap;
    ChsVehicleFeedback fb;
    for (long i = 0; i < nsteps; ++i) {
        DrvInput d;
        drv_script_eval(&sc, (double)i * DT, &d);

        Chs_GetSnapshot(w, &snap);
        double roll, pitch, yaw;
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);

        /* Lane keeping OVERRIDES the script's steering. The scripts describe
         * pedals and gears; holding a lane is the closed-loop part and the
         * script has no way to know where the car actually is. */
        const double steer_dev = drv_lane_keep(&lk, snap.bodies[0].position.y, yaw);

        ChsDriverInput in;
        in.steering = -steer_dev;      /* the one device->Chrono flip */
        in.throttle = d.throttle;
        in.braking  = d.brake;
        in.clutch   = d.clutch;
        in.gear     = d.gear;
        in._pad     = 0;
        Chs_SetDriverInput(w, in);
        Chs_Step(w, DT);

        Chs_GetSnapshot(w, &snap);
        Chs_GetFeedback(w, &fb);
        euler_zyx(&snap.bodies[0].rotation, &roll, &pitch, &yaw);

        RefVehicleState v;
        memset(&v, 0, sizeof v);
        v.t = (double)i * DT;
        v.step = (uint64_t)i;
        v.x = snap.bodies[0].position.x;
        v.y = snap.bodies[0].position.y;
        v.yaw = yaw;
        v.speed = fb.speed_mps;
        v.wheel_speed = snap_wheel_speed(&snap, course.r_roll);
        v.engine_rpm = fb.engine_rpm;
        v.gear = fb.gear;
        v.clutch = d.clutch;
        ref_update(&ref, &v);       /* EVERY physics sample, not a sampled one */
    }

    Chs_GetSnapshot(w, &snap);
    printf("  finished at x=%.2f y=%.3f, %.2f m/s\n\n",
           snap.bodies[0].position.x, snap.bodies[0].position.y, fb.speed_mps);

    RefEvent ev[REF_MAX_EVENTS];
    const int32_t n = ref_get_events(&ref, ev, REF_MAX_EVENTS);
    printf("  %-8s %-14s %-8s %s\n", "t (s)", "event", "object", "detail");
    for (int32_t i = 0; i < n; ++i)
        printf("  %-8.3f %-14s %-8d %.4f\n",
               ev[i].t, ref_event_name(ev[i].type), ev[i].object_id, ev[i].detail);
    if (n == 0) printf("  (no events)\n");

    printf("\n  gates passed: %d of %d\n", ref.next_gate, course.n_gates);
    printf("  events: %d", n);
    if (ref.dropped_events) printf(" (+%d dropped)", ref.dropped_events);
    printf("\n");
    print_verdict(&course, &ref);

    Chs_DestroyWorld(w);
    drv_script_free(&sc);
    return 0;
}

/* ===========================================================================
 *  HILL — the measurement that decides how to fix the rollback rule.
 *
 *  Every creep measurement so far was on FLAT ground, where the only forces in
 *  play are the small spurious ones. A hill start is not that regime: at 12%
 *  gravity supplies m*g*sin(theta) = 1800 * 9.81 * 0.1191 = 2101 N of real
 *  backward force, which may simply swamp whatever drives a 0.17 m/s creep.
 *  Whether it does is the whole question, and it cannot be answered from the
 *  flat-ground data.
 *
 *  Three phases, and each isolates one claim:
 *
 *    HOLD    brake on, in neutral. A real car does not move. Any motion here
 *            is the tyre model failing to hold a static load, which is the
 *            defect -- and it is now competing against real gravity rather
 *            than against nothing.
 *
 *    ROLL    brake released, still neutral. The car MUST roll back, and it
 *            must do so at g*sin(theta). This is the quantity the 0.30 m rule
 *            measures, and comparing it against the closed-form free-roll
 *            distance says whether the tyre model is adding or removing force
 *            it should not.
 *
 *    LAUNCH  1st gear, clutch engaged, throttle. Does the car arrest the
 *            rollback and pull away? CLAUDE.md's analysis says this is
 *            grip-limited, not torque-limited.
 * ======================================================================== */
/* One hill measurement, returned rather than printed, so a sweep can use the
 * same code path the single-shot report uses. */
typedef struct {
    double hold_drift;   /* m moved with brake 1.0 over 4 s                  */
    double roll_err;     /* % error of free roll against g*sin(theta)        */
    double launch_net;   /* m gained under power                             */
    double applied_sigma0, applied_sigma1;   /* read back from the tyre       */
    int    ok;           /* world built                                      */
} HillResult;

/* In neutral from the first step, as do_hill is. (A `hold_gear` argument
 * chose which gear the pre-task-10 neutral workaround held; since task 10
 * neutral holds none, and the argument and its `driveline` sweep are gone.) */
static HillResult hill_measure(const char* path, double grade,
                               double sigma0, double sigma1,
                               double blend_begin, double blend_end) {
    HillResult R; memset(&R, 0, sizeof R);

    ChsTerrainPatch ramp;
    memset(&ramp, 0, sizeof ramp);
    const double run = 240.0;
    ramp.length = run * sqrt(1.0 + grade * grade);
    ramp.width = 20.0;
    ramp.pitch_rad = -atan2(run * grade, run);
    ramp.friction = 0.9;

    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, &ramp, 1);
    if (!w) return R;
    R.ok = 1;

    if (sigma0 > 0.0 || sigma1 > 0.0 || blend_begin > 0.0) {
        Chs_SetTireStandstill(w, sigma0, sigma1, blend_begin, blend_end);
        /* READ IT BACK. Chs_SetTireStandstill skips any tyre that is not a
         * ChTMeasyTire, so a failed dynamic_pointer_cast would make the whole
         * sweep silently measure nothing -- every row identical, which is
         * exactly what a real null result looks like. */
        double g0 = -1, g1 = -1, gb = -1, ge = -1;
        Chs_GetTireStandstill(w, &g0, &g1, &gb, &ge);
        R.applied_sigma0 = g0;
        R.applied_sigma1 = g1;
    }

    const double theta = atan(grade);
    const double a_free = 9.81 * sin(theta);

    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
    ChsVehicleFeedback fb;
    ChsSnapshot snap;

    in.braking = 1.0;
    in.gear = 0; in.clutch = 1.0;
    run_(w, in, 0.5, &fb);
    run_(w, in, 1.5, &fb);
    Chs_GetSnapshot(w, &snap);
    const double x0 = snap.bodies[0].position.x;

    in.gear = 0; in.clutch = 1.0; in.throttle = 0.0;
    run_(w, in, 4.0, &fb);
    Chs_GetSnapshot(w, &snap);
    R.hold_drift = snap.bodies[0].position.x - x0;

    const double xr = snap.bodies[0].position.x;
    in.braking = 0.0;
    run_(w, in, 2.0, &fb);
    Chs_GetSnapshot(w, &snap);
    const double rolled = snap.bodies[0].position.x - xr;
    const double expect = -0.5 * a_free * 4.0;
    R.roll_err = 100.0 * (rolled - expect) / expect;

    const double xl = snap.bodies[0].position.x;
    in.gear = 1; in.clutch = 0.0; in.throttle = 0.6;
    run_(w, in, 4.0, &fb);
    Chs_GetSnapshot(w, &snap);
    R.launch_net = snap.bodies[0].position.x - xl;

    Chs_DestroyWorld(w);
    return R;
}

/* Sweep the Dahl damping. sigma0 holds the car; sigma1 is what taxes free
 * roll. The two are swept separately because they do different jobs, and a
 * single knob would trade one failure for the other. */
static int do_standstill(const char* path, double grade) {
    printf("\n=== TMeasy STANDSTILL TUNING, %.0f%% GRADE ===\n", grade * 100.0);
    printf("  Wanted: HOLD near zero AND roll error near zero. Chrono's stock\n");
    printf("  values are sigma0=100000, sigma1=5000.\n\n");
    printf("  %-10s %-10s %-12s %-12s %-11s %s\n",
           "sigma0", "sigma1", "hold (m)", "roll err %", "launch (m)", "verdict");

    /* sigma0 is the BRISTLE STIFFNESS and is what provides the static hold;
     * sigma1 is damping and was already shown not to move either number.
     * Sweeping sigma0 is what the Chrono patch exists for. */
    const double s0[] = { 1e5, 5e5, 1e6, 5e6, 1e7, 1e8 };
    const double s1[] = { 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0 };
    for (unsigned i = 0; i < sizeof s0 / sizeof s0[0]; ++i) {
        const HillResult R = hill_measure(path, grade, s0[i], s1[i], 0.0, 0.0);
        if (!R.ok) { printf("  build failed\n"); return 1; }
        const int held = fabs(R.hold_drift) < 0.05;
        const int rolls = fabs(R.roll_err) < 15.0;
        printf("  %-10.0f %-10.1f %-12.4f %-12.1f %-11.3f %s%s  [applied s0=%.0f]\n",
               s0[i], s1[i], R.hold_drift, R.roll_err, R.launch_net,
               held ? "holds " : "SLIDES ", rolls ? "+ rolls" : "+ roll wrong",
               R.applied_sigma0);
    }
    printf("\n  A row that both holds and rolls is the one to adopt.\n");
    return 0;
}

/* ===========================================================================
 *  ROLLBACK (task 7) -- the referee's detection floor, measured.
 *
 *  The regulation allows no rollback at all, so the floor must sit above
 *  whatever the MODEL moves a held car by. A car braked to a stop on the
 *  slope settles back a little with its wheels locked: the tyres' bristles
 *  (Dahl sigma0 = 1e5 N/m) are deflected FORWARD by the braking force and
 *  then BACK by gravity along the slope. A real tyre does the same. Measured
 *  here along the road -- the signed speed integrated, as the referee does --
 *  after full-brake stops from three speeds, on the course's own 15 degrees.
 * ======================================================================== */
static double settle_after_stop(const char* path, double grade, double v0,
                                double r_roll, double* wheel_worst)
{
    ChsTerrainPatch p[2];
    memset(p, 0, sizeof p);
    const double run = 60.0, th = atan(grade), toe = 5.0;
    p[0].center_x = toe - 20.0; p[0].length = 40.0; p[0].width = 20.0; p[0].friction = 0.9;
    p[1].center_x = toe + run / 2.0; p[1].center_z = grade * run / 2.0;
    p[1].length = run * sqrt(1.0 + grade * grade); p[1].width = 20.0;
    p[1].pitch_rad = -th; p[1].friction = 0.9;
    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, p, 2);
    const DrvLaneKeep lk = DRV_LANEKEEP_DEFAULT;
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 1, 0};
    ChsVehicleFeedback fb; ChsSnapshot s;
    double roll, pitch, yaw, t = 0.0;
    run_(w, in, 1.5, &fb);
    in.braking = 0.0; in.clutch = 0.0;
    for (;;) {                                    /* up the slope at v0 */
        Chs_GetSnapshot(w, &s); Chs_GetFeedback(w, &fb);
        if (s.bodies[0].position.x > toe + 2.75 + 6.0 || t > 60.0) break;
        euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
        in.steering = -drv_lane_keep(&lk, s.bodies[0].position.y, yaw);
        const double e = v0 - fb.speed_mps;
        in.throttle = e > 0.0 ? fmin(0.9, 0.3 + 1.0 * e) : 0.0;
        Chs_SetDriverInput(w, in); Chs_Step(w, DT); t += DT;
    }
    in.steering = 0.0; in.throttle = 0.0; in.clutch = 1.0; in.braking = 1.0;
    Chs_SetDriverInput(w, in);
    /* The referee's stop: the first sample under REF_PARK_STOP_SPEED. */
    for (long i = 0; i < 5000; ++i) {
        Chs_Step(w, DT); Chs_GetFeedback(w, &fb);
        if (fabs(fb.speed_mps) < REF_PARK_STOP_SPEED) break;
    }
    double d = 0.0, worst = 0.0, dw = 0.0;
    *wheel_worst = 0.0;
    for (long i = 0; i < (long)(3.0 / DT); ++i) {  /* then held 3 s */
        Chs_Step(w, DT); Chs_GetFeedback(w, &fb); Chs_GetSnapshot(w, &s);
        d  += fb.speed_mps * DT;                            /* the body   */
        dw += snap_wheel_speed(&s, r_roll) * DT;            /* the wheels */
        worst = fmin(worst, d);
        *wheel_worst = fmin(*wheel_worst, dw);
    }
    Chs_DestroyWorld(w);
    return worst;
}

static int do_rollback(const char* path)
{
    RefCourse course; char err[256] = {0};
    if (ref_load_course(&course, "/work/course/simA_referee.txt", err, sizeof err) != 0) {
        printf("course load failed: %s\n", err); return 1;
    }
    const double grade = tan(15.0 * M_PI / 180.0);   /* the course's hill, task 11 */
    printf("\n=== ROLLBACK: the referee's detection floor (task 7) ===\n");
    printf("  The regulation allows none; the floor, %.3f m, must clear the MODEL's\n"
           "  own motion of a held car. Full-brake stops on 15 degrees, then held 3 s.\n"
           "  The referee judges the WHEELS' travel; the body's is shown beside it.\n\n",
           REF_ROLLBACK_FLOOR);
    printf("  %-12s %-18s %s\n", "stop from", "body back", "wheels back (judged)");
    static const double v0s[] = { 0.5, 1.0, 2.0 };
    double worst = 0.0, body_worst = 0.0;
    for (int k = 0; k < 3; ++k) {
        double wheel = 0.0;
        const double body = settle_after_stop(path, grade, v0s[k], course.r_roll, &wheel);
        worst = fmin(worst, wheel); body_worst = fmin(body_worst, body);
        printf("  %4.1f m/s     %+.4f m          %+.4f m\n", v0s[k], body, wheel);
    }
    const int ok = -worst < 0.5 * REF_ROLLBACK_FLOOR;   /* noise under half the floor */
    printf("\n  wheels: worst %.4f m against the %.3f m floor -- %s\n", worst,
           REF_ROLLBACK_FLOOR, ok ? "a held car cannot be failed"
                                  : "THE FLOOR IS INSIDE THE MODEL'S NOISE");
    printf("  body: worst %.4f m -- the tyres' compliance, which a floor on body\n"
           "  travel would have had to clear\n", body_worst);
    return ok ? 0 : 1;
}


/* ===========================================================================
 *  ROLLACCT (task 6) -- where the free roll's deficit goes.
 *
 *  On 12% a car in neutral with the brake off accelerates at less than
 *  g*sin(theta): -15.4% in `hill` after task 10. Task 6 began as "the tyre
 *  sticks from rest"; after tasks 13 and 15 the deficit looked like plain
 *  physics -- rolling resistance, and the masses that spin with the wheels.
 *  This checks that account TERM BY TERM, against rows that move one term:
 *    - the car is stopped on the slope the way a driver stops there: driven
 *      UP it in 1st at idle, braked, put in neutral -- not a level car
 *      spawned onto the slope, which is a drop test;
 *    - the acceleration is FITTED where the model is in its steady regime,
 *      0.5 < |v| < 2.0 m/s: past the Dahl standstill core, and with TMeasy's
 *      rolling resistance (a tanh of wheel spin) saturated;
 *    - rows ADD a known inertia: the pedal down in gear g leaves the gearbox's
 *      0.3 kg m^2 input shaft geared to the wheels, 29 to 775 kg of car;
 *    - the other terms move by regenerating the car (ROLL_RES, TR_DS_INERTIA,
 *      DL_DS_INERTIA in gen_innova.py) and passing the values here, so the
 *      account printed is the account of the car that ran.
 * ======================================================================== */
typedef struct { double a_fit, d2, v2; int n_fit, ok; } RollRun;

static RollRun roll_on_slope(const char* path, double grade, int gear)
{
    RollRun R; memset(&R, 0, sizeof R);
    ChsTerrainPatch p[2];
    memset(p, 0, sizeof p);
    const double run = 60.0, th = atan(grade), toe = 5.0;
    p[0].center_x = toe - 20.0; p[0].length = 40.0;            /* flat: -35..5 */
    p[0].width = 20.0; p[0].friction = 0.9;
    p[1].center_x = toe + run / 2.0;                           /* ramp: 5..65  */
    p[1].center_z = grade * run / 2.0;
    p[1].length = run * sqrt(1.0 + grade * grade);
    p[1].width = 20.0; p[1].pitch_rad = -th;                   /* - climbs     */
    p[1].friction = 0.9;
    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, p, 2);
    if (!w) return R;

    const DrvLaneKeep lk = DRV_LANEKEEP_DEFAULT;
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 1, 0};
    ChsVehicleFeedback fb; ChsSnapshot s;
    double roll, pitch, yaw;
    run_(w, in, 1.5, &fb);                                      /* settle, braked */

    /* Up the slope in 1st at idle: the pedal comes up over 2 s and the
     * governor does the rest. Stop with the rear axle 6 m onto the ramp. */
    in.braking = 0.0;
    double t = 0.0;
    for (;;) {
        Chs_GetSnapshot(w, &s);
        if (s.bodies[0].position.x > toe + 2.75 + 6.0 || t > 40.0) break;
        euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
        in.steering = -drv_lane_keep(&lk, s.bodies[0].position.y, yaw);
        in.clutch = fmax(0.0, 1.0 - t / 2.0);
        Chs_SetDriverInput(w, in); Chs_Step(w, DT); t += DT;
    }
    if (t > 40.0) { Chs_DestroyWorld(w); return R; }
    in.steering = 0.0; in.clutch = 1.0; in.braking = 1.0;       /* stop     */
    run_(w, in, 2.0, &fb);
    in.gear = gear;                                             /* 0: neutral */
    run_(w, in, 2.0, &fb);

    /* Brake off. Fit v(t) over the steady window by least squares. */
    Chs_GetSnapshot(w, &s);
    const double x0 = s.bodies[0].position.x;
    in.braking = 0.0;
    Chs_SetDriverInput(w, in);
    double st = 0.0, sv = 0.0, stt = 0.0, stv = 0.0;
    int n = 0;
    const long n2 = (long)(2.0 / DT + 0.5), n5 = (long)(5.0 / DT + 0.5);  /* SHIM_DT-proof */
    for (long i = 1; i <= n5; ++i) {
        Chs_Step(w, DT);
        Chs_GetFeedback(w, &fb);
        const double tt = i * DT, v = fb.speed_mps;           /* < 0: rolling back */
        if (i == n2) {
            Chs_GetSnapshot(w, &s);
            R.d2 = (s.bodies[0].position.x - x0) / cos(th);
            R.v2 = v;
        }
        if (v < -0.5 && v > -2.0) { st += tt; sv += v; stt += tt * tt; stv += tt * v; ++n; }
        if (v < -3.0 && i > n2) break;
    }
    if (n > (long)(0.1 / DT)) {
        R.a_fit = -(n * stv - st * sv) / (n * stt - st * st);  /* magnitude */
        R.n_fit = n;
        R.ok = 1;
    }
    Chs_DestroyWorld(w);
    return R;
}

/* The same stop on FLAT ground, then the brake off in neutral: does the car
 * stand still after DRIVING, where a freshly spawned one rolls at ~5.5 mm/s
 * (task 14's `creep`)? The hill start comes after a drive, not a spawn. */
static void creep_after_drive(const char* path, int weave, double* d20, double* v20)
{
    ChsWorld w = Chs_CreateVehicle(path, 0.45);
    const DrvLaneKeep lk = DRV_LANEKEEP_DEFAULT;
    ChsDriverInput in = {0.0, 0.0, 1.0, 1.0, 1, 0};
    ChsVehicleFeedback fb; ChsSnapshot s;
    double roll, pitch, yaw, t = 0.0;
    run_(w, in, 1.5, &fb);
    in.braking = 0.0;
    for (;;) {
        Chs_GetSnapshot(w, &s);
        if (s.bodies[0].position.x > 15.0 || t > 40.0) break;
        euler_zyx(&s.bodies[0].rotation, &roll, &pitch, &yaw);
        in.steering = -drv_lane_keep(&lk, s.bodies[0].position.y, yaw);
        /* A weave slips the tyres sideways both ways before the straight
         * run-in: whatever lateral state the spawn left is overwritten. */
        if (weave && s.bodies[0].position.x < 9.0)
            in.steering = 0.25 * sin(2.0 * M_PI * t / 2.0);
        in.clutch = fmax(0.0, 1.0 - t / 2.0);
        Chs_SetDriverInput(w, in); Chs_Step(w, DT); t += DT;
    }
    in.steering = 0.0; in.clutch = 1.0; in.braking = 1.0;
    run_(w, in, 2.0, &fb);
    in.gear = 0;
    run_(w, in, 2.0, &fb);
    Chs_GetSnapshot(w, &s);
    const double x0 = s.bodies[0].position.x;
    in.braking = 0.0;
    run_(w, in, 20.0, &fb);
    Chs_GetSnapshot(w, &s);
    *d20 = s.bodies[0].position.x - x0;
    *v20 = fb.speed_mps;
    Chs_DestroyWorld(w);
}

static int do_rollacct(const char* path, double grade, double rr, double tr_ds)
{
    const double g = 9.81, M = 1800.0, th = atan(grade), a_free = g * sin(th);
    /* Chrono's ChTMeasyTire relates spin to speed through R_eff =
     * (2 R0 + r_stat)/3, r_stat the loaded radius: 0.3318 m at this car's
     * static deflection (`attitude`: wheel centres 0.3217 / 0.3234 m). */
    const double R0 = 0.33645, Reff = 0.3318, FD = 3.909;
    const double gears[] = { 0.0, 4.313, 2.330, 1.436, 1.000, 0.838 };
    /* Everything that spins with the wheels, as gen_innova.py builds it,
     * referred to the wheel as I (n / R_eff)^2. Printed, so a stale copy of a
     * generator value is visible here rather than silently wrong. */
    const double I_corners = 4.0 * (0.62 + 0.62 + 0.35) + 2.0 * 0.07 + 2.0 * 0.06;
    const double m_corners = I_corners / (Reff * Reff);
    const double m_carrier = 0.6 / (Reff * Reff);
    const double m_dl_ds   = 0.5 * FD * FD / (Reff * Reff);
    const double m_tr_ds   = tr_ds * FD * FD / (Reff * Reff);
    const double F_grav = M * g * sin(th);
    const double F_rr   = rr * M * g * cos(th) * R0 / Reff;

    printf("\n=== THE FREE-ROLL ACCOUNT, %.0f%% GRADE (task 6) ===\n", grade * 100.0);
    printf("  Stopped on the slope as a driver stops there, brake off, fitted where\n");
    printf("  0.5 < |v| < 2.0 m/s. g sin(theta) = %.4f m/s^2.\n\n", a_free);
    printf("  gravity along the slope, M g sin(theta)         %8.1f N\n", F_grav);
    printf("  rolling resistance, rr Fz R0/R_eff (rr %.3f)   %8.1f N   TMeasy: My = rr Fz R0 tanh(omega)\n",
           rr, F_rr);
    printf("  spinning with the wheels, I (n/R_eff)^2:\n");
    printf("    4 wheels + tyres + axle shafts + spindles   %5.2f kg m^2 at 1:1      %6.1f kg\n",
           I_corners, m_corners);
    printf("    differential carrier                         0.60 kg m^2 at 1:1      %6.1f kg\n",
           m_carrier);
    printf("    driveline driveshaft                         0.50 kg m^2 at %.3f:1  %6.1f kg\n",
           FD, m_dl_ds);
    printf("    transmission output shaft (a proxy?)         %.2f kg m^2 at %.3f:1  %6.1f kg\n",
           tr_ds, FD, m_tr_ds);
    printf("    gearbox input shaft, pedal down in gear g    0.30 kg m^2 at n_g %.3f:1\n\n", FD);

    const double base_a = m_corners + m_carrier + m_dl_ds;     /* hypothesis A */
    const double base_b = base_a + m_tr_ds;                    /* hypothesis B */
    printf("  A: the transmission's output shaft does NOT count (its speed is\n");
    printf("     reset from the driveline every step; only the gearbox reaction\n");
    printf("     is passed on). B: it counts like any other shaft.\n\n");
    printf("  %-24s %9s %9s %9s %9s %9s %s\n", "case", "a meas", "a pred A", "a pred B",
           "err A %", "err B %", "  2 s from rest: d, v");
    double ss_a = 0.0, ss_b = 0.0, worst_a = 0.0, neutral_a = 0.0;
    int bad = 0;
    for (int k = 0; k <= 5; ++k) {
        const int gear = k;
        const double n_in = gears[k] * FD;
        const double m_in = k ? 0.3 * n_in * n_in / (Reff * Reff) : 0.0;
        const double pa = (F_grav - F_rr) / (M + base_a + m_in);
        const double pb = (F_grav - F_rr) / (M + base_b + m_in);
        const RollRun r = roll_on_slope(path, grade, gear);
        char label[40];
        if (k) snprintf(label, sizeof label, "pedal down in %d (+%.0f kg)", k, m_in);
        else   snprintf(label, sizeof label, "neutral");
        if (!r.ok) { printf("  %-24s  RUN FAILED\n", label); ++bad; continue; }
        const double ea = 100.0 * (r.a_fit - pa) / pa, eb = 100.0 * (r.a_fit - pb) / pb;
        ss_a += ea * ea; ss_b += eb * eb;
        worst_a = fmax(worst_a, fabs(ea));
        if (k == 0) neutral_a = fabs(ea);
        printf("  %-24s %9.4f %9.4f %9.4f %+9.2f %+9.2f   %+.3f m, %+.3f m/s", label,
               r.a_fit, pa, pb, ea, eb, r.d2, r.v2);
        if (k == 0)
            printf("  (%.1f%% of free roll)", 100.0 * (r.d2 / (-0.5 * a_free * 4.0) - 1.0));
        printf("\n");
    }
    const double rms_a = sqrt(ss_a / 6.0), rms_b = sqrt(ss_b / 6.0);
    printf("\n  rms error: A %.2f%%, B %.2f%%  ->  %s\n", rms_a, rms_b,
           rms_a < rms_b ? "A: the output-shaft proxy is not inertia"
                         : "B: the output-shaft proxy is inertia");
    /* ASSERTED: the account must close. Neutral within 1%, every known-
     * inertia row within 2%, under A. A drift here means a term changed --
     * a mass, an inertia, the rolling resistance, or how Chrono couples the
     * powertrain -- and the hill start's physics with it. */
    const int closes = !bad && neutral_a < 1.0 && worst_a < 2.0 && rms_a < rms_b;
    printf("  %s\n", closes
           ? "ACCOUNT CLOSES: the deficit is rolling resistance and spinning mass."
           : "ACCOUNT DOES NOT CLOSE -- a term has changed; find it before trusting the hill.");
    if (!closes) bad = 1;

    /* The flat-ground creep (task 14): a steady ~4 N push, 0.02% of the
     * car's weight, that the near-zero-speed rolling resistance (tanh of
     * wheel spin, effectively viscous there) balances at ~6 mm/s. Not the
     * solver: identical at PSOR 1000. */
    double d20 = 0.0, v20 = 0.0;
    creep_after_drive(path, 0, &d20, &v20);
    printf("\n  flat, stopped after a 15 m straight drive, neutral, brake off: "
           "%+.4f m in 20 s, %+.4f m/s\n", d20, v20);
    creep_after_drive(path, 1, &d20, &v20);
    printf("  flat, the same after a weave (lateral slip both ways):         "
           "%+.4f m in 20 s, %+.4f m/s\n", d20, v20);
    printf("  (task 14's `creep`: a freshly SPAWNED car rolls ~0.10 m in 20 s)\n");
    return bad ? 1 : 0;
}


/* The other half of task 1 (mz-below-1ms). m_frblend_begin is ONE constant
 * doing TWO jobs: it gates the aligning moment (mzlow measures that) and it
 * opens the Dahl standstill blend (this measures that). Lowering it to buy
 * steering feel narrows the band in which the bristle model runs, which is the
 * band the hill start lives in -- so the steering-feel choice is not free and
 * has to be paid for here or rejected here.
 *
 * Reuses hill_measure, the same code path the single-shot `hill` report uses,
 * so a row here and a `hill` run are comparable rather than merely similar. */
static int do_blendhill(const char* path, double grade, double blend_end) {
    printf("\n=== HILL vs TMeasy BLEND BEGIN, %.0f%% GRADE ===\n", grade * 100.0);
    printf("  blend_begin gates Mz (steering feel below it is ZERO) and also\n");
    printf("  sets where the Dahl bristle model stops being 100%% of the tyre\n");
    printf("  force. Chrono's stock value is 1.0 m/s. sigma0/sigma1 are left\n");
    printf("  stock throughout. blend_end = %.2f m/s (Chrono stock: 3.0).\n\n",
           blend_end);
    printf("  %-11s %-12s %-12s %-11s %s\n",
           "blend_begin", "hold (m)", "roll err %", "launch (m)", "verdict");

    const double b[] = { 1.0, 0.70, 0.50, 0.30, 0.20, 0.10 };
    for (unsigned i = 0; i < sizeof b / sizeof b[0]; ++i) {
        const HillResult R = hill_measure(path, grade, 0.0, 0.0, b[i], blend_end);
        if (!R.ok) { printf("  build failed\n"); return 1; }
        const int held  = fabs(R.hold_drift) < 0.05;
        const int rolls = fabs(R.roll_err) < 25.0;
        const int pulls = R.launch_net > 0.1;
        printf("  %-11.2f %-12.4f %-12.1f %-11.3f %s%s%s\n",
               b[i], R.hold_drift, R.roll_err, R.launch_net,
               held ? "holds " : "SLIDES ",
               rolls ? "+ rolls " : "+ roll wrong ",
               pulls ? "+ launches" : "+ NO LAUNCH");
    }
    printf("\n  A blend_begin is only adoptable if its row here is no worse\n");
    printf("  than the 1.00 row. Steering feel does not outrank the hill start.\n");
    return 0;
}

static int do_hill(const char* path, double grade,
                   double blend_begin, double blend_end) {
    /* A single ramp through the ORIGIN, so the car spawns on the slope rather
     * than having to drive to it. center_z = 0 at x = 0. */
    ChsTerrainPatch ramp;
    memset(&ramp, 0, sizeof ramp);
    const double run = 240.0;
    ramp.center_x = 0.0;
    ramp.center_y = 0.0;
    ramp.center_z = 0.0;
    ramp.length   = run * sqrt(1.0 + grade * grade);
    ramp.width    = 20.0;
    ramp.pitch_rad = -atan2(run * grade, run);   /* negative climbs +X */
    ramp.friction = 0.9;

    ChsWorld w = Chs_CreateVehicleEx(path, 0.45, &ramp, 1);
    if (!w) { printf("Chs_CreateVehicleEx failed\n"); return 1; }

    /* mz-below-1ms regression hook: m_frblend_begin is shared between the Mz
     * gate and the Dahl standstill blend, so any value adopted for steering
     * feel has to be re-run here. Read back, because a skipped cast would
     * make a changed run look identical to an unchanged one. 0 leaves the
     * shim default; pass 1.0 / 3.0 to measure stock Chrono. */
    if (blend_begin > 0.0) {
        Chs_SetTireStandstill(w, 0.0, 0.0, blend_begin, blend_end);
        double gb = -1, ge = -1;
        Chs_GetTireStandstill(w, NULL, NULL, &gb, &ge);
        printf("\n  [blend range applied: %.2f .. %.2f m/s]\n", gb, ge);
    }

    const double theta = atan(grade);
    const double a_free = 9.81 * sin(theta);

    printf("\n=== HILL START, %.0f%% GRADE ===\n", grade * 100.0);
    printf("  slope angle %.2f deg, free-roll acceleration g*sin(t) = %.3f m/s^2\n",
           theta * 180.0 / M_PI, a_free);
    printf("  (all creep measurements before this were on FLAT ground, where\n");
    printf("   there is no gravity component to compete with.)\n\n");

    ChsDriverInput in = {0, 0, 0, 1.0, 0, 0};
    ChsVehicleFeedback fb;
    ChsSnapshot snap;

    /* Settle on the slope. */
    in.braking = 1.0;
    run_(w, in, 2.0, &fb);
    Chs_GetSnapshot(w, &snap);
    const double x_settled = snap.bodies[0].position.x;

    /* ---- HOLD: brake on, 4 s -------------------------------------------- */
    in.braking = 1.0; in.gear = 0; in.clutch = 1.0; in.throttle = 0.0;
    run_(w, in, 4.0, &fb);
    Chs_GetSnapshot(w, &snap);
    const double hold_drift = snap.bodies[0].position.x - x_settled;
    const double hold_v = snap.bodies[0].linear_velocity.x;

    printf("  HOLD   (brake 1.0, neutral, 4 s)\n");
    printf("    moved %+.4f m, speed %+.4f m/s\n", hold_drift, hold_v);
    printf("    %s\n\n", fabs(hold_drift) < 0.05
           ? "HELD: the brake keeps the car on the hill."
           : "DID NOT HOLD: the car slides with the brake fully applied.");

    /* ---- ROLL: brake off, 2 s ------------------------------------------- */
    Chs_GetSnapshot(w, &snap);
    const double x_roll0 = snap.bodies[0].position.x;
    in.braking = 0.0;
    run_(w, in, 2.0, &fb);
    Chs_GetSnapshot(w, &snap);
    const double rolled = snap.bodies[0].position.x - x_roll0;
    const double roll_v = snap.bodies[0].linear_velocity.x;
    const double expect = -0.5 * a_free * 2.0 * 2.0;

    printf("  ROLL   (brake released, neutral, 2 s)\n");
    printf("    rolled %+.4f m, expected %+.4f m from free roll at g*sin(t)\n",
           rolled, expect);
    printf("    speed  %+.4f m/s, expected %+.4f\n", roll_v, -a_free * 2.0);
    const double roll_err = expect != 0.0 ? 100.0 * (rolled - expect) / expect : 0.0;
    printf("    error  %+.1f%%  %s\n\n", roll_err,
           fabs(roll_err) < 25.0 ? "-- gravity dominates; the tyre is not "
                                   "materially adding or removing force"
                                 : "-- the tyre model is distorting free roll");

    /* ---- LAUNCH --------------------------------------------------------- */
    Chs_GetSnapshot(w, &snap);
    const double x_launch0 = snap.bodies[0].position.x;
    const double v_launch0 = snap.bodies[0].linear_velocity.x;
    in.gear = 1; in.clutch = 0.0; in.throttle = 0.6; in.braking = 0.0;

    /* Sampled over 8 s rather than reported at 4. The 4 s figure is kept
     * because it is the one every earlier run in CLAUDE.md quotes, but on its
     * own it cannot distinguish "the launch failed" from "the launch is still
     * arresting the rollback it started with" -- and once free roll works, it
     * STARTS with a rollback. A single endpoint conflates the two. */
    printf("  LAUNCH (1st, clutch engaged, 60%% throttle)\n");
    printf("    entering the launch at %+.3f m/s\n", v_launch0);
    printf("    %-6s %-11s %-11s %s\n", "t(s)", "net(m)", "vx(m/s)", "rpm");
    double after = 0.0;
    for (int k = 1; k <= 8; ++k) {
        run_(w, in, 1.0, &fb);
        Chs_GetSnapshot(w, &snap);
        const double net = snap.bodies[0].position.x - x_launch0;
        if (k == 4) after = net;
        printf("    %-6d %-11.4f %-11.3f %.0f\n",
               k, net, snap.bodies[0].linear_velocity.x, fb.engine_rpm);
    }
    printf("    net at 4 s %+.4f m; %s\n", after,
           snap.bodies[0].linear_velocity.x > 0.1
           ? "PULLED AWAY up the slope by 8 s."
           : "did NOT pull away even by 8 s.");

    Chs_DestroyWorld(w);
    return 0;
}

int main(int argc, char** argv) {
    if (getenv("SHIM_DT")) DT = atof(getenv("SHIM_DT"));
    const char* mode = argc > 1 ? argv[1] : "assemble";
    const char* path = argc > 2 ? argv[2] : "/work/vehicle";
    if (!strcmp(mode, "assemble")) return do_assemble(path);
    if (!strcmp(mode, "sweep"))
        return do_sweep(path, argc > 3 ? atof(argv[3]) : 0.0,
                              argc > 4 ? atof(argv[4]) : 3.0);
    if (!strcmp(mode, "clutch"))   return do_clutch(path);
    if (!strcmp(mode, "diff"))     return do_diff(path);
    if (!strcmp(mode, "solvers"))  return do_solvers(path);
    if (!strcmp(mode, "hold"))     return do_hold(path);
    if (!strcmp(mode, "attitude")) return do_attitude(path);
    if (!strcmp(mode, "holdtrace"))
        return do_holdtrace(path, argc > 3 ? atof(argv[3]) : 0.25,
                            argc > 4 ? atoi(argv[4]) : -1,
                            argc > 5 && !strcmp(argv[5], "land"));
    if (!strcmp(mode, "neutral"))  return do_neutral(path);
    if (!strcmp(mode, "perf"))     return do_perf(path, argc > 3 ? atoi(argv[3]) : 3000);
    if (!strcmp(mode, "creep"))
        return do_creep(path, argc > 3 ? atof(argv[3]) : 0.0,
                              argc > 4 ? atof(argv[4]) : 3.0);
    if (!strcmp(mode, "rollback")) return do_rollback(path);
    if (!strcmp(mode, "rollacct"))
        return do_rollacct(path, argc > 3 ? atof(argv[3]) : 0.12,
                           argc > 4 ? atof(argv[4]) : 0.012,
                           argc > 5 ? atof(argv[5]) : 0.5);
    if (!strcmp(mode, "hill"))
        return do_hill(path, argc > 3 ? atof(argv[3]) : 0.12,
                             argc > 4 ? atof(argv[4]) : 0.0,
                             argc > 5 ? atof(argv[5]) : 3.0);
    if (!strcmp(mode, "mzlow"))
        return do_mzlow(path, argc > 3 ? atof(argv[3]) : 0.0,
                              argc > 4 ? atof(argv[4]) : 3.0);
    if (!strcmp(mode, "blendhill"))
        return do_blendhill(path, argc > 3 ? atof(argv[3]) : 0.12,
                                  argc > 4 ? atof(argv[4]) : 3.0);
    if (!strcmp(mode, "standstill"))
        return do_standstill(path, argc > 3 ? atof(argv[3]) : 0.12);
    if (!strcmp(mode, "judge"))
        return do_judge(path, argc > 3 ? argv[3] : "slalom");
    if (!strcmp(mode, "terrain"))
        return do_terrain(path, argc > 3 ? argv[3] : "/work/course/simA_terrain.txt");
    if (!strcmp(mode, "drift"))
        return do_drift(path, argc > 3 ? argv[3] : "pullaway");
    if (!strcmp(mode, "determ"))
        return do_determinism(path, argc > 3 ? argv[3] : "pullaway",
                              argc > 4 ? atoi(argv[4]) : 2);
    fprintf(stderr,
            "usage: test_shim assemble|sweep|mzlow|clutch|diff|neutral|perf|determ|judge "
            "[datapath] [steps|script|grade|blend] [passes|blend]\n");
    return 64;
}
