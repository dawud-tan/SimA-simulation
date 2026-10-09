/* ===========================================================================
 *  chrono_shim.h — the ABI wall between Project Chrono and Unreal Engine.
 *
 *  THIS HEADER IS PURE C. It is the ONLY file the Unreal side may include.
 *
 *  Nothing here may ever become a C++ type. Not std::vector, not ChVector3d,
 *  not a class with a virtual function, not an Eigen matrix. Only:
 *      - fixed-width integers and doubles
 *      - opaque pointers (void* handles)
 *      - structs of the above, with explicit padding
 *
 *  WHY (measured on this machine, not assumed):
 *
 *  1. TWO C++ RUNTIMES IN ONE PROCESS.
 *     libChrono_core.so links libstdc++.so.6 dynamically and has 85 undefined
 *     references into it. UnrealEditor statically links libc++ (libc++.a from
 *     Epic's bundled clang 20.1.8). Both will be resident in the same address
 *     space. std::string, std::vector and exception objects have different
 *     layouts in each. 2873 of Chrono's 28574 exported symbols pass std::
 *     types; every one of them is a trap if called from UE code.
 *
 *  2. AVX2 CHANGES STRUCT LAYOUT.
 *     Chrono is built -march=skylake, so EIGEN_MAX_ALIGN_BYTES=32. A default
 *     UE translation unit gets 16. Measured consequence:
 *
 *         struct Holder { char tag; Eigen::Quaterniond q; };
 *             AVX2 build : 64 bytes
 *             SSE2 build : 48 bytes
 *
 *     sizeof(Quaterniond) is 32 in BOTH builds -- so a spot check passes --
 *     but anything embedding it disagrees by 16 bytes. That is silent heap
 *     corruption with no link error and no crash at the call site.
 *
 *  Passing only PODs by value sidesteps both problems completely: a double is
 *  a double under every compiler, runtime and -march on this ABI.
 * ======================================================================== */

#ifndef CHRONO_SHIM_H
#define CHRONO_SHIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Bump when the layout of any struct below changes. The UE module checks this
 * at load time and refuses to run against a mismatched .so, which turns a
 * silent memory bug into a clear startup error. */
#define CHRONO_SHIM_ABI_VERSION 5   /* 5: + parking hold */

/* --- Plain data ------------------------------------------------------- */

typedef struct { double x, y, z; }    ChsVec3;   /* 24 bytes, align 8 */
typedef struct { double w, x, y, z; } ChsQuat;   /* 32 bytes, align 8 */

/* One rigid body's state, as the render thread needs it. */
typedef struct {
    ChsVec3 position;
    ChsQuat rotation;
    ChsVec3 linear_velocity;
    ChsVec3 angular_velocity;
} ChsBodyState;

/* A complete snapshot handed from the 1 kHz physics thread to the game thread.
 * `sim_time` and `step_index` let the consumer interpolate and detect drops. */
typedef struct {
    double       sim_time;      /* seconds since sim start */
    uint64_t     step_index;    /* monotonic; gaps mean dropped frames */
    double       last_step_us;  /* cost of the step that produced this */
    int32_t      body_count;
    int32_t      _pad;          /* explicit: never let the compiler choose */
    ChsBodyState bodies[16];    /* fixed array: no allocation across the wall */
} ChsSnapshot;

/* --- Lifetime --------------------------------------------------------- */

typedef void* ChsWorld;

/* Returns CHRONO_SHIM_ABI_VERSION as compiled into the .so. */
int32_t  Chs_GetAbiVersion(void);
/* Chrono's version string, copied into caller-owned storage. Never returns a
 * pointer into the library -- that would hand UE a std::string's buffer. */
void     Chs_GetChronoVersion(char* out_buf, int32_t buf_size);

ChsWorld Chs_CreateWorld(double gravity_z);
void     Chs_DestroyWorld(ChsWorld world);

/* Adds a box. Returns its body index, or -1 if the world is full. */
int32_t  Chs_AddBox(ChsWorld world, double sx, double sy, double sz,
                    double density, ChsVec3 pos, int32_t is_fixed);

/* Advances by exactly `dt`. Returns microseconds spent, for the jitter
 * histogram in Milestone 3. */
double   Chs_Step(ChsWorld world, double dt);

/* Fills `out`. Returns bodies written, or -1 on bad input. */
int32_t  Chs_GetSnapshot(ChsWorld world, ChsSnapshot* out);


/* ======================================================================== *
 *  MILESTONE 5 -- the vehicle, driver inputs, and force-feedback source.
 *
 *  Up to ABI 1 the shim carried a box-stacking test scene. These entry points
 *  put the Milestone 4 Innova behind the same wall, because force feedback has
 *  to come from the tyres: the whole point of the exercise is that the wheel
 *  goes light when the front tyres saturate, and nothing but the tyre model
 *  knows when that happens.
 * ======================================================================== */

/* Driver inputs. Passed BY VALUE: 40 bytes is nothing next to a 185 us step,
 * and it removes every question about who owns the pointer and how long it
 * stays valid.
 *
 * SIGN: `steering` is POSITIVE-LEFT, matching Chrono's DriverInputs and
 * ISO 8855 (+Y left). The wheel's own convention is positive-CLOCKWISE. The
 * negation happens on the Unreal side, once, at the call to this function.
 *
 * CLUTCH POLARITY (verified against Chrono's source, not assumed):
 * ChManualTransmissionShafts::Synchronize does
 *     m_clutch->SetModulation(1.0 - driver_inputs.m_clutch)
 * so 1.0 = pedal fully DOWN = fully disengaged, 0.0 = fully engaged. That is
 * the same sense as the pedal axis, so it passes straight through. Getting it
 * backwards would make the hill start trivially easy instead of the
 * clutch-control exercise it is meant to be. */
typedef struct {
    double  steering;   /* -1 .. +1, positive steers LEFT                    */
    double  throttle;   /*  0 .. 1                                           */
    double  braking;    /*  0 .. 1                                           */
    double  clutch;     /*  0 .. 1, 1 = disengaged                           */
    int32_t gear;       /* -1 reverse, 0 neutral, 1..5 forward               */
    int32_t _pad;
} ChsDriverInput;       /* 40 bytes */

/* Everything the force-feedback controller needs, sampled at the step rate.
 * The raw per-tyre terms are exposed alongside the cooked torque so the feel
 * can be re-tuned on the Unreal side without another shim rebuild -- and so a
 * bad FFB signal can be traced to the tyre it came from.
 *
 * PARKING TORQUE (task 16). column_torque_nm also carries the front tyres'
 * parking (bore) torque, which TMeasy lacks and the shim models for the
 * force feedback ONLY: the car never feels it, its steering being position-
 * commanded. tire_mz_nm stays TMeasy's own, so the parking share is
 *     column_torque_nm * 17.5 - (tire_mz_nm[0] + tire_mz_nm[1]).
 * It is zero above TMeasy's blend_end (3.0 m/s). CHS_PARKING_TORQUE=0
 * (environment, read at vehicle creation; diagnostic) turns it off. */
typedef struct {
    double  steer_torque_nm;    /* AT THE RIM: after ratio, assist, damping   */
    double  column_torque_nm;   /* before assist -- what the rack really feels */
    double  tire_mz_nm[2];      /* FL, FR raw self-aligning moment (TMeasy)   */
    double  tire_fz_n[2];       /* FL, FR vertical load; 0 = wheel airborne   */
    double  road_steer_rad[2];  /* FL, FR actual road-wheel steer angle       */
    double  speed_mps;
    double  engine_rpm;
    int32_t gear;               /* gear the DRIVER selected (see the shim)    */
    int32_t front_grounded;     /* bit 0 = FL loaded, bit 1 = FR loaded       */
} ChsVehicleFeedback;   /* 88 bytes */

/* Builds the Innova on flat rigid terrain with the MINRES solver (measured at
 * 185.8 us/step against 382.5 us for the default -- see CLAUDE.md).
 * `data_path` is the directory containing the innova JSON set, e.g. ".../vehicle".
 * Returns NULL on failure. The returned handle is used with Chs_Step,
 * Chs_GetSnapshot and Chs_DestroyWorld exactly like a box world. */
ChsWorld Chs_CreateVehicle(const char* data_path, double init_z);

/* Latches the driver inputs used by every subsequent Chs_Step. Cheap: it
 * stores four doubles and, only when the gear actually changes, calls
 * SetGear. Safe to call at any rate from the thread that owns the world. */
void     Chs_SetDriverInput(ChsWorld world, ChsDriverInput in);

/* Fills `out` with the current feedback. Returns 0, or -1 if this world is a
 * box scene rather than a vehicle. */
int32_t  Chs_GetFeedback(ChsWorld world, ChsVehicleFeedback* out);

/* ======================================================================== *
 *  MILESTONE 6.2b -- TERRAIN.
 *
 *  Up to ABI 2 the vehicle world hardcoded one flat 300 x 300 m RigidTerrain
 *  patch. The SIM A hill start needs a slope, and a slope cannot be faked: the
 *  whole exercise is holding a car against gravity on a gradient with the
 *  clutch, and the traction analysis that says it is a grip-limited rather
 *  than a torque-limited problem only means anything if the gradient is real.
 *
 *  WHY PATCHES AND NOT A HEIGHTMAP OR A MESH:
 *  RigidTerrain::AddPatch accepts a box, an OBJ mesh, or a BMP heightmap. The
 *  mesh and heightmap forms both take a FILENAME, which would put file
 *  discovery, image decoding and mesh loading behind the ABI wall for a shape
 *  that is three flat rectangles. The box form takes numbers, and numbers are
 *  what a POD ABI is good at. A ramp is a box rotated about Y.
 * ======================================================================== */

/* One rigid terrain patch.
 *
 * GEOMETRY: `center` is the centre of the patch's DRIVING SURFACE -- its top
 * face -- not the centre of the box. That is Chrono's convention for the
 * ChCoordsys passed to AddPatch, and getting it wrong buries the road half a
 * thickness underground.
 *
 * `pitch_rad` rotates the patch about the Y axis. Chrono is right-handed with
 * +Y left, so a rotation about +Y takes +X toward -Z: a patch whose surface
 * RISES going forward needs a NEGATIVE pitch. `length` is measured along the
 * patch's own inclined X axis, so a ramp covering a horizontal run r at
 * gradient g is length = r * sqrt(1 + g*g), not r.
 *
 * These are the two mistakes that produce a hill the car falls through and a
 * hill that is subtly too short, respectively; both are stated here because
 * neither is visible in a JSON diff. */
typedef struct {
    double  center_x, center_y, center_z;
    double  length;      /* along the patch's own (possibly inclined) X       */
    double  width;       /* along Y                                          */
    double  pitch_rad;   /* about Y; NEGATIVE = surface rises with +X         */
    double  friction;    /* 0.9 matches the flat default                      */
    double  _pad;
} ChsTerrainPatch;      /* 64 bytes */

/* As Chs_CreateVehicle, but with an explicit terrain.
 *
 * `patches` is CALLER-OWNED and is copied during the call; the shim retains no
 * pointer into it (ABI rule 5). Passing NULL or n_patches <= 0 gives exactly
 * the ABI-2 behaviour -- one flat 300 x 300 m patch -- so the old entry point
 * is now a thin wrapper over this one and cannot drift from it. */
ChsWorld Chs_CreateVehicleEx(const char* data_path, double init_z,
                             const ChsTerrainPatch* patches, int32_t n_patches);

/* Terrain height under a point, in metres.
 *
 * The referee needs this and cannot get it any other way: "rolled back 0.3 m"
 * on a slope is a distance along the ROAD, and a rule that used world Z or
 * world X would be measuring the gradient instead of the driver. Returns 0.0
 * for a box world or a bad handle. */
double   Chs_GetTerrainHeight(ChsWorld world, double x, double y);

/* ======================================================================== *
 *  TMeasy STANDSTILL TUNING  (requires the local Chrono patch)
 *
 *  Below `blend_begin` m/s, ChTMeasyTire uses 100% a Dahl bristle model:
 *      F = sigma0*z + sigma1*dz/dt
 *  The damping term is viscous in the SLIP velocity. At Chrono's stock
 *  sigma1 = 5000 N.s/m that is about 500 N per tyre at 0.1 m/s of slip, so
 *  ~2000 N across four against the 2101 N gravity component of a 12% grade --
 *  a freely rolling car is taxed nearly the whole force that should be
 *  accelerating it.
 *
 *  MEASURED with the stock values (05-verify-vehicle-shim.sh hill):
 *      HOLD  brake 1.0 on a 12% slope: slides 0.246 m in 4 s -- does NOT hold
 *      ROLL  neutral, brake off:       49% short of g*sin(theta)
 *  Both make the hill-start rollback rule unmeasurable. Chrono's own comment
 *  calls the model "experimental and needs some testing".
 *
 *  sigma0 is what provides the standstill HOLD and is not the problem;
 *  sigma1 is. They are separated here so they can be tuned independently
 *  against that measurement rather than as one knob.
 *
 *  These entry points need docker/patches/0001-tmeasy-standstill-setters.patch,
 *  which is applied during the container build. Against an unpatched Chrono the
 *  shim will not compile -- deliberately, because silently ignoring the call
 *  would leave the hill start quietly broken.
 *
 *  Call AFTER Chs_CreateVehicleEx; applies to all four tyres. Passing a
 *  non-positive value for any argument leaves that parameter at its current
 *  value, so one term can be changed without restating the others.
 *
 *  --- THE BLEND RANGE IS NOT STOCK BY DEFAULT --------------------------
 *
 *  Chs_CreateVehicleEx applies CHS_TIRE_BLEND_BEGIN / _END below instead of
 *  Chrono's 1.0 / 3.0. To measure stock behaviour, pass 1.0 and 3.0
 *  explicitly -- a non-positive argument now means "leave the shim default",
 *  not "leave Chrono's".
 *
 *  Returns 0, or -1 if this world is not a vehicle.
 * ======================================================================== */
int32_t  Chs_SetTireStandstill(ChsWorld world, double sigma0, double sigma1,
                               double blend_begin, double blend_end);

/* Reads back what the tyres are actually using, so a tuning sweep can report
 * the values it ran with rather than the values it intended. */
int32_t  Chs_GetTireStandstill(ChsWorld world, double* sigma0, double* sigma1,
                               double* blend_begin, double* blend_end);

/* ---- The blend range every vehicle world starts with --------------------
 *
 * Chrono ships 1.0 / 3.0. That 1.0 is ONE constant doing TWO jobs:
 *
 *   1. ChTMeasyTire::Advance computes the aligning moment only when
 *      `m_data.vel.x() >= m_frblend_begin`. Below it Mz is *identically
 *      zero*, so at 1.0 m/s the steering goes completely dead -- and both
 *      SIM A parking exercises and the tight end of the slalom are performed
 *      entirely under that speed. `sweep` never saw it because it measures at
 *      8 m/s, eight times above the gate.
 *
 *   2. begin..end is the window over which the Dahl bristle force fades out.
 *
 * Only the first is changed here. blend_end stays at Chrono's 3.0; see the
 * rejected experiment at the bottom of this comment for why.
 *
 * MEASURED, 05-verify-vehicle-shim.sh mzlow -- column torque at a 0.60
 * parking lock, mean over 1 s at a held speed, with peak-to-peak beside it
 * (the pedal and the speed were steady to 0.012 and 0.010 in every row, so
 * the swing is the tyre's and not the harness's):
 *
 *     v m/s   stock 1.0/3.0        this default 0.20/3.0
 *     0.09    -0.028               -0.028            still gated, and rightly
 *     0.21    -0.079  p2p 0.019    -3.256  p2p 0.845     41x the torque
 *     0.39    -0.127  p2p 0.027    -3.525  p2p 0.256     28x
 *     0.65    -0.181  p2p 0.024    -3.622  p2p 0.128     20x
 *     0.99    -4.980               -3.762
 *
 * 0.20 rather than 0.10: at 0.10 the two front tyres straddle the gate at a
 * 0.09 m/s crawl, so one contributes Mz and the other does not, and the rack
 * gets half a signal with a 1.9 N.m p2p wobble on it. 0.20 keeps both tyres
 * on the same side of the gate through the whole manoeuvring band.
 *
 * NOT free, and checked rather than assumed:
 *   creep       unchanged to three decimals across begin 1.0 / 0.30 / 0.10
 *   sweep       peak column torque 7.044 -> 7.046 N.m at steer 0.45, trail
 *               collapse still present, friction-circle onset still steer
 *               0.70. Unaffected because it runs at 7-8 m/s.
 *   hill  0.12  HOLD -0.0043 m (stock -0.0048), still holds. ROLL -50.4%
 *               against stock -45.5%. LAUNCH still pulls away up the slope.
 *
 * The ROLL row deserves its 5 points of honesty: it is WORSE on the distance
 * integral and BETTER on the thing task `breakaway-from-rest` is about. Speed
 * after 2 s of free roll from rest goes -0.092 m/s (stock) to -0.867 m/s --
 * the stock tyre is stuck and this one is not. Both are far from the -2.338
 * that gravity alone gives, so neither supports a rollback rule yet.
 *
 * Treat the hill numbers as indicative, not tight: sweeping blend_begin from
 * 1.0 to 0.10 at blend_end 3.0 gives roll errors of -45.5 / -69.2 / -29.0 /
 * -97.4 / -50.4 / -46.4%. The sim is deterministic, so that spread is real
 * sensitivity rather than run-to-run noise -- but it means the hill start
 * cannot currently resolve a 5-point difference, and no tuning should be
 * justified by one.
 *
 * REJECTED: blend_end 8.0.
 * Raising it damps the mid-band chatter beautifully -- column-torque p2p at
 * 2.86 m/s falls from 8.44 N.m (stock, about a -4.13 mean, i.e. swinging
 * through zero) to 1.67, and at 1.82 m/s from 4.55 to 0.04 -- and it makes
 * the hill sweep repeatable (-33.0 to -36.1% across all six blend_begin
 * values instead of -29 to -97). It also taxes DRIVE force across the whole
 * launch range, for the same reason sigma1 taxes free roll: a driving wheel
 * needs slip, and the Dahl damping term is viscous in slip. Measured on a 12%
 * grade, 1st, 60% throttle: the car never pulls away, ending 11.2 m further
 * down the hill after 8 s. Not adoptable. The mid-band chatter it would have
 * fixed is a real pre-existing defect and is left open.
 *
 * Baked in here rather than left to the caller for the same reason the
 * parking hold is: the harnesses and the UE plugin must measure the same
 * vehicle, and a value that only production sets is a value no host test
 * covers. */
#define CHS_TIRE_BLEND_BEGIN 0.20
#define CHS_TIRE_BLEND_END   3.00

/* ======================================================================== *
 *  PARKING HOLD -- OFF BY DEFAULT since task 15; kept as an opt-in.
 *
 *  It was built because a braked car would not stay on a 12% slope: it slid
 *  at a steady 0.06 m/s. Task 15 found why: the vehicle used ChBrakeSimple,
 *  whose ChLinkLockBrake sticking logic is dead code in Chrono 10 (it sets
 *  `must_stick` and nothing reads it), so a braked wheel at rest was a bare
 *  Coulomb torque flipping sign every step -- it CHATTERED, measured at
 *  0.36 rad/s with a sign flip on 19 of 20 steps, and the tyre could never
 *  settle. The vehicle now uses ChBrakeShafts (gen_innova.py BRAKE_MODEL),
 *  a clutch that a VI solver holds rigidly below its torque limit: the wheel
 *  is stuck, and the brake ALONE holds a car stopped on 12%, 25% and 45%
 *  (1-4 mm of tyre pre-slide, then still) and lets it slide on 120%.
 *
 *  With the brake doing its job the hold only disturbs the car, so it is off.
 *  WHAT IT DOES when enabled: while the brake is past 0.8 and the car is
 *  nearly stopped, a force on the chassis opposes its velocity IN THE ROAD
 *  PLANE, sized to arrest it in one step and clamped to mu times the load the
 *  tyres actually carry. It first opposed the full 3-D velocity, which froze
 *  the suspension -- a braked spawn held the body at the bottom of its first
 *  compression, 0.113 m ride height and 74% front load against 0.2375 m and
 *  51.3% -- and it is applied at the centre of mass, so it still resists the
 *  small fore-aft motion of the body as it pitches. Enable it only for a
 *  vehicle built with BRAKE_MODEL=simple, whose brake cannot hold.
 *
 *  Never active once the brake is released, so it cannot touch a hill-start
 *  rollback measurement. Returns 0, or -1 if this world is not a vehicle. */
int32_t  Chs_SetParkingHold(ChsWorld world, int32_t enable);

/* Force-feedback shaping, adjustable at runtime.
 *   assist_frac  0..1, fraction of column torque the power steering removes.
 *                The Innova has engine-driven hydraulic assist; 0.7 is the
 *                starting estimate and is [EST], not measured.
 *   damping_nm_s viscous term opposing steering RATE. This is not cosmetic:
 *                the wheel's rotor inertia plus one frame of transport delay
 *                closes a positive-feedback loop, and an undamped constant-
 *                force channel will oscillate audibly around centre.
 *   gain         final scalar into the device, before its own soft clip. */
void     Chs_SetFfbTuning(ChsWorld world, double assist_frac,
                          double damping_nm_s, double gain);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* CHRONO_SHIM_H */
