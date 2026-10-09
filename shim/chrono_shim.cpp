// ============================================================================
//  chrono_shim.cpp — C++ implementation behind the pure-C wall.
//
//  This file is compiled INSIDE the Chrono container, with the same compiler,
//  the same -march=skylake, and the same libstdc++ as Chrono itself. It is the
//  only place where Chrono headers and C++ types are allowed to appear.
//
//  Unreal never sees this file, never includes a Chrono header, and never
//  links Chrono directly -- it links only libchrono_shim.so through
//  chrono_shim.h.
// ============================================================================

#include "chrono_shim.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "chrono/ChVersion.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/functions/ChFunctionSineStep.h"

// Milestone 5: the vehicle moves behind the wall. Chrono 10 renamed
// ChVehicleModelData.h -> ChVehicleDataPath.h, and WheeledVehicle.h lives
// under wheeled_vehicle/vehicle/ (see CLAUDE.md, Milestone 4 conventions).
#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h"

namespace stdc = std::chrono;
using namespace chrono;
using namespace chrono::vehicle;

namespace {

// Matches ChsSnapshot::bodies. Kept as a constant so the static_assert below
// fails loudly if the two ever drift apart.
constexpr int kMaxBodies = 16;

// Everything C++ lives inside this struct, which UE only ever holds as an
// opaque void*. Its layout is a private detail of this .so.
struct World {
    // --- box mode (Milestone 3 test scene) ---
    // Unused in vehicle mode: WheeledVehicle constructs and owns its own
    // ChSystem, so this member just sits there. Kept by value rather than
    // hoisted behind a pointer so the ABI-1 entry points stay untouched.
    ChSystemSMC                            sys;
    std::shared_ptr<ChContactMaterialSMC>  material;
    std::vector<std::shared_ptr<ChBody>>   bodies;

    double                                 sim_time    = 0.0;
    uint64_t                               step_index  = 0;
    double                                 last_step_us = 0.0;

    // --- vehicle mode (Milestone 5) ---
    bool                              is_vehicle = false;
    std::shared_ptr<WheeledVehicle>   veh;
    std::shared_ptr<RigidTerrain>     terrain;
    DriverInputs                      inputs{0.0, 0.0, 0.0, 0.0};
    int                               box_gear = 1;   // gear the gearbox holds
    int                               req_gear = 0;   // gear the driver selected
    double                            prev_steering = 0.0;
    ChsVehicleFeedback                fb{};

    // Parking hold, see Chs_SetParkingHold. OFF by default since task 15:
    // with BrakeShafts the brake itself holds, and the hold then only
    // distorts the settled attitude (measured -- see the header).
    bool         parking_hold = false;
    bool         has_hold_acc = false;
    unsigned int hold_acc = 0;

    // The driveline's two ChShaftsClutch elements (task 13): the pedal clutch
    // in the transmission, and the rear differential's lock.
    std::shared_ptr<ChShaftsClutch> pedal_clutch, diff_lock;

    // Solver convergence statistics -- diagnostic, CHS_SOLVER_STATS only.
    std::string solver_name = "psor";
    bool   solver_stats = false;
    // True neutral (task 10): pass gear 0 to the transmission. Only a VI
    // solver may: see Chs_SetDriverInput. Cleared for a linear one.
    bool   true_neutral = true;

    // Idle governor and stall (task 8), see Chs_Step. idle_rpm 0 = neither.
    // gov_throttle is the fuel the governor demanded on the last step.
    double idle_rpm = 0.0;
    double gov_throttle = 0.0;
    bool   engine_running = true;
    bool   stall_armed = false;     // set once the engine has reached idle
    double crank_s = 0.0;           // time the restart condition has held
    long   st_n = 0;
    int    st_it_max = 0;
    double st_it_sum = 0.0, st_err_sum = 0.0, st_err_max = 0.0;

    // Parking torque (task 16), see ParkingTorque. Force feedback only.
    bool   park_on = false;
    bool   park_primed = false;
    double park_k = 0.0, park_w = 0.0, park_mu = 0.0, park_r0 = 0.0;
    double park_psi[2] = {0.0, 0.0};          // twist of each front patch, rad
    double park_prev_delta[2] = {0.0, 0.0};
    std::shared_ptr<ChTMeasyTire> park_tire[2];

    // FFB shaping, see Chs_SetFfbTuning.
    double assist_frac  = 0.7;
    double damping_nm_s = 0.02;
    double ffb_gain     = 1.0;

    // Tyre operating envelope -- diagnostic, CHS_TIRE_ENVELOPE only (Q1):
    // what a tyre table would have to cover. Per step and tyre: speed along
    // the wheel heading, slip angle, slip ratio, camber, load; per front
    // wheel, the road-wheel steer rate. Percentiles at destroy.
    bool tire_env = false;
    std::vector<float> env[4][5];
    std::vector<float> env_steer_rate[2], env_steer_rate_slow[2];
    double env_prev_steer[2] = {0.0, 0.0};
};

// Record one step of the tyre envelope (CHS_TIRE_ENVELOPE). Reads the slip
// state TMeasy itself computed in Synchronize (ChTire::CalculateKinematics):
// slip angle atan(Vy/|Vx|), slip ratio -(Vx - omega R)/|Vx|, both set to 0
// below |Vx| = 1e-4 m/s.
void RecordTireEnvelope(World* w, double dt) {
    for (int a = 0; a < 2; ++a)
        for (int s = 0; s < 2; ++s) {
            const auto side = static_cast<VehicleSide>(s);
            auto tire  = w->veh->GetTire(a, side);
            auto wheel = w->veh->GetWheel(a, side);
            const WheelState ws = wheel->GetState();
            const ChVector3d axle = ws.rot.GetAxisY();
            ChVector3d fwd = Vcross(axle, ChVector3d(0, 0, 1));
            fwd.Normalize();
            const int i = 2 * a + s;
            w->env[i][0].push_back((float)ws.lin_vel.Dot(fwd));
            w->env[i][1].push_back((float)tire->GetSlipAngle());
            w->env[i][2].push_back((float)tire->GetLongitudinalSlip());
            w->env[i][3].push_back((float)tire->GetCamberAngle());
            w->env[i][4].push_back((float)tire->ReportTireForce(w->terrain.get()).force.z());
            if (a == 0) {
                const double st = w->veh->GetSteeringAngle(0, side);
                const double rate = (st - w->env_prev_steer[s]) / dt;
                w->env_prev_steer[s] = st;
                if (w->step_index > 0) {
                    w->env_steer_rate[s].push_back((float)rate);
                    if (std::abs(ws.lin_vel.Dot(fwd)) < 0.1)
                        w->env_steer_rate_slow[s].push_back((float)rate);
                }
            }
        }
}

void PrintTireEnvelope(World* w) {
    auto pct = [](std::vector<float> v, bool absval, const char* name, double scale,
                  const char* unit) {
        if (v.empty()) { std::fprintf(stderr, "    %-22s (no samples)\n", name); return; }
        if (absval) for (auto& x : v) x = std::abs(x);
        std::sort(v.begin(), v.end());
        auto q = [&](double p) { return scale * v[(size_t)(p * (v.size() - 1))]; };
        std::fprintf(stderr, "    %-22s min %9.3f  p1 %9.3f  p50 %9.3f  p99 %9.3f  p99.9 %9.3f  "
                     "max %9.3f %s\n", name, q(0), q(0.01), q(0.5), q(0.99), q(0.999), q(1.0), unit);
    };
    constexpr double kDeg = 57.29577951308232;
    const char* names[4] = {"FL", "FR", "RL", "RR"};
    std::fprintf(stderr, "CHS_TIRE_ENVELOPE %zu steps\n", w->env[0][0].size());
    for (int i = 0; i < 4; ++i) {
        std::fprintf(stderr, "  %s\n", names[i]);
        pct(w->env[i][0], false, "speed vx", 1.0, "m/s");
        pct(w->env[i][1], false, "slip angle", kDeg, "deg");
        // Slip only means something where the denominator does: |vx| > 0.5 m/s.
        std::vector<float> sa, kap;
        for (size_t k = 0; k < w->env[i][0].size(); ++k)
            if (std::abs(w->env[i][0][k]) > 0.5f) {
                sa.push_back(w->env[i][1][k]);
                kap.push_back(w->env[i][2][k]);
            }
        pct(sa, false, "slip angle |vx|>0.5", kDeg, "deg");
        pct(w->env[i][2], false, "slip ratio", 1.0, "");
        pct(kap, false, "slip ratio |vx|>0.5", 1.0, "");
        pct(w->env[i][3], false, "camber", kDeg, "deg");
        pct(w->env[i][4], false, "load Fz", 1.0, "N");
    }
    for (int s = 0; s < 2; ++s) {
        std::fprintf(stderr, "  %s road-wheel steer rate\n", names[s]);
        pct(w->env_steer_rate[s], true, "|rate|, all", kDeg, "deg/s");
        pct(w->env_steer_rate_slow[s], true, "|rate|, |vx| < 0.1", kDeg, "deg/s");
    }
    // Time spent in each speed band, front left: where the table must be good.
    const double edges[] = {0.0, 0.05, 0.2, 0.5, 1.0, 2.0, 4.0, 8.0, 1e9};
    std::fprintf(stderr, "  FL time by |vx| band:");
    for (int b = 0; b < 8; ++b) {
        size_t n = 0;
        for (float x : w->env[0][0]) if (std::abs(x) >= edges[b] && std::abs(x) < edges[b + 1]) ++n;
        std::fprintf(stderr, "  [%g,%g) %.1f%%", edges[b], edges[b + 1] > 1e8 ? 99.0 : edges[b + 1],
                     100.0 * n / std::max<size_t>(1, w->env[0][0].size()));
    }
    std::fprintf(stderr, "\n");
}

// Overall steering ratio: steering-wheel angle / road-wheel angle.
// 17.5 comes from gen_innova.py:153 and is tagged [EST] there. It is NOT
// derivable from the JSON -- the rack-and-pinion file stores a pinion radius,
// which encodes the ratio only together with the steering-arm length. If the
// generator's STEER_RATIO changes, change it here too.
//
// Consequence worth knowing: 34.6 deg road-wheel lock x 17.5 = 605 deg per
// side, i.e. 3.4 turns lock-to-lock. A G923 turns 2.5 (900 deg). The device
// range cannot cover the car, so the Unreal side scales; see ChronoWheelInput.
constexpr double kSteerRatio = 17.5;

constexpr double kRadSToRpm = 60.0 / (2.0 * 3.14159265358979323846);

// Vehicle JSON set, relative to the data path handed to Chs_CreateVehicle.
constexpr const char* kVehJson = "innova/innova_Vehicle.json";
constexpr const char* kEngJson = "innova/innova_Engine.json";
constexpr const char* kTrnJson = "innova/innova_Transmission.json";
constexpr const char* kTireF   = "innova/innova_TireF.json";
constexpr const char* kTireR   = "innova/innova_TireR.json";

inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
inline double clamp11(double v) { return v < -1.0 ? -1.0 : (v > 1.0 ? 1.0 : v); }

// std::exception must never cross the wall: its layout differs between
// libstdc++ and libc++, and unwinding across the boundary is undefined.
// Every entry point swallows exceptions and reports failure as a return code.
template <typename Fn>
auto guard(Fn&& fn, decltype(fn()) on_error) -> decltype(fn()) {
    try {
        return fn();
    } catch (...) {
        return on_error;
    }
}


// ---------------------------------------------------------------------------
// Where force feedback actually comes from.
//
// A tyre generating lateral force does not generate it at the centre of the
// contact patch. The patch deforms, the pressure distribution shifts rearward,
// and the resultant acts a short distance behind the wheel centre -- the
// PNEUMATIC TRAIL. Add the MECHANICAL TRAIL from caster angle and the lateral
// force acquires a moment arm about the steering axis. That moment is Mz, the
// self-aligning torque, and it is the entire physical basis of steering feel.
//
// The cue that matters for a driving test is what happens as the tyre
// saturates: pneumatic trail COLLAPSES TOWARDS ZERO before peak lateral force
// is reached, because the rear of the patch is already sliding and stops
// contributing. So Mz peaks and falls away while cornering force is still
// climbing. That is why a real wheel goes LIGHT just before the front end
// washes out, and it is the single most valuable thing force feedback teaches.
// TMeasy reproduces it; a torque synthesised from lateral acceleration or
// steering angle -- the usual shortcut -- does not, and teaches the wrong
// reflex.
//
// SIGN CONVENTION: every torque here is in Chrono's frame, positive-LEFT, the
// same convention as DriverInputs::m_steering. The single flip to the device's
// positive-clockwise convention happens on the Unreal side.
// Mean distance from the centre of a 2a x 2b rectangle: the lever arm of a
// uniformly loaded patch that slides as it turns. Closed form, d = hypot(a, b).
double MeanRadius(double a, double b) {
    const double d = std::hypot(a, b);
    return (d + a * a / (2.0 * b) * std::log((b + d) / a)
              + b * b / (2.0 * a) * std::log((a + d) / b)) / 3.0;
}

// PARKING TORQUE (task 16). Turn a parked wheel and the tread does not slide
// at once: static friction holds it while the patch and carcass twist, and
// the moment builds until the patch slides at mu Fz rbar. Stop turning and
// the twist stays wound in; roll and fresh, untwisted tread enters the patch.
// TMeasy has none of this ("No parking slip calculations / No dynamic
// parking torque"), so at a standstill the wheel was dead (Q1). Per tyre:
//
//     d(psi)/dt = d(delta)/dt - (|v| / sigma) psi,   |K psi| <= mu Fz rbar
//     M = -K psi
//
// elastic-perfectly-plastic: once the patch slides, psi is held at the limit.
// K from gen_innova.py (TMeasy's cornering stiffness through the brush
// model, [DERIV]); the patch length TMeasy's own, 2 sqrt(R0 depth), live;
// sigma = half of it [EST] (a rigid-carcass brush relaxes in ~0.45 of it).
// delta is the road-wheel angle relative to the CHASSIS, so path curvature
// (turn slip from yaw) is left out, as TMeasy leaves it out while rolling.
//
// Faded out over TMeasy's blend band by the same weight TMeasy gives its Dahl
// standstill force, 1 - SineStep(|v|, begin, end): this is that bristle
// regime's moment, and above blend_end (3.0 m/s) TMeasy's slip-based Mz has
// the wheel alone, so `sweep` is untouched. |v|, not TMeasy's signed vel.x:
// rolling relaxes the twist whichever way the car moves.
//
// FORCE FEEDBACK ONLY. The steering is position-commanded, so in the real car
// too this torque reaches only the rack and the driver's hands.
double ParkingTorque(World* w, int s, double fz, double dt) {
    double& psi = w->park_psi[s];
    const double delta = w->fb.road_steer_rad[s];
    const double turn = w->park_primed ? delta - w->park_prev_delta[s] : 0.0;
    w->park_prev_delta[s] = delta;

    const auto& tm = w->park_tire[s];
    const double depth = tm->GetDeflection();
    if (fz <= 1.0 || depth <= 0.0) { psi = 0.0; return 0.0; }   // airborne

    const double a = std::sqrt(w->park_r0 * depth);   // half the patch length
    const double b = 0.5 * w->park_w;

    const WheelState ws = w->veh->GetWheel(0, static_cast<VehicleSide>(s))->GetState();
    ChVector3d fwd = Vcross(ws.rot.GetAxisY(), ChVector3d(0, 0, 1));
    fwd.Normalize();
    const double v = std::abs(ws.lin_vel.Dot(fwd));

    psi = psi * std::exp(-v * dt / a) + turn;
    const double psi_max = w->park_mu * fz * MeanRadius(a, b) / w->park_k;
    psi = std::clamp(psi, -psi_max, psi_max);

    const double rolling = ChFunctionSineStep::Eval(v, tm->GetFrictionBlendBegin(), 0.0,
                                                    tm->GetFrictionBlendEnd(), 1.0);
    return -(1.0 - rolling) * w->park_k * psi;
}

void UpdateFeedback(World* w, double dt) {
    ChsVehicleFeedback& fb = w->fb;

    double mz_sum = 0.0;
    int grounded = 0;
    for (int s = 0; s < 2; ++s) {
        const auto side = static_cast<VehicleSide>(s);
        auto tire = w->veh->GetTire(0, side);          // axle 0 = front
        const TerrainForce tf = tire->ReportTireForce(w->terrain.get());

        // moment.z() is about the global vertical. With the car level that is
        // the steering axis to within the caster angle -- a few degrees, worth
        // under 0.5% of the magnitude, and far below the uncertainty in the
        // [EST] hardpoints that set the trail in the first place.
        fb.tire_mz_nm[s]     = tf.moment.z();
        fb.tire_fz_n[s]      = tf.force.z();
        fb.road_steer_rad[s] = w->veh->GetSteeringAngle(0, side);

        mz_sum += tf.moment.z();
        if (w->park_on) mz_sum += ParkingTorque(w, s, tf.force.z(), dt);
        if (tf.force.z() > 1.0) grounded |= (1 << s);   // >1 N: actually loaded
    }
    fb.front_grounded = grounded;
    w->park_primed = true;

    // Both front tyres act on one rack, so their moments SUM, then divide by
    // the overall steering ratio to reach the rim. A wheel lifted over a kerb
    // contributes nothing, which falls out for free: no load, no Mz.
    fb.column_torque_nm = mz_sum / kSteerRatio;

    // Power steering. The Innova's assist is engine-driven hydraulic, so it is
    // essentially constant rather than speed-scheduled like an electric rack;
    // one fraction is the right shape of model. 0.7 is [EST].
    double rim = fb.column_torque_nm * (1.0 - w->assist_frac) * w->ffb_gain;

    // Viscous damping on steering RATE. This is not seasoning. The wheel's
    // rotor inertia, the driver's arms, and one frame of transport delay form a
    // loop; a pure position-derived constant force closes it with positive gain
    // and the wheel buzzes around centre. Damping is what makes it stable, and
    // a real steering box has plenty of it from the fluid and the joints.
    const double rate = (w->inputs.m_steering - w->prev_steering) / (dt > 0.0 ? dt : 1e-3);
    w->prev_steering = w->inputs.m_steering;
    rim -= w->damping_nm_s * rate;

    fb.steer_torque_nm = rim;
    fb.speed_mps  = w->veh->GetSpeed();
    fb.engine_rpm = w->veh->GetEngine()->GetMotorSpeed() * kRadSToRpm;
    // The gear the DRIVER selected, not the one the gearbox is holding: in
    // neutral those differ on purpose (see Chs_SetDriverInput), and a HUD or a
    // referee rule must see what the driver did.
    fb.gear       = w->req_gear;
}

}  // namespace

// The struct the two sides share must agree byte-for-byte. These fire at
// compile time inside the container; the runtime ABI check in the UE module
// catches the case where the .so is stale relative to the header.
static_assert(sizeof(ChsVec3)  == 24, "ChsVec3 layout changed");
static_assert(sizeof(ChsQuat)  == 32, "ChsQuat layout changed");
static_assert(sizeof(ChsBodyState) == 24 + 32 + 24 + 24, "ChsBodyState has padding");
static_assert(sizeof(ChsDriverInput)     == 40, "ChsDriverInput layout changed");
static_assert(sizeof(ChsVehicleFeedback) == 88, "ChsVehicleFeedback layout changed");
static_assert(sizeof(ChsTerrainPatch)    == 64, "ChsTerrainPatch layout changed");

extern "C" {

int32_t Chs_GetAbiVersion(void) { return CHRONO_SHIM_ABI_VERSION; }

void Chs_GetChronoVersion(char* out_buf, int32_t buf_size) {
    if (!out_buf || buf_size <= 0) return;
    // Copy into caller storage. Returning CHRONO_VERSION directly would be
    // handing UE a pointer owned by this .so's runtime.
    std::snprintf(out_buf, static_cast<size_t>(buf_size), "%s", CHRONO_VERSION);
}

ChsWorld Chs_CreateWorld(double gravity_z) {
    return guard([&]() -> ChsWorld {
        auto* w = new World();
        w->sys.SetGravitationalAcceleration(ChVector3d(0, 0, gravity_z));
        // Single-threaded: this is the per-core real-time budget, and OpenMP
        // thread pools inside a 1 kHz loop are a jitter source.
        w->sys.SetNumThreads(1, 0, 1);

        w->material = chrono_types::make_shared<ChContactMaterialSMC>();
        w->material->SetYoungModulus(2e7f);
        w->material->SetFriction(0.9f);
        w->material->SetRestitution(0.1f);

        w->bodies.reserve(kMaxBodies);
        return static_cast<ChsWorld>(w);
    }, nullptr);
}

void Chs_DestroyWorld(ChsWorld world) {
    if (!world) return;
    {
        auto* w = static_cast<World*>(world);
        if (w->solver_stats && w->st_n > 0)
            std::fprintf(stderr, "CHS_SOLVER_STATS %s: %ld steps, iterations mean %.1f max %d, "
                         "error mean %.3e max %.3e\n", w->solver_name.c_str(), w->st_n,
                         w->st_it_sum / w->st_n, w->st_it_max,
                         w->st_err_sum / w->st_n, w->st_err_max);
        if (w->tire_env) PrintTireEnvelope(w);
    }
    // Allocated by this .so's operator new, so it must be freed here too --
    // UE's allocator is a different heap entirely.
    delete static_cast<World*>(world);
}

int32_t Chs_AddBox(ChsWorld world, double sx, double sy, double sz,
                   double density, ChsVec3 pos, int32_t is_fixed) {
    if (!world) return -1;
    auto* w = static_cast<World*>(world);
    if (static_cast<int>(w->bodies.size()) >= kMaxBodies) return -1;

    return guard([&]() -> int32_t {
        auto b = chrono_types::make_shared<ChBodyEasyBox>(
            sx, sy, sz, density, true, true, w->material);
        b->SetPos(ChVector3d(pos.x, pos.y, pos.z));
        b->SetFixed(is_fixed != 0);
        w->sys.Add(b);
        w->bodies.push_back(b);
        return static_cast<int32_t>(w->bodies.size()) - 1;
    }, -1);
}

double Chs_Step(ChsWorld world, double dt) {
    if (!world) return -1.0;
    auto* w = static_cast<World*>(world);

    return guard([&]() -> double {
        const auto t0 = stdc::steady_clock::now();
        if (w->is_vehicle) {
            // A vehicle is not stepped with DoStepDynamics directly. Chrono's
            // vehicle system is a Synchronize/Advance pair: Synchronize pushes
            // driver inputs and terrain state into every subsystem, Advance
            // integrates. Calling DoStepDynamics alone would integrate the
            // multibody system with last step's tyre forces and no steering.
            // ---- parking hold -------------------------------------------
            // Supplies the static friction the force-element tyre cannot. See
            // Chs_SetParkingHold in the header for the measurement and for why
            // gating on the BRAKE (rather than on speed alone) is what keeps
            // this clear of the hill-start rollback measurement.
            if (w->has_hold_acc) {
                auto body = w->veh->GetChassisBody();
                body->EmptyAccumulator(w->hold_acc);

                constexpr double kBrakeOn  = 0.8;    // pedal fraction
                constexpr double kStopped  = 0.25;   // m/s
                constexpr double kMu       = 0.85;   // matches the tyre

                if (w->parking_hold && w->inputs.m_braking > kBrakeOn) {
                    // IN THE ROAD PLANE ONLY (task 15). Friction acts along
                    // the road and cannot hold vertical motion, so neither
                    // may this. It first opposed the full 3-D velocity, and
                    // while braked and slow that froze the SUSPENSION: a
                    // braked spawn held the body at the bottom of its first
                    // spring compression (0.113 m ride height and 74% front
                    // load, against 0.2375 m and 51.3% free).
                    ChVector3d road_pt, n;
                    double road_h = 0.0;
                    float road_mu = 0.0f;
                    w->terrain->GetProperties(body->GetPos(), road_pt, road_h, n, road_mu);
                    const ChVector3d v3 = body->GetPosDt();
                    const ChVector3d v  = v3 - n * v3.Dot(n);
                    const double sp = v.Length();
                    if (sp > 1e-9 && sp < kStopped) {
                        // The load the tyres actually press on the road --
                        // which is also zero while the car is in the air, so
                        // a spawn drop is not "held" in mid-air.
                        double load = 0.0;
                        for (int a = 0; a < 2; ++a)
                            for (int sd = 0; sd < 2; ++sd) {
                                auto tire = w->veh->GetTire(a, static_cast<VehicleSide>(sd));
                                const double fn = tire->ReportTireForce(w->terrain.get()).force.Dot(n);
                                if (fn > 0.0) load += fn;
                            }
                        const double m = w->veh->GetMass();
                        // Force that would arrest the car in exactly one step.
                        const double need = m * sp / (dt > 0.0 ? dt : 1e-3);
                        // CLAMPED TO THE FRICTION CIRCLE, mu times the normal
                        // load. Without this the hold would be a kinematic
                        // weld and a slope steeper than the tyres can hold
                        // would still be held, which would make the rule
                        // untestable in the other direction.
                        const double fmax = kMu * load;
                        const double f = need < fmax ? need : fmax;
                        if (f > 0.0)
                            body->AccumulateForce(w->hold_acc, v * (-f / sp),
                                                  body->GetPos(), false);
                    }
                }
            }

            // ---- idle governor (task 8) ----------------------------------
            // Chrono's EngineShafts has none: at throttle 0 the engine coasts
            // down under its losses until something stops it, and every
            // clutch engagement drags it down with nothing to push back. A
            // diesel's pump governor fuels it to hold idle, with or without
            // the pedal. Modelled as the minimum-speed half of a min-max
            // governor: fuel = max(pedal, (idle - rpm) / band), so it is full
            // 200 rpm under idle and nothing at or above it. Proportional,
            // so it droops a little under load (a real one does too). The
            // loop is ~40 ms against a 1 ms step: 0.025 of the step's
            // stability margin. The driver's pedal in w->inputs is untouched.
            //
            // A governor that always fuels would make the engine UNSTALLABLE:
            // the torque map gives 250 N m at 0 rpm (Chrono has no firing
            // speed), so held on the brakes in gear it pushed on at 0 rpm and
            // turned the rear wheels through their brakes. So the engine
            // DIES below a firing speed, once it has run, and is dead -- no
            // fuel, only its losses -- until the driver restarts it: pedal
            // down or neutral for a second, the proxy for the key, since the
            // ABI has no ignition. The referee's STALL rule (rpm < 400 for
            // 0.75 s, stopped) then scores an engine that really died.
            DriverInputs drive = w->inputs;
            if (w->idle_rpm > 0.0) {
                constexpr double kGovBandRpm = 200.0;   // [EST]
                constexpr double kStallRpm   = 300.0;   // [EST] minimum firing speed
                constexpr double kCrankS     = 1.0;     // [EST] key to running
                const double rpm = w->veh->GetEngine()->GetMotorSpeed() * kRadSToRpm;
                if (w->engine_running) {
                    if (rpm >= 0.9 * w->idle_rpm) w->stall_armed = true;
                    if (w->stall_armed && rpm < kStallRpm) {
                        w->engine_running = false;
                        w->stall_armed = false;
                        w->crank_s = 0.0;
                    }
                } else {
                    const bool crank = w->req_gear == 0 || w->inputs.m_clutch >= 0.9;
                    w->crank_s = crank ? w->crank_s + dt : 0.0;
                    if (w->crank_s >= kCrankS) w->engine_running = true;
                }
                if (w->engine_running) {
                    double gov = (w->idle_rpm - rpm) / kGovBandRpm;
                    gov = gov < 0.0 ? 0.0 : (gov > 1.0 ? 1.0 : gov);
                    w->gov_throttle = gov;
                    if (gov > drive.m_throttle) drive.m_throttle = gov;
                } else {
                    w->gov_throttle = 0.0;
                    drive.m_throttle = 0.0;
                }
            }

            const double t = w->sim_time;
            w->terrain->Synchronize(t);
            w->veh->Synchronize(t, drive, *w->terrain);
            w->terrain->Advance(dt);
            w->veh->Advance(dt);
            if (w->solver_stats) {
                if (auto* it = w->veh->GetSystem()->GetSolver()->AsIterative()) {
                    const int    n = it->GetIterations();
                    const double e = it->GetError();
                    w->st_n      += 1;
                    w->st_it_sum += n;
                    w->st_err_sum += e;
                    if (n > w->st_it_max)  w->st_it_max  = n;
                    if (e > w->st_err_max) w->st_err_max = e;
                }
            }
            UpdateFeedback(w, dt);
            if (w->tire_env) RecordTireEnvelope(w, dt);
        } else {
            w->sys.DoStepDynamics(dt);
        }
        const auto t1 = stdc::steady_clock::now();

        w->sim_time    += dt;
        w->step_index  += 1;
        w->last_step_us = stdc::duration<double, std::micro>(t1 - t0).count();
        return w->last_step_us;
    }, -1.0);
}

int32_t Chs_GetSnapshot(ChsWorld world, ChsSnapshot* out) {
    if (!world || !out) return -1;
    auto* w = static_cast<World*>(world);

    return guard([&]() -> int32_t {
        // memset so padding bytes are deterministic -- this struct gets
        // memcpy'd into a ring buffer, and uninitialised padding would make
        // otherwise-identical snapshots compare unequal.
        std::memset(out, 0, sizeof(*out));
        out->sim_time     = w->sim_time;
        out->step_index   = w->step_index;
        out->last_step_us = w->last_step_us;

        if (w->is_vehicle) {
            // Slot 0 is the chassis REFERENCE frame, not its centre of mass.
            // ChBodyAuxRef separates the two, and the visual mesh is authored
            // against the reference frame; using GetPos() here would offset the
            // whole car by the CG offset -- a subtle, constant, wrong-looking
            // shift that is easy to misread as a units bug.
            const auto& rf = w->veh->GetRefFrame();
            out->bodies[0].position = ChsVec3{rf.GetPos().x(), rf.GetPos().y(), rf.GetPos().z()};
            out->bodies[0].rotation = ChsQuat{rf.GetRot().e0(), rf.GetRot().e1(),
                                              rf.GetRot().e2(), rf.GetRot().e3()};
            const auto cv = rf.GetPosDt();
            const auto cw = rf.GetAngVelLocal();
            out->bodies[0].linear_velocity  = ChsVec3{cv.x(), cv.y(), cv.z()};
            out->bodies[0].angular_velocity = ChsVec3{cw.x(), cw.y(), cw.z()};

            // Slots 1..4: spindles, FL FR RL RR. The renderer needs these
            // separately because the wheels steer and spin independently of
            // the chassis.
            int idx = 1;
            for (int a = 0; a < 2; ++a) {
                for (int s = 0; s < 2; ++s) {
                    const auto side = static_cast<VehicleSide>(s);
                    const auto p = w->veh->GetSpindlePos(a, side);
                    const auto q = w->veh->GetSpindleRot(a, side);
                    const auto v = w->veh->GetSpindleLinVel(a, side);
                    const auto o = w->veh->GetSpindleAngVel(a, side);
                    out->bodies[idx].position = ChsVec3{p.x(), p.y(), p.z()};
                    out->bodies[idx].rotation = ChsQuat{q.e0(), q.e1(), q.e2(), q.e3()};
                    out->bodies[idx].linear_velocity  = ChsVec3{v.x(), v.y(), v.z()};
                    out->bodies[idx].angular_velocity = ChsVec3{o.x(), o.y(), o.z()};
                    ++idx;
                }
            }
            out->body_count = idx;
            return idx;
        }

        const int n = static_cast<int>(w->bodies.size());
        out->body_count = n;

        for (int i = 0; i < n && i < kMaxBodies; ++i) {
            const auto& b = w->bodies[static_cast<size_t>(i)];
            const auto  p = b->GetPos();
            const auto  q = b->GetRot();
            const auto  v = b->GetPosDt();
            const auto  wv = b->GetAngVelLocal();

            // Field-by-field copy. Never reinterpret_cast a ChVector3d to a
            // ChsVec3: their layouts are only coincidentally similar, and that
            // coincidence is exactly what -march changes.
            out->bodies[i].position         = ChsVec3{p.x(), p.y(), p.z()};
            out->bodies[i].rotation         = ChsQuat{q.e0(), q.e1(), q.e2(), q.e3()};
            out->bodies[i].linear_velocity  = ChsVec3{v.x(), v.y(), v.z()};
            out->bodies[i].angular_velocity = ChsVec3{wv.x(), wv.y(), wv.z()};
        }
        return n;
    }, -1);
}


/* ---- Milestone 5: vehicle ---------------------------------------------- */

ChsWorld Chs_CreateVehicle(const char* data_path, double init_z) {
    /* ABI 2 behaviour, expressed in terms of ABI 3 so the two cannot drift.
     * Passing no patches selects the single flat 300 x 300 m default. */
    return Chs_CreateVehicleEx(data_path, init_z, nullptr, 0);
}

ChsWorld Chs_CreateVehicleEx(const char* data_path, double init_z,
                             const ChsTerrainPatch* patches, int32_t n_patches) {
    if (!data_path) return nullptr;
    return guard([&]() -> ChsWorld {
        // SetVehicleDataPath is global state inside Chrono_vehicle, so this is
        // not re-entrant across two differently-rooted vehicles. One vehicle
        // per process is all this simulator will ever need.
        //
        // GetVehicleDataFile does a plain string CONCATENATION -- it inserts no
        // separator. Without the trailing slash you get
        // "/work/vehicleinnova/innova_Vehicle.json" and a "could not open"
        // that reads like a missing file rather than a malformed path.
        std::string root(data_path);
        if (!root.empty() && root.back() != '/') root.push_back('/');
        SetVehicleDataPath(root);

        auto* w = new World();
        w->is_vehicle = true;

        w->veh = std::make_shared<WheeledVehicle>(GetVehicleDataFile(kVehJson),
                                                  ChContactMethod::SMC);
        w->veh->Initialize(ChCoordsys<>(ChVector3d(0, 0, init_z), QUNIT));
        w->veh->SetChassisVisualizationType(VisualizationType::NONE);

        auto engine = ReadEngineJSON(GetVehicleDataFile(kEngJson));
        // The idle speed for the governor (task 8). Not a Chrono key --
        // EngineShafts has no governor -- but gen_innova.py writes it into the
        // engine's own file so there is one source for it, and Chrono ignores
        // members it does not read. CHS_IDLE_GOVERNOR=0 (diagnostic) turns
        // the governor off: the pre-task-8 engine.
        {
            rapidjson::Document d;
            ReadFileJSON(GetVehicleDataFile(kEngJson), d);
            if (d.IsObject() && d.HasMember("Idle Speed [rpm]") && d["Idle Speed [rpm]"].IsNumber())
                w->idle_rpm = d["Idle Speed [rpm]"].GetDouble();
            if (const char* g = std::getenv("CHS_IDLE_GOVERNOR"))
                if (std::atoi(g) == 0) w->idle_rpm = 0.0;
        }
        auto trans  = ReadTransmissionJSON(GetVehicleDataFile(kTrnJson));
        w->veh->InitializePowertrain(
            chrono_types::make_shared<ChPowertrainAssembly>(engine, trans));

        // Seed the cached gear from the transmission's ACTUAL state. Chrono
        // initialises a manual gearbox in 1st, not neutral. Leaving the cache
        // at 0 makes Chs_SetDriverInput believe neutral is already selected, so
        // the very first SetGear(0) is skipped and the car sits in gear while
        // reporting neutral -- which on a hill start looks exactly like the
        // clutch model being wrong.
        w->box_gear = w->veh->GetTransmission()->GetCurrentGear();
        w->req_gear = w->box_gear;

        for (auto& axle : w->veh->GetAxles()) {
            const bool front = (axle == w->veh->GetAxles()[0]);
            for (auto& wheel : axle->GetWheels()) {
                auto tire = ReadTireJSON(GetVehicleDataFile(front ? kTireF : kTireR));
                w->veh->InitializeTire(tire, wheel, VisualizationType::NONE);
            }
        }

        // The TMeasy blend range. Not Chrono's 1.0 / 3.0: at 1.0 the aligning
        // moment is gated off below 1 m/s and the steering is dead through
        // every parking manoeuvre in the test. See CHS_TIRE_BLEND_BEGIN in the
        // header for the measurement and for the regressions that had to pass
        // before it was adopted. sigma0/sigma1 are left stock -- those were
        // swept and found not to move anything.
        //
        // Applied here, before the terrain and the solver, so that every path
        // into a vehicle world gets it: Chs_CreateVehicle wraps this function,
        // and Chs_SetTireStandstill can still override it afterwards.
        for (auto& axle : w->veh->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tm = std::dynamic_pointer_cast<ChTMeasyTire>(wheel->GetTire());
                if (tm) tm->SetFrictionBlendRange(CHS_TIRE_BLEND_BEGIN,
                                                  CHS_TIRE_BLEND_END);
            }
        }

        // The parking torque (task 16), from the front tyre's own JSON, where
        // gen_innova.py writes it beside TMeasy's keys; Chrono ignores it. It
        // needs TMeasy (the deflection, the blend range), so a Fiala or rigid
        // tyre gets none. CHS_PARKING_TORQUE=0 (diagnostic) turns it off: the
        // pre-task-16 wheel, same code.
        {
            rapidjson::Document d;
            ReadFileJSON(GetVehicleDataFile(kTireF), d);
            bool ok = d.IsObject() && d.HasMember("Parking Torque")
                      && d["Parking Torque"].IsObject();
            for (int s = 0; s < 2; ++s) {
                w->park_tire[s] = std::dynamic_pointer_cast<ChTMeasyTire>(
                    w->veh->GetTire(0, static_cast<VehicleSide>(s)));
                ok = ok && w->park_tire[s];
            }
            if (ok) {
                const auto& p = d["Parking Torque"];
                auto num = [](const rapidjson::Value& o, const char* k) {
                    return o.HasMember(k) && o[k].IsNumber() ? o[k].GetDouble() : 0.0;
                };
                w->park_k  = num(p, "Twist Stiffness [Nm/rad]");
                w->park_w  = num(p, "Patch Width [m]");
                w->park_mu = num(d, "Coefficient of Friction");
                if (d.HasMember("Design") && d["Design"].IsObject())
                    w->park_r0 = num(d["Design"], "Unloaded Radius [m]");
                w->park_on = w->park_k > 0.0 && w->park_w > 0.0
                             && w->park_mu > 0.0 && w->park_r0 > 0.0;
            }
            if (const char* e = std::getenv("CHS_PARKING_TORQUE"))
                if (std::atoi(e) == 0) w->park_on = false;
        }

        w->terrain = chrono_types::make_shared<RigidTerrain>(w->veh->GetSystem());

        if (!patches || n_patches <= 0) {
            // ABI-2 default: one flat patch, big enough for anything.
            auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
            mat->SetFriction(0.9f);
            mat->SetYoungModulus(2e7f);
            w->terrain->AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300, 300);
        } else {
            for (int32_t i = 0; i < n_patches; ++i) {
                const ChsTerrainPatch& p = patches[i];

                // A fresh material per patch. They could share one when the
                // friction matches, but a patch is also where a low-grip
                // surface would be expressed, and one material per patch keeps
                // that a data change rather than a code change.
                auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
                mat->SetFriction(static_cast<float>(p.friction > 0.0 ? p.friction : 0.9));
                mat->SetYoungModulus(2e7f);

                // Rotation about Y. Chrono is right-handed with +Y left, so a
                // positive rotation about +Y tips +X down toward -Z; the
                // generator therefore emits a NEGATIVE pitch for a climb. The
                // coordsys origin is the centre of the DRIVING SURFACE, not of
                // the box -- Chrono adds the thickness downward from it.
                const ChQuaternion<> rot = QuatFromAngleY(p.pitch_rad);
                w->terrain->AddPatch(
                    mat,
                    ChCoordsys<>(ChVector3d(p.center_x, p.center_y, p.center_z), rot),
                    p.length, p.width);
            }
        }
        w->terrain->Initialize();

        // One force accumulator on the chassis for the parking hold. Added
        // once here so the hot path only clears and fills it.
        w->hold_acc = w->veh->GetChassisBody()->AddAccumulator();
        w->has_hold_acc = true;

        // THE SOLVER: PSOR, 50 iterations, cold start (task 13).
        //
        // A clutch is not a joint. ChShaftsClutch holds its two shafts
        // together only while the torque that takes stays inside a box,
        // |lambda| <= modulation * limit; past it, the shafts slip. That is a
        // variational inequality, and only a VI solver -- one that projects
        // each multiplier onto its box (PSOR, APGD, BB, ADMM) -- solves it.
        // MINRES, used here until task 13, is a LINEAR Krylov solver: it
        // treats the box as an equality, so every clutch was welded shut --
        // the pedal did nothing and the rear differential (its lock clutch
        // at modulation 0, i.e. open) was a spool. Measured: `clutch` moved
        // off with the pedal on the floor; `diff` spun both rear wheels at
        // 1.002x each other where the ground asked for 0.714.
        //
        // Accuracy, measured with `solvers` against the EXACT direct solve
        // (SPARSE_QR) of the same physics: in a steady 5 m/s turn, QR with an
        // open differential and PSOR at 1000 iterations agree to four
        // figures (yaw 16.652 deg/s, roll 1.465 deg, column -2.298 N m), and
        // PSOR at 50 is within 0.2-0.6% of that; at rest PSOR 50 equals
        // PSOR 1000. Cost 140-160 us/step against MINRES's 185-208.
        //
        // NO WARM START: warm-started PSOR diverges on this car (residual
        // 1-3 against 0.02-0.09 cold) and gets the turn 50% wrong.
        auto* sys = w->veh->GetSystem();
        sys->SetSolverType(ChSolver::Type::PSOR);
        sys->GetSolver()->AsIterative()->SetMaxIterations(50);
        sys->GetSolver()->AsIterative()->EnableWarmStart(false);
        // DIAGNOSTIC ONLY -- unset in production. Swaps the solver:
        // CHS_SOLVER=minres reproduces the pre-task-13 car, clutches welded.
        //
        // `qr` and `lu` are Chrono's direct sparse solvers: far too slow for
        // the 1 kHz loop, but they solve the joints EXACTLY, which makes them
        // the reference an iterative solver's statics are judged against
        // (they drop the clutch box too, like MINRES). CHS_SOLVER_ITERS,
        // _TOL and _WARM tune an iterative solver; CHS_SOLVER_STATS=1 prints
        // its per-step iterations and residual when the world is destroyed.
        if (const char* s = std::getenv("CHS_SOLVER")) {
            const std::string name(s);
            if (name == "psor")        sys->SetSolverType(ChSolver::Type::PSOR);
            else if (name == "apgd")   sys->SetSolverType(ChSolver::Type::APGD);
            else if (name == "bb")     sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
            else if (name == "minres") sys->SetSolverType(ChSolver::Type::MINRES);
            else if (name == "qr")     sys->SetSolverType(ChSolver::Type::SPARSE_QR);
            else if (name == "lu")     sys->SetSolverType(ChSolver::Type::SPARSE_LU);
            if (auto* it = sys->GetSolver()->AsIterative()) {
                it->SetMaxIterations(std::getenv("CHS_SOLVER_ITERS")
                                         ? std::atoi(std::getenv("CHS_SOLVER_ITERS"))
                                         : (name == "minres" ? 60 : 150));
                if (const char* tol = std::getenv("CHS_SOLVER_TOL"))
                    it->SetTolerance(std::atof(tol));
                if (const char* ws = std::getenv("CHS_SOLVER_WARM"))
                    it->EnableWarmStart(std::atoi(ws) != 0);
            }
            w->solver_name = name;
            // A LINEAR solver cannot take Chrono's 1e20 neutral ratio (M5:
            // MINRES dropped the car through the ground); keep the old
            // top-gear workaround for these diagnostic runs.
            w->true_neutral = !(name == "minres" || name == "qr" || name == "lu");
        }
        w->solver_stats = std::getenv("CHS_SOLVER_STATS") != nullptr;
        w->tire_env = std::getenv("CHS_TIRE_ENVELOPE") != nullptr;   // diagnostic (Q1)
        // DIAGNOSTIC: CHS_NEUTRAL=hold restores the pre-task-10 neutral (top
        // gear held, clutch forced open) under any solver, for a same-code
        // before/after.
        if (const char* n = std::getenv("CHS_NEUTRAL"))
            if (std::string(n) == "hold") w->true_neutral = false;
        sys->SetNumThreads(1, 0, 1);

        // Find the driveline's two clutches by their SHAFTS, not by guessing
        // from torque limits: the differential lock is the clutch between the
        // driven axle's two axle shafts ("Suspension Indexes": [1], the rear);
        // the other one is the pedal's.
        {
            auto rear = w->veh->GetAxles()[1]->m_suspension;
            const ChShaft* al = rear->GetAxle(VehicleSide::LEFT).get();
            const ChShaft* ar = rear->GetAxle(VehicleSide::RIGHT).get();
            for (auto& item : sys->GetOtherPhysicsItems()) {
                auto c = std::dynamic_pointer_cast<ChShaftsClutch>(item);
                if (!c) continue;
                const bool lock = (c->GetShaft1() == al && c->GetShaft2() == ar) ||
                                  (c->GetShaft1() == ar && c->GetShaft2() == al);
                (lock ? w->diff_lock : w->pedal_clutch) = c;
            }
        }
        // DIAGNOSTIC: remove the differential lock's constraint entirely, so a
        // LINEAR solver (MINRES, QR) sees an open differential too.
        if (std::getenv("CHS_OPEN_DIFF") && w->diff_lock) w->diff_lock->SetDisabled(true);
        if (w->solver_stats)
            std::fprintf(stderr, "CHS_CLUTCHES pedal %s (limit %.0f N m), diff lock %s (limit %.0f N m)%s\n",
                         w->pedal_clutch ? "found" : "MISSING",
                         w->pedal_clutch ? w->pedal_clutch->GetTorqueLimit() : 0.0,
                         w->diff_lock ? "found" : "MISSING",
                         w->diff_lock ? w->diff_lock->GetTorqueLimit() : 0.0,
                         std::getenv("CHS_OPEN_DIFF") ? ", lock DISABLED" : "");

        return static_cast<ChsWorld>(w);
    }, nullptr);
}

void Chs_SetDriverInput(ChsWorld world, ChsDriverInput in) {
    if (!world) return;
    auto* w = static_cast<World*>(world);
    if (!w->is_vehicle) return;

    guard([&]() -> int {
        w->inputs.m_steering = clamp11(in.steering);
        w->inputs.m_throttle = clamp01(in.throttle);
        w->inputs.m_braking  = clamp01(in.braking);
        w->inputs.m_clutch   = clamp01(in.clutch);

        int g = in.gear;
        const int gmax = w->veh->GetTransmission()->GetMaxGear();
        if (g > gmax) g = 0;      // 6th gate on a 5-speed: neutral, not a crash
        if (g < -1)   g = -1;
        w->req_gear = g;

        // NEUTRAL, since task 10, is passed to the transmission -- under a VI
        // solver. ChTransmission::SetGear(0) sets the ChShaftsGearbox ratio
        // to CHRONO_NEUTRAL_GEAR_RATIO, #defined 1e20, and
        // ChManualTransmissionShafts::Synchronize opens the clutch in gear 0.
        // The gearbox row then reads 1e20*w_in - w_out = 0: the input shaft
        // is pinned to the gearbox casing and the driveshaft turns free.
        //
        // M5 could not do this. MINRES, a Krylov method on the WHOLE system,
        // met a 1e40 spread in the Schur diagonal, stopped converging, and
        // returned garbage for every reaction, tyre contacts included: the
        // stationary car fell from z = +0.264 m to -2.017 m in one second.
        // PSOR solves one row at a time, projecting each exactly, so that row
        // just pins the input shaft and nothing else sees its scale
        // (measured: `neutral`, and CHS_SOLVER_STATS=1 on it).
        //
        // A LINEAR solver (CHS_SOLVER=minres|qr|lu, diagnostic) still gets
        // the M5 workaround below: the gearbox held in a finite gear and the
        // clutch forced open. An H-pattern shifter passes THROUGH neutral on
        // every change, so this path is exercised constantly.
        if (g == 0 && !w->true_neutral) {
            w->inputs.m_clutch = 1.0;          // fully disengaged

            // ...and hold the gearbox in its HIGHEST ratio, not its last one.
            //
            // Holding a gear leaves the gearbox INPUT shaft -- behind the
            // clutch -- geared to the wheels, and reflected inertia goes as
            // the SQUARE of the ratio: its 0.3 kg m^2 is worth ~751 kg of car
            // in 1st (16.9:1) and ~28 kg in 5th (3.3:1). The highest gear is
            // the smallest ratio, so the least phantom inertia available
            // without a true neutral (task 10).
            //
            // This was first MEASURED with the clutch welded shut (task 13):
            // "held in 1st -52.4%, in 5th -14.6%" of g*sin(theta) in free
            // roll, tracking an equivalent mass of 2047 kg in 1st -- that was
            // the ENGINE's inertia too, which a working clutch now removes.
            const int top = w->veh->GetTransmission()->GetMaxGear();
            if (top > 0 && top != w->box_gear) {
                w->veh->GetTransmission()->SetGear(top);
                w->box_gear = top;
            }
        } else if (g != w->box_gear) {
            w->veh->GetTransmission()->SetGear(g);
            w->box_gear = g;
        }
        return 0;
    }, 0);
}

int32_t Chs_GetFeedback(ChsWorld world, ChsVehicleFeedback* out) {
    if (!world || !out) return -1;
    auto* w = static_cast<World*>(world);
    if (!w->is_vehicle) return -1;
    std::memcpy(out, &w->fb, sizeof(*out));
    return 0;
}

double Chs_GetTerrainHeight(ChsWorld world, double x, double y) {
    if (!world) return 0.0;
    auto* w = static_cast<World*>(world);
    if (!w->is_vehicle || !w->terrain) return 0.0;
    return guard([&]() -> double {
        // ChTerrain::GetHeight takes a world point; z is ignored for a
        // RigidTerrain box lookup but is part of the signature.
        return w->terrain->GetHeight(ChVector3d(x, y, 0.0));
    }, 0.0);
}

int32_t Chs_SetParkingHold(ChsWorld world, int32_t enable) {
    if (!world) return -1;
    auto* w = static_cast<World*>(world);
    if (!w->is_vehicle) return -1;
    w->parking_hold = (enable != 0);
    return 0;
}

int32_t Chs_SetTireStandstill(ChsWorld world, double sigma0, double sigma1,
                             double blend_begin, double blend_end) {
    if (!world) return -1;
    auto* w = static_cast<World*>(world);
    if (!w->is_vehicle) return -1;
    return guard([&]() -> int32_t {
        for (auto& axle : w->veh->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                // Only TMeasy has these; a Fiala or Rigid tyre set simply is
                // not this model, so skip rather than fail. That keeps
                // TYRE_MODEL=fiala/rigid runnable through the same shim.
                auto tm = std::dynamic_pointer_cast<ChTMeasyTire>(wheel->GetTire());
                if (!tm) continue;
                const double s0 = sigma0 > 0.0 ? sigma0 : tm->GetDahlSigma0();
                const double s1 = sigma1 > 0.0 ? sigma1 : tm->GetDahlSigma1();
                tm->SetDahlParameters(s0, s1);
                if (blend_begin > 0.0 && blend_end > blend_begin) {
                    tm->SetFrictionBlendRange(blend_begin, blend_end);
                }
            }
        }
        return 0;
    }, -1);
}

int32_t Chs_GetTireStandstill(ChsWorld world, double* sigma0, double* sigma1,
                              double* blend_begin, double* blend_end) {
    if (!world) return -1;
    auto* w = static_cast<World*>(world);
    if (!w->is_vehicle) return -1;
    return guard([&]() -> int32_t {
        for (auto& axle : w->veh->GetAxles()) {
            for (auto& wheel : axle->GetWheels()) {
                auto tm = std::dynamic_pointer_cast<ChTMeasyTire>(wheel->GetTire());
                if (!tm) continue;
                if (sigma0)      *sigma0      = tm->GetDahlSigma0();
                if (sigma1)      *sigma1      = tm->GetDahlSigma1();
                if (blend_begin) *blend_begin = tm->GetFrictionBlendBegin();
                if (blend_end)   *blend_end   = tm->GetFrictionBlendEnd();
                return 0;      /* all four are set identically */
            }
        }
        return -1;
    }, -1);
}

void Chs_SetFfbTuning(ChsWorld world, double assist_frac,
                      double damping_nm_s, double gain) {
    if (!world) return;
    auto* w = static_cast<World*>(world);
    w->assist_frac  = clamp01(assist_frac);
    w->damping_nm_s = damping_nm_s < 0.0 ? 0.0 : damping_nm_s;
    w->ffb_gain     = gain;
}

}  // extern "C"
