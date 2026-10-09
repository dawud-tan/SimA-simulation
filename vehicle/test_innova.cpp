// ===========================================================================
//  Milestone 4 verification harness for the Innova Reborn model.
//
//  Modes:
//    assemble   load the JSON model and report the assembled mass properties
//    ackermann  sweep the steering rack and measure actual left/right wheel
//               angles against the ideal Ackermann relation
//    torque     dump the 2GD-FTV torque curve as the engine model sees it
//    accel      full-throttle acceleration in 1st..5th
//    hill       hill-start on a 20 deg ramp (the SIM A element)
//    perf       per-step cost of the full vehicle at 1 kHz
//    tire       TMeasy vs ANCF cost per step on ChTireTestRig (M4)
//    ancfland   Q1: one tyre settled under load -- sound? cost per step?
//               ancfland <tyre> [dt] [minres|qr|lu] [seconds]
//    ancfpark   Q1: one tyre steered at standstill -- the parking torque
//               ancfpark <tmeasy|innova|polaris|hmmwv|file> [rate deg/s] [amp deg] [dt] [solver]
//    ancfroll   Q1: one tyre rolling at a held speed and slip angle
//               ancfroll <tyre> [speed m/s] [slip deg] [dt] [solver]
//               env: RIG_LOAD (N), RIG_NODE_R (m), RIG_STICK (k_t, N/m: a
//               friction that sticks), RIG_ITERS, RIG_THREADS, RIG_ROLL_T,
//               RIG_LAND_T, RIG_SETTLE_T, RIG_SETTLE_C, RIG_HOLD (s), RIG_STIFF
//               (implicit contact), RIG_TOL, RIG_LOCK, RIG_PRINT_DT,
//               RIG_INTEGRATOR=hht, RIG_NEWTON, RIG_SURFACE=mesh, RIG_TAIL,
//               RIG_CONTACT_E (Pa)
// ===========================================================================
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <functional>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolver.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"
#include "chrono/functions/ChFunctionConst.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono_vehicle/ChPowertrainAssembly.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/functions/ChFunctionInterp.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkTSDA.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <utility>

#include <chrono>
namespace stdc = std::chrono;
using namespace chrono;
using namespace chrono::vehicle;

static const std::string kVeh = "innova/innova_Vehicle.json";
static const std::string kEng = "innova/innova_Engine.json";
static const std::string kTrn = "innova/innova_Transmission.json";
static const std::string kTF  = "innova/innova_TireF.json";
static const std::string kTR  = "innova/innova_TireR.json";

// Build a fully initialised vehicle on flat rigid terrain.
static std::shared_ptr<WheeledVehicle> MakeVehicle(std::shared_ptr<RigidTerrain>& terrain,
                                                   double init_z = 0.45) {
    std::cout << "  [1] data path: " << GetVehicleDataPath() << std::endl;
    std::cout << "  [2] vehicle file: " << GetVehicleDataFile(kVeh) << std::endl;
    auto veh = std::make_shared<WheeledVehicle>(GetVehicleDataFile(kVeh), ChContactMethod::SMC);
    std::cout << "  [3] vehicle constructed" << std::endl;
    veh->Initialize(ChCoordsys<>(ChVector3d(0, 0, init_z), QUNIT));
    std::cout << "  [4] vehicle initialized" << std::endl;
    veh->SetChassisVisualizationType(VisualizationType::NONE);

    auto engine = ReadEngineJSON(GetVehicleDataFile(kEng));
    auto trans  = ReadTransmissionJSON(GetVehicleDataFile(kTrn));
    std::cout << "  [5] engine+trans read" << std::endl;
    veh->InitializePowertrain(chrono_types::make_shared<ChPowertrainAssembly>(engine, trans));
    std::cout << "  [6] powertrain initialized" << std::endl;

    for (auto& axle : veh->GetAxles()) {
        const bool front = (axle == veh->GetAxles()[0]);
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(GetVehicleDataFile(front ? kTF : kTR));
            veh->InitializeTire(tire, wheel, VisualizationType::NONE);
        }
    }

    std::cout << "  [7] tires initialized" << std::endl;
    terrain = chrono_types::make_shared<RigidTerrain>(veh->GetSystem());
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetFriction(0.9f);
    mat->SetYoungModulus(2e7f);
    auto patch = terrain->AddPatch(mat, ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), 300, 300);
    patch->SetColor(ChColor(0.6f, 0.6f, 0.6f));
    terrain->Initialize();
    std::cout << "  [8] terrain ready" << std::endl;
    return veh;
}

// ---------------------------------------------------------------------------
static int DoAssemble(const std::string& solver) {
    std::shared_ptr<RigidTerrain> terrain;
    auto veh = MakeVehicle(terrain);
    if (solver == "minres") {
        veh->GetSystem()->SetSolverType(ChSolver::Type::MINRES);
        veh->GetSystem()->GetSolver()->AsIterative()->SetMaxIterations(60);
    }
    std::cout << "  solver: " << (solver.empty() ? "default" : solver) << "\n";

    std::cout << "\n=== ASSEMBLED VEHICLE ===\n";
    std::cout << "  " << veh->GetName() << "\n";
    std::cout << "  total mass        : " << std::fixed << std::setprecision(1)
              << veh->GetMass() << " kg\n";
    std::cout << "  wheelbase         : " << veh->GetWheelbase() << " m\n";
    std::cout << "  track (front)     : " << veh->GetWheeltrack(0) << " m\n";
    std::cout << "  track (rear)      : " << veh->GetWheeltrack(1) << " m\n";
    std::cout << "  min turn radius   : " << veh->GetMinTurningRadius() << " m\n";
    std::cout << "  max steer angle   : " << veh->GetMaxSteeringAngle() * CH_RAD_TO_DEG << " deg\n";
    std::cout << "  num axles         : " << veh->GetNumberAxles() << "\n";
    std::cout << "  bodies in system  : " << veh->GetSystem()->GetBodies().size() << "\n";
    std::cout << "  links in system   : " << veh->GetSystem()->GetLinks().size() << "\n";

    // Settle under gravity, then read static corner loads. This is the single
    // best check that spring rates and ride height agree with the mass: if the
    // car sinks to the bump stops or floats, the numbers are wrong.
    DriverInputs in{0, 0, 0, 0};
    std::cout << "\n  t[s]   chassis_z   FL_wheel_z   RL_wheel_z   Fz_FL     Fz_RL\n";
    for (int i = 0; i < 4000; ++i) {
        terrain->Synchronize(i * 1e-3);
        veh->Synchronize(i * 1e-3, in, *terrain);
        terrain->Advance(1e-3);
        veh->Advance(1e-3);
        if (i % 500 == 0) {
            const double fw = veh->GetAxle(0)->GetWheel(LEFT)->GetPos().z();
            const double rw = veh->GetAxle(1)->GetWheel(LEFT)->GetPos().z();
            const double f0 = veh->GetAxle(0)->GetWheel(LEFT)->GetTire()
                                 ->ReportTireForce(terrain.get()).force.z();
            const double f1 = veh->GetAxle(1)->GetWheel(LEFT)->GetTire()
                                 ->ReportTireForce(terrain.get()).force.z();
            std::cout << "  " << std::setw(4) << std::setprecision(2) << i*1e-3
                      << "   " << std::setw(9) << std::setprecision(4) << veh->GetPos().z()
                      << "   " << std::setw(10) << fw
                      << "   " << std::setw(10) << rw
                      << "   " << std::setw(7) << std::setprecision(0) << f0
                      << "   " << std::setw(7) << f1 << "\n";
        }
    }
    std::cout << "\n=== AFTER 2 s SETTLING ===\n";
    const auto pos = veh->GetPos();
    std::cout << "  chassis z         : " << std::setprecision(4) << pos.z() << " m\n";
    double total = 0;
    for (int a = 0; a < 2; ++a)
        for (int s = 0; s < 2; ++s) {
            const auto& wheel = veh->GetAxle(a)->GetWheel(s == 0 ? LEFT : RIGHT);
            const double fz = wheel->GetTire()->ReportTireForce(terrain.get()).force.z();
            std::cout << "  Fz axle" << a << (s == 0 ? " L" : " R") << "        : "
                      << std::setprecision(1) << fz << " N  (" << fz / 9.81 << " kg)\n";
            total += fz;
        }
    std::cout << "  sum Fz            : " << total << " N  (" << total / 9.81 << " kg)\n";
    std::cout << "  expected          : " << veh->GetMass() * 9.81 << " N\n";
    return 0;
}

// ---------------------------------------------------------------------------
// Ackermann: sweep the rack and compare the achieved inner/outer wheel angles
// with the ideal relation
//     cot(delta_outer) - cot(delta_inner) = track / wheelbase
static int DoAckermann() {
    std::shared_ptr<RigidTerrain> terrain;
    auto veh = MakeVehicle(terrain);

    const double L = veh->GetWheelbase();
    const double T = veh->GetWheeltrack(0);
    std::cout << "\n=== ACKERMANN GEOMETRY ===\n";
    std::cout << "  wheelbase " << L << " m, front track " << T << " m\n";
    std::cout << "  ideal:  cot(outer) - cot(inner) = T/L = "
              << std::fixed << std::setprecision(4) << T / L << "\n\n";
    std::cout << "  steer     inner     outer   cot(o)-cot(i)    ideal    Ackermann%\n";
    std::cout << "  -----   -------   -------   -------------   -------   ----------\n";

    DriverInputs in{0, 0, 0, 0};
    for (double s = 0.1; s <= 1.001; s += 0.1) {
        in.m_steering = s;
        // Let the steering settle quasi-statically.
        for (int i = 0; i < 400; ++i) {
            terrain->Synchronize(0); veh->Synchronize(0, in, *terrain);
            terrain->Advance(1e-3);  veh->Advance(1e-3);
        }
        // Spindle rotation about Z relative to chassis = road-wheel steer angle.
        const auto chassis_rot = veh->GetChassisBody()->GetRot();
        auto steer_of = [&](VehicleSide side) {
            const auto sp = veh->GetAxle(0)->m_suspension->GetSpindle(side);
            const auto rel = chassis_rot.GetConjugate() * sp->GetRot();
            // Use the SPIN AXIS (spindle local Y), not the local X. The spindle
            // rotates about Y as the wheel rolls, so anything derived from its
            // X axis is corrupted by wheel rotation -- which is what made the
            // first attempt report 66 deg of inner steer and 200% Ackermann.
            // The spin axis is invariant under that rotation.
            const auto spin = rel.Rotate(ChVector3d(0, 1, 0));
            // A wheel steered by delta has spin axis (-sin d, cos d, 0).
            return std::atan2(-spin.x(), spin.y());
        };
        // Positive steering input turns left: LEFT is inner.
        const double dl = steer_of(LEFT), dr = steer_of(RIGHT);
        const double inner = std::fabs(dl), outer = std::fabs(dr);
        // Below ~8 deg the metric cot(o)-cot(i) is a difference of two large
        // numbers and is dominated by noise; quote it only where it means
        // something. Industry practice is to characterise Ackermann at a
        // reference inner angle around 20 deg.
        if (inner * CH_RAD_TO_DEG < 12.0) continue;
        const double lhs = 1.0 / std::tan(outer) - 1.0 / std::tan(inner);
        const double ideal = T / L;
        std::cout << "  " << std::setw(5) << std::setprecision(2) << s
                  << "   " << std::setw(6) << std::setprecision(3) << inner * CH_RAD_TO_DEG
                  << "d  " << std::setw(6) << outer * CH_RAD_TO_DEG
                  << "d   " << std::setw(13) << std::setprecision(4) << lhs
                  << "   " << std::setw(7) << ideal
                  << "   " << std::setw(9) << std::setprecision(1) << 100.0 * lhs / ideal << "%\n";
    }
    std::cout << "\n  100% = true Ackermann (all four wheels roll about one centre).\n"
                 "  Production cars usually run 60-90%: full Ackermann costs\n"
                 "  high-speed cornering grip. For a parking test, high is right.\n";
    return 0;
}


// ---------------------------------------------------------------------------
// Load each subsystem JSON on its own so a parser crash names the guilty file.
static int DoProbe() {
    struct Item { const char* label; std::function<void()> fn; };
    std::vector<Item> items = {
      {"Chassis",      []{ ReadChassisJSON(GetVehicleDataFile("innova/innova_Chassis.json")); }},
      {"FrontSusp",    []{ ReadSuspensionJSON(GetVehicleDataFile("innova/innova_FrontSusp.json")); }},
      {"RearSusp",     []{ ReadSuspensionJSON(GetVehicleDataFile("innova/innova_RearSusp.json")); }},
      {"Steering",     []{ ReadSteeringJSON(GetVehicleDataFile("innova/innova_Steering.json")); }},
      {"Driveline",    []{ ReadDrivelineWVJSON(GetVehicleDataFile("innova/innova_Driveline.json")); }},
      {"Engine",       []{ ReadEngineJSON(GetVehicleDataFile("innova/innova_Engine.json")); }},
      {"Transmission", []{ ReadTransmissionJSON(GetVehicleDataFile("innova/innova_Transmission.json")); }},
      {"TireF",        []{ ReadTireJSON(GetVehicleDataFile("innova/innova_TireF.json")); }},
      {"TireR",        []{ ReadTireJSON(GetVehicleDataFile("innova/innova_TireR.json")); }},
    };
    for (auto& it : items) {
        std::cout << "  probing " << std::setw(14) << std::left << it.label << std::flush;
        try { it.fn(); std::cout << "  OK\n"; }
        catch (const std::exception& e) { std::cout << "  EXCEPTION: " << e.what() << "\n"; }
    }
    std::cout << "  (a crash stops at the guilty file)\n";
    return 0;
}


// ---------------------------------------------------------------------------
// TMeasy vs ANCF: the Milestone 4 decision, measured.
//
// Both tyres are mounted on ChTireTestRig and stepped identically. The claim
// under test is a TIMING claim, so the ANCF exemplar is Chrono's shipped
// Polaris tyre (0.33 m radius -- close to the Innova's 0.336 m). Its exact
// compound is irrelevant to the cost-per-step question, and no orthotropic
// layer data exists publicly for the Innova's 205/65R16.
static int DoTire(int steps) {
    struct Result { std::string name; int dof; double mean, p99, max; };
    std::vector<Result> results;

    auto run = [&](const std::string& label, const std::string& tire_json,
                   double tire_step, bool abs_path = false) -> void {
        ChSystemSMC sys;
        sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
        // Q1 found this missing: a Chrono 10 ChSystem has NO collision system
        // until one is set, so the ANCF tyre this mode timed in M4 never
        // touched the road (and its MINRES/100 solve was not converged).
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
        sys.SetNumThreads(1, 0, 1);
        // Stiff FEA needs an implicit integrator and an iterative linear solve;
        // see the Milestone 1 deep dive.
        sys.SetSolverType(ChSolver::Type::MINRES);
        sys.GetSolver()->AsIterative()->SetMaxIterations(100);
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

        auto wheel = ReadWheelJSON(GetVehicleDataFile("innova/innova_Wheel.json"));
        auto tire  = ReadTireJSON(abs_path ? tire_json : GetVehicleDataFile(tire_json));

        ChTireTestRig rig(wheel, tire, &sys);
        rig.SetGravitationalAcceleration(9.81);
        rig.SetNormalLoad(4500);                 // ~ Innova front corner load
        rig.SetTireStepsize(tire_step);
        rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
        rig.SetTireVisualizationType(VisualizationType::NONE);
        // Without terrain the rig's default TerrainType is NONE and Advance()
        // dereferences a null terrain. Rigid, since we are timing the TYRE.
        ChTireTestRig::TerrainPatchSize patch;   // has a default ctor, not aggregate init
        patch.length = 10.0; patch.width = 1.0; patch.depth = 0.2;
        rig.SetTerrainRigid(patch, 0.9, 0.01, 2e7);
        rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunctionConst>(3.0));
        rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunctionConst>(9.0));
        rig.Initialize(ChTireTestRig::Mode::TEST);

        for (int i = 0; i < 50; ++i) rig.Advance(1e-3);   // warm up

        std::vector<double> cost;
        cost.reserve(steps);
        for (int i = 0; i < steps; ++i) {
            const auto t0 = stdc::steady_clock::now();
            rig.Advance(1e-3);
            const auto t1 = stdc::steady_clock::now();
            cost.push_back(stdc::duration<double, std::micro>(t1 - t0).count());
        }
        std::sort(cost.begin(), cost.end());
        double mean = 0; for (double v : cost) mean += v; mean /= cost.size();
        results.push_back({label, (int)sys.GetNumCoordsPosLevel(), mean,
                           cost[size_t(0.99*(cost.size()-1))], cost.back()});
        std::cout << "  " << label << " done (" << sys.GetNumCoordsPosLevel()
                  << " position coords)\n";
    };

    std::cout << "\n  running... (ANCF is slow by construction)\n";
    run("TMeasy  (runtime)", "innova/innova_TireF.json", 1e-3);
    // Absolute path into Chrono's shipped data: this exemplar is not part of
    // the Innova model, it is a cost proxy.
    run("ANCF-4  (offline)",
        std::string(CHRONO_DATA_DIR) + "vehicle/Polaris/Polaris_ANCF4Tire_Lumped.json",
        1e-4, true);

    std::cout << "\n=== TMeasy vs ANCF, ONE TYRE, 1 ms of simulated time ===\n"
              << "  model                coords     mean us      p99      max   vs 1 kHz budget\n"
              << "  ------------------  -------   ---------  -------  -------   ---------------\n";
    for (const auto& r : results) {
        std::cout << "  " << std::setw(18) << std::left << r.name << std::right
                  << "  " << std::setw(7) << r.dof
                  << "   " << std::setw(9) << std::fixed << std::setprecision(1) << r.mean
                  << "  " << std::setw(7) << r.p99
                  << "  " << std::setw(7) << r.max
                  << "   " << std::setw(9) << std::setprecision(2) << r.mean/1000.0 << "x\n";
    }
    if (results.size() == 2) {
        const double ratio = results[1].mean / results[0].mean;
        std::cout << "\n  ANCF costs " << std::setprecision(0) << ratio
                  << "x a TMeasy tyre. Four tyres plus chassis at 1 kHz:\n"
                  << "    TMeasy : " << std::setprecision(1) << 4*results[0].mean
                  << " us of the 1000 us budget\n"
                  << "    ANCF   : " << 4*results[1].mean << " us -- "
                  << std::setprecision(1) << 4*results[1].mean/1000.0 << "x OVER budget\n";
    }
    return 0;
}

// ---------------------------------------------------------------------------
static int DoPerf(int steps, const std::string& solver) {
    std::shared_ptr<RigidTerrain> terrain;
    auto veh = MakeVehicle(terrain);
    auto* sys = veh->GetSystem();
    sys->SetNumThreads(1, 0, 1);

    // The vehicle is a set of CLOSED kinematic loops (each wishbone corner is a
    // loop) plus a shafts powertrain. That constraint structure, not the body
    // count, is what costs -- so the solver choice matters far more here than
    // it did for the free-body benchmark in Milestone 1.
    if (solver == "sparseqr") {
        sys->SetSolverType(ChSolver::Type::SPARSE_QR);
    } else if (solver == "sparselu") {
        sys->SetSolverType(ChSolver::Type::SPARSE_LU);
    } else if (solver == "bb") {
        sys->SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
        sys->GetSolver()->AsIterative()->SetMaxIterations(60);
    } else if (solver == "minres") {
        sys->SetSolverType(ChSolver::Type::MINRES);
        sys->GetSolver()->AsIterative()->SetMaxIterations(60);
    }
    std::cout << "  solver: " << (solver.empty() ? "default" : solver) << "\n";

    DriverInputs in{0, 0.3, 0, 0};
    for (int i = 0; i < 500; ++i) {           // warm up
        terrain->Synchronize(0); veh->Synchronize(0, in, *terrain);
        terrain->Advance(1e-3);  veh->Advance(1e-3);
    }

    std::vector<double> cost;
    cost.reserve(steps);
    double t = 0;
    for (int i = 0; i < steps; ++i) {
        const auto t0 = stdc::steady_clock::now();
        terrain->Synchronize(t); veh->Synchronize(t, in, *terrain);
        terrain->Advance(1e-3);  veh->Advance(1e-3);
        const auto t1 = stdc::steady_clock::now();
        cost.push_back(stdc::duration<double, std::micro>(t1 - t0).count());
        t += 1e-3;
    }
    std::sort(cost.begin(), cost.end());
    double mean = 0; for (double v : cost) mean += v; mean /= cost.size();
    std::cout << "\n=== FULL VEHICLE AT 1 kHz (" << steps << " steps, single thread) ===\n"
              << std::fixed << std::setprecision(1)
              << "  bodies " << veh->GetSystem()->GetBodies().size()
              << ", links " << veh->GetSystem()->GetLinks().size() << "\n"
              << "  mean " << mean << " us   p99 " << cost[size_t(0.99 * (cost.size() - 1))]
              << " us   max " << cost.back() << " us\n"
              << "  budget used: " << std::setprecision(2) << 100.0 * mean / 1000.0
              << "% of 1000 us\n"
              << "  realtime factor: " << std::setprecision(1) << 1000.0 / mean << "x\n";
    return 0;
}

// ===========================================================================
// Q1: can an ANCF tyre, run offline, tell the steering wheel what TMeasy
// cannot? One wheel at a time, the same rig for every tyre model.
//
//   ancfland  the tyre lowered onto the road under its load: does this
//             configuration (tyre, step, solver) settle, and what does a
//             step cost? Cheap; run it before the others.
//   ancfpark  the wheel does NOT roll. It is steered about a vertical axis
//             through its centre (zero scrub radius) on a rig built here:
//             ChTireTestRig's slip axis is 0.3 m off the wheel (see ParkRig).
//             What resists it is the bore (parking) torque.
//   ancfroll  ChTireTestRig: the wheel rolls freely at a held speed and slip
//             angle; steady Fy and Mz, what a steady-state table would hold.
//
// Tyres: `tmeasy` (the car's own front tyre), `innova` (gen_innova.py's
// 205/65 R16 ANCF guess), `polaris` and `hmmwv` (Chrono's shipped ANCF-4
// tyres), or a path to any tyre JSON.
// ===========================================================================

static double EnvD(const char* name, double def) {
    const char* s = std::getenv(name);
    return (s && *s) ? std::atof(s) : def;
}

static std::string RigTyreFile(const std::string& name) {
    if (name == "tmeasy")  return GetVehicleDataFile("innova/innova_TireF.json");
    if (name == "innova")  return GetVehicleDataFile("innova/innova_TireANCF.json");   // gen_innova.py
    if (name == "polaris") return std::string(CHRONO_DATA_DIR) + "vehicle/Polaris/Polaris_ANCF4Tire_Lumped.json";
    if (name == "hmmwv")   return std::string(CHRONO_DATA_DIR) + "vehicle/hmmwv/tire/HMMWV_ANCF4Tire_Lumped.json";
    return name;
}

// ---------------------------------------------------------------------------
// A contact friction that can STICK -- an experiment, RIG_STICK=<k_t N/m>.
//
// Chrono 10's SMC contact keeps no tangential history: ChContactSMC.cpp sets
// delta_t = |v_t| * dt for both OneStep and MultiStep ("TODO: implement
// proper MultiStep mode"), so its friction force (kt*dt + gt)*|v_t| is
// VISCOUS, capped at mu*Fn. A node at rest relative to the road carries no
// tangential force at all. That is the opposite of a tread element in the
// adhesion zone of a contact patch, and it would starve exactly what Q1 asks
// about: a parked tyre's twist torque.
//
// This replaces only the TANGENTIAL law, through Chrono's own hook
// (ChSystemSMC::SetContactForceTorqueAlgorithm; no patch): each contact
// carries a bristle, an elastic tangential displacement z integrated from the
// relative velocity, F_t = -(k_t z + c_t v_t), with z capped so that
// k_t|z| <= mu Fn (elastic-perfectly-plastic, a spring-slider). The NORMAL
// force is Chrono's own Hertz law, copied unchanged. Contacts are keyed by
// the pair of contactables: with a node cloud on a flat rigid patch each
// node has at most one contact.
//
// The contact forces are explicit in Chrono's implicit integrator, so k_t is
// stability-limited by the node mass and the step: sqrt(k_t/m) dt < ~1.
// ---------------------------------------------------------------------------
class BristleContactSMC : public ChSystemSMC::ChContactForceTorqueSMC {
  public:
    BristleContactSMC(double kt, double zeta) : m_kt(kt), m_zeta(zeta) {}

    ChWrenchd CalculateForceTorque(const ChSystemSMC& sys, const ChVector3d& normal_dir,
                                   const ChVector3d& p1, const ChVector3d& p2,
                                   const ChVector3d& vel1, const ChVector3d& vel2,
                                   const ChContactMaterialCompositeSMC& mat, double delta,
                                   double eff_radius, double mass1, double mass2,
                                   ChContactable* objA, ChContactable* objB) const override {
        if (delta <= 0) return {VNULL, VNULL};
        const double dT = sys.GetStep();
        const ChVector3d relvel = vel2 - vel1;
        const double vn = relvel.Dot(normal_dir);
        const ChVector3d vt = relvel - vn * normal_dir;
        const double eff_mass = mass1 * mass2 / (mass1 + mass2);

        // Normal: Chrono's Hertz law with material properties, as in
        // ChDefaultContactForceTorqueSMC (the system default).
        constexpr double eps = std::numeric_limits<double>::epsilon();
        const double sqrt_Rd = std::sqrt(eff_radius * delta);
        const double Sn = 2 * mat.E_eff * sqrt_Rd;
        double loge = (mat.cr_eff < eps) ? std::log(eps) : std::log(mat.cr_eff);
        loge = (mat.cr_eff > 1 - eps) ? std::log(1 - eps) : loge;
        const double beta = loge / std::sqrt(loge * loge + CH_PI * CH_PI);
        const double kn = CH_2_3 * Sn;
        const double gn = -2 * std::sqrt(5.0 / 6) * beta * std::sqrt(Sn * eff_mass);
        double fn = kn * delta - gn * vn;
        Bristle& b = m_hist[{objA, objB}];
        if (fn <= 0) { b.z = VNULL; b.t = -1; return {VNULL, VNULL}; }

        // Integrate the bristle once per step; a contact that lapsed for more
        // than a step starts again unstrained.
        const double t = sys.GetChTime();
        if (b.t < 0 || t - b.t > 1.5 * dT) b.z = VNULL;
        if (t != b.t) { b.z += vt * dT; b.t = t; }
        b.z -= b.z.Dot(normal_dir) * normal_dir;          // stay in the tangent plane
        const double cap = mat.mu_eff * fn;
        const double zl = b.z.Length();
        if (m_kt * zl > cap && zl > 0) b.z *= cap / (m_kt * zl);   // slides
        const double ct = 2 * m_zeta * std::sqrt(m_kt * eff_mass);
        ChVector3d ft = -(m_kt * b.z + ct * vt);
        const double fl = ft.Length();
        if (fl > cap && fl > 0) ft *= cap / fl;
        return {fn * normal_dir + ft, VNULL};
    }

  private:
    struct Bristle { ChVector3d z{VNULL}; double t = -1; };
    double m_kt, m_zeta;
    mutable std::map<std::pair<const void*, const void*>, Bristle> m_hist;
};

// The system every Q1 rig runs in. An FEA tyre is stiff: implicit Euler and
// a linear solve each step. MINRES/100 is what M4 timed; QR and LU are the
// exact solves, to check it.
static void ConfigureRigSystem(ChSystemSMC& sys, const std::string& solver) {
    // A Chrono 10 ChSystem starts with NO collision system (ChSystem.cpp:
    // collision_system(nullptr)); nothing then generates a contact, and an
    // FEA tyre floats. A force-element tyre never notices: it queries the
    // terrain itself. M4's `tire` mode timed ANCF like that.
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    sys.SetNumThreads((int)EnvD("RIG_THREADS", 1), 0, 1);
    const double kt = EnvD("RIG_STICK", 0.0);
    if (kt > 0) {
        sys.SetContactForceTorqueAlgorithm(
            std::make_unique<BristleContactSMC>(kt, EnvD("RIG_STICK_ZETA", 0.5)));
        std::cout << "  contact: bristle friction (sticks), k_t " << kt << " N/m per contact\n";
    } else {
        std::cout << "  contact: Chrono default SMC (Hertz; friction viscous in v_t)\n";
    }
    // RIG_STIFF=1: contact forces enter the implicit solve with their
    // Jacobians (ChSystemSMC::SetContactStiff), instead of explicitly.
    const bool stiff = EnvD("RIG_STIFF", 0) > 0;
    sys.SetContactStiff(stiff);
    if (solver == "qr" || solver == "lu") {
        sys.SetSolverType(solver == "qr" ? ChSolver::Type::SPARSE_QR : ChSolver::Type::SPARSE_LU);
        // With explicit contact the matrix pattern -- mesh plus joints -- never
        // changes, so analyse it once. Stiff contact changes it every step.
        if (auto* ds = dynamic_cast<ChDirectSolverLS*>(sys.GetSolver().get()))
            ds->LockSparsityPattern(!stiff && EnvD("RIG_LOCK", 1) > 0);
    } else {
        sys.SetSolverType(ChSolver::Type::MINRES);
        auto* it = sys.GetSolver()->AsIterative();
        it->SetMaxIterations((int)EnvD("RIG_ITERS", 100));
        it->SetTolerance(EnvD("RIG_TOL", 1e-8));
        it->EnableDiagonalPreconditioner(true);
    }
    // RIG_INTEGRATOR=hht: Chrono's own FEA-tyre test (demo_VEH_Test_TireFEA:
    // HHT, alpha -0.2, at most 5 Newton iterations to 1e-2, the Jacobian
    // every step, 1e-4 s). Default: linearised implicit Euler, one solve.
    const char* integ = std::getenv("RIG_INTEGRATOR");
    if (integ && std::string(integ) == "hht") {
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto hht = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        hht->SetAlpha(-0.2);
        hht->SetMaxIters((int)EnvD("RIG_NEWTON", 5));
        hht->SetAbsTolerances(1e-2);
        hht->SetJacobianUpdateMethod(ChTimestepperImplicit::JacobianUpdate::EVERY_STEP);
        hht->SetStepControl(false);
        std::cout << "  integrator: HHT (Newton)\n";
    } else {
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);
        std::cout << "  integrator: implicit Euler, linearised\n";
    }
}

static std::shared_ptr<ChTire> ReadRigTyre(const std::string& tyre, double dt, bool& fea) {
    auto tire = ReadTireJSON(RigTyreFile(tyre));
    fea = std::dynamic_pointer_cast<ChDeformableTire>(tire) != nullptr;
    if (fea) {   // RIG_SURFACE=mesh: triangles, as Chrono's FEA-tyre test; else a node cloud
        const char* surf = std::getenv("RIG_SURFACE");
        const bool mesh = surf && std::string(surf) == "mesh";
        tire->SetContactSurfaceType(mesh ? ChTire::ContactSurfaceType::TRIANGLE_MESH
                                         : ChTire::ContactSurfaceType::NODE_CLOUD,
                                    EnvD("RIG_NODE_R", 0.01), 7);   // radius / thickness, m
    }
    tire->SetStepsize(dt);
    return tire;
}

static void PrintRigTyre(const std::string& tyre, const std::shared_ptr<ChTire>& tire, bool fea,
                         double load, double dt, const std::string& solver) {
    std::printf("  tyre %s (%s), radius %.4f m, mass %.2f kg, load %.0f N, dt %g s, solver %s\n",
                tyre.c_str(), fea ? "FEA" : "force element", tire->GetRadius(),
                tire->GetTireMass(), load, dt, solver.c_str());
    if (fea) std::printf("  contact node radius %g m\n", EnvD("RIG_NODE_R", 0.01));
}

// RIG_CONTACT_E: the FEA tyre's contact Young's modulus, overriding its JSON
// (2e6 Pa in Chrono's ANCF tyres). With Hertz contact on 1 cm node spheres,
// 2 MPa lets each node sink ~11 mm under a 4.68 kN corner load spread over
// ~15 contacts -- deeper than the sphere -- and the contact geometry fails.
static void OverrideTyreContact(const std::shared_ptr<ChTire>& tire) {
    const double e = EnvD("RIG_CONTACT_E", 0.0);
    auto def = std::dynamic_pointer_cast<ChDeformableTire>(tire);
    if (e > 0 && def && def->GetContactMaterial()) {
        def->GetContactMaterial()->SetYoungModulus((float)e);
        std::printf("  tyre contact Young's modulus overridden: %g Pa\n", e);
    }
}

// ---------------------------------------------------------------------------
// The parking rig. ChTireTestRig cannot do this test. Its slip-angle joint
// (ChTireTestRig.cpp, m_slip_lock, initialised at the origin) turns the wheel
// about a vertical axis through the RIG's origin, and the spindle sits 0.3 m
// beside it (3 * dim, dim = 0.1). Rolling, that is a small offset. Parked, it
// tows the patch 5.2 cm along the road for every 10 deg of "steer": TMeasy on
// it read Fx from -3.1 to +3.9 kN, a locked wheel being dragged. So this is
// the rig a parking-torque test needs: the wheel locked to a steering motor
// whose axis is vertical through the wheel centre (zero scrub radius, caster
// and inclination), on a carrier that slides vertically and is the load.
// ---------------------------------------------------------------------------
struct ParkRig {
    std::unique_ptr<ChSystemSMC> sys;
    std::shared_ptr<RigidTerrain> terrain;
    std::shared_ptr<ChWheel> wheel;
    std::shared_ptr<ChTire> tire;
    std::shared_ptr<ChSpindle> spindle;
    std::shared_ptr<ChBody> carrier;
    std::shared_ptr<ChLinkMotorRotationAngle> steer;
    bool fea = false;
    double r0 = 0;   // unloaded radius (TMeasy's GetRadius() is its rolling radius once loaded)

    // ChTireTestRig::Advance's sequence.
    void Advance(double dt) {
        const double t = sys->GetChTime();
        terrain->Synchronize(t);
        tire->Synchronize(t, *terrain);
        spindle->EmptyTireAccumulator();
        wheel->Synchronize();
        terrain->Advance(dt);
        tire->Advance(dt);
        sys->DoStepDynamics(dt);
    }
    // Wheel centre above the road.
    double Height() const { return spindle->GetPos().z() - terrain->GetHeight(spindle->GetPos()); }
};

static void MakeParkRig(ParkRig& r, const std::string& tyre, double load, double dt,
                        const std::string& solver, std::shared_ptr<ChFunction> steer_angle) {
    r.sys = std::make_unique<ChSystemSMC>();
    r.sys->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    ConfigureRigSystem(*r.sys, solver);
    r.tire = ReadRigTyre(tyre, dt, r.fea);
    r.wheel = ReadWheelJSON(GetVehicleDataFile("innova/innova_Wheel.json"));
    // Start just clear of the road. An FEA tyre's contact is a cloud of
    // spheres of radius RIG_NODE_R round its nodes, so clear means that much
    // higher: starting 8 mm into the road was a kilonewton shock at t = 0.
    const double clear = (r.fea ? EnvD("RIG_NODE_R", 0.01) : 0.0) + 0.001;
    const ChVector3d c(0, 0, r.tire->GetRadius() + clear);

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    r.sys->AddBody(ground);

    r.carrier = chrono_types::make_shared<ChBody>();
    r.carrier->SetPos(c);
    r.carrier->SetInertiaXX(ChVector3d(1, 1, 1));
    r.sys->AddBody(r.carrier);
    auto slide = chrono_types::make_shared<ChLinkLockPrismatic>();   // along Z
    slide->Initialize(r.carrier, ground, ChFrame<>(c, QUNIT));
    r.sys->AddLink(slide);
    // Damps the settle on the slide; it cannot act on the steering.
    auto damper = chrono_types::make_shared<ChLinkTSDA>();
    damper->Initialize(r.carrier, ground, false, c, c - ChVector3d(0, 0, 1));
    damper->SetSpringCoefficient(0);
    damper->SetDampingCoefficient(EnvD("RIG_SETTLE_C", 2e4));
    r.sys->AddLink(damper);

    r.spindle = chrono_types::make_shared<ChSpindle>();
    r.spindle->SetPos(c);
    r.spindle->SetMass(0);
    r.spindle->SetInertiaXX(ChVector3d(0.01, 0.02, 0.01));
    r.sys->AddBody(r.spindle);
    // Motor axis = Z of the frame: vertical, through the wheel centre. A
    // rotation motor also locks the other five relative motions, so the
    // wheel cannot spin: a parked wheel, steered.
    r.steer = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    r.steer->Initialize(r.spindle, r.carrier, ChFrame<>(c, QUNIT));
    r.steer->SetAngleFunction(steer_angle);
    r.sys->AddLink(r.steer);

    r.wheel->Initialize(nullptr, r.spindle, LEFT);
    r.wheel->SetVisualizationType(VisualizationType::NONE);
    r.wheel->SetTire(r.tire);
    r.tire->Initialize(r.wheel);
    r.tire->SetVisualizationType(VisualizationType::NONE);
    OverrideTyreContact(r.tire);
    r.r0 = r.tire->GetRadius();   // before any contact: the unloaded radius
    // A force-element tyre's GetRadius() is only meaningful after Initialize
    // (TMeasy's is a state), so place it again now; it has no mesh to move.
    if (!r.fea) {
        const ChVector3d dz(0, 0, r.r0 + clear - c.z());
        r.spindle->SetPos(r.spindle->GetPos() + dz);
        r.carrier->SetPos(r.carrier->GetPos() + dz);
    }
    // The carrier makes up the load: everything above the road weighs `load`.
    // A force-element tyre's mass is on the spindle; an FEA tyre's in its mesh.
    const double above = r.spindle->GetMass() + (r.fea ? r.tire->GetTireMass() : 0.0);
    r.carrier->SetMass(std::max(1.0, load / 9.81 - above));

    r.terrain = chrono_types::make_shared<RigidTerrain>(r.sys.get());
    ChContactMaterialData minfo;   // as ChTireTestRig::SetTerrainRigid(.., 0.9, 0.01, 2e7)
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    r.terrain->AddPatch(minfo.CreateMaterial(ChContactMethod::SMC), ChCoordsys<>(VNULL, QUNIT), 4,
                        4, 0.1);
    r.terrain->Initialize();
    PrintRigTyre(tyre, r.tire, r.fea, load, dt, solver);
}

// Settle on the parking rig, steer held at 0, and report it. Returns false if
// the tyre never comes to rest under its load (it diverged, or bounces on).
static bool ParkSettle(ParkRig& r, double dt, double t_settle, double load) {
    const int every = std::max(1, (int)std::lround(EnvD("RIG_PRINT_DT", 0.1) / dt));
    std::vector<double> cost;
    for (long i = 0; r.sys->GetChTime() < t_settle; ++i) {
        const auto c0 = stdc::steady_clock::now();
        r.Advance(dt);
        cost.push_back(stdc::duration<double, std::micro>(stdc::steady_clock::now() - c0).count());
        const double h = r.Height();
        if (i % every == 0 || !std::isfinite(h)) {
            const TerrainForce f = r.tire->ReportTireForce(r.terrain.get());
            std::printf("  SETTLE t %5.2f  centre height %8.4f m  Fz %8.1f N  contacts %4u",
                        r.sys->GetChTime(), h, f.force.z(), r.sys->GetNumContacts());
            // An iterative solve that stops at its cap is not a solution.
            if (auto* it = r.sys->GetSolver()->AsIterative())
                std::printf("  solver its %d residual %.2e", it->GetIterations(), it->GetError());
            std::printf("  %.0f us/step\n", cost.back());
        }
        if (!std::isfinite(h)) { std::printf("  DIVERGED (non-finite state)\n"); return false; }
    }
    double mean = 0; for (double c : cost) mean += c; mean /= cost.size();
    // An FEA tyre touches the road through spheres (or swept triangles) of
    // radius RIG_NODE_R round its mesh: its free contact radius is r0 + that.
    const double r_free = r.r0 + (r.fea ? EnvD("RIG_NODE_R", 0.01) : 0.0);
    const double h = r.Height(), defl = r_free - h;
    const double vz = r.spindle->GetPosDt().z();
    std::printf("  settled at t %.2f: loaded radius %.4f m, deflection %.1f mm -> secant vertical "
                "stiffness %.0f kN/m; %u contacts; vertical speed %.4f m/s\n", t_settle, h,
                1e3 * defl, defl > 0 ? 1e-3 * load / defl : 0.0,
                r.sys->GetNumContacts(), vz);
    if (r.fea)
        std::printf("  FEA coords %u; step cost mean %.0f us -> %.1f s of CPU per simulated "
                    "second\n", r.sys->GetNumCoordsPosLevel(), mean, mean * 1e-6 / dt);
    if (std::abs(vz) >= 0.01) std::printf("  NOTE: still sinking at %.4f m/s\n", vz);
    return std::abs(vz) < 0.01;
}

// ancfland: a tyre lowered onto the road under its load and left there. The
// cheapest check that a configuration (tyre, step, solver) is sound before an
// hour is spent steering it, and the cost per step of that configuration.
static int DoAncfLand(int argc, char** argv) {
    const std::string tyre = argc > 2 ? argv[2] : "polaris";
    const double dt = argc > 3 ? std::atof(argv[3]) : 1e-3;
    const std::string solver = argc > 4 ? argv[4] : "minres";
    const double t_settle = argc > 5 ? std::atof(argv[5]) : 1.0;
    ParkRig r;
    MakeParkRig(r, tyre, EnvD("RIG_LOAD", 4680.0), dt, solver,
                chrono_types::make_shared<ChFunctionConst>(0.0));
    return ParkSettle(r, dt, t_settle, EnvD("RIG_LOAD", 4680.0)) ? 0 : 1;
}

static int DoAncfPark(int argc, char** argv) {
    const std::string tyre = argc > 2 ? argv[2] : "polaris";
    const double rate = (argc > 3 ? std::atof(argv[3]) : 10.0) * CH_DEG_TO_RAD;  // steer rate
    const double amp  = (argc > 4 ? std::atof(argv[4]) : 10.0) * CH_DEG_TO_RAD;  // amplitude
    const double dt   =  argc > 5 ? std::atof(argv[5]) : 1e-3;
    const std::string solver = argc > 6 ? argv[6] : "minres";
    const double load = EnvD("RIG_LOAD", 4680.0);   // Innova front corner, static (task 14)
    const double t_settle = EnvD("RIG_SETTLE_T", 1.0), hold = EnvD("RIG_HOLD", 1.0);

    // Steer profile after the settle: up, hold, through zero to the other
    // side, hold, back. A real tyre winds up elastically, saturates when the
    // patch slides, KEEPS its torque through a hold, and shows hysteresis on
    // reversal. A purely viscous contact gives a torque proportional to the
    // steer RATE, gone in the holds. The holds tell the two apart.
    const double tr = amp / rate;
    const double t1 = tr, t2 = t1 + hold, t3 = t2 + 2 * tr, t4 = t3 + hold, t5 = t4 + tr,
                 t_end = t5 + EnvD("RIG_TAIL", 0.5);
    auto sa = chrono_types::make_shared<ChFunctionInterp>();
    sa->AddPoint(0, 0);
    sa->AddPoint(t_settle, 0);
    sa->AddPoint(t_settle + t1, amp);
    sa->AddPoint(t_settle + t2, amp);
    sa->AddPoint(t_settle + t3, -amp);
    sa->AddPoint(t_settle + t4, -amp);
    sa->AddPoint(t_settle + t5, 0);

    ParkRig r;
    MakeParkRig(r, tyre, load, dt, solver, sa);
    // A finite state is required; a slow vertical creep is reported, not
    // fatal: the twist test does not need the last millimetre of sink, and
    // the trace carries Fz throughout.
    if (!ParkSettle(r, dt, t_settle, load) && !std::isfinite(r.Height())) {
        std::printf("  diverged while settling\n");
        return 1;
    }

    // Two measures of the same torque. M_steer is the reaction in the steering
    // motor: what a rack must supply to turn the wheel, about the true axis.
    // Mz is the tyre's own report about the vertical through the wheel centre
    // (a deformable tyre sums its rim-connection reactions, so it includes the
    // tyre's inertia; a force-element tyre reports its patch moment).
    std::printf("\n  steer rate %.1f deg/s, amplitude +/-%.1f deg, holds %.1f s\n",
                rate * CH_RAD_TO_DEG, amp * CH_RAD_TO_DEG, hold);
    std::printf("  TRACE     t  steer_deg  M_steer_Nm    Mz_Nm     Fx_N     Fy_N     Fz_N  "
                "contacts\n");
    // Read 10 ms INSIDE each phase: on a corner itself the inertia kick below
    // is what the motor reports, not the tyre.
    struct Key { double t; const char* what; };
    const Key keys[] = {{t1 - 0.01, "ramp to +A, 10 ms to go"},
                        {t2 - 0.01, "hold at +A, 10 ms to go"},
                        {t2 + tr, "ramp back, crossing 0"},
                        {t3 - 0.01, "ramp to -A, 10 ms to go"},
                        {t4 - 0.01, "hold at -A, 10 ms to go"},
                        {t_end - 0.005, "back at 0, end of tail"}};
    double key_m[6] = {0}, key_mz[6] = {0}, key_sa[6] = {0};
    size_t next_key = 0;
    double peak_up = 0, peak_dn = 0;
    const int every = std::max(1, (int)std::lround(0.01 / dt));
    std::vector<double> cost;
    for (long i = 0;; ++i) {
        const double t = r.sys->GetChTime() - t_settle;
        if (t > t_end) break;
        const auto c0 = stdc::steady_clock::now();
        r.Advance(dt);
        cost.push_back(stdc::duration<double, std::micro>(stdc::steady_clock::now() - c0).count());
        const TerrainForce f = r.tire->ReportTireForce(r.terrain.get());
        const double steer = r.steer->GetMotorAngle() * CH_RAD_TO_DEG;
        const double m = r.steer->GetMotorTorque(), mz = f.moment.z();
        if (!std::isfinite(m)) { std::printf("  DIVERGED at t %.3f\n", t); return 1; }
        const double tt = r.sys->GetChTime() - t_settle;
        // The profile's corners step the steer RATE, so the wheel's own
        // inertia kicks the motor for a step (I * d(omega)/dt: ~250 N.m for
        // TMeasy's wheel at 20 deg/s and 1 ms). Peaks skip 5 ms round them.
        bool corner = false;
        for (double tc : {0.0, t1, t2, t3, t4, t5}) corner |= std::abs(tt - tc) < 0.005;
        if (!corner && tt <= t2) peak_up = std::max(peak_up, std::abs(m));
        else if (!corner && tt <= t4) peak_dn = std::max(peak_dn, std::abs(m));
        while (next_key < 6 && tt >= keys[next_key].t) {
            key_m[next_key] = m; key_mz[next_key] = mz; key_sa[next_key] = steer; ++next_key;
        }
        if (i % every == 0)
            std::printf("  TRACE %6.3f %9.3f %11.3f %9.3f %8.1f %8.1f %8.1f %6u\n", tt, steer, m,
                        mz, f.force.x(), f.force.y(), f.force.z(), r.sys->GetNumContacts());
    }
    double mean = 0; for (double c : cost) mean += c; mean /= cost.size();

    std::printf("\n=== PARKING TORQUE, %s, %.0f N, steer %.1f deg/s, dt %g, %s ===\n",
                tyre.c_str(), load, rate * CH_RAD_TO_DEG, dt,
                EnvD("RIG_STICK", 0) > 0 ? "bristle contact" : "Chrono contact");
    std::printf("  %-26s %9s %11s %9s\n", "", "steer deg", "M_steer N.m", "Mz N.m");
    for (int k = 0; k < 6; ++k)
        std::printf("  %-26s %9.2f %11.3f %9.3f\n", keys[k].what, key_sa[k], key_m[k], key_mz[k]);
    std::printf("  peak |M_steer| up+hold %.3f N.m, reverse+hold %.3f N.m\n", peak_up, peak_dn);
    if (std::abs(key_m[0]) > 0.01)
        std::printf("  held/ramp ratio at +A: %.3f (an elastic patch keeps ~1, a viscous one ~0)\n",
                    key_m[1] / key_m[0]);
    else
        std::printf("  held/ramp ratio at +A: n/a, no torque at the end of the ramp\n");
    std::printf("  step cost mean %.0f us -> %.1f s of CPU per simulated second\n", mean,
                mean * 1e-6 / dt);
    return 0;
}

// ---------------------------------------------------------------------------
// ancfroll: ChTireTestRig, which is right for a ROLLING tyre (its 0.3 m axis
// offset only shifts the wheel sideways by 0.3 (1 - cos alpha)).
// The rig keeps a raw ChSystem* and removes its bodies in its destructor, so
// the rig must die first: members are destroyed in reverse order.
// ---------------------------------------------------------------------------
struct Rig {
    std::unique_ptr<ChSystemSMC> sys;
    std::unique_ptr<ChTireTestRig> rig;
    std::shared_ptr<ChTire> tire;
    bool fea = false;
};

static void MakeRig(Rig& r, const std::string& tyre, double load, double dt,
                    const std::string& solver, double drop_speed, double delay,
                    std::shared_ptr<ChFunction> long_speed,   // null: carrier free
                    std::shared_ptr<ChFunction> ang_speed,    // null: wheel free
                    std::shared_ptr<ChFunction> slip_angle) {
    r.sys = std::make_unique<ChSystemSMC>();
    ConfigureRigSystem(*r.sys, solver);
    auto wheel = ReadWheelJSON(GetVehicleDataFile("innova/innova_Wheel.json"));
    r.tire = ReadRigTyre(tyre, dt, r.fea);

    r.rig = std::make_unique<ChTireTestRig>(wheel, r.tire, r.sys.get());
    r.rig->SetGravitationalAcceleration(9.81);
    r.rig->SetNormalLoad(load);
    r.rig->SetTireStepsize(dt);
    r.rig->SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    r.rig->SetTireVisualizationType(VisualizationType::NONE);
    ChTireTestRig::TerrainPatchSize patch;
    patch.length = 40.0; patch.width = 1.0; patch.depth = 0.2;
    r.rig->SetTerrainRigid(patch, 0.9, 0.01, 2e7);
    if (long_speed) r.rig->SetLongSpeedFunction(long_speed);
    if (ang_speed)  r.rig->SetAngSpeedFunction(ang_speed);
    r.rig->SetSlipAngleFunction(slip_angle);
    r.rig->SetTimeDelay(delay);
    r.rig->Initialize(ChTireTestRig::Mode::TEST, drop_speed);
    OverrideTyreContact(r.tire);
    PrintRigTyre(tyre, r.tire, r.fea, load, dt, solver);
}

// Advance the rig through the drop until its outputs are live (the motion
// functions start then). Returns false if it never lands.
static bool RigLand(Rig& r, double dt) {
    const int every = std::max(1, (int)std::lround(0.1 / dt));
    for (long i = 0; i < (long)(EnvD("RIG_LAND_T", 5.0) / dt); ++i) {
        r.rig->Advance(dt);
        const ChVector3d sp = r.rig->GetSpindle()->GetPos();
        if (i % every == 0 || !std::isfinite(sp.z()))
            std::printf("  LAND t %5.2f  spindle z %8.4f\n", r.sys->GetChTime(), sp.z());
        if (!std::isfinite(sp.z())) { std::printf("  DIVERGED (non-finite state)\n"); return false; }
        if (r.rig->OutputEnabled()) return true;
    }
    return false;
}

static int DoAncfRoll(int argc, char** argv) {
    const std::string tyre = argc > 2 ? argv[2] : "polaris";
    const double v    = argc > 3 ? std::atof(argv[3]) : 3.0;
    const double slip = (argc > 4 ? std::atof(argv[4]) : 3.0) * CH_DEG_TO_RAD;
    const double dt   = argc > 5 ? std::atof(argv[5]) : 1e-3;
    const std::string solver = argc > 6 ? argv[6] : "minres";
    const double load = EnvD("RIG_LOAD", 4680.0);
    const double t_v = 0.5, t_s0 = 1.0, t_s1 = 1.5, t_end = EnvD("RIG_ROLL_T", 3.0);

    auto speed = chrono_types::make_shared<ChFunctionInterp>();
    speed->AddPoint(0, 0); speed->AddPoint(t_v, v);
    auto sa = chrono_types::make_shared<ChFunctionInterp>();
    sa->AddPoint(0, 0); sa->AddPoint(t_s0, 0); sa->AddPoint(t_s1, slip);

    Rig r;
    MakeRig(r, tyre, load, dt, solver, 0.2, 0.5, speed, nullptr /* rolls freely */, sa);
    if (!RigLand(r, dt)) { std::cout << "  never landed\n"; return 1; }
    const double t0 = r.sys->GetChTime();

    std::printf("\n  speed %.2f m/s, slip angle %.2f deg from t=%.1f s\n", v,
                slip * CH_RAD_TO_DEG, t_s1);
    std::printf("  TRACE     t   slip_deg     Fx_N     Fy_N     Fz_N     Mz_Nm   kappa\n");
    const int every = std::max(1, (int)std::lround(0.05 / dt));
    double n = 0, fy = 0, mz = 0, fx = 0, fz = 0, sl = 0, ks = 0;
    std::vector<double> cost;
    for (long i = 0;; ++i) {
        const double t = r.sys->GetChTime() - t0;
        if (t > t_end) break;
        const auto c0 = stdc::steady_clock::now();
        r.rig->Advance(dt);
        cost.push_back(stdc::duration<double, std::micro>(stdc::steady_clock::now() - c0).count());
        const TerrainForce f = r.rig->ReportTireForce();
        const double tt = r.sys->GetChTime() - t0;
        const double kap = r.rig->GetLongitudinalSlip();
        if (!std::isfinite(f.force.y())) { std::printf("  DIVERGED at t %.3f\n", tt); return 1; }
        if (i % every == 0)
            std::printf("  TRACE %6.3f %9.3f %8.1f %8.1f %8.1f %9.3f %7.4f\n", tt,
                        r.rig->GetSlipAngle() * CH_RAD_TO_DEG, f.force.x(), f.force.y(),
                        f.force.z(), f.moment.z(), kap);
        if (tt > t_end - 0.5) {   // the last half second: the steady state
            n += 1; fy += f.force.y(); mz += f.moment.z(); fx += f.force.x(); fz += f.force.z();
            sl += r.rig->GetSlipAngle(); ks += kap;
        }
    }
    double mean = 0; for (double c : cost) mean += c; mean /= cost.size();
    std::printf("\n=== ROLLING, %s, %.0f N, %.2f m/s, slip %.2f deg, dt %g, %s ===\n",
                tyre.c_str(), load, v, slip * CH_RAD_TO_DEG, dt,
                EnvD("RIG_STICK", 0) > 0 ? "bristle contact" : "Chrono contact");
    std::printf("  mean over the last 0.5 s: Fx %.1f N, Fy %.1f N, Fz %.1f N, Mz %.3f N.m, "
                "slip %.3f deg, kappa %.4f\n", fx / n, fy / n, fz / n, mz / n,
                sl / n * CH_RAD_TO_DEG, ks / n);
    std::printf("  cornering stiffness estimate |Fy|/alpha: %.0f N/rad\n",
                std::abs(fy / n) / std::max(1e-9, std::abs(sl / n)));
    std::printf("  step cost mean %.0f us -> %.1f s of CPU per simulated second\n", mean,
                mean * 1e-6 / dt);
    return 0;
}

int main(int argc, char** argv) {
    SetChronoDataPath(CHRONO_DATA_DIR);
    SetVehicleDataPath(std::string(VEHICLE_DATA_DIR));

    // Line-buffered even into a log file: the ANCF modes run for many minutes,
    // and a trace that only appears at exit cannot be watched.
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const std::string mode = (argc > 1) ? argv[1] : "assemble";
    std::cout << "Chrono " << CHRONO_VERSION << "  |  mode: " << mode << "\n";
    try {
        if (mode == "assemble")  return DoAssemble(argc > 2 ? argv[2] : "");
        if (mode == "ackermann") return DoAckermann();
        if (mode == "probe")     return DoProbe();
        if (mode == "tire")      return DoTire(argc > 2 ? std::atoi(argv[2]) : 300);
        if (mode == "perf")      return DoPerf(argc > 2 ? std::atoi(argv[2]) : 3000,
                                              argc > 3 ? argv[3] : "");
        if (mode == "ancfpark")  return DoAncfPark(argc, argv);
        if (mode == "ancfroll")  return DoAncfRoll(argc, argv);
        if (mode == "ancfland")  return DoAncfLand(argc, argv);
        std::cerr << "unknown mode\n";
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "EXCEPTION: " << e.what() << "\n";
        return 1;
    }
}
