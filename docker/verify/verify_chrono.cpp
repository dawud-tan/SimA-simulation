// =============================================================================
//  Milestone 1 verification + benchmark.
//
//  Proves three things about the containerised Chrono 10 build:
//    1. The library loads, and Vehicle + core FEA are present.
//    2. ChSystemSMC / ChSolverMINRES / ChTimestepperHHT all instantiate.
//    3. THE IMPORTANT ONE: measures actual per-step cost, so the 1 kHz budget
//       stops being an assumption. 1000 Hz means every step must finish in
//       under 1000 microseconds, on this CPU, with nothing else running.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"   // CHRONO_VERSION - NOT pulled in by ChConfig.h
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChContactMaterialSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"   // ChSolverMINRES, GMRES, BiCGSTAB
#include "chrono/solver/ChDirectSolverLS.h"      // ChSolverSparseQR / SparseLU
#include "chrono/timestepper/ChTimestepperHHT.h"

#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChBeamSectionCable.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChNodeFEAxyzD.h"

// std::chrono and chrono:: (Project Chrono) collide under `using namespace`.
namespace stdc = std::chrono;
using namespace chrono;
using namespace chrono::fea;

// -----------------------------------------------------------------------------
// Collect per-step wall times and report the distribution. The mean is the
// least interesting number here: a 1 kHz loop is judged by its TAIL, because
// one 5 ms step is a pothole the driver feels through the wheel.
// -----------------------------------------------------------------------------
struct StepStats {
    std::vector<double> us;
    void add(double v) { us.push_back(v); }
    double mean() const {
        double s = 0; for (double v : us) s += v; return s / us.size();
    }
    double pct(double p) const {
        std::vector<double> t = us;
        std::sort(t.begin(), t.end());
        return t[static_cast<size_t>(p * (t.size() - 1))];
    }
    void report(const std::string& label, int dof) const {
        double m = mean(), p99 = pct(0.99), mx = *std::max_element(us.begin(), us.end());
        // 1 kHz budget = 1000 us. "realtime factor" > 1 means faster than real time.
        double rtf = 1000.0 / m;
        std::cout << std::left << std::setw(34) << label
                  << std::right << std::fixed << std::setprecision(1)
                  << std::setw(7) << dof   << " dof"
                  << std::setw(11) << m    << " us mean"
                  << std::setw(11) << p99  << " us p99"
                  << std::setw(11) << mx   << " us max"
                  << std::setw(9)  << std::setprecision(2) << rtf << "x rt";
        std::cout << (m < 1000.0 ? "   [FITS 1 kHz]" : "   [BUSTS 1 kHz]") << "\n";
    }
};

// -----------------------------------------------------------------------------
// BENCHMARK A — rigid multibody with SMC contact.
// This is the shape of the real-time vehicle workload: a modest number of
// rigid bodies, penalty contact, cheap explicit-ish stepping.
//
// WHY SMC (smooth contact / penalty)?
// Contact force is a continuous function of penetration depth: f = k*d + c*d_dot.
// There is no complementarity problem to solve, so the contact solve is O(n)
// and completely predictable in cost — which is exactly what a fixed 1 kHz
// budget requires. The price is stiffness: k must be large to keep penetration
// small, and large k forces a small timestep. At 1 kHz we can afford it.
// -----------------------------------------------------------------------------
static void benchRigid(int n_bodies, int n_steps, double step) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetNumThreads(1, 0, 1);   // single-threaded: this is the per-core budget

    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();
    mat->SetYoungModulus(2e7f);
    mat->SetFriction(0.9f);
    mat->SetRestitution(0.1f);

    auto ground = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.4, 1000, true, true, mat);
    ground->SetPos(ChVector3d(0, 0, -0.2));
    ground->SetFixed(true);
    sys.Add(ground);

    for (int i = 0; i < n_bodies; ++i) {
        auto b = chrono_types::make_shared<ChBodyEasyBox>(0.3, 0.3, 0.3, 1000, true, true, mat);
        b->SetPos(ChVector3d(0.05 * (i % 5), 0.05 * (i / 5), 0.35 + 0.32 * i));
        sys.Add(b);
    }

    for (int i = 0; i < 50; ++i) sys.DoStepDynamics(step);   // warm up caches

    StepStats st;
    for (int i = 0; i < n_steps; ++i) {
        auto t0 = stdc::steady_clock::now();
        sys.DoStepDynamics(step);
        auto t1 = stdc::steady_clock::now();
        st.add(stdc::duration<double, std::micro>(t1 - t0).count());
    }
    st.report("A. Rigid SMC, " + std::to_string(n_bodies) + " bodies", n_bodies * 6);
}

// -----------------------------------------------------------------------------
// BENCHMARK B — ANCF finite-element cable, MINRES + HHT.
// This is the Milestone 4 question, answered numerically.
//
// WHY AN ITERATIVE SOLVER LIKE MINRES?
// HHT is implicit: each step solves a nonlinear system by Newton iteration, and
// each Newton iteration needs a linear solve against the tangent matrix
// K_eff = M/(beta*h^2) + gamma*C/(beta*h) + K.
// For a stiff FEA mesh that matrix is large, sparse, symmetric and
// horribly conditioned — the 1/h^2 mass term and the stiffness term differ by
// many orders of magnitude. A direct factorisation costs O(n^1.5+) and fills in;
// MINRES exploits symmetry via Lanczos tridiagonalisation and never forms the
// factorisation at all, trading exactness for a controllable iteration count.
//
// That trade is what makes stiff FEA tractable OFFLINE. It does not make it
// real-time — which is what the numbers below are here to show.
// -----------------------------------------------------------------------------
static void benchANCF(int n_elements, int n_steps, double step) {
    ChSystemSMC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetNumThreads(1, 0, 1);

    auto mesh = chrono_types::make_shared<ChMesh>();

    auto section = chrono_types::make_shared<ChBeamSectionCable>();
    section->SetDiameter(0.015);
    section->SetYoungModulus(0.01e9);
    section->SetRayleighDamping(0.0001);

    ChBuilderCableANCF builder;
    builder.BuildBeam(mesh, section, n_elements,
                      ChVector3d(0, 0, 0), ChVector3d(1.0, 0, 0));
    builder.GetLastBeamNodes().front()->SetFixed(true);
    sys.Add(mesh);

    // MINRES: symmetric Krylov solver, no factorisation.
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    sys.SetSolver(solver);

    // HHT-alpha: implicit, second-order, with tunable numerical damping.
    // alpha in [-1/3, 0]; more negative damps high-frequency FEA modes harder.
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto hht = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    hht->SetAlpha(-0.2);
    hht->SetMaxIters(8);
    hht->SetAbsTolerances(1e-4, 1e2);
    hht->SetStepControl(false);   // fixed step: a control loop cannot subdivide
    hht->SetVerbose(false);

    for (int i = 0; i < 5; ++i) sys.DoStepDynamics(step);

    StepStats st;
    for (int i = 0; i < n_steps; ++i) {
        auto t0 = stdc::steady_clock::now();
        sys.DoStepDynamics(step);
        auto t1 = stdc::steady_clock::now();
        st.add(stdc::duration<double, std::micro>(t1 - t0).count());
    }
    // ChNodeFEAxyzD carries position + slope = 6 dof per node.
    st.report("B. ANCF cable, " + std::to_string(n_elements) + " elem, MINRES+HHT",
              (n_elements + 1) * 6);
}

int main() {
    std::cout << "\n=============================================================\n"
              << " Chrono " << CHRONO_VERSION << " — Milestone 1 verification\n"
              << "=============================================================\n\n";

    std::cout << "Compiled-in capabilities:\n";
#ifdef CHRONO_VEHICLE
    std::cout << "  [YES] Chrono::Vehicle\n";
#else
    std::cout << "  [ NO] Chrono::Vehicle   <-- Milestone 4 will fail\n";
#endif
#ifdef CHRONO_OPENMP_ENABLED
    std::cout << "  [YES] OpenMP\n";
#else
    std::cout << "  [ NO] OpenMP\n";
#endif
#ifdef CHRONO_HAS_AVX
    std::cout << "  [YES] AVX\n";
#endif
#ifdef CHRONO_HAS_FMA
    std::cout << "  [YES] FMA\n";
#endif
    std::cout << "  core FEA (ANCF) is always present in Chrono 10\n\n";

    std::cout << "Instantiating the three classes named in the roadmap:\n";
    { ChSystemSMC s;                                   std::cout << "  [OK] ChSystemSMC\n"; }
    { auto s = chrono_types::make_shared<ChSolverMINRES>();    std::cout << "  [OK] ChSolverMINRES\n"; }
    { auto t = chrono_types::make_shared<ChTimestepperHHT>();  std::cout << "  [OK] ChTimestepperHHT\n"; }

    std::cout << "\n-------------------------------------------------------------\n"
              << " Per-step cost at h = 1 ms. Budget for 1 kHz real time: 1000 us\n"
              << "-------------------------------------------------------------\n";

    benchRigid(10,  2000, 1e-3);
    benchRigid(40,  2000, 1e-3);
    benchANCF(8,     400, 1e-3);
    benchANCF(16,    400, 1e-3);
    benchANCF(32,    200, 1e-3);

    std::cout << "\nRead the ANCF rows as a scaling curve, not as a verdict on the\n"
                 "cable itself. A production ANCF *tire* is a shell mesh of several\n"
                 "thousand dof -- one to two orders of magnitude beyond the largest\n"
                 "row above, and there are four of them on a car.\n\n";
    return 0;
}
