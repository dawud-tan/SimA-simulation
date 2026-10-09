#!/usr/bin/env python3
# ============================================================================
#  gen_course.py — the SIM A practical test course, generated from dimensions.
#
#  Same contract as gen_innova.py: EDIT THIS FILE, NEVER THE JSON. Every value
#  carries a provenance tag and everything derivable is recomputed on each run.
#
#  PROVENANCE TAGS
#    [SPEC]  a published vehicle figure
#    [DERIV] computed here from [SPEC] values by geometry or physics
#    [EST]   an engineering estimate
#    [REG?]  a COURSE dimension attributed to the SIM A regulation that has
#            NOT been verified against a primary source. Treat as [EST].
#    [REG]   a COURSE dimension read in the regulation's own text (task 11):
#            Perkapolri No. 9/2012 tentang SIM, Lampiran A "Ujian Praktik SIM
#            A" -- the LAST PUBLISHED figures. Its successor, Perpol No. 5/2021
#            (as amended by Perpol No. 2/2023), Pasal 18 ayat (6), delegates
#            the current ones: "Materi yang diujikan dan ketentuan pelaksanaan
#            ujian praktik ditetapkan dengan Keputusan Kakorlantas Polri" -- a
#            decree not in the legal databases (JDIH BPK, peraturan.go.id).
#            Reporting after 2021 (detik, Oct 2022) still describes the 2012
#            items and cites Perkapolri 9/2012. So: sourced, and possibly
#            superseded by a document that could not be read.
#
#  WHY [REG?] EXISTS AND WHY IT IS NOT [SPEC]
#  Milestone 4 was derailed by Step 0's confident claim that the IMV platform
#  had extensive published documentation; it does not, and ~90% of the vehicle
#  had to be derived. The lesson was to tag what is actually known. The exact
#  cone spacings and bay dimensions of the Indonesian SIM A course are, for
#  this project right now, in the same position: plausible figures are easy to
#  find and hard to verify. So each one is stated as [REG?] and printed next to
#  the MINIMUM THIS CAR CAN PHYSICALLY ACHIEVE, which is derived and is not in
#  doubt. If a configured dimension is below the derived minimum, the exercise
#  is impossible in an Innova and the report says so in those words.
#
#  That comparison is the useful engineering output either way: it answers
#  "can this car do this exercise, and by how much margin" without needing the
#  regulation to be settled first.
#
#  COORDINATES: Chrono/ISO 8855 -- X forward, Y LEFT, Z up, metres,
#  right-handed. The same frame the physics publishes, so the referee needs no
#  conversion. Unreal's left-handed centimetres are handled once, in
#  ChronoInterpolator.h, exactly as for the vehicle.
#
#  Indonesia drives on the LEFT, so kerbside exercises are on the +Y side.
# ============================================================================

import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "simA_course.json")
GEN_INNOVA = os.path.join(HERE, "..", "vehicle", "gen_innova.py")

_notes = []


def N(tag, msg):
    _notes.append((tag, msg))
    print(f"  [{tag:5s}] {msg}")


# ---------------------------------------------------------------------------
#  Vehicle dimensions, PARSED from gen_innova.py rather than copied.
#
#  A copy would drift. The vehicle generator is the single source of truth for
#  what this car is, and a course whose derived minima were computed from a
#  stale wheelbase would be wrong in a way nothing would catch -- the JSON
#  would still load and the car would still drive, it would just be scored
#  against the wrong geometry.
# ---------------------------------------------------------------------------

def parse_innova(names):
    try:
        src = open(GEN_INNOVA).read()
    except OSError as e:
        sys.exit(f"cannot read {GEN_INNOVA}: {e}\n"
                 "The course is derived from the vehicle; it cannot be "
                 "generated without it.")
    out = {}
    for n in names:
        m = re.search(rf"^{n}\s*=\s*([0-9.]+)", src, re.M)
        if not m:
            sys.exit(f"{n} not found in gen_innova.py -- the generator changed "
                     f"shape and this parser needs updating.")
        out[n] = float(m.group(1))
    return out


V = parse_innova(["WHEELBASE", "TRACK_F", "LENGTH", "WIDTH", "TURN_RADIUS", "WDIST_F"])

WHEELBASE = V["WHEELBASE"]      # [SPEC] m
TRACK     = V["TRACK_F"]        # [SPEC] m
LENGTH    = V["LENGTH"]         # [SPEC] m
WIDTH     = V["WIDTH"]          # [SPEC] m
TURN_R    = V["TURN_RADIUS"]    # [SPEC] m, published minimum turning radius

# The rolling radius, for the referee's rollback rule (task 7), which measures
# how far the WHEELS roll back: spin x radius. gen_innova.py builds the tyre
# from its size on one line; parsed from there like everything above. TMeasy's
# own rolling radius is (2 R0 + r_static)/3 (ChTMeasyTire.cpp), and
# gen_innova.py's static deflection is 3%.
_m = re.search(r"^TYRE_W, TYRE_AR, RIM_IN\s*=\s*([0-9.]+),\s*([0-9.]+),\s*([0-9.]+)",
               open(GEN_INNOVA).read(), re.M)
if not _m:
    sys.exit("tyre size line not found in gen_innova.py -- the parser needs updating")
_tw, _ar, _rim = (float(g) for g in _m.groups())
R0     = _rim * 0.0254 / 2.0 + _tw * _ar                     # [DERIV] unloaded radius
R_ROLL = (2.0 * R0 + 0.97 * R0) / 3.0                        # [DERIV] TMeasy rolling radius

# The steering lock the MODEL actually reaches. Use the MEASURED figure, not
# gen_innova.py's DELTA_MAX design intent: the referee scores the simulated
# car, and a course the model cannot physically drive is a broken test whatever
# the real Innova could do.
#
# This is the ACKERMANN AVERAGE, cot(d_avg) = (cot(d_inner)+cot(d_outer))/2 --
# not the arithmetic mean of the two road-wheel angles. The average is what
# sets the turning radius, and with 66-89% Ackermann the inner and outer
# differ by nearly 10 degrees at full lock, so the distinction is worth ~1 deg.
#
# Measured with vehicle/run.sh ackermann after task 14 (the car at its design
# ride height, PINION_CAL refitted): inner 40.72 deg, outer 31.12 deg ->
# 35.36 deg. (35.22 before, measured with the front 84 mm into bump.)
DELTA_MAX_DEG = 35.36           # [SPEC] measured, vehicle/run.sh ackermann

# Overhang split. Toyota publishes overall length but not the split; an MPV on
# a RWD platform carries more behind the rear axle than in front.
FRONT_OVERHANG = 0.880          # [EST] m
REAR_OVERHANG  = LENGTH - WHEELBASE - FRONT_OVERHANG   # [DERIV]

print("=== vehicle, parsed from gen_innova.py ===")
N("SPEC", f"wheelbase {WHEELBASE:.3f} m, track {TRACK:.3f} m, "
          f"L {LENGTH:.3f} x W {WIDTH:.3f} m")
N("EST", f"overhangs front {FRONT_OVERHANG:.3f} m, rear {REAR_OVERHANG:.3f} m")

# ---------------------------------------------------------------------------
#  Turning geometry -- the number every derived minimum below depends on.
# ---------------------------------------------------------------------------

print("\n=== turning geometry ===")

DELTA_MAX = math.radians(DELTA_MAX_DEG)

# Radius of the REAR AXLE CENTRE at full lock. This is the natural radius for
# every manoeuvre below, because the rear axle centre is the point that traces
# a circular arc under the kinematic bicycle model -- the instantaneous centre
# of rotation lies on the rear axle line.
R_REAR = WHEELBASE / math.tan(DELTA_MAX)                 # [DERIV]
N("DERIV", f"rear-axle-centre radius at {DELTA_MAX_DEG:.1f} deg lock: "
           f"R = L/tan(d) = {R_REAR:.3f} m")

# Cross-check against the published turning radius. A manufacturer's "minimum
# turning radius" is normally kerb-to-kerb: the circle swept by the OUTER FRONT
# WHEEL's contact patch. That wheel sits one wheelbase ahead of the rear axle
# and half a track outboard of the centreline, so
#     R_outer^2 = L^2 + (R_rear + track/2)^2
R_OUTER = math.hypot(WHEELBASE, R_REAR + TRACK / 2.0)    # [DERIV]
err = 100.0 * (R_OUTER - TURN_R) / TURN_R
N("DERIV", f"implied kerb-to-kerb radius {R_OUTER:.3f} m vs published "
           f"{TURN_R:.3f} m  ({err:+.1f}%)")
if abs(err) < 5.0:
    N("DERIV", f"lock and published turning circle AGREE -- the "
               f"{DELTA_MAX_DEG:.2f} deg measurement is consistent with the "
               f"{TURN_R:.1f} m spec.")
else:
    N("EST", "lock and published turning circle DISAGREE; one of DELTA_MAX or "
             "the interpretation of TURN_RADIUS is wrong.")

# NOTE, worth knowing before trusting gen_innova.py's own steering derivation:
# that file computes delta_avg = atan(WHEELBASE / TURN_RADIUS) = 27.0 deg,
# which treats the published 5.4 m as a REAR-AXLE-CENTRE radius. Under the
# kerb-to-kerb reading used here the same spec implies ~35 deg, which is what
# DELTA_MAX was independently set to. The two readings differ by 8 degrees of
# road-wheel angle; the kerb-to-kerb reading is the one consistent with the
# measured full lock, so it is the one used for the course.

# Body corners in the rear-axle-centre frame (x forward, y left).
CORNERS = [
    (WHEELBASE + FRONT_OVERHANG,  WIDTH / 2.0),   # front left
    (WHEELBASE + FRONT_OVERHANG, -WIDTH / 2.0),   # front right
    (-REAR_OVERHANG,              -WIDTH / 2.0),  # rear right
    (-REAR_OVERHANG,               WIDTH / 2.0),  # rear left
]

# Swept radii about the instantaneous centre, used for the aisle-width and
# clearance derivations. At full lock the ICR is at (0, +-R_REAR) in this frame.
def swept_radii(R):
    """Min and max distance from the ICR to any body corner, turning at radius R.

    The MAXIMUM is traced by the outer front corner and is what decides whether
    the car fits through a gap. The MINIMUM is the inner rear corner and is what
    decides whether it clips the inside of a turn -- the classic reason a long
    vehicle cuts a kerb it seemed to have cleared."""
    ds = [math.hypot(cx, R - cy) for cx, cy in CORNERS]
    return min(ds), max(ds)


R_IN, R_OUT = swept_radii(R_REAR)
N("DERIV", f"body swept band at full lock: inner {R_IN:.3f} m, "
           f"outer {R_OUT:.3f} m (width {R_OUT - R_IN:.3f} m)")

# ---------------------------------------------------------------------------
#  Kinematic bicycle model, used to derive the parking minima numerically.
#
#  Closed-form parking formulas exist but every published one carries its own
#  assumptions about which corner binds and whether the car ends flush. Rather
#  than reproduce a formula whose derivation cannot be checked here, the
#  manoeuvre is integrated directly and the binding corner is found by looking.
#  It is a dozen lines and it is exact for the model the referee will score.
# ---------------------------------------------------------------------------

def integrate(pose, steer, ds, n):
    """Advances the rear axle centre along a constant-steer arc.

    ds may be negative, which is reversing. Yields every intermediate pose so
    the caller can test the whole swept path, not just the endpoints -- the
    corner that hits something almost never does so at an endpoint."""
    x, y, psi = pose
    k = math.tan(steer) / WHEELBASE
    for _ in range(n):
        x += ds * math.cos(psi)
        y += ds * math.sin(psi)
        psi += ds * k
        yield (x, y, psi)


def body(pose):
    """The four body corners in world coordinates for a rear-axle-centre pose."""
    x, y, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    return [(x + cx * c - cy * s, y + cx * s + cy * c) for cx, cy in CORNERS]


# ---------------------------------------------------------------------------
#  EXERCISE 1 — SLALOM
#
#  Physics: weaving between cones is a curvature problem, not a speed problem.
#  Approximating the path as a sinusoid of amplitude A and wavelength 2d (one
#  full left-right cycle spans two cone gaps),
#
#      y(x) = A sin(pi x / d)      =>   |y''|max = A (pi/d)^2
#
#  and the car cannot exceed curvature 1/R_REAR, so
#
#      A pi^2 / d^2  <=  1 / R_REAR      =>      d  >=  pi sqrt(A R_REAR)
#
#  This is a LOWER bound: it ignores that the car has length and that the body
#  must miss the cones, both of which push d higher. It is the right bound to
#  state because it is the one that cannot be argued with -- below it, no
#  steering input whatsoever gets the car through.
# ---------------------------------------------------------------------------

print("\n=== exercise 1: slalom ===")

# Lampiran A 3.b: "patok 9 buah ditambah 1 untuk batas garis start dan 1 untuk
# batas garis finish ... jarak antara patok yang satu dengan yang lain 1,5 kali
# panjang kendaraan bermotor uji". (Was 5 cones at 8.0 m, [REG?].)
SLALOM_GAP      = 1.5 * LENGTH   # [REG] m, 1.5 x vehicle length
SLALOM_CONES    = 9              # [REG]
SLALOM_LATERAL  = 1.5      # [EST] m, lateral offset the car must achieve -- how
                           # far the CAR weaves, not a course dimension

A = SLALOM_LATERAL / 2.0
d_min = math.pi * math.sqrt(A * R_REAR)                  # [DERIV]
N("DERIV", f"minimum cone spacing for +-{A:.2f} m of weave at full lock: "
           f"d >= pi*sqrt(A*R) = {d_min:.2f} m")
N("REG", f"configured spacing {SLALOM_GAP:.2f} m (1.5 x length, Perkapolri 9/2012) "
          f"-> margin {SLALOM_GAP - d_min:+.2f} m "
          f"({'FEASIBLE' if SLALOM_GAP >= d_min else 'IMPOSSIBLE for this car'})")

# Body-vs-cone clearance. The car pivots about the rear axle, so the point that
# sweeps widest is the outer FRONT corner; at the moment of maximum steer the
# body is at its worst yaw relative to the path.
half_diag = math.hypot(WHEELBASE + FRONT_OVERHANG, WIDTH / 2.0)
N("DERIV", f"front outer corner is {half_diag:.3f} m from the rear axle "
           f"centre -- the radius that must clear each cone")

# ---------------------------------------------------------------------------
#  EXERCISE 2 — PARALLEL PARKING (parkir paralel)
#
#  Derived, not tabulated. The manoeuvre is integrated backwards out of the
#  parked pose: drive FORWARD out of the bay on full lock, then counter-lock to
#  straighten. Reversibility of the kinematic model makes that identical to
#  reversing in, and integrating outwards from a known final pose removes the
#  search over start positions entirely.
#
#  The slot length required is then just the furthest forward any body corner
#  reaches while still at a lateral position that overlaps the parked cars.
# ---------------------------------------------------------------------------

print("\n=== exercise 2: parallel parking ===")

# Lampiran A 3.c(5): "ukuran tempat parkir yang panjangnya 1,5 kali panjang
# kendaraan bermotor uji dan lebar 1,5 meter". A 1.5 m wide space cannot hold
# this 1.83 m car, and the bay clause two lines up reads "1,5 lebar kendaraan";
# so the width is read as 1.5 x vehicle WIDTH -- an INTERPRETATION of a
# printed figure that cannot be meant literally. (Was car + 2 x 0.30 m [EST].)
PP_BAY_WIDTH  = 1.5 * WIDTH                              # [REG] interpreted
PP_SIDE_CLEAR = (PP_BAY_WIDTH - WIDTH) / 2.0             # [DERIV]
PP_SLOT_LEN   = 1.5 * LENGTH                             # [REG]


def parallel_min_slot(bay_width):
    """Minimum slot length for the two-arc manoeuvre, integrated.

    Frame: kerb line at y = 0, bay occupies y in [-bay_width, 0], and the
    parked car behind ends at x = 0. Returns the smallest slot length in which
    the manoeuvre fits."""
    # Parked pose: centred across the bay, rear bumper at x = 0.
    y_park = -bay_width / 2.0
    x_park = REAR_OVERHANG
    pose = (x_park, y_park, 0.0)

    # Drive out: full lock AWAY from the kerb (kerb is +y here after the sign
    # convention below, so steer negative), then counter-lock to straighten.
    # Arc angle for each half of a lateral shift of `shift`.
    shift = bay_width / 2.0 + WIDTH / 2.0 + 0.20   # clear of the bay mouth
    if shift > 2 * R_REAR:
        return float("inf")
    theta = math.acos(1.0 - shift / (2.0 * R_REAR))

    arc_len = R_REAR * theta
    steps = 240
    ds = arc_len / steps

    worst_x = -1e9
    path = [pose]
    for p in integrate(pose, -DELTA_MAX, ds, steps):
        path.append(p)
    for p in integrate(path[-1], +DELTA_MAX, ds, steps):
        path.append(p)

    for p in path:
        for (bx, by) in body(p):
            # Only corners at a depth that overlaps the parked cars can hit
            # them. A corner already out in the traffic lane is free.
            if by <= 0.0 + 1e-9 and by >= -bay_width - 1e-9:
                worst_x = max(worst_x, bx)
    return worst_x


PP_MIN = parallel_min_slot(PP_BAY_WIDTH)                 # [DERIV]
N("DERIV", f"bay width {PP_BAY_WIDTH:.3f} m (car {WIDTH:.3f} + "
           f"2 x {PP_SIDE_CLEAR:.2f} clearance)")
N("DERIV", f"minimum slot length by integrating the two-arc manoeuvre: "
           f"{PP_MIN:.3f} m = {PP_MIN / LENGTH:.2f} x car length")
N("REG", f"configured slot {PP_SLOT_LEN:.3f} m (1.5 x length, Perkapolri 9/2012) "
          f"-> margin {PP_SLOT_LEN - PP_MIN:+.3f} m "
          f"({'FEASIBLE' if PP_SLOT_LEN >= PP_MIN else 'IMPOSSIBLE for this car'})")

# ---------------------------------------------------------------------------
#  EXERCISE 3 — BAY / PERPENDICULAR PARKING (parkir seri, reversed in)
#
#  The binding dimension is the AISLE WIDTH, not the bay. Reversing into a 90
#  degree bay, the car turns about the ICR and the outer front corner sweeps a
#  circle of radius R_OUT. The aisle must contain that sweep plus the depth the
#  car has already entered the bay.
# ---------------------------------------------------------------------------

print("\n=== exercise 3: bay parking ===")

# Lampiran A 3.c(3), parkir seri: "cara mundur sekali tepat pada posisi parkir
# yang baik ... ukuran tempat parkir 1,5 lebar kendaraan dan panjang 1,5 kali
# kendaraan bermotor uji". The aisle is NOT given -- only "banyaknya patok untuk
# lintasan jalan dengan tempat parkir 22 buah" -- so it stays an estimate.
# (Was 2.50 x 5.00 m, aisle 6.00 m, all [REG?].)
BAY_WIDTH  = 1.5 * WIDTH   # [REG] m
BAY_DEPTH  = 1.5 * LENGTH  # [REG] m
AISLE      = 6.00          # [EST] m, not in the regulation


def bay_min_aisle(bay_width, bay_depth):
    """Aisle width needed to reverse into a 90 degree bay, single cut.

    Integrated, not solved, and it took two wrong answers to get here -- both
    worth recording because they are the same mistake in different clothes.

      1. A closed form from the swept radii gave 3.3 m. It tracked the ICR and
         the outer corner but missed that the car is still PARTLY IN THE BAY
         while its nose is already across the aisle.
      2. Integrating a bare 90 degree arc gave 2.4 m -- and that arc does not
         end with the car in the aisle at all. A quarter circle of radius R
         translates the rear axle centre laterally by exactly R, which here
         leaves the centreline 0.29 m past the bay mouth: half the car is still
         in the bay's depth. It was measuring a manoeuvre that does not finish.

    The real single-cut manoeuvre has a STRAIGHT segment: reverse straight down
    the bay, having arrived there on a 90 degree arc from a start pose that was
    fully in the aisle and parallel to it. Integrated outwards from the parked
    pose that is: straight out, then arc until parallel.

    The terminating condition is the thing the first two versions lacked --
    every body corner must finish at y <= 0, i.e. genuinely in the aisle.

    Frame: bay mouth on y = 0, bay occupying y in [0, bay_depth], x in
    [0, bay_width]. The aisle is the strip y < 0."""
    y_park = bay_depth - REAR_OVERHANG - 0.20
    steps = 360
    ds_arc = (R_REAR * math.pi / 2.0) / steps

    best = float("inf")
    # More straight-out only pushes the car deeper into the aisle, so the
    # optimum is the shortest segment that still finishes clear. Sweep anyway:
    # it costs nothing and it does not assume the monotonicity.
    for k in range(0, 200):
        straight = k * 0.05
        pose = (bay_width / 2.0, y_park, -math.pi / 2.0)
        path = [pose]
        n = max(1, int(straight / 0.02))
        if straight > 0.0:
            for pp in integrate(pose, 0.0, 0.02, n):
                path.append(pp)
        for pp in integrate(path[-1], +DELTA_MAX, ds_arc, steps):
            path.append(pp)

        # Did it finish in the aisle?
        if max(c[1] for c in body(path[-1])) > 0.0:
            continue
        deepest = -min(c[1] for pp in path for c in body(pp))
        best = min(best, deepest)
    return best


AISLE_MIN = bay_min_aisle(BAY_WIDTH, BAY_DEPTH)          # [DERIV]
N("DERIV", f"bay must be at least car width + clearance: "
           f"{WIDTH + 0.4:.2f} m")
N("DERIV", f"minimum aisle by integrating a single-cut 90 deg reverse: "
           f"{AISLE_MIN:.2f} m")
N("EST", "that minimum assumes ZERO clearance from the parked row -- the "
         "start pose has the car's near side exactly on the bay mouth line, "
         "touching the noses of the parked cars. It is a geometric floor, not "
         "a design figure; published 90 deg aisle widths of ~6 m carry margin "
         "for two-way traffic, doors and driver skill that this does not.")
N("REG", f"configured bay {BAY_WIDTH:.2f} x {BAY_DEPTH:.2f} m (1.5 W x 1.5 L), aisle "
          f"{AISLE:.2f} m -> aisle margin {AISLE - AISLE_MIN:+.2f} m "
          f"({'FEASIBLE' if AISLE >= AISLE_MIN and BAY_WIDTH >= WIDTH + 0.4 else 'TIGHT/IMPOSSIBLE'})")
if BAY_DEPTH < LENGTH:
    N("REG?", f"bay depth {BAY_DEPTH:.2f} m is shorter than the car "
              f"({LENGTH:.2f} m); the nose will overhang the bay mouth.")

# ---------------------------------------------------------------------------
#  EXERCISE 4 — HILL START (tanjakan)
#
#  CLAUDE.md already carries the traction analysis: in 1st gear the tractive
#  force available is about 1.00x vehicle weight, but the REAR tyres cap near
#  7,100 N of the 17,700 N the engine could otherwise deliver. So the limit is
#  grip on the driven axle, not torque -- which is what makes it a clutch
#  exercise rather than a throttle one.
#
#  On a slope the rear axle also unloads as weight transfers rearward... on a
#  RWD car it LOADS, which helps. The number below is the gradient at which the
#  rear tyres can still hold and launch the car.
# ---------------------------------------------------------------------------

print("\n=== exercise 4: hill start ===")

# Lampiran A 1.h, "untuk materi tanjakan dan turunan: sudut kemiringan 15 derajat;
# tinggi kemiringan dari dasar permukaan 2,5 M; ... panjang jalan datar jembatan
# 1,5 kali panjang kendaraan bermotor uji". Up, a flat "bridge", and down again.
# 3.d(6): the stop lines on the ascent, the bridge and the descent lie
# "sekurang-kurangnya berada ditengah-tengah panjang jalan".
# 3.d(2),(7): stop on the ascent, pull away "tidak ada reaksi kendaraan mundur";
# it FAILS on "mati mesin, mundur pada saat berhenti ditanjakan". No distance is
# allowed at all -- so ROLLBACK_MAX is ZERO, and what the referee needs is a
# detection floor above the model's own noise (task 7).
# (Was 12% over 20 m to 2.4 m, flat on top, rollback 0.30 m -- all [REG?].)
HILL_ANGLE   = 15.0                                   # [REG] degrees
HILL_GRADE   = math.tan(math.radians(HILL_ANGLE))     # [DERIV] 26.8%
HILL_RISE    = 2.5                                    # [REG] m
HILL_LEN     = HILL_RISE / HILL_GRADE                 # [DERIV] m of run
BRIDGE_LEN   = 1.5 * LENGTH                           # [REG] m, flat on top
ROLLBACK_MAX = 0.0                                    # [REG] m: none

MU = 0.9                   # matches the RigidTerrain patch in chrono_shim.cpp
WDIST_R = 1.0 - V["WDIST_F"]   # rear static weight fraction, parsed (was a copy)
grade_max = MU * WDIST_R                                  # [DERIV], small-angle
N("DERIV", f"static rear axle carries {WDIST_R:.0%} of weight; at mu={MU} the "
           f"grade a RWD launch can hold is ~{grade_max:.0%} before weight "
           f"transfer is counted")
N("REG", f"configured {HILL_ANGLE:.0f} deg = {HILL_GRADE:.1%} (Perkapolri 9/2012) -> "
         f"{'within traction' if HILL_GRADE < grade_max else 'BEYOND traction'}")
# Holding the car on the clutch: gravity along the slope through 1st and the
# final drive, against the clutch's 600 N m (gen_innova.py). The fraction of
# pedal travel a hill start lives in.
_c_hold = (1800.0 * 9.81 * math.sin(math.radians(HILL_ANGLE)) * 0.3318
           / (4.313 * 3.909) / 600.0)
N("DERIV", f"holding {HILL_ANGLE:.0f} deg in 1st takes {_c_hold:.1%} of the "
           f"clutch's capacity")
N("REG", f"rollback allowed: {ROLLBACK_MAX:.2f} m (\"tidak ada reaksi "
         f"kendaraan mundur\"); how HARD that is to meet is clutch-pedal "
         f"ergonomics, NOT tunable without a real pedal (see CLAUDE.md).")

# ---------------------------------------------------------------------------
#  LAYOUT — place the exercises end to end along +X.
# ---------------------------------------------------------------------------

cones, lines, gates, zones, patches = [], [], [], [], []

# Exercise ids, shared by the objects, the exercise table and the referee file.
_EXERCISE_ID = {"start": 0, "slalom": 1, "parallel": 2, "bay": 3,
                "hill": 4, "hill_stop": 4, "hill_crest": 4, "approach": 0,
                "crest": 4, "all": 0}


def cone(x, y, exercise):
    cones.append({"id": len(cones), "x": round(x, 4), "y": round(y, 4),
                  "radius": 0.15, "height": 0.70, "exercise": exercise})


def gate(x0, y0, x1, y1, exercise):
    """A gate, in DRIVING ORDER: its seq is its position in the test. Returns
    its id, so the exercise table below names real gates, not counted ones."""
    gates.append({"id": len(gates), "seq": len(gates),
                  "x0": round(x0, 4), "y0": round(y0, 4),
                  "x1": round(x1, 4), "y1": round(y1, 4), "exercise": exercise})
    return gates[-1]["id"]


def line(x0, y0, x1, y1, kind, exercise):
    lines.append({"id": len(lines), "kind": kind, "exercise": exercise,
                  "x0": round(x0, 4), "y0": round(y0, 4),
                  "x1": round(x1, 4), "y1": round(y1, 4)})


def zone(kind, poly, exercise, heading):
    """A parking space. `heading` is the yaw of a correctly parked car -- the
    referee judges the angle against it, so it is part of the exercise, not a
    property of the rectangle: the bay is reversed INTO, and a car driven in
    nose first is 180 degrees out, not "on the axis"."""
    zones.append({"id": len(zones), "kind": kind, "exercise": exercise,
                  "heading": round(heading, 6),
                  "poly": [[round(px, 4), round(py, 4)] for px, py in poly]})
    return zones[-1]["id"]


# The exercises of the test, for the referee's verdict (task 4). Each STARTS
# when the car crosses its entry gate and passes by its own rule: the exit
# gate (gated), a PARK_OK in its zone (park), or the exit gate after a stop
# on the way (hill). Its order in the test is its entry gate's seq.
exercises = []


def exercise(name, kind, entry, exit_gate=-1, zone_id=-1):
    exercises.append({"id": _EXERCISE_ID[name], "name": name, "kind": kind,
                      "entry_gate": entry, "exit_gate": exit_gate, "zone": zone_id})


# -- start straight ---------------------------------------------------------
X = 0.0
START = {"x": 0.0, "y": 0.0, "yaw": 0.0, "z": 0.45}
# The first gate is the slalom's ENTRY. It used to be a free-standing "start"
# gate; a test needs to know where each exercise begins, and this is where the
# first one does.
slalom_entry = gate(15.0, -3.0, 15.0, 3.0, "slalom")
X = 20.0

# -- slalom -----------------------------------------------------------------
# Cones on the centreline; the car weaves around alternate sides.
slalom_x0 = X
for i in range(SLALOM_CONES):
    cone(slalom_x0 + i * SLALOM_GAP, 0.0, "slalom")
    side = SLALOM_LATERAL if (i % 2 == 0) else -SLALOM_LATERAL
    slalom_exit = gate(slalom_x0 + i * SLALOM_GAP, side * 0.1,
                       slalom_x0 + i * SLALOM_GAP, side * 2.0, "slalom")
exercise("slalom", "gated", slalom_entry, exit_gate=slalom_exit)   # last cone gate ends it
X = slalom_x0 + (SLALOM_CONES - 1) * SLALOM_GAP + 10.0

# -- parallel parking, kerb on the LEFT (+Y): Indonesia drives on the left ---
pp_x0 = X
KERB_Y = 3.0
# Entry: across the road 6 m before the slot, after the last slalom cone.
parallel_entry = gate(pp_x0 - 6.0, -3.0, pp_x0 - 6.0, KERB_Y, "parallel")
parallel_zone = zone("parking_bay", [
    (pp_x0,                KERB_Y),
    (pp_x0 + PP_SLOT_LEN,  KERB_Y),
    (pp_x0 + PP_SLOT_LEN,  KERB_Y - PP_BAY_WIDTH),
    (pp_x0,                KERB_Y - PP_BAY_WIDTH),
], "parallel", 0.0)                 # parked facing the traffic, +X
# The kerb runs 8 m past the slot and NO further. It used to run 20 m, which
# put it across the bay exercise's aisle, 1 m in front of the bay mouth: every
# correct reverse into the bay crossed it and scored a LINE_CROSS against the
# PARALLEL exercise (found planning the task-3 parking pilot).
# It STARTS 4 m before the slot, past the entry gate. It used to start 20 m
# before, which is inside the slalom (last cone 10 m before the slot): the
# weave around that cone swings the body across y = KERB_Y, and a fault
# against an exercise that has not started fails it (task 4), so a clean
# slalom failed the parallel park before it began.
KERB_START = pp_x0 - 4.0
KERB_END = pp_x0 + PP_SLOT_LEN + 8.0
line(KERB_START, KERB_Y, KERB_END, KERB_Y, "kerb", "parallel")
# The two parked cars that define the slot, as cones at their inner corners.
cone(pp_x0, KERB_Y - PP_BAY_WIDTH, "parallel")
cone(pp_x0 + PP_SLOT_LEN, KERB_Y - PP_BAY_WIDTH, "parallel")
exercise("parallel", "park", parallel_entry, zone_id=parallel_zone)
X = pp_x0 + PP_SLOT_LEN + 15.0

# -- bay parking, reversed in from an aisle ---------------------------------
bay_x0 = X
BAY_Y = 4.0
# Entry: across the aisle 8 m before the bay. It must lie beyond where the
# PARALLEL manoeuvre ever takes the car -- it pulls ~3 m past its slot before
# reversing in -- or setting up the parallel park would start the bay.
BAY_ENTRY_X = bay_x0 - 8.0
assert BAY_ENTRY_X > pp_x0 + PP_SLOT_LEN + 4.0, "bay entry inside the parallel set-up"
bay_entry = gate(BAY_ENTRY_X, BAY_Y - AISLE, BAY_ENTRY_X, BAY_Y, "bay")
bay_zone = zone("parking_bay", [
    (bay_x0,             BAY_Y),
    (bay_x0 + BAY_WIDTH, BAY_Y),
    (bay_x0 + BAY_WIDTH, BAY_Y + BAY_DEPTH),
    (bay_x0,             BAY_Y + BAY_DEPTH),
], "bay", -math.pi / 2.0)          # reversed in: nose out toward the aisle, -Y
for dx in (0.0, BAY_WIDTH):
    line(bay_x0 + dx, BAY_Y, bay_x0 + dx, BAY_Y + BAY_DEPTH, "bay_edge", "bay")
line(bay_x0, BAY_Y + BAY_DEPTH, bay_x0 + BAY_WIDTH, BAY_Y + BAY_DEPTH,
     "bay_end", "bay")
# The aisle is a WIDTH across the approach, not a distance along it: the far
# side is a boundary the car must not cross while manoeuvring.
# Starts 5 m before the bay, not 15: from 15 m it reached back to the end of
# the parallel slot, and the parallel manoeuvre's front swing (~0.2 m past
# y = -2 at full lock) crossed it -- the same leak in the other direction.
AISLE_START = bay_x0 - 5.0
line(AISLE_START, BAY_Y - AISLE, bay_x0 + 20.0, BAY_Y - AISLE,
     "aisle_edge", "bay")
# Each exercise's lines must stay in its own space, or a correct manoeuvre in
# one scores a fault in the other. Checked here, where the layout is decided.
assert KERB_END < AISLE_START, "kerb and aisle edge overlap between exercises"
exercise("bay", "park", bay_entry, zone_id=bay_zone)
X = bay_x0 + 20.0

# -- hill: up, across the bridge, down ---------------------------------------
hill_x0 = X                          # toe of the ascent
hill_x1 = hill_x0 + HILL_LEN         # top of the ascent
hill_x2 = hill_x1 + BRIDGE_LEN       # end of the bridge
hill_x3 = hill_x2 + HILL_LEN         # foot of the descent
hill_entry = gate(hill_x0 - 2.0, -4.0, hill_x0 - 2.0, 4.0, "hill")   # at the toe
# The scored stop line, halfway up the ascent. The bridge and the descent have
# stop lines too (3.d(3),(4)); they are not built -- see "cannot yet do".
gate(hill_x0 + HILL_LEN * 0.5, -4.0, hill_x0 + HILL_LEN * 0.5, 4.0, "hill_stop")
hill_exit = gate(hill_x3, -4.0, hill_x3, 4.0, "hill_crest")         # at the foot
exercise("hill", "hill", hill_entry, exit_gate=hill_exit)
X = hill_x3 + 10.0

# Gates are created in driving order, so seq IS the order of the test; the
# verdict relies on that to tell a skipped exercise from one still to come.
assert [g["seq"] for g in gates] == sorted(g["seq"] for g in gates)
assert [g["x0"] for g in gates] == sorted(g["x0"] for g in gates), "gates out of driving order"

# Every cone and line must lie past its own exercise's entry gate. The
# referee fails an exercise on a fault even before it has started, so an
# object reaching back into the previous exercise's space turns a correct
# manoeuvre there into a failure here. Three leaks of this kind were found by
# driving the course (kerb into the bay aisle, aisle edge into the parallel
# swing, kerb into the slalom); this states the rule once instead of pairwise.
_entry_x = {e["id"]: gates[e["entry_gate"]]["x0"] for e in exercises}
for _o in [dict(c, x0=c["x"], x1=c["x"]) for c in cones] + lines:
    _ex = _EXERCISE_ID[_o["exercise"]]
    assert min(_o["x0"], _o["x1"]) > _entry_x[_ex], \
        f"{_o['exercise']} object at x={min(_o['x0'], _o['x1'])} is before its entry gate"

COURSE_LEN = X

# ---------------------------------------------------------------------------
#  TERRAIN PATCHES
#
#  These are consumed by Chs_CreateVehicleEx (ABI 3), so they are physics, not
#  decoration. Two things about Chrono's RigidTerrain box patch decide the
#  arithmetic here, and both are silent if you get them wrong:
#
#  1. The ChCoordsys origin is the centre of the patch's DRIVING SURFACE (its
#     top face), not the centre of the box. Chrono extrudes the thickness
#     downward from it.
#  2. `length` is measured along the patch's OWN X axis, which for a ramp is
#     inclined. A ramp covering a horizontal run r at gradient g has length
#     r*sqrt(1+g^2). Using r makes the hill quietly too short and leaves a step
#     at the crest.
#
#  The patches TILE and must not overlap. An earlier version had one flat patch
#  spanning the whole course with the ramp on top of it; the flat would then
#  still be present under the ramp, giving duplicate contacts at the toe where
#  the two surfaces meet.
# ---------------------------------------------------------------------------

FLAT_WIDTH = 40.0          # covers the parking bays out to y = +9
HILL_WIDTH = 8.0
MU_TERRAIN = 0.9           # matches the SMC material in chrono_shim.cpp


def patch(kind, exercise, x_lo, x_hi, z_lo, z_hi, width):
    """Emits one patch spanning [x_lo, x_hi] horizontally, rising z_lo -> z_hi."""
    run = x_hi - x_lo
    rise = z_hi - z_lo
    grade = rise / run if run else 0.0
    patches.append({
        "kind": kind, "exercise": exercise,
        # Kept for the renderer and the plot, which think in world extents.
        "x0": round(x_lo, 4), "x1": round(x_hi, 4),
        "z0": round(z_lo, 4), "z1": round(z_hi, 4),
        "grade": round(grade, 6), "width": width,
        # Consumed by the shim. Centre of the DRIVING SURFACE.
        "center_x": round(0.5 * (x_lo + x_hi), 4),
        "center_y": 0.0,
        "center_z": round(0.5 * (z_lo + z_hi), 4),
        # Inclined length, not horizontal run.
        "length": round(math.hypot(run, rise), 4),
        # NEGATIVE pitch climbs: a rotation about +Y takes +X toward -Z.
        "pitch_rad": round(-math.atan2(rise, run), 8),
        "friction": MU_TERRAIN,
    })


patch("flat", "approach", -20.0,   hill_x0, 0.0,       0.0,       FLAT_WIDTH)
patch("ramp", "hill",     hill_x0, hill_x1, 0.0,       HILL_RISE, HILL_WIDTH)
patch("flat", "crest",    hill_x1, hill_x2, HILL_RISE, HILL_RISE, HILL_WIDTH)
patch("ramp", "hill",     hill_x2, hill_x3, HILL_RISE, 0.0,       HILL_WIDTH)
patch("flat", "crest",    hill_x3, hill_x3 + 20.0, 0.0, 0.0,      FLAT_WIDTH)

N("DERIV", f"hill rises {HILL_RISE:.3f} m over {HILL_LEN:.3f} m of run "
           f"({HILL_GRADE:.1%}); ramp patch length {math.hypot(HILL_LEN, HILL_RISE):.4f} m "
           f"(NOT {HILL_LEN:.3f} -- it is inclined), pitch "
           f"{math.degrees(-math.atan2(HILL_RISE, HILL_LEN)):.4f} deg; "
           f"{BRIDGE_LEN:.3f} m bridge, then the same down")

# ---------------------------------------------------------------------------
#  Emit
# ---------------------------------------------------------------------------

doc = {
    "meta": {
        "generator": "gen_course.py",
        "frame": "chrono ISO 8855: X forward, Y left, Z up, metres",
        "note": "GENERATED FILE -- edit gen_course.py, never this.",
        "course_length_m": round(COURSE_LEN, 3),
    },
    "vehicle": {
        "length": LENGTH, "width": WIDTH, "wheelbase": WHEELBASE,
        "track": TRACK, "front_overhang": FRONT_OVERHANG, "r_roll": round(R_ROLL, 4),
        "rear_overhang": round(REAR_OVERHANG, 4),
        "delta_max_deg": DELTA_MAX_DEG,
        "r_rear_axle": round(R_REAR, 4),
        "r_swept_outer": round(R_OUT, 4),
        "r_swept_inner": round(R_IN, 4),
    },
    "derived_minima": {
        "slalom_cone_spacing": round(d_min, 4),
        "parallel_slot_length": round(PP_MIN, 4),
        "bay_aisle_width": round(AISLE_MIN, 4),
        "hill_grade_traction_limit": round(grade_max, 4),
    },
    "configured": {
        "slalom_cone_spacing": SLALOM_GAP,
        "parallel_slot_length": PP_SLOT_LEN,
        "bay_aisle_width": AISLE,
        "hill_grade": HILL_GRADE,
        "rollback_max": ROLLBACK_MAX,
    },
    "start": START,
    "cones": cones,
    "lines": lines,
    "gates": gates,
    "zones": zones,
    "exercises": exercises,
    "patches": patches,
}

with open(OUT, "w") as f:
    json.dump(doc, f, indent=2)
    f.write("\n")

# The physics side reads a PLAIN TEXT projection of the patches rather than the
# JSON. Not duplication for its own sake: the loader runs in the ChronoBridge
# plugin on the path that builds the Chrono world, and a JSON parser there would
# be a dependency on that path for six numbers a line. Same reasoning as the
# driver scripts. Both files are written by this one function call, so they
# cannot drift.
TERRAIN_OUT = os.path.join(HERE, "simA_terrain.txt")
with open(TERRAIN_OUT, "w") as f:
    f.write("# GENERATED by gen_course.py -- do not edit.\n")
    f.write("# Consumed by course_terrain.cpp -> Chs_CreateVehicleEx (ABI 3).\n")
    f.write("#\n")
    f.write("# center_x center_y center_z length width pitch_rad friction\n")
    f.write("#\n")
    f.write("# center_* is the centre of the DRIVING SURFACE, not of the box.\n")
    f.write("# length is along the patch's own (possibly inclined) X axis.\n")
    f.write("# pitch_rad is about Y; NEGATIVE means the surface rises with +X.\n")
    for pp in patches:
        f.write(f"{pp['center_x']:.4f} {pp['center_y']:.4f} {pp['center_z']:.4f} "
                f"{pp['length']:.4f} {pp['width']:.4f} {pp['pitch_rad']:.8f} "
                f"{pp['friction']:.3f}   # {pp['kind']} / {pp['exercise']}\n")

# The referee's view of the course, also plain text and for the same reason as
# the terrain file: it is consumed by a pure-C rule engine that ships in the
# plugin and is exercised by a host harness, and neither should need a JSON
# parser for six numbers a line. One keyword per line so the parser can stay a
# switch on the first token.
REFEREE_OUT = os.path.join(HERE, "simA_referee.txt")
_LINE_KIND = {"kerb": 0, "bay_edge": 1, "bay_end": 2, "aisle_edge": 3}

with open(REFEREE_OUT, "w") as f:
    f.write("# GENERATED by gen_course.py -- do not edit.\n")
    f.write("# Consumed by referee.cpp. One keyword per line.\n")
    f.write("#\n")
    f.write("# CONE id x y radius exercise\n")
    f.write("# LINE id kind x0 y0 x1 y1 exercise\n")
    f.write("# GATE id seq x0 y0 x1 y1 exercise\n")
    f.write("# EXERCISE id kind entry_gate exit_gate zone   (kind 0 gated, 1 park, 2 hill)\n")
    f.write("# ZONE id exercise heading n x0 y0 ... x(n-1) y(n-1)   (convex)\n")
    f.write("# START x y yaw\n")
    f.write("# CAR length width front_overhang rear_overhang wheelbase r_roll\n")
    f.write(f"START {START['x']:.4f} {START['y']:.4f} {START['yaw']:.4f}\n")
    f.write(f"CAR {LENGTH:.4f} {WIDTH:.4f} {FRONT_OVERHANG:.4f} "
            f"{REAR_OVERHANG:.4f} {WHEELBASE:.4f} {R_ROLL:.4f}\n")
    for c in cones:
        f.write(f"CONE {c['id']} {c['x']:.4f} {c['y']:.4f} {c['radius']:.4f} "
                f"{_EXERCISE_ID.get(c['exercise'], 0)}\n")
    for l in lines:
        f.write(f"LINE {l['id']} {_LINE_KIND.get(l['kind'], 0)} "
                f"{l['x0']:.4f} {l['y0']:.4f} {l['x1']:.4f} {l['y1']:.4f} "
                f"{_EXERCISE_ID[l['exercise']]}\n")
    for g in gates:
        f.write(f"GATE {g['id']} {g['seq']} "
                f"{g['x0']:.4f} {g['y0']:.4f} {g['x1']:.4f} {g['y1']:.4f} "
                f"{_EXERCISE_ID[g['exercise']]}\n")
    _EX_KIND = {"gated": 0, "park": 1, "hill": 2}
    for x in exercises:
        f.write(f"EXERCISE {x['id']} {_EX_KIND[x['kind']]} {x['entry_gate']} "
                f"{x['exit_gate']} {x['zone']}   # {x['name']}\n")
    for z in zones:
        verts = " ".join(f"{px:.4f} {py:.4f}" for px, py in z["poly"])
        f.write(f"ZONE {z['id']} {_EXERCISE_ID.get(z['exercise'], 0)} "
                f"{z['heading']:.6f} {len(z['poly'])} {verts}\n")

print(f"\n=== wrote {REFEREE_OUT} ===")
print(f"\n=== wrote {TERRAIN_OUT} ===")
print(f"\n=== wrote {OUT} ===")
print(f"  {len(cones)} cones, {len(lines)} lines, {len(gates)} gates, "
      f"{len(zones)} zones, {len(patches)} patches")
print(f"  course extends {COURSE_LEN:.1f} m along +X")

tags = {}
for t, _ in _notes:
    tags[t] = tags.get(t, 0) + 1
print("  provenance:", ", ".join(f"{k}={v}" for k, v in sorted(tags.items())))

print("""
=== WHERE THIS COURSE DEPARTS FROM THE REGULATION (Perkapolri 9/2012, Lampiran A) ===

1. THE FIGURES MAY BE SUPERSEDED. They are the last PUBLISHED ones. Perpol
   5/2021 (amended by Perpol 2/2023), Pasal 18(6), hands the current ones to a
   Keputusan Kakorlantas Polri that is not in the legal databases.

2. NOT BUILT: 3.a, forward and reverse 50 m through a lane the car's width +
   60 cm wide; the slalom's REVERSE pass (3.b is "maju dan mundur"); the
   stops on the bridge (foot brake) and on the descent (hand brake), 3.d(3),(4).
   The ABI has no hand brake.

3. READ, NOT QUOTED: the parallel space's printed width, "1,5 meter", cannot
   hold this 1.83 m car and is read as 1.5 x vehicle width. The hill road's
   width is given twice, as 1.5 x vehicle LENGTH (1.h(3)) and 1.5 x vehicle
   WIDTH (3.d(5)); the terrain is {w:.0f} m wide and has no edge lines.

4. SCORING is the referee's, not the regulation's: the regulation fails a
   stage on 2 consecutive cone touches; the referee fails an exercise on its
   first fault.
""".format(w=HILL_WIDTH))
