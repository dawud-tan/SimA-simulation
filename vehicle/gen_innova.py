#!/usr/bin/env python3
"""
Generate the Chrono::Vehicle JSON model for a Toyota Kijang Innova Reborn
2.4 G MT (AN140, IMV platform).

WHY A GENERATOR AND NOT HAND-WRITTEN JSON:
Almost nothing about a suspension is a published number. Wheelbase and track
are; hardpoint coordinates, spring rates and inertias are not, for any of the
candidate vehicles. They must be DERIVED from what is published, and a derivation
you cannot re-run is a derivation you cannot correct.

Every value below is tagged:
    [SPEC]  published manufacturer figure
    [DERIV] computed from [SPEC] values by stated engineering relations
    [EST]   engineering estimate, to be refined against ChSuspensionTestRig

Chrono::Vehicle frame convention: X forward, Y LEFT, Z up (ISO 8855).
Suspension-local coordinates are relative to that subsystem's reference frame.
"""

import json, math, os, sys

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "innova")
os.makedirs(OUT, exist_ok=True)
notes = []
def N(tag, msg): notes.append(f"  [{tag}] {msg}")

# ===========================================================================
#  1. PUBLISHED SPECIFICATION
# ===========================================================================
WHEELBASE      = 2.750           # [SPEC] m
TRACK_F        = 1.540           # [SPEC] m
TRACK_R        = 1.540           # [SPEC] m
KERB_MASS      = 1800.0          # [SPEC] kg, 2.4 G MT approx
LENGTH         = 4.735           # [SPEC] m
WIDTH          = 1.830           # [SPEC] m
HEIGHT         = 1.795           # [SPEC] m
TURN_RADIUS    = 5.4             # [SPEC] m, minimum turning radius

# 2GD-FTV 2.393 L turbodiesel
P_MAX_KW       = 110.0           # [SPEC] 149 PS
P_MAX_RPM      = 3400.0          # [SPEC]
T_MAX_NM       = 343.0           # [SPEC]
T_PLATEAU_RPM  = (1400.0, 2800.0)# [SPEC] flat torque band
IDLE_RPM       = 800.0           # [EST]
REDLINE_RPM    = 4200.0          # [EST] diesel

# Tyre 205/65 R16
TYRE_W, TYRE_AR, RIM_IN = 0.205, 0.65, 16.0

HALF_F, HALF_R = TRACK_F / 2.0, TRACK_R / 2.0

# ===========================================================================
#  2. DERIVED GLOBALS
# ===========================================================================
R_UNLOADED = RIM_IN * 0.0254 / 2 + TYRE_W * TYRE_AR
R_LOADED   = R_UNLOADED * 0.97          # ~3% static deflection
N("DERIV", f"tyre 205/65R16 -> unloaded radius {R_UNLOADED:.4f} m, loaded {R_LOADED:.4f} m")

# Body-on-frame MPV with a front-mounted iron diesel: front-biased.
WDIST_F    = 0.53                                       # [EST]
M_F, M_R   = KERB_MASS * WDIST_F, KERB_MASS * (1 - WDIST_F)
# CG measured BACK from the front axle. The FRONT axle carries the fraction
# (distance CG->REAR axle)/wheelbase, so a 53% front bias puts the CG at
# 0.47*wheelbase behind the front axle -- NOT 0.53. Getting this backwards
# silently produces a rear-biased car that understeers the wrong way.
CG_X       = -WHEELBASE * (1.0 - WDIST_F)
CG_Z       = 0.62                                       # [EST] m above ground
N("EST",   f"weight distribution {WDIST_F*100:.0f}/{(1-WDIST_F)*100:.0f} -> "
           f"front {M_F:.0f} kg, rear {M_R:.0f} kg; CG x={CG_X:.3f} z={CG_Z:.2f}")

# The driver's eye [EST], task 9: the cockpit is built around it and the
# mirrors are aimed from it. Heights above the GROUND at design ride height:
# a floor 0.40 up, the hip point 0.33 above the floor (an MPV's upright
# seat), the eye 0.63 above the hip point (a seated adult, back reclined
# ~25 deg) = 1.36 m. x: 1.35 m behind the front axle, 0.35 m ahead of the
# B-pillar. y: RIGHT-hand drive [SPEC] -- Indonesia drives on the left --
# which is -Y in Chrono's frame (Y is LEFT).
EYE_X, EYE_Y, EYE_H = -1.35, -0.38, 0.40 + 0.33 + 0.63
N("EST",   f"driver's eye x={EYE_X} y={EYE_Y}, {EYE_H:.2f} m above the ground")

# Component masses, each [EST]. Defined ONCE, here, because both the springs
# (sprung/unsprung split) and the chassis mass and CG (section 4b) must be
# derived from the same bodies Chrono will build.
SPINDLE_F, UPRIGHT, UCA_M, LCA_M = 14.0, 19.0, 5.5, 9.0
SPINDLE_R, TRI_M, LONG_M         = 12.0, 12.0, 9.0
WHEEL_M, TYRE_M, STEER_LINK_M    = 11.5, 11.5, 9.0
AXLE_TUBE  = 110.0       # [EST] kg, whole live axle

# Unsprung mass: what the TYRES carry without it passing through a spring.
# Wishbones and links pivot on the body, so half of each counts. Until task 14
# this was a free [EST] (42 kg a front corner; the bare axle tube at the rear)
# while the spring preload below used the FULL corner load, unsprung weight
# included -- so rates, preload and Chrono's bodies all disagreed.
UNSPRUNG_F = SPINDLE_F + UPRIGHT + WHEEL_M + TYRE_M + 0.5 * (UCA_M + LCA_M)   # per corner
UNSPRUNG_R = AXLE_TUBE + 2 * (SPINDLE_R + WHEEL_M + TYRE_M) + 0.5 * (TRI_M + 2 * LONG_M)
SPRUNG_F   = M_F - 2 * UNSPRUNG_F          # front axle's sprung share, both corners
SPRUNG_R   = M_R - UNSPRUNG_R
N("DERIV", f"unsprung {UNSPRUNG_F:.1f} kg per front corner, {UNSPRUNG_R:.1f} kg whole rear "
           f"axle -> sprung {SPRUNG_F/2:.1f} kg per front corner, {SPRUNG_R/2:.1f} kg per rear")

# --- ride rates from target natural frequency ------------------------------
# k = (2*pi*f)^2 * m_sprung_per_corner. A comfort-tuned MPV sits near 1.2 Hz
# front, slightly higher at the rear (flat ride: rear >= front so that pitch
# induced by a bump decays rather than builds).
F_RIDE_F, F_RIDE_R = 1.20, 1.35          # [EST] Hz
k_wheel_f = (2 * math.pi * F_RIDE_F) ** 2 * (SPRUNG_F / 2)
k_wheel_r = (2 * math.pi * F_RIDE_R) ** 2 * (SPRUNG_R / 2)
N("DERIV", f"ride freq {F_RIDE_F} Hz front -> wheel rate {k_wheel_f:,.0f} N/m")
N("DERIV", f"ride freq {F_RIDE_R} Hz rear  -> wheel rate {k_wheel_r:,.0f} N/m")

# Damping from target ratio zeta, c_crit = 2*sqrt(k*m), at the WHEEL. The
# spring and shock coefficients need the motion ratios, and those are measured
# on the hardpoints -- section 4b.
ZETA_F, ZETA_R = 0.32, 0.30              # [EST]
c_wheel_f = ZETA_F * 2 * math.sqrt(k_wheel_f * SPRUNG_F / 2)
c_wheel_r = ZETA_R * 2 * math.sqrt(k_wheel_r * SPRUNG_R / 2)

# ===========================================================================
#  3. FRONT DOUBLE WISHBONE HARDPOINTS + ACKERMANN
# ===========================================================================
# Ride height sets the wheel-centre plane; suspension frame origin is at the
# wheel centre height, so z=0 is the wheel centre.
UBJ = (0.000,  0.660,  0.170)     # upper ball joint  [EST]
LBJ = (0.000,  0.700, -0.165)     # lower ball joint  [EST]
UCA_F = (0.150, 0.360,  0.185)
UCA_B = (-0.150, 0.360, 0.185)
LCA_F = (0.185, 0.310, -0.140)
LCA_B = (-0.185, 0.310, -0.140)

TIEROD_Z   = -0.100               # [EST] tierod plane AT THE UPRIGHT
STEER_ARM  = 0.145                # [EST] m, ball joint behind kingpin axis

# --- rack height: DERIVED, not estimated ------------------------------------
# The inner (rack) end of the tierod used to be placed at TIEROD_Z as well --
# inner and outer at the same height. That is the classic bump-steer mistake
# and it had a measurable consequence: front toe swung from 0.097 deg unloaded
# to ~1.3 deg at static ride height, a 13x change over a few centimetres of
# travel. A real front end has a fraction of a degree over its WHOLE travel.
#
# Why it matters beyond tyre wear: the driveline reacts engine torque as a ROLL
# moment on the chassis (innova_Driveline.json sets Motor Block direction
# [1,0,0], so the driveshaft spins about X). Roll puts one front wheel into
# bump and the other into droop. With a sane bump-steer curve those two toe
# changes very nearly cancel; with this one they did not, leaving a net steer
# of about +0.05 deg that the driver never asked for. Measured consequence:
# with `steering` held at exactly 0.0 the car curved at roughly 900 m radius --
# 0.55 m of lateral drift over 30.8 m.
#
# THE CONSTRUCTION: for the outer tierod end to travel on the same arc as the
# point of the kingpin axis at its height, the tierod must be a virtual
# extension of the wishbone geometry. So the inner end sits on the line joining
# the two INNER pivots, at the same height fraction along it as the outer end
# sits along the line joining the two BALL JOINTS.
_t_tie = (UBJ[2] - TIEROD_Z) / (UBJ[2] - LBJ[2])
RACK_Z = UCA_F[2] + _t_tie * (LCA_F[2] - UCA_F[2])        # [DERIV]
N("DERIV", f"tierod outer end is {_t_tie:.1%} of the way down UBJ->LBJ; "
           f"zero-bump-steer rack height is z={RACK_Z:.4f} m "
           f"(was {TIEROD_Z:.4f}, an error of {1000*(RACK_Z-TIEROD_Z):+.1f} mm)")

# --- true Ackermann construction -------------------------------------------
# For 100% Ackermann the steering-arm ball joint must lie on the line from the
# kingpin axis to the CENTRE OF THE REAR AXLE. Then, at any steer angle, the
# perpendiculars to all four wheels meet at one point and no tyre is dragged
# sideways -- which at parking speed is the difference between rolling and
# scrubbing, and scrubbing is what knocks cones over.
t = (UBJ[2] - TIEROD_Z) / (UBJ[2] - LBJ[2])
kp_y = UBJ[1] + t * (LBJ[1] - UBJ[1])      # kingpin y at tierod height
kp_x = UBJ[0] + t * (LBJ[0] - UBJ[0])
# Line kingpin -> rear axle centre (x = -WHEELBASE, y = 0)
# ACK_SCALE = 1.0 is the textbook construction: put the steering-arm ball joint
# exactly on the kingpin-to-rear-axle line. Whether that yields true Ackermann
# on the ASSEMBLED linkage is an empirical question, and the answer depends on
# the whole 3D geometry -- which is why this is a calibrated constant and not a
# derivation.
#
# REFITTED after the tierod inner end moved to its zero-bump-steer height.
# The old value of 0.18 was fitted against the old (wrong) geometry, where the
# rack end sat 23 mm below where the wishbone geometry wanted it. Correcting
# that changed the linkage enough that 0.18 collapsed Ackermann from ~90% to
# 3.6%. This is the standing hazard with any empirically fitted constant: it
# encodes a compensation for a specific error, so fixing the error invalidates
# the constant, silently and in the wrong direction.
#
# Measured with calibrate_steering.sh: 1.0 gives 66% at half lock rising to 89%
# at full lock. Production cars run 60-90%; for a parking test high is right.
ACK_SCALE = float(os.environ.get("ACK_SCALE", "1.0"))
frac = STEER_ARM / WHEELBASE
tie_up_y = kp_y - ACK_SCALE * frac * kp_y
tie_up = (kp_x - STEER_ARM, tie_up_y, TIEROD_Z)
ack_deg = math.degrees(math.atan2(STEER_ARM, kp_y - tie_up_y)) if kp_y != tie_up_y else 90.0
N("DERIV", f"kingpin at tierod height y={kp_y:.4f}; 100% Ackermann puts the "
           f"steering-arm ball joint at y={tie_up_y:.4f} (arm {STEER_ARM} m)")

RACK_HALF = 0.330                                        # [EST] m
# Inner end at the DERIVED rack height, not at the outer end's height.
# RACK_HALF (the y) stays [EST]: the zero-bump-steer construction also wants
# y ~ 0.320 here, 10 mm inboard of this, but rack half-length is a real
# packaging dimension and moving it would shift the Ackermann calibration that
# ACK_SCALE and PINION_CAL were empirically fitted to. Height is the term that
# dominates bump steer; change one thing at a time.
tie_ch = (tie_up[0], RACK_HALF, RACK_Z)

# --- steering ratio ---------------------------------------------------------
# Ackermann average angle from the published turning radius:
#   tan(delta) = wheelbase / R
delta_avg = math.atan(WHEELBASE / TURN_RADIUS)
# Inner wheel runs a few degrees more; allow head-room to full lock.
DELTA_MAX = math.radians(35.0)                            # [EST]
STEER_RATIO = 17.5                                        # [EST]
sw_max_deg = math.degrees(DELTA_MAX) * STEER_RATIO
rack_travel = STEER_ARM * math.sin(DELTA_MAX)
# The simple rack_travel = arm*sin(delta) relation ignores tierod obliquity, so
# the first cut over-steered: 49.8 deg measured against a 35 deg design.
#
# REFITTED alongside ACK_SCALE, and anchored on the one PUBLISHED number rather
# than on a design intent. Toyota's 5.4 m minimum turning radius is kerb-to-kerb
# -- the outer front wheel's contact patch -- so
#     R_rear = sqrt(5.4^2 - L^2) - track/2 = 3.877 m
#     delta_avg = atan(L / R_rear) = 35.35 deg
# where delta_avg is the ACKERMANN average, cot(d_avg) = (cot(d_in)+cot(d_out))/2,
# not the arithmetic mean of the two angles.
#
# Measured with calibrate_steering.sh at ACK_SCALE=1.0: 0.98 gave inner 40.61,
# outer 30.96 at full lock, i.e. delta_avg = 35.21 deg. REFITTED by task 14:
# that fit was made with the front sitting 84 mm into bump (the car sat nose-up
# on mis-derived springs), and the linkage ratio changes with ride height. At
# design ride height 0.98 gave 36.74 deg; 0.954 gives the published 35.35.
PINION_CAL = float(os.environ.get("PINION_CAL", "0.954"))

# Overridable so the creep diagnostic can null it out. 0.012 is a normal
# passenger-tyre figure; the env var exists to test whether Chrono's TMeasy
# applies this as a moment that does not vanish at zero wheel speed.
ROLL_RES = float(os.environ.get("ROLL_RES", "0.012"))
pinion_r = PINION_CAL * rack_travel / math.radians(sw_max_deg)
N("DERIV", f"turning radius {TURN_RADIUS} m -> Ackermann avg angle "
           f"{math.degrees(delta_avg):.1f} deg")
N("DERIV", f"max road-wheel {math.degrees(DELTA_MAX):.0f} deg x ratio {STEER_RATIO} "
           f"-> {sw_max_deg:.0f} deg steering wheel ({sw_max_deg*2/360:.2f} turns lock-to-lock)")
N("DERIV", f"rack travel {rack_travel*1000:.1f} mm -> pinion radius {pinion_r*1000:.2f} mm")

# ===========================================================================
#  4. DRIVETRAIN
# ===========================================================================
# 2GD-FTV torque curve. Shape is anchored on the three published points:
# idle, the 1400-2800 rpm 343 Nm plateau, and 110 kW at 3400 rpm.
# P = T * omega  =>  T at rated power = 110000 / (3400*2pi/60) = 309 Nm.
T_at_rated = P_MAX_KW * 1000 / (P_MAX_RPM * 2 * math.pi / 60)
N("DERIV", f"110 kW @ 3400 rpm implies {T_at_rated:.0f} Nm at rated power "
           f"(vs {T_MAX_NM} Nm plateau) -- consistent with a diesel falling off after 2800")

# IMPORTANT: these maps are in RPM, not rad/s. EngineShafts::SetEngineTorqueMap
# applies CH_RPM_TO_RAD_S internally; pre-converting here would scale the whole
# curve by 9.55 and give an engine that makes peak torque at 143 rpm.
def rpm2rads(r): return r * 2 * math.pi / 60.0   # kept for the sanity checks below
torque_map = [
    [-100,  0.0],
    [0,    250.0],   # cranking / off-idle
    [800,  260.0],
    [1200, 315.0],
    [1400, T_MAX_NM],   # [SPEC] plateau start
    [2000, T_MAX_NM],   # [SPEC]
    [2800, T_MAX_NM],   # [SPEC] plateau end
    [3400, T_at_rated], # [DERIV] from rated power
    [3800, 270.0],
    [4200, 180.0],      # redline
    [4400,   0.0],
]
# Engine friction and pumping losses, IN RPM like the torque map (task 8).
# Until task 8 the x-values were wrapped in rpm2rads() -- directly under the
# warning above -- so Chrono read 146.6 ... 460.8 "rpm" and the -60 N m peak
# arrived at 461 rpm instead of 4400. And the map gave -20 N m at 0 rpm,
# reaching zero only at -100: a declutched engine was driven BACKWARDS, and
# idled at -98 rpm.
# A loss OPPOSES rotation, whichever way the crank turns, and is zero at rest:
# odd-symmetric through the origin, with a 100 rpm ramp so the sign change is
# not a step (Chrono's ChFunctionInterp clamps beyond the ends).
# Magnitudes [EST]: FMEP ~1.2 bar at idle to ~2.6 bar at 4400 rpm for a
# 2.4 L diesel, T = FMEP * Vd / (4 pi) -> ~23 N m and ~50 N m.
losses_map = [
    [-1400,  25.0],
    [-100,   20.0],
    [0,       0.0],
    [100,   -20.0],
    [1400,  -25.0],
    [2800,  -35.0],
    [4200,  -50.0],
    [4400,  -60.0],
]

# The transmission's OUTPUT shaft. An experiment knob (task 6): Chrono resets
# this shaft's speed from the driveline every step and passes on only the
# gearbox's reaction, so its inertia may not act as inertia at all.
# `rollacct` measures whether it does: TR_DS_INERTIA=5.0 python3 gen_innova.py.
TR_DS_INERTIA = float(os.environ.get("TR_DS_INERTIA", "0.5"))   # [EST] kg m^2

# 5-speed manual + final drive. [EST] -- typical for a 2.4 diesel RWD MPV.
GEARS = [4.313, 2.330, 1.436, 1.000, 0.838]
REV_GEAR = -4.220
FINAL_DRIVE = 3.909
top_speed = rpm2rads(P_MAX_RPM) * R_LOADED / (GEARS[-1] * FINAL_DRIVE) * 3.6
N("EST",   f"gear set {GEARS} + final {FINAL_DRIVE} -> "
           f"{top_speed:.0f} km/h at rated rpm in 5th (sanity check only)")
grad_1st = T_MAX_NM * GEARS[0] * FINAL_DRIVE / R_LOADED / (KERB_MASS * 9.81)
N("DERIV", f"1st gear tractive force / weight = {grad_1st:.2f} "
           f"-> can hold {math.degrees(math.atan(grad_1st)):.0f} deg gradient at peak torque "
           f"(SIM A ramp is 20 deg)")

# ===========================================================================
#  4b. SPRINGS FROM THE GEOMETRY AND STATIC EQUILIBRIUM   (rewritten, task 14)
# ===========================================================================
# A spring's rate, free length and stops are not free parameters: they are what
# makes the car sit at its DESIGN ride height -- every spindle at z = 0 of its
# suspension frame -- carrying its own weight. Task 14 measured the car sitting
# 84 mm low at the front and 289 mm low at the rear (4.4 deg nose-up), from
# three errors in the derivation this replaces:
#   1. FRONT MOTION RATIO. The spring sat at y = LBJ_y*MR_F, a fraction of the
#      distance from the car's CENTRELINE; a lever ratio is measured from the
#      arm's PIVOT, at y = 0.31. Actual 0.46 against the 0.70 assumed: the
#      front was 2.3x too soft and short of preload.
#   2. REAR SPRING SIGN. A "Spring Curve Data" table written as the force,
#      f = -k*d. Chrono's NonlinearSpringForce computes F = P - mapK(len - rest),
#      so the table was negated twice: the rear "spring" PULLED when compressed
#      and the body sat on the three-link geometry. Now linear, like the front.
#   3. PRELOAD FROM THE FULL CORNER LOAD, unsprung weight included -- which the
#      tyre carries, not the spring.
# So: motion ratios are MEASURED on the hardpoints; rates follow from them; the
# free length is what holds the SPRUNG corner load at the design length; and
# Chrono's own progressive stops ("Minimum/Maximum Length", ADAMS/Car rubber)
# sit a stated wheel travel either side of design.
def seg_len(a, b):
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))

def rot_x(p, y0, z0, ang):
    """p rotated by `ang` about the line (y0, z0) running along X."""
    dy, dz = p[1] - y0, p[2] - z0
    c, sn = math.cos(ang), math.sin(ang)
    return (p[0], y0 + c*dy - sn*dz, z0 + sn*dy + c*dz)

# Front: coil-over on the lower arm at ARM_FRAC_F of the arm's span from its
# pivot axis (the 0.70 the old code meant), top mount nearly above it.
LCA_AXIS   = (0.5*(LCA_F[1] + LCA_B[1]), 0.5*(LCA_F[2] + LCA_B[2]))   # (y, z) of the X-line
ARM_FRAC_F = 0.70                                                    # [EST]
F_SPR_ARM  = (0.0, LCA_AXIS[0] + ARM_FRAC_F*(LBJ[1] - LCA_AXIS[0]),
              LCA_AXIS[1] + ARM_FRAC_F*(LBJ[2] - LCA_AXIS[1]) + 0.02)  # seat 2 cm above the arm
F_SPR_CH   = (0.0, F_SPR_ARM[1] - 0.02, 0.28)                        # [EST]
# Rear: coil on the axle; the shock has its own mounts.
R_SPR_CH, R_SPR_AX = (0.0, 0.52, 0.26), (0.0, 0.55, -0.03)           # [EST]
R_SHK_CH, R_SHK_AX = (-0.12, 0.58, 0.24), (-0.14, 0.60, -0.05)       # [EST]
# Rear link hardpoints (rear suspension frame). Chrono puts each link's COM at
# the midpoint of its two ends; the CG sums below use the same numbers.
TRI_CH,  TRI_AX  = (0.42, 0.30, 0.17), (0.0, 0.0, 0.14)              # [EST]
LONG_CH, LONG_AX = (0.52, 0.52, -0.06), (-0.30, 0.56, -0.08)         # [EST]
def _mid(a, b, i):
    return 0.5 * (a[i] + b[i])

# MOTION RATIOS, measured: spring-length change per unit of wheel travel.
# Front: rotate the lower arm a hair about its pivot line; wheel travel is
# taken at the lower ball joint. Rear, heave: the whole axle moves vertically.
_da = 1e-5
_lbj2 = rot_x(LBJ, LCA_AXIS[0], LCA_AXIS[1], _da)
MR_F = (abs(seg_len(F_SPR_CH, rot_x(F_SPR_ARM, LCA_AXIS[0], LCA_AXIS[1], _da))
            - seg_len(F_SPR_CH, F_SPR_ARM)) / abs(_lbj2[2] - LBJ[2]))
def _mr_vertical(top, bot, dz=1e-5):
    return abs(seg_len(top, (bot[0], bot[1], bot[2] + dz)) - seg_len(top, bot)) / dz
MR_R, MR_R_SHK = _mr_vertical(R_SPR_CH, R_SPR_AX), _mr_vertical(R_SHK_CH, R_SHK_AX)
N("DERIV", f"motion ratios, measured on the hardpoints: front {MR_F:.3f} "
           f"(spring at {ARM_FRAC_F:.0%} of the lower arm), rear {MR_R:.3f}, rear shock {MR_R_SHK:.3f}")

k_spring_f = k_wheel_f / MR_F ** 2
k_spring_r = k_wheel_r / MR_R ** 2
c_shock_f  = c_wheel_f / MR_F ** 2          # co-located with the spring
c_shock_r  = c_wheel_r / MR_R_SHK ** 2
N("DERIV", f"springs {k_spring_f:,.0f} N/m front, {k_spring_r:,.0f} N/m rear; "
           f"shocks {c_shock_f:,.0f} / {c_shock_r:,.0f} Ns/m (zeta {ZETA_F} / {ZETA_R})")

L_inst_f = seg_len(F_SPR_CH, F_SPR_ARM)     # at design ride height
L_inst_r = seg_len(R_SPR_CH, R_SPR_AX)
F_static_f = (SPRUNG_F / 2) * 9.81 / MR_F   # along the spring: the SPRUNG corner load
F_static_r = (SPRUNG_R / 2) * 9.81 / MR_R
free_len_f = L_inst_f + F_static_f / k_spring_f
free_len_r = L_inst_r + F_static_r / k_spring_r
BUMP_TRAVEL, REBOUND_TRAVEL = 0.090, 0.100  # [EST] m of wheel travel, design -> stop
stops_f = (L_inst_f - BUMP_TRAVEL * MR_F, L_inst_f + REBOUND_TRAVEL * MR_F)
stops_r = (L_inst_r - BUMP_TRAVEL * MR_R, L_inst_r + REBOUND_TRAVEL * MR_R)
N("DERIV", f"front spring {L_inst_f:.4f} m at design carrying {F_static_f:,.0f} N -> free "
           f"{free_len_f:.4f} m, stops {stops_f[0]:.4f}..{stops_f[1]:.4f}")
N("DERIV", f"rear  spring {L_inst_r:.4f} m at design carrying {F_static_r:,.0f} N -> free "
           f"{free_len_r:.4f} m, stops {stops_r[0]:.4f}..{stops_r[1]:.4f}")

# ---- chassis CG solved so the ASSEMBLED vehicle hits the target -----------
# Setting the chassis CG at the target longitudinal position is not enough: the
# 110 kg live rear axle and the rear unsprung corners sit at x = -wheelbase and
# drag the assembled CG backwards. First attempt asked for 53/47 and measured
# 48.9/51.1. Solve for the chassis CG that makes the SUM land where we want.
#   m_total * x_target = m_chassis * x_chassis + sum(m_i * x_i)
# ---- chassis mass: whatever is left after every other subsystem -----------
# Chrono sums the masses of every body it creates. Specifying the chassis as
# the full kerb mass double-counts spindles, arms, axle, wheels and tyres.
other = (2*(SPINDLE_F+UPRIGHT+UCA_M+LCA_M)          # front corners
         + 2*SPINDLE_R + AXLE_TUBE + TRI_M + 2*LONG_M   # rear
         + STEER_LINK_M + 4*WHEEL_M + 4*TYRE_M)
chassis_mass = KERB_MASS - other
N("DERIV", f"subsystem masses total {other:.0f} kg -> chassis body "
           f"{chassis_mass:.0f} kg (sums to the {KERB_MASS:.0f} kg kerb mass)")

m_front_uns = 2*(SPINDLE_F+UPRIGHT+UCA_M+LCA_M) + 2*(WHEEL_M+TYRE_M)   # at x=0
m_rear_uns  = 2*SPINDLE_R + AXLE_TUBE + TRI_M + 2*LONG_M + 2*(WHEEL_M+TYRE_M)
moment_uns  = (m_front_uns*0.0 + m_rear_uns*(-WHEELBASE) + STEER_LINK_M*(-0.145)
               + TRI_M*_mid(TRI_CH, TRI_AX, 0) + 2*LONG_M*_mid(LONG_CH, LONG_AX, 0))  # link COMs sit off the axle
CG_X_CHASSIS = (KERB_MASS*CG_X - moment_uns) / chassis_mass
N("DERIV", f"unsprung {m_front_uns:.0f} kg front + {m_rear_uns:.0f} kg rear pull the CG back; "
           f"chassis CG must sit at x={CG_X_CHASSIS:.4f} for the assembly to reach "
           f"x={CG_X:.4f} ({WDIST_F*100:.0f}% front)")

# CG HEIGHT, in the right frame (task 14). CG_Z is metres above GROUND; the
# chassis frame's origin is at wheel-centre height (every suspension frame puts
# z = 0 at the wheel centre), so writing CG_Z straight into the centroidal
# frame -- as this did until task 14 -- put the body's CG R_LOADED too high:
# the assembled car's CG was 0.82 m up at design ride height, not 0.62, and
# every load transfer a third too big. Solve, as for x, for the body CG that
# puts the ASSEMBLY at CG_Z.
z_uns = (2*(UCA_M*UBJ[2] + LCA_M*LBJ[2])            # arm COMs (JSON), chassis-frame z
         + TRI_M*_mid(TRI_CH, TRI_AX, 2) + 2*LONG_M*_mid(LONG_CH, LONG_AX, 2)
         + STEER_LINK_M*RACK_Z)                     # everything else at z = 0
CG_Z_CHASSIS = (KERB_MASS*(CG_Z - R_LOADED) - z_uns) / chassis_mass
N("DERIV", f"chassis CG at z={CG_Z_CHASSIS:.4f} in the chassis frame (wheel-centre height) "
           f"puts the assembly {CG_Z:.2f} m above ground")

# ===========================================================================
#  5. EMIT JSON
# ===========================================================================
def W(name, obj):
    p = os.path.join(OUT, name)
    with open(p, "w") as f:
        json.dump(obj, f, indent=2)
    return name

def inertia_box(m, lx, ly, lz):
    """Solid-cuboid inertia; good enough for links whose exact shape is unknown."""
    return [m*(ly**2+lz**2)/12, m*(lx**2+lz**2)/12, m*(lx**2+ly**2)/12]

# --- chassis ---------------------------------------------------------------
# Ladder-frame MPV: model as a cuboid of the body's outer dimensions. Iyy
# dominates pitch response and Izz dominates yaw; both matter for the test.
chassis_m = chassis_mass
Ixx, Iyy, Izz = inertia_box(chassis_m, LENGTH*0.85, WIDTH*0.90, HEIGHT*0.55)
W("innova_Chassis.json", {
    "Name": "Innova Reborn Chassis", "Type": "Chassis", "Template": "RigidChassis",
    "Components": [{
        "Centroidal Frame": {"Location": [CG_X_CHASSIS, 0, round(CG_Z_CHASSIS, 4)], "Orientation": [1,0,0,0]},
        "Mass": chassis_m,
        "Moments of Inertia": [Ixx, Iyy, Izz],
        "Products of Inertia": [0, 0, 0],
        # REQUIRED. RigidChassis::Create calls comp["Void"].GetBool() with no
        # HasMember guard, so omitting this segfaults inside the JSON parser
        # rather than producing a readable error.
        "Void": False,
    }],
    # The seat above (EYE_*). Chrono uses it for its own driver's-eye camera,
    # not for dynamics. AChronoVehicleActor READS it (task 9) for its camera
    # and for aiming the mirrors, so this is the one place the seat is set.
    # Chassis frame: the origin is at wheel-centre height, R_LOADED up.
    # (It was [CG_X+0.45, -0.38, 0.97] while the editor's eye sat elsewhere.)
    "Driver Position": {"Location": [EYE_X, EYE_Y, round(EYE_H - R_LOADED, 4)],
                        "Orientation": [1,0,0,0]},
    "Visualization": {"Primitives": [{"Type": "Box",
        "Location": [CG_X, 0, 0.62], "Orientation": [1,0,0,0],   # visual only, not the CG
        "Dimensions": [LENGTH*0.85, WIDTH*0.90, HEIGHT*0.55]}]},
})

# --- front double wishbone -------------------------------------------------
W("innova_FrontSusp.json", {
    "Name": "Innova Front Double Wishbone", "Type": "Suspension",
    "Template": "DoubleWishbone",
    "Camber Angle (deg)": -0.5, "Toe Angle (deg)": 0.1,
    "Spindle": {"Mass": SPINDLE_F, "COM": [0, HALF_F, 0],
                "Inertia": [0.04, 0.07, 0.04], "Radius": 0.10, "Width": 0.04},
    "Upright": {"Mass": UPRIGHT, "COM": [0, HALF_F-0.09, 0],
                "Moments of Inertia": [0.10, 0.10, 0.06],
                "Products of Inertia": [0,0,0], "Radius": 0.03},
    "Upper Control Arm": {"Mass": UCA_M, "COM": [0, (UBJ[1]+UCA_F[1])/2, UBJ[2]],
                "Moments of Inertia": inertia_box(5.5, 0.30, 0.30, 0.05),
                "Products of Inertia": [0,0,0], "Radius": 0.02,
                "Location Chassis Front": list(UCA_F), "Location Chassis Back": list(UCA_B),
                "Location Upright": list(UBJ)},
    "Lower Control Arm": {"Mass": LCA_M, "COM": [0, (LBJ[1]+LCA_F[1])/2, LBJ[2]],
                "Moments of Inertia": inertia_box(9.0, 0.37, 0.39, 0.05),
                "Products of Inertia": [0,0,0], "Radius": 0.03,
                "Location Chassis Front": list(LCA_F), "Location Chassis Back": list(LCA_B),
                "Location Upright": list(LBJ)},
    "Tierod": {"Location Chassis": list(tie_ch), "Location Upright": list(tie_up)},
    "Spring": {"Location Chassis": [round(v, 4) for v in F_SPR_CH],
               "Location Arm": [round(v, 4) for v in F_SPR_ARM],
               "Free Length": round(free_len_f,4), "Spring Coefficient": round(k_spring_f,1),
               "Minimum Length": round(stops_f[0],4), "Maximum Length": round(stops_f[1],4)},
    "Shock": {"Location Chassis": [round(v, 4) for v in F_SPR_CH],
              "Location Arm": [round(v, 4) for v in F_SPR_ARM],
              "Damping Coefficient": round(c_shock_f,1)},
    "Axle": {"Inertia": 0.35},
})

# --- rear solid three-link axle --------------------------------------------
# The real car is a 4-link with a lateral (Panhard) rod. Chrono's closest
# template is SolidThreeLinkAxle: two longitudinal links locating the axle
# fore-aft plus one triangulated link taking lateral load. The essential
# kinematics -- axle wind-up under torque, roll steer, lateral location -- are
# captured; the redundancy of the real 4-link is not.
W("innova_RearSusp.json", {
    "Name": "Innova Rear Solid Axle (4-link approximated as three-link)",
    "Type": "Suspension", "Template": "SolidThreeLinkAxle",
    "Spindle": {"Mass": SPINDLE_R, "COM": [0, HALF_R, 0],
                "Inertia": [0.03, 0.06, 0.03], "Radius": 0.10, "Width": 0.04},
    "Axle Tube": {"Mass": AXLE_TUBE, "COM": [0,0,0],
                  "Inertia": [round(AXLE_TUBE*TRACK_R**2/12,2), 1.2,
                              round(AXLE_TUBE*TRACK_R**2/12,2)],
                  "Radius": 0.045},
    "Triangular Link": {"Mass": TRI_M, "Inertia": [0.12, 0.12, 0.12],
                        "Location Chassis": list(TRI_CH), "Location Axle": list(TRI_AX)},
    "Longitudinal Link": {"Mass": LONG_M, "Inertia": [0.25, 0.05, 0.25],
                          "Location Chassis": list(LONG_CH), "Location Axle": list(LONG_AX)},
    "Spring": {"Location Chassis": list(R_SPR_CH), "Location Axle": list(R_SPR_AX),
               "Free Length": round(free_len_r,4), "Spring Coefficient": round(k_spring_r,1),
               "Minimum Length": round(stops_r[0],4), "Maximum Length": round(stops_r[1],4)},
    "Shock": {"Location Chassis": list(R_SHK_CH), "Location Axle": list(R_SHK_AX),
              "Damping Curve Data": [[-1.0, -c_shock_r*1.0],[-0.3, -c_shock_r*0.35],
                                     [0.0, 0.0],
                                     [0.3, c_shock_r*0.18],[1.0, c_shock_r*0.5]]},
    "Axle": {"Inertia": 0.35},
})

# --- steering --------------------------------------------------------------
W("innova_Steering.json", {
    "Name": "Innova Rack and Pinion", "Type": "Steering", "Template": "RackPinion",
    "Steering Link": {"Mass": STEER_LINK_M, "Inertia": [0.6, 0.02, 0.6],
                      "COM": 0.0, "Radius": 0.02, "Length": 2*RACK_HALF},
    "Pinion": {"Radius": round(pinion_r, 6),
               "Maximum Angle (deg)": round(sw_max_deg, 1)},
})

# --- wheels, brakes --------------------------------------------------------
W("innova_Wheel.json", {"Name":"Innova Wheel","Type":"Wheel","Template":"Wheel",
    "Mass": WHEEL_M, "Inertia": [0.35, 0.62, 0.35], "Radius": 0.203, "Width": 0.205})
# Brake torque sized to lock the wheels: mu*N*r at ~1g with load transfer.
brake_f = 1.1 * (M_F/2 + 250) * 9.81 * R_LOADED
brake_r = 1.1 * (M_R/2 - 150) * 9.81 * R_LOADED
N("DERIV", f"brake torque front {brake_f:,.0f} Nm, rear {brake_r:,.0f} Nm "
           f"(sized to lock at ~1 g including load transfer)")
# BRAKE_MODEL (task 15): "simple" is ChBrakeSimple, a ChLinkLockBrake whose
# sticking logic is dead code in Chrono 10 -- `must_stick` is set and never
# read -- so at rest it is a bare Coulomb torque, -T*sign(w), flipping sign
# every step: a braked wheel CHATTERS instead of holding. "shafts" is
# ChBrakeShafts, a ChShaftsClutch between the axle shaft and an
# upright-fixed brake shaft: under a VI solver (task 13) it holds rigidly
# while the demand is inside braking*T_max and slips at the limit beyond it,
# which is what pad friction does. Its JSON needs "Shaft Inertia" (read with
# no HasMember guard -- omit it and Chrono segfaults); 0.4 is Chrono's generic.
#
# DEFAULT "shafts" since task 15, MEASURED: a braked wheel at rest is stuck
# (0.004 rad/s, no sign flips; "simple" chattered at 0.36 rad/s, flipping
# every step), and the car stopped on 12/25/45% holds on the brake alone
# (1-4 mm of tyre pre-slide, then still) while 120% -- past friction --
# slides. "simple" is kept to reproduce the old car.
BRAKE_MODEL = os.environ.get("BRAKE_MODEL", "shafts").lower()
if BRAKE_MODEL not in ("simple", "shafts"):
    sys.exit(f"BRAKE_MODEL must be simple or shafts, not {BRAKE_MODEL!r}")
_brake = ({"Template": "BrakeShafts", "Shaft Inertia": 0.4} if BRAKE_MODEL == "shafts"
          else {"Template": "BrakeSimple"})
W("innova_BrakeFront.json", {"Name":"Innova Front Brake","Type":"Brake",
    **_brake, "Maximum Torque": round(brake_f,1)})
W("innova_BrakeRear.json", {"Name":"Innova Rear Brake","Type":"Brake",
    **_brake, "Maximum Torque": round(brake_r,1)})

# --- powertrain ------------------------------------------------------------
W("innova_Engine.json", {
    "Name": "Toyota 2GD-FTV 2.4L Turbodiesel", "Type": "Engine",
    "Template": "EngineShafts",
    "Motor Block Inertia": 0.6, "Motorshaft Inertia": 0.5,
    "Torque Map": [[a, round(b,1)] for a, b in torque_map],
    "Losses Map": [[a, round(b,1)] for a, b in losses_map],
    # Not a Chrono key: EngineShafts has no idle governor, and Chrono ignores
    # members it does not read. The shim's governor reads it (task 8), so the
    # idle speed has ONE source, this file.
    "Idle Speed [rpm]": IDLE_RPM,
})
W("innova_Transmission.json", {
    "Name": "Innova 5-Speed Manual", "Type": "Transmission",
    "Template": "ManualTransmissionShafts",
    "Transmission Block Inertia": 0.4, "Input Shaft Inertia": 0.3,
    "Motorshaft Inertia": 0.5, "Driveshaft Inertia": TR_DS_INERTIA,
    "Clutch Torque Limit": 600.0,
    "Gear Box": {"Reverse Gear Ratio": 1.0/REV_GEAR,
                 "Forward Gear Ratios": [round(1.0/g, 6) for g in GEARS]},
})
# NOTE the exact key names. ShaftsDriveline2WD::Create reads
# ["Shaft Direction"]["Motor Block"] and ["Shaft Inertia"]["Driveshaft"] with
# no HasMember guard -- older Chrono spellings ("Motorblock Direction",
# "Driveshaft Inertia") segfault inside the parser instead of erroring.
W("innova_Driveline.json", {
    "Name": "Innova RWD Driveline", "Type": "Driveline",
    "Template": "ShaftsDriveline2WD",
    "Shaft Direction": {"Motor Block": [1,0,0], "Axle": [0,1,0]},
    "Shaft Inertia": {"Driveshaft": 0.5, "Differential Box": 0.6},
    "Gear Ratio": {"Conical Gear": 1.0/FINAL_DRIVE},
})

# --- tyres -----------------------------------------------------------------
# TMeasy: semi-empirical, parameterised by load and the friction/slip curve.
# Cheap enough for the 1 kHz loop -- this is the runtime tyre.
FZ_NOM_F = M_F/2 * 9.81
FZ_NOM_R = M_R/2 * 9.81

# --- parking torque (task 16) -------------------------------------------------
# TMeasy has no parking (bore) torque, so the shim adds one to the FORCE
# FEEDBACK only: a twist state per front tyre, elastic until the patch slides
# at M_max = mu Fz rbar, relaxed as the wheel rolls (chrono_shim.cpp,
# ParkingTorque). Two numbers come from here; the patch length, the load and
# mu are read live or from this same file.
#
# The twist stiffness follows from TMeasy's OWN cornering stiffness through
# the brush model: uniform bristles k per unit area over a 2a x 2b patch give
#     C_Falpha = 4 k b a^2      (cornering stiffness)
#     K_psi    = k 4ab (a^2 + b^2)/3 = C_Falpha (a^2 + b^2) / (3a)
# so the parked tyre twists with the stiffness the rolling tyre corners with.
# TMeasy's Dahl standstill bristle (sigma0 = 1e5 N/m) would give ~10x softer,
# and the patch would not slide before full lock; no published twist
# stiffness for this size was found. The real car's standstill rim torque
# (Q1 item 4.3) is what would replace this.
#
# C_Falpha is reproduced from ChTMeasyTire::GuessPassCar70Par (Chrono 10):
# pn = tireLoad/2 (the load-index path sets the inflation ratio to 1),
# dfy0 = InterpQ(15.9826 pn, 12.8509 * 2 pn) at q = Fz/pn, and the deflection
# from CZ = tireLoad / (0.16 x section height). Contact length as TMeasy's own
# P_len = 2 sqrt(R0 depth).
LOAD_INDEX = 95                          # [SPEC] 205/65 R16 95H
LI_KG      = 690.0                       # [SPEC] load index 95
PATCH_W    = 0.16                        # [EST] tread contact width, ~0.78 x 205 mm
def parking_torque(fz):
    tire_load = LI_KG * 9.81
    pn = 0.5 * tire_load
    q = fz / pn
    w1, w2 = 15.9826 * pn, 12.8509 * 2.0 * pn
    dfy0 = q * (2.0 * w1 - 0.5 * w2 - (w1 - 0.5 * w2) * q)          # N/rad
    depth = fz / (tire_load / (0.16 * TYRE_W * TYRE_AR))
    a, b = math.sqrt(R_UNLOADED * depth), PATCH_W / 2.0
    k_psi = dfy0 * (a * a + b * b) / (3.0 * a)
    N("DERIV", f"parking torque at {fz:.0f} N: C_Falpha {dfy0/1e3:.1f} kN/rad, "
               f"patch {2*a*100:.1f} x {PATCH_W*100:.0f} cm -> twist stiffness "
               f"{k_psi:.0f} N m/rad")
    return {"Twist Stiffness [Nm/rad]": round(k_psi, 1),
            "Patch Width [m]": PATCH_W}

def tmeasy(name, fz):
    """
    TMeasy from LOAD INDEX rather than hand-written coefficients.

    Given Design + Vehicle Type + Load Index, TMeasy synthesises a complete,
    physically consistent parameter set (slip stiffnesses, peak and sliding
    forces at nominal and double-nominal load). That is far more defensible
    than inventing Pacejka-style numbers for which no measured source exists.

    205/65 R16 95H: load index 95 = 690 kg per tyre, comfortably above the
    477 kg front corner load.

    NOTE the "[unit]" suffixes. TMeasyTire::Create reads
    d["Design"]["Unloaded Radius [m]"] and friends with no HasMember guard, so
    dropping the suffix segfaults inside the parser rather than erroring.
    """
    return {"Name": name, "Type": "Tire", "Template": "TMeasyTire",
        "Design": {
            "Unloaded Radius [m]": round(R_UNLOADED, 5),
            "Mass [kg]": TYRE_M,
            "Inertia [kg.m2]": [0.35, 0.62, 0.35],
            "Width [m]": TYRE_W,
            "Rim Radius [m]": RIM_IN*0.0254/2,
            "Roundness of Cross Section": 0.1,
        },
        "Coefficient of Friction": 0.85,
        "Rolling Resistance Coefficient": ROLL_RES,
        "Vehicle Type": "Passenger",
        "Load Index": LOAD_INDEX,
        "Visualization": {"Width": TYRE_W, "Radius": round(R_UNLOADED,5),
                          "Rim Radius": RIM_IN*0.0254/2},
        # Not a Chrono key: the shim reads it (task 16), Chrono ignores it.
        "Parking Torque": parking_torque(fz),
    }
def fiala(name, fz):
    """
    Fiala, as an ALTERNATIVE to TMeasy -- selected with TYRE_MODEL=fiala.

    WHY THIS EXISTS: Chrono's TMeasy documents "No parking slip calculations /
    No dynamic parking torque / Simplified stand still handling", and the
    measured consequence is a 0.17 m/s phantom creep on flat ground from rest
    (docker/05-verify-vehicle-shim.sh creep). That is fatal to the hill-start
    rollback rule, which is a 0.30 m threshold on exactly that quantity.

    Fiala is the candidate because it still computes an ALIGNING MOMENT --
    ChFialaTire::FialaPatchForces(fx, fy, mz, ...) -- so unlike RigidTire it
    does not destroy the Milestone 5 force-feedback argument. Pac02 was ruled
    out on data grounds: it wants an ADAMS/Car .tir Magic Formula set of about
    a hundred coefficients, and no source for a 205/65R16 exists here. That is
    the M4 hardpoint problem again, and inventing them would be worse than not
    having them.

    Unlike TMeasy, Fiala CANNOT synthesise itself from a load index. Every
    coefficient below is [EST], derived from standard passenger-tyre ratios to
    the static corner load rather than measured. That is a real fidelity step
    DOWN from the TMeasy set, and it is the price of the experiment.
    """
    # Vertical rate, chosen so static deflection matches the 3% that
    # R_LOADED = R_UNLOADED * 0.97 already assumes elsewhere in this file.
    # Deriving it keeps ride height consistent between the two tyre models,
    # so a creep comparison is not confounded by the car sitting differently.
    defl = 0.03 * R_UNLOADED
    k_z  = fz / defl
    # Heavily damped on purpose: contact must stay stable at a 1 ms step.
    c_z  = math.sqrt(k_z * UNSPRUNG_F)
    return {"Name": name, "Type": "Tire", "Template": "FialaTire",
        "Mass": TYRE_M, "Inertia": [0.35, 0.62, 0.35],
        "Coefficient of Friction": 0.85,
        "Fiala Parameters": {
            "Unloaded Radius": round(R_UNLOADED, 5),
            "Width": TYRE_W,
            "Vertical Stiffness": round(k_z, 1),
            "Vertical Damping": round(c_z, 1),
            "Rolling Resistance": ROLL_RES,
            # Longitudinal slip stiffness ~15x and cornering stiffness ~12x the
            # vertical load are the usual passenger-tyre ratios. [EST].
            "CSLIP": round(15.0 * fz, 1),
            "CALPHA": round(12.0 * fz, 1),
            "UMIN": 0.65,
            "UMAX": 0.85,
            # Relaxation length. A passenger tyre is 0.5-1.0 m; the HMMWV set
            # ships 2.0 for a much larger tyre.
            "X Relaxation Length": 0.6,
            "Y Relaxation Length": 0.6,
        },
    }


def rigid(name, fz):
    """
    RigidTire -- a rigid cylinder in real frictional contact with the terrain.

    The ONLY model measured here that holds a static load at zero speed, because
    it is the only one that is not a force element: it does not derive
    longitudinal force from slip, it collides. That is why it does not creep.

    The price is that it produces NO aligning moment, so it destroys the
    Milestone 5 force-feedback signal entirely, and its grip transition is
    Coulomb rather than a slip curve -- sharper and more binary than a real
    tyre near the limit.

    Kept as a generated variant rather than a hand-edited file so the hill
    experiment is repeatable: TYRE_MODEL=rigid python3 gen_innova.py
    """
    return {"Name": name, "Type": "Tire", "Template": "RigidTire",
        "Radius": round(R_UNLOADED, 5), "Width": TYRE_W,
        "Mass": TYRE_M, "Inertia": [0.35, 0.62, 0.35],
        "Contact Material": {
            "Coefficient of Friction": 0.85,
            "Coefficient of Restitution": 0.1,
            "Properties": {"Young Modulus": 2e7, "Poisson Ratio": 0.3},
            "Coefficients": {"Normal Stiffness": 2e5, "Normal Damping": 40.0,
                             "Tangential Stiffness": 2e5, "Tangential Damping": 20.0},
        },
    }


TYRE_MODEL = os.environ.get("TYRE_MODEL", "tmeasy").lower()
if TYRE_MODEL == "fiala":
    N("EST", "TYRE_MODEL=fiala -- every Fiala coefficient is [EST], unlike the "
             "TMeasy set which synthesises itself from the load index.")
    W("innova_TireF.json", fiala("Innova 205/65R16 Fiala Front", FZ_NOM_F))
    W("innova_TireR.json", fiala("Innova 205/65R16 Fiala Rear",  FZ_NOM_R))
elif TYRE_MODEL == "tmeasy":
    W("innova_TireF.json", tmeasy("Innova 205/65R16 TMeasy Front", FZ_NOM_F))
    W("innova_TireR.json", tmeasy("Innova 205/65R16 TMeasy Rear",  FZ_NOM_R))
elif TYRE_MODEL == "rigid":
    N("EST", "TYRE_MODEL=rigid -- NO aligning moment, so no force feedback. "
             "For the standstill experiment only.")
    W("innova_TireF.json", rigid("Innova Rigid Front", FZ_NOM_F))
    W("innova_TireR.json", rigid("Innova Rigid Rear",  FZ_NOM_R))
else:
    sys.exit(f"TYRE_MODEL={TYRE_MODEL!r} unknown; use tmeasy, fiala or rigid")

# --- an ANCF 205/65 R16, for the offline rig only (Q1) -----------------------
# NOT used by the car. test_innova's ancfland/ancfpark/ancfroll modes take it
# as `innova`. No ply or material data is published for any 205/65 R16, so this
# is what a no-data parameterisation looks like, kept so that it can be
# MEASURED against the only targets there are (CLAUDE.md, Q1 item 4):
#   [DERIV] geometry from the size code: rim radius, section height, width
#   [EST]   construction: Chrono's HMMWV ANCF-4 (lumped) plies and materials,
#           read from the SDK's copy of Chrono 10's data, unchanged. A 37x12.5
#           R16.5 military tyre's carcass on a passenger car's cross-section.
#   [EST]   inflation 230 kPa, a placard-typical figure for this size
#   [EST]   mesh 60 x 12: 3.5 cm elements round the circumference, so a ~13 cm
#           contact patch spans ~4 node rows
ANCF_SRC = os.path.join(os.path.dirname(OUT), "..", "chrono-sdk", "share", "chrono",
                        "data", "vehicle", "hmmwv", "tire", "HMMWV_ANCF4Tire_Lumped.json")
def ancf(name):
    src = json.load(open(ANCF_SRC))
    rim_r = RIM_IN * 0.0254 / 2
    height = R_UNLOADED - rim_r                  # section height, 0.133 m
    # The HMMWV profile runs from bead to bead in (radial, lateral) metres.
    # Scale it to this section: radial by height, lateral by width.
    px = [p[1] for p in src["Profile"]]
    py = [p[2] for p in src["Profile"]]
    sx = height / max(px)
    sy = (TYRE_W / 2) / max(abs(v) for v in py)
    prof = [[round(p[0], 6), round(p[1] * sx, 6), round(p[2] * sy, 6)] for p in src["Profile"]]
    out = dict(src)
    out.update({"Name": name, "Tire Radius": round(R_UNLOADED, 5), "Rim Radius": rim_r,
                "Rim Width": round(TYRE_W * 0.8, 4), "Default Pressure": 230.0e3,
                "Number Elements Circumference": 60, "Profile": prof})
    for sec, n in (("Bead Section", 1), ("Sidewall Section", 2), ("Tread Section", 3)):
        out[sec] = dict(src[sec], **{"Number Elements": n})
    return out
if os.path.exists(ANCF_SRC):
    W("innova_TireANCF.json", ancf("Innova 205/65R16 ANCF-4 (HMMWV construction) [Q1 rig only]"))
    N("EST", "innova_TireANCF.json: 205/65R16 geometry on Chrono's HMMWV ANCF plies "
             "-- offline rig only, never the car (Q1)")
else:
    N("SKIP", f"innova_TireANCF.json not written: {ANCF_SRC} missing (export the SDK)")

# --- vehicle ---------------------------------------------------------------
P = "innova/"
W("innova_Vehicle.json", {
    "Name": "Toyota Kijang Innova Reborn 2.4 G MT",
    "Type": "Vehicle", "Template": "WheeledVehicle",
    "Chassis": {"Input File": P+"innova_Chassis.json"},
    "Axles": [
      {"Suspension Input File": P+"innova_FrontSusp.json",
       "Suspension Location": [0, 0, 0], "Steering Index": 0,
       "Left Wheel Input File": P+"innova_Wheel.json",
       "Right Wheel Input File": P+"innova_Wheel.json",
       "Left Brake Input File": P+"innova_BrakeFront.json",
       "Right Brake Input File": P+"innova_BrakeFront.json"},
      {"Suspension Input File": P+"innova_RearSusp.json",
       "Suspension Location": [-WHEELBASE, 0, 0],
       "Left Wheel Input File": P+"innova_Wheel.json",
       "Right Wheel Input File": P+"innova_Wheel.json",
       "Left Brake Input File": P+"innova_BrakeRear.json",
       "Right Brake Input File": P+"innova_BrakeRear.json"},
    ],
    "Steering Subsystems": [
      {"Input File": P+"innova_Steering.json",
       "Location": [-0.145, 0, -0.10], "Orientation": [1,0,0,0]},
    ],
    # "Suspension Indexes" lists which axles the driveline drives, and it lives
    # HERE, not in the driveline file. WheeledVehicle::Create reads
    # d["Driveline"]["Suspension Indexes"] unguarded; omitting it leaves the
    # driven-axle list empty and ChShaftsDriveline2WD::Initialize segfaults.
    # [1] = rear axle only: the Innova is RWD.
    "Driveline": {"Input File": P+"innova_Driveline.json",
                  "Suspension Indexes": [1]},
    "Wheelbase": WHEELBASE,
    "Minimum Turning Radius": TURN_RADIUS,
    "Maximum Steering Angle (deg)": round(math.degrees(DELTA_MAX),1),
})

print("=" * 74)
print(" Innova Reborn 2.4 G MT — Chrono::Vehicle model generated")
print("=" * 74)
for n in notes: print(n)
print(f"\n  wrote {len(os.listdir(OUT))} files to {OUT}")
