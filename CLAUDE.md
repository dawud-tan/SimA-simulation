# SIM A Driving Licence Simulator

High-fidelity simulation of the Indonesian SIM A practical driving test
(slalom, parallel parking, bay parking, hill start), built on Project Chrono
multibody dynamics rendered through Unreal Engine.

**The user is an experienced systems/low-level programmer who was new to both
Unreal and Chrono at the start of this project.** Explain UE and Chrono
concepts; do not explain threading, ABI, linking, or C++.

Work milestone by milestone. For each: physics theory → engine architecture →
minimal commented implementation → verification commands. Do not emit
monolithic code blocks.

---

## TASK BACKLOG

Ordered by leverage, not by size. Each is self-contained: a session can be
started with `proceed to "task <name>"` and this section is the brief.

**What "done" means here:** a measurement, not a compile. Every task below
names its verification command. If a claim is not backed by a number from one
of the harnesses, it is not finished. Every "Full account in ..." names a
section of `ENGINEERING_LOG.md`.

---

### 1. `mz-below-1ms` — restore steering feel at parking speed — **DONE**

Adopted `CHS_TIRE_BLEND_BEGIN = 0.20` (was Chrono's 1.0), baked into
`Chs_CreateVehicleEx`. Column torque at a 0.60 parking lock and 0.21 m/s goes
**−0.079 → −3.256 N·m, 41×**. `blend_end` stays at Chrono's 3.0; raising it was
measured, was better on ripple, and was **rejected** because it stops the car
launching on a hill. Full account in "Task 1: the aligning moment below 1 m/s"
in the log, including three things it turned up that are not this task.

New harness modes: `mzlow`, `blendhill`.

---

### 2. `mirror-reflection` — make the glass show the render target — **DONE**

All three mirrors MEASURED correct in the editor: coloured markers placed by
the geometry land within **0.17 px** of prediction in the render targets and
within **0.24 px** on the glass. Five faults stood between "black" and that —
culled glass, a dead `FlipU` parameter, a mirror aimed at the sky, a 90° UV
transpose, and Unreal's capture clip plane being inert all along — plus the
frustum itself, which was built in the wrong basis: `mm_ue_mirror_capture` now
builds the whole capture in Unreal's conventions, with 24 new host assertions
(`mirror/run.sh selftest`, 53 in all). **`bFlipOffAxisSkew` is deleted** — no
setting of it could ever have been right. **`bFlipMirrorU` is `false`**,
measured. The driver sits on the RIGHT. Full account in "Task 2: the mirror,
end to end" in the log.

**Verify.** `bash mirror/run.sh render` — three editor runs, one per mirror,
each with `-ChronoMirrorProbe`. Exits 0 only if the runtime frustum check
passes (corners on their NDC corners AND the glass clipped) and every marker
lands within 3 px of its predicted place in the target and within 6 px on the
glass.

---

### 3. `parking-completion` — detect a finished park — **DONE**

The referee judges every stop that lasts `REF_PARK_DWELL` inside a parking
zone — judging the pose recorded AT the stop, per the design note — and emits
`PARK_OK` / `PARK_OUTSIDE` / `PARK_ANGLE`. Zones reach it as `ZONE` records
with the heading a parked car must have (the bay is reversed into: nose-in is
180° out). 28 new host assertions (`referee/run.sh selftest`, 59 in all),
mutation-tested. End to end, a closed-loop pilot drives the real car in
Chrono: the bay and the parallel slot both judged **`PARK_OK`** (clearance
0.121 / 0.106 m), and a deliberately short bay stop **`PARK_OUTSIDE`**.

Planning that run turned up two defects that were not this task and are fixed:
**the course's exercises leaked lines into each other** (every correct bay park
crossed the parallel kerb), and **the editor fed the referee world-X velocity
as "speed"**, so a car reversing along world Y looked stopped. Full account in
"Task 3: parking completion" in the log.

**Verify.** `referee/run.sh selftest`, then
`docker/05-verify-vehicle-shim.sh judge park_bay|park_bay_short|park_parallel`.

---

### 4. `referee-verdict` — turn events into a result — **DONE**

The referee now produces a verdict. Each exercise goes ARMED → RUNNING when
the car crosses its entry gate, → PASSED by its own rule (exit gate; a
`PARK_OK` within 3 attempts; exit gate after a confirmed stop) and → FAILED
on its first fault — or when it is abandoned or skipped. The test PASSES only
if every exercise passes and nothing faulted outside them. Pure C in
`referee.cpp`, fed before the event log's capacity check, so a full log can
never change a result. 26 new host assertions (85 in all), 7 mutants.

End to end, a pilot now drives the WHOLE course in Chrono for the first time —
slalom, parallel park, bay park, hill start — and the referee returns
**PASS, 4 of 4**, with the same verdict digest from two separate processes.
`course_cone` and `course_nostop` FAIL for the right reason. Full account in
"Task 4: the verdict" in the log.

Driving it turned up the largest defect since Milestone 4: **every clutch in
the driveline is welded shut** — see task 13, which now leads the backlog.

**Verify.** `referee/run.sh selftest`, then
`docker/05-verify-vehicle-shim.sh verdict course` (two processes, digests
compared) and `... judge course_cone|course_nostop`.

---

### 13. `welded-clutch` — the solver cannot open a clutch — **DONE**

Production now runs **PSOR, 50 iterations, cold start**, a VI solver that
enforces a clutch's torque box, where MINRES welded every `ChShaftsClutch`
shut. `clutch` now asserts `CLUTCH WORKS` (pedal down: held, engine free at
3523 rpm) and `diff` asserts `OPEN` (inner/outer 0.7141 vs ground 0.7151).
Chosen by measurement against the EXACT direct solve: QR with an open diff and
converged PSOR agree to four figures in a steady turn, and PSOR-50 is within
0.2–0.6% of both. It is **cheaper** than MINRES: 136 vs 192 µs/step mean, max
340 vs 440.

It overturned three recorded conclusions:
- **The flat-ground creep was the engine, not the tyres.** Neutral and pedal
  down now stand still (−0.006 m in 20 s, was −3.8 m); in gear with the pedal
  up the engine's −20 N·m of losses at 0 rpm still drives the car backwards.
- **Hill free roll** improves from −50.4% to −36.7% of `g·sin θ`.
- **Parking-speed steering feel was mostly the spool**: column torque at
  0.30 m/s drops −3.256 → −0.330 N·m. The 8 m/s FFB curve is unchanged.

The whole test still PASSES 4/4, reproducibly (`7ba5b428b8a58ce5`). Full
account in "Task 13: the clutch, opened" in the log, with the defects it exposed:
the engine idles backwards (task 8), the parking hold freezes the suspension
(task 15), and task 14's weight split was a settling artefact.

**Verify.** `... clutch` and `... diff` (both now fail by exit code),
`... solvers`, then `determ`, `creep`, `hill`, `mzlow`, `sweep`,
`verdict course`. `CHS_SOLVER=minres` reproduced the old car exactly
(digest `c829c405af66692a`). at the time; since task 15 that also needs `BRAKE_MODEL=simple` and the hold
enabled, because a linear solver welds the new brake clutches too.

---

### 14. `static-attitude` — the car sat 4.4° nose-up — **DONE**

The car now sits at its DESIGN ride height, level, with the designed 53%
front: pitch −0.05° (was −4.37°), every corner within 4 mm of design (front
was +84 mm into bump, rear +289 mm), front 52.99% (was 51.2%). QR, the exact
solve, agrees to 0.4 mm. The new `attitude` mode asserts it. Four
derivation errors in `gen_innova.py`, not one: the front spring's motion
ratio was measured from the car's centreline instead of the arm's pivot
(0.46, not 0.70); the rear "spring curve" had its sign inverted, so the rear
spring PULLED; preload counted the unsprung weight; and `CG_Z`, metres above
GROUND, was written into a frame whose origin is at wheel-centre height, so
the assembled CG sat 0.82 m up instead of 0.62.

It changed more than the attitude. **Zero-steer drift fell 4×** (0.338 →
0.086 m over 32.7 m; the rear three-link's roll steer depends on ride
height). Full lock came out at 36.7° at the real ride height, so
`PINION_CAL` was refitted, 0.98 → 0.954 (35.36°, kerb-to-kerb 5.398 m). And
`sweep`'s "past grip" flag was exposed as kinematic. It now uses the measured
lateral acceleration, and the FFB cue is confirmed: the torque peaks at steer
0.60 and has fallen 17% when the car's grip peaks at 0.80, 0.77 g. The whole
test still PASSES 4/4 (`0b19f42c2e895875`). Full account in "Task 14: the car
sat on the wrong springs".

**Verify.** `docker/05-verify-vehicle-shim.sh attitude` (asserted), then
`vehicle/run.sh ackermann` (full lock 35.36°), `sweep`, `drift`, and
`verdict course`.

---

### 15. `planar-parking-hold` — the hold freezes the suspension — **DONE**

The brief's fix is in: when enabled, the hold opposes only the in-plane
velocity and is clamped to μ × the load the tyres actually carry. The new
`hold` harness, written first, found why the hold was needed at all. Chrono
10's `ChLinkLockBrake` **cannot stick**: its sticking logic is dead code, so a
braked wheel at rest chattered, flipping direction on 19 of 20 steps. The car
now uses **`BrakeShafts`**, a clutch-type brake that task 13's VI solver holds
rigidly below its torque limit. The brake alone now holds a car stopped on
12%, 25% and 45% (1–4 mm of tyre pre-slide, then still) and lets it slide at
120%. So the **parking hold is OFF by default**: with a working brake it only
disturbed the car.

It changed the hill: free ROLL from a braked stop is now **−16.7%** of
`g·sin θ` (was −36.7%). That is about what rolling resistance plus the rotating
masses predict (task 6). The whole test still PASSES 4/4. Full account in
"Task 15: the brake that could not hold" in the log.

**Verify.** `docker/05-verify-vehicle-shim.sh hold` (asserted: chatter,
flat, slope, grades, iterations), then `hill 0.12` and `verdict course`.

---

### 5. `hud` — tell the driver what is happening — **DONE**

`AChronoHUD`, a canvas HUD: code and engine fonts, no UMG asset. It shows
speed, gear, revs to the redline, the three pedals and the wheel as
commanded, a STALLED banner with the restart hint, and the four exercises
with their states. Exercise names are derived from the course data, so there
is no copied table. It also shows the referee's calls in words, the verdict,
and the 1 kHz loop's timing. Verified from rendered frames: `-ChronoShotUI`
saves the viewport as drawn.

Seeing it running found three defects the HUD did not cause:
- **Scripted drives ran out during the loading screen.** The physics thread
  starts with the world, up to 100 s before the first frame. A **session
  start** now holds a script at t = 0 until frames are steady (under 40 ms
  for 0.5 s), and the referee judges from that moment on.
- **The referee's drain skipped a sample whenever the physics thread
  published mid-drain**: 135 one-sample gaps in two minutes of play. It
  indexed the ring back from a head that moved under it. It now reads by
  absolute index (`ReadAt`). 1 gap in two minutes, and that one is the
  screenshot. A new host harness, `ring/run.sh`, reproduces the old bug
  (1,172 skips) and proves the fix (0 skipped, never a wrong or torn sample).
- **There was no sky.** Now an `ASkyAtmosphere`.

And the measurement tasks 13 and 15 left open: **the 1 kHz loop under
rendering** with PSOR, BrakeShafts and the governor, 110 s of steady play:
mean 205–220 µs (21% of the budget), max 1043 µs, **2 steps over budget in
~109,000**, lateness 0.8–1.1 µs mean, `rt=yes`. Full account in "Task 5: the
HUD, and what seeing the car found".

**Verify.** `bash ring/run.sh`; then the game offscreen with
`-ChronoShot=110 -ChronoShotUI -ChronoScript=<repo>/driver/scripts/circle.txt
-RenderOffscreen -unattended -ResX=1280 -ResY=720`: the image in
`Saved/Screenshots/*/ChronoHUD.png`, and the `physics:` and `SAMPLE_GAP`
lines in `Saved/Logs/SimA.log`.

---

### 6. `breakaway-from-rest` — why the tyre sticks — **DONE**

**The tyre does not stick.** The free-roll deficit is exactly rolling
resistance plus the masses that spin with the wheels, and the new `rollacct`
measures it term by term. The car stops on the slope the way a driver stops
there: driven up in 1st, braked, neutral. The acceleration is fitted where
the model is in its steady regime. On 12%: **0.9752 m/s² measured against
0.9770 predicted (−0.19%)** from gravity 2,104 N, TMeasy rolling resistance
213 N, and 1,800 + 135 kg. Each term was checked by moving it.
Regenerated with `ROLL_RES=0`, the prediction holds to +0.12%. With the pedal
down in each of 1st to 5th, the gearbox input shaft adds a known 29 to 775 kg,
and all five rows are predicted within ~1%.

It turned up one trap. The transmission's output-shaft "inertia" is **not
inertia**: counting it fits every row 3% worse. Chrono resets that shaft from
the driveline each step and passes only the gearbox reaction. It is a
stability parameter of that explicit exchange, and **diverges to NaN between
0.8 and 0.9 kg·m²**. The stock 0.5 has a 1.6× margin; do not raise it
(`TR_DS_INERTIA` exists to show this).

The flat-ground creep (5.5–5.9 mm/s) is **not** spawn memory, as task 14
guessed. It is identical after a straight drive and after a weave, identical
at PSOR 1000, and within 15% at a 0.5 ms step. It is a steady **~4 N push**
(measured with `ROLL_RES=0`: 0.002 m/s²), 0.02% of the car's weight, balanced
by TMeasy's near-zero rolling resistance. Mechanism unidentified. It is
bounded: ≤ 3.5 cm over a 5 s hill start, and irrelevant on a slope. Full
account in "Task 6: the free-roll account".

**Verify.** `docker/05-verify-vehicle-shim.sh rollacct` (asserted: the account
must close). With a regenerated car, pass what it was generated with:
`ROLL_RES=0 python3 vehicle/gen_innova.py` then `... rollacct 0.12 0.0`.

---

### 7. `rollback-rule` — the hill-start displacement threshold — **DONE**

The referee now fails a hill start that rolls back. **The regulation allows
no rollback** ("tidak ada reaksi kendaraan mundur", task 11), so
`REF_ROLLBACK_FLOOR` (5 cm) is a detection floor, not a tolerance. It counts
backward travel after any stop inside the hill exercise, as **the WHEELS'
travel, not the body's**. Measured on the course's 15°: a car braked firmly
to a stop settles back **5.8–7.6 cm on its tyres' compliance with its wheels
locked**. A floor on body travel would have to sit above that, and would miss
real rollbacks. Its wheels move only 1.2–1.3 cm. The pilot's clutch start
rolls the wheels back 0.8 cm and PASSES. A bad start fails with `ROLLBACK`:
brake released at half the holding capacity, half a second to react
(`course_rollback`), 12.1 cm. 13 new host tests (98 in all), 5 mutants
caught. `ref_wheel_speed` handles the snapshot's mixed frames (spindles in
world axes, chassis in its own).

**Verify.** `referee/run.sh selftest`, `... rollback` (asserted: the held
car's wheel settle is under half the floor), `... verdict course` (PASS),
`... judge course_rollback` (must FAIL, rollback).

---

### 8. `idle-governor` — stop the engine bogging — **DONE**

The engine now idles, drives and stalls like a diesel. The losses map is in
rpm and passes through zero at rest, opposing rotation either way; its
x-values had been in rad/s and it gave −20 N·m at 0 rpm. The shim adds what
Chrono's `EngineShafts` lacks. An **idle governor**: fuel = max(pedal,
(idle − rpm)/200 rpm), idle read from the engine JSON. A **stall**: without
one, the governor made the engine unstallable, pushing 250 N·m at 0 rpm.
Now it dies below 300 rpm and restarts after 1 s with the pedal down or in
neutral, the ABI's proxy for the key.
`CHS_IDLE_GOVERNOR=0` is the pre-task-8 engine. Declutched it idles at **783
rpm** (was −98). In 1st with the pedal up the car **idles forward at 1.60 m/s**
(predicted 1.59), where it crept backwards at 0.18 m/s. On the brakes it
**stalls and holds**. The hill `LAUNCH` now **pulls away**. **The brief's
verify criterion was wrong**: "1st, pedal up, must stop moving" predates the
governor, and a governed diesel idles forward. That is what is asserted.

Because the car idles along at 1.6 m/s, the **pilot now crawls on the
clutch** below idle speed, as a driver does: throttle off, a PI loop on
clutch capacity. The hill start is done the same way. The launch-floor
workaround is gone. `mzlow` crawls the same way; its sub-1.2 m/s probes had
silently run at 1.16. The whole test still PASSES 4/4 (`1e8c06f3ddc60388`),
never below 654 rpm with the clutch carrying torque, every leg within 1.5 cm of its planned end except the
parallel park's 9.0 cm, as before. Full account in "Task 8: an engine that
idles, and stalls".

**Verify.** `creep` (idle creep and the stall asserted), `assemble` (783
rpm), `clutch`, `neutral`, `hill 0.12`, `verdict course`.

---

### 9. `cockpit` — an interior worth driving from — **DONE**

The car is a shell to sit in, still built from engine cubes and cylinders. It
has real openings (windscreen, side windows, tailgate glass), pillars, roof,
door cards, a dashboard and the binnacle, and a steering wheel that turns
with the input. The wheels have silver rims, and the mirrors are in the
cockpit, each with a housing. Read, not copied: the footprint from the
course's CAR record, the seat from `innova_Chassis.json` (set once in
`gen_innova.py`: x −1.35, y −0.38 RHD, **1.36 m** up), and full lock from
`innova_Steering.json` (612.5° at the rim). The scenery is cut around the
course's patches, so the black void past them is gone. All measured:
- **The rim** (`-ChronoLookWheel`, a camera down the column): steer 0.2 must
  put the stripe at 0.2 × 612.5 = **122.50°** clockwise as the driver sees it;
  found **122.50°**. Four mutants (sign, scale, offset, wrong side) all fail.
- **The HUD's test panel moved** off the interior mirror (it covered 85.4%)
  onto the dashboard, bottom right. The HUD now projects every glass each
  frame and logs `HUD over mirrors`: interior and right door **0.0%** covered.
- **The void**: 0.00% black in all three mirror targets and below the horizon.
- **Mirrors with the body in place**: ALL MIRRORS MEASURED CORRECT, targets
  within 0.10 px, glass within 1.09 px (tolerance 6).
- `determ pullaway` **`08eb1d195ace64ff`**, unchanged. The 1 kHz loop under
  cockpit rendering: 177–180 µs mean, **0 missed in ~103,000 steps**.
- **Frame time, like for like** (`circle.txt`, same day): box car 27.84 ms,
  cockpit **28.48 ms, +0.6 ms (2%)**. The 25.4 ms baseline did not
  reproduce: the unchanged box car is 2.4 ms slower today, cause unknown.

It corrected one record: `HighResShot` includes the canvas HUD (task 5 said
it did not). Full account in "Task 9: the cockpit" in the log.

**Verify.** `bash cockpit/run.sh render` (wheel and seat, exit 0 only if both
pass), `bash cockpit/run.sh frametime`, `bash mirror/run.sh render`.

---

### 17. `glance` — let the driver see the left mirror — **DONE**

From the right-hand seat the left door mirror is 55° left, outside the
forward view, and both parking spaces are on the left. Now the driver can
glance at either door mirror and back: a `glance` axis (−1 left, +1 right)
beside the controls that never reaches Chrono. **Keys, held** (Z/C, or
comma/period), integrated at 1 kHz at 4 /s; the head returns by itself on
release. An **optional 7th script column**, and the **wheel's D-pad**
(`ABS_HAT0X`, an unverified G923 default: task 12, item 7). `drv_glance_ease`
(smoothstep) means no jolt from any source. The camera slerps toward the
glass about a fixed eye, so the mirrors' images do not change. Measured: the
glance run puts the left glass **1.1 px from the frame centre, 0.0% under a
panel**; 20 new driver assertions (66), 4 new decode checks;
`determ pullaway` and `verdict course` unchanged. It found the HUD's mirror
report was stale at the shot (logged only on state changes, last one
mid-turn); the shot timer now asks for one "at the shot". Full account in
"Task 17: the glance".

**Verify.** `driver/run.sh selftest`, `wheel/run.sh decode`,
`cockpit/run.sh render` (the glance run).

### 18. `frame-budget` — where 28 ms a frame went — **DONE**

**The 60 Hz render target is met: 16.67 ms frames, the GPU at 8.8 ms.** The
health line now splits each frame as `stat unit` does, and
`-ChronoProfileGPU` dumps a per-pass profile. As found the frame was
GPU-bound (24.1 of 25.8 ms), and **TSR was 10.9 ms of 19.8**: UE5's default
anti-aliasing at Epic, with a 2560×1440 history. Also measured: a
translucency lighting volume for a scene with no translucent material (0.79
ms, plus 0.75 in each mirror capture) and motion blur (0.8). Now TAA, no
volume, no blur: GPU 24.1 → 8.5 ms. **Frames are capped at 60** in
`AChronoGameMode`. Uncapped the game ran ~100 fps and the 1 kHz loop missed
10 in 113,000. `t.MaxFPS` in an ini does not hold, because
`UGameUserSettings` re-applies `FrameRateLimit` over it. `-ChronoMaxFPS=<n>`
overrides, 0 = uncapped. At 60 Hz the loop misses **0–6 per 112,000 steps**
(TSR's 37 fps: 0). Page faults, the frame rate and the GPU's interrupt CPU
were each ruled out by a run; the source is unidentified. Task 5 accepted 2
in 109,000. Full account in "Task 18: the frame budget". It also made task
5's wrong claim true: `HighResShot` frames now hide the HUD ("The scene-only
screenshot, made so").

**Verify.** `cockpit/run.sh profile` (split + GPU passes), `frametime` (16.67
ms, gpu-bound under the cap), then `mirror/run.sh render` and
`cockpit/run.sh render`.

---

### 10. `true-neutral` — remove the gear-holding workaround — **DONE**

Neutral now goes to the transmission, 1e20 ratio and all, with no Chrono
patch. PSOR solves one constraint row at a time, exactly, so the 1e20 row
pins the gearbox input shaft to its casing and no other row sees its scale.
MINRES could not do that. Linear solvers (`CHS_SOLVER=minres|qr|lu`) keep
M5's workaround, and `CHS_NEUTRAL=hold` restores it under any solver for a
same-code comparison. Free roll on 12% improves **−16.6% → −15.4%** of
`g·sin θ`: the freed input shaft is worth 30 kg of car, predicted ~1.5
points. It also removed a defect nobody had seen: selecting neutral at
4 m/s in 1st **kicked the car to 4.26 m/s**, because the workaround's shift
to 5th dumped the input shaft's spin into it. The `neutral` harness now
drives at speed and asserts that neutral is a neutral: engine free with the
pedal up, car coasting, 2nd drives after. `driveline`, which swept the held
gear, is deleted. The PSOR residual stat is meaningless in neutral now (the
1e20 row reports ~1e14); convergence is checked physically instead. Full
account in "Task 10: a true neutral".

**Verify.** `... neutral` (asserted), `... hill 0.12`, `... determ upshift`.

---

### 11. `regulation-dimensions` — replace the `[REG?]` figures — **DONE**

Sourced, with one caveat. **Perpol 5/2021 (and its 2023 amendment, Perpol
2/2023) contains no course dimensions**: Pasal 18 ayat (6), read verbatim,
hands them to a *Keputusan Kakorlantas Polri* that is in no legal database
reachable here. The last **published** figures are Perkapolri No. 9/2012,
Lampiran A "Ujian Praktik SIM A", read in its own text (JDIH BPK), and
reporting after 2021 still cites them. They are now `[REG]`: sourced, and
possibly superseded by an unpublished decree.

They changed the course. **Slalom**: 9 cones at 1.5 × car length = **7.10
m** (was 5 at 8.0). **Parallel**: 1.5 × length (as before) by a printed
"lebar 1,5 meter", which cannot hold a 1.83 m car, **read as 1.5 × width**.
**Bay**: 1.5W × 1.5L = **2.745 × 7.10 m** (was 2.5 × 5.0). **Hill**: **15°
(26.8%, was 12%)** up to 2.5 m, a 1.5L bridge, and a 15° descent, with the
stop line halfway up. **Rollback allowed: none** — "tidak ada reaksi kendaraan
mundur". All feasible for this car. The whole test still **PASSES 4/4**
(`2b9c9e72f728f6b9`), after fixing three things it exposed: a plan buffer
that truncated SILENTLY at 82 m, a pilot hill-start bite fixed for 12% (15°
needs 15% of clutch capacity; the bite now comes from the slope), and a
parking set-up placed off the road instead of off the space. Departures from
the regulation are listed in "Task 11: the regulation, read".

**Verify.** `course/run.sh gen` (the `[REG]` lines and the departures list),
`... terrain`, `... verdict course`, `... judge park_bay_short` (must be
PARK_OUTSIDE).

---

### 12. `wheel-bringup` — the hardware unknowns

**Needs the G923 physically attached.** Everything is code-complete and
offline-tested (`wheel/run.sh decode`, 21 assertions).

1. FFB sign — whether positive `FF_CONSTANT` turns clockwise. `run.sh ffb`.
2. Shifter gate codes — `BTN_TRIGGER_HAPPY1..7` is an unverified default.
   `run.sh monitor`.
3. `EVIOCSFF` cost — decides whether the separate wheel thread stays necessary.
4. `/dev/uinput` access — `sudo bash scripts/04-uinput-access.sh`, then
   `run.sh virtual`.
5. Clutch ergonomics — `DRV_RATE_CLUTCH_DN`. **Do not tune hill-start
   difficulty without a real pedal.**
6. FFB gain — since task 14 the car pulls 0.77 g and the rim torque peaks at
   **2.97 N·m, above the G923's 2.3** (`sweep`). `Chs_SetFfbTuning`'s gain,
   or a soft clip, has to be chosen with the wheel in hand.
7. D-pad codes — the glance (task 17) reads the D-pad as `ABS_HAT0X`, the HID
   hat a Logitech wheel reports under Linux; unverified for the G923.
   `run.sh monitor`, press left and right.

---

### 16. `parking-torque` — a wheel that resists turning when parked — **DONE**

The wheel now resists being turned at a standstill: **−26.5 N·m at the
column** (7.9 at the rim), where TMeasy alone gave −0.30. It holds 99% of
that when the wheel stops, reverses with the turn, and fades as the car rolls:
−17.2 N·m of parking share turning at 0.30 m/s, −6.0 at 0.80, zero above 3
m/s. A twist state per front tyre in the shim's `ParkingTorque`, **force
feedback only**. No ABI change. Its two stored numbers live in the tyre JSON,
from `gen_innova.py`: the twist stiffness, 3,720 N·m/rad, `[DERIV]` from
TMeasy's own cornering stiffness through the brush model, and the patch width,
0.16 m `[EST]`. The patch length is TMeasy's own, read live; μ comes from the
JSON. The fade is TMeasy's own Dahl weight, so `sweep` is bit-identical.

It found that **TMeasy has no aligning moment in reverse**: its `Mz` gate tests
SIGNED speed. Reversing at 0.30 m/s, turning or holding, the old wheel read
0.000. Both parking exercises are reversed in, so this term is now their only
steering feel. The rim torque is 3.4× the G923's 2.3 N·m: task 12, item 6.
Full account in "Task 16: the parking torque" in the log.

**Verify.** `docker/05-verify-vehicle-shim.sh mzlow 0.20` (asserted: the
parking rows, standstill and rolling), then `sweep` with and without
`CHS_PARKING_TORQUE=0` (identical), `determ pullaway` (`08eb1d195ace64ff`),
`verdict course` (PASS, `317eca4c2c8d9b64`), `perf`.

---

## Open questions

Questions the user will ask, recorded here so the session that answers them
starts from what the repo already knows.

### Q1. `ancf-offline-ffb` — **ANSWERED 2026-10-01**

The brief and its answer, with the measurements, are "Q1: ANCF, rFactor 2,
and the parking torque" in `ENGINEERING_LOG.md`. It proposed task 16, now done.

---

## Read this first if you are new to this repo

`README.md` has the from-scratch setup for a fresh Fedora machine and a summary
of what works. `ENGINEERING_LOG.md` is the engineering log: what was measured,
what was tried and rejected, and why each decision went the way it did. THIS
file is the brief distilled from it, loaded into every session. When they
disagree with the README, these two are authoritative about *reasoning* and
the README about *procedure*.

Two habits this codebase depends on:

1. **Edit the generator, never its output.** `gen_innova.py` owns
   `vehicle/innova/*.json`; `gen_course.py` owns `simA_course.json`,
   `simA_terrain.txt` and `simA_referee.txt`. `course/run.sh check` fails if
   you hand-edit them.
2. **Measure before concluding.** Nearly every wrong turn recorded in the log was a
   plausible hypothesis that a five-minute experiment refuted. The harnesses
   exist so that is cheap: the whole host suite runs in about five seconds.

## Status

| # | Milestone | State |
|---|-----------|-------|
| 0 | Host assessment + remediation | **done** |
| 1 | Chrono 10 in Docker | **done** |
| 2 | UE 5.8.2 build + ChronoBridge plugin | **done** |
| 3 | 1 kHz thread + lock-free ring + interpolation | **done** |
| 4 | Innova multibody model, Ackermann, TMeasy vs ANCF | **done** |
| 5 | Logitech G923 evdev/hidraw + FFB | **code complete, hardware bring-up blocked** |
| 6 | Mirrors + SIM A course + referee triggers | **done** — course, referee, verdict, mirrors, HUD, cockpit, glance; the whole test drives and judges PASS end to end, rendered at 60 Hz |

### Reasoning archive

This file records *what* was decided and measured. The *why* — the derivations,
the diagrams, the failed approaches — lives in four published artifacts:

| Artifact | Covers |
|---|---|
| [Step 0 feasibility](https://claude.ai/code/artifact/1e841092-b769-4ca9-b5ef-46f7217c3db6) | Hardware limits, vehicle selection, why RT/144 Hz/ANCF are out |
| [M2 The ABI Wall](https://claude.ai/code/artifact/4d53430a-8338-4ab7-b790-20ab85d8e6a7) | Two C++ runtimes, AVX2 layout divergence, UE modules, RPATH |
| [M3 The Kilohertz Loop](https://claude.ai/code/artifact/ac4104ac-219b-4886-ab02-632023f8fa6c) | Deadline timing, C-states, seqlock, interpolation vs latest-sample |
| [M4 Building the Innova](https://claude.ai/code/artifact/6174c115-eb80-4e5a-8234-32cf85f0fca3) | Deriving an undocumented vehicle, Ackermann, TMeasy vs ANCF |

**Read one with `WebFetch` on its URL** (works via the user's claude.ai login;
`curl` does not — it gets the SPA shell or a 403). `/artifacts` in the terminal
also lists them.

Do this when a settled decision is being questioned or re-derived — the artifact
holds the measurement that settled it. It is **not** free: an artifact fetch
returns raw HTML, roughly 15k tokens, so treat it as a deliberate lookup rather
than routine background reading. If this file or `ENGINEERING_LOG.md` already
answers the question, that is enough.

---

## Hardware timeline — the wheel comes LAST

**The G923 will not be bought until every milestone is done.** This is
deliberate, confirmed 2026-09-03, and it inverts the usual order: Milestone 5
was built and landed with no wheel in existence, and Milestone 6 will be too.

So: never block a milestone on hardware arriving, and never let "test it when
the wheel is plugged in" be the verification step for new work. Build a
hardware-independent check instead. The pattern that already works here is an
offline self-test against a synthetic fd (`wheel/run.sh decode`), a pure-C
client that links only the shim, and a uinput virtual device.

The open hardware-dependent unknowns are task 12 in the backlog (and the end
of the log's Milestone 5 section). They stay open on purpose — do not try to resolve or guess them.

---

## Hard constraints — do not design around these being fixable

Verified by direct probing in Step 0, not from spec sheets.

- **No hardware ray tracing.** Radeon RX 550 (Polaris/GCN4) and Intel UHD 630
  expose neither `VK_KHR_ray_query` nor `VK_KHR_ray_tracing_pipeline`. Only
  `llvmpipe` (CPU) does. Architectural — AMD RT starts at RDNA 2.
  → Milestone 6 uses planar `USceneCaptureComponent2D` reflections, which are
  exact for flat mirrors anyway.
- **No 144 Hz display.** EDID caps the LG FHD at 100 Hz, and that panel is on
  the *Intel iGPU*; the AMD card drives a 60 Hz panel. **Render target is 60 Hz.**
  Met since task 18: frames capped at 60, the GPU at ~8.8 ms of the 16.7.
- **ANCF tyres cannot run in real time.** Measured: 49,709 µs/step for one tyre
  against a 1000 µs budget (1,125× a TMeasy tyre; 199× over for four).
  → TMeasy in the loop, ANCF offline on `ChTireTestRig` for validation.
  **Corrected by Q1:** that tyre never touched the road (no collision system
  was set) and its solve was not converged. In contact and converged, the
  shipped ANCF-4 tyre needs a 0.1 ms step and 0.29–0.44 s per step: 3,000–4,400×
  real time. The conclusion stands, by a wider margin.
- Mechanical disk (7200 rpm), no SSD. 8C/8T i7-9700, no SMT, **no AVX-512**.
  16 GiB RAM.

## Host setup already applied

`ulimit -r` = 95 · governor `performance` · `/WIN_D` mounted **ntfs3** ·
zram 16 G zstd · Docker data-root `/home/docker-data` · **containerd** root
`/home/containerd-data` (the snapshotter stores image layers there, *not* under
Docker's data-root — this is what filled `/` twice).

Disk: `/` 60% · `/home` 76% · `/WIN_D` 74%. Build artifacts belong on `/WIN_D`.

---

## Layout

```
/WIN_D/protek/SimA-simulation/     (was /WIN_D/protek/chronoproject until 2026-09-05)
  CLAUDE.md       the brief (auto-loaded)  ENGINEERING_LOG.md  the full record
  scripts/        host remediation (00,01,02,03), UE clone/build (10,11)
                  content generation (20 level, 21 mirror material, 22 both)
  docker/         Dockerfile + 01-build-image 02-verify 03-export-sdk 04-build-shim
  shim/           chrono_shim.{h,cpp}, .map  <- the ABI wall
  chrono-sdk/     exported from container: include/ lib/ share/
  vehicle/        gen_innova.py, test_innova.cpp, run.sh, dbg.sh, innova/*.json
  wheel/          run.sh, test_wheel.c, virtual_g923.c   <- G923 bring-up harness
                  (the device layer itself lives in the plugin; see below)
  driver/         run.sh, test_driver.c, scripts/*.txt   <- keyboard + script
                  (same arrangement: the logic lives in the plugin)
  course/         gen_course.py, plot_course.py, run.sh, simA_course.json
                  simA_terrain.txt (physics), simA_referee.txt (rules)
  referee/        run.sh, test_referee.c   <- rule-engine harness
  mirror/         run.sh, test_mirror.c    <- planar-mirror geometry harness
                  probe.py                 <- measures a -ChronoMirrorProbe frame
  cockpit/        run.sh, probe.py         <- task 9: the wheel, the seat view,
                                              frame time; measured from frames
  SimA/           UE project
    Plugins/ChronoBridge/Source/ChronoBridge/
      Public/  ChronoBridgeModule.h ChronoStateRing.h ChronoPhysicsThread.h
               ChronoInterpolator.h ChronoWorldSubsystem.h ChronoWheelInput.h
               driver_source.h            <- CANONICAL keyboard/script logic
      Private/ ChronoBridgeModule.cpp ChronoPhysicsThread.cpp
               ChronoWorldSubsystem.cpp ChronoWheelInput.cpp
               ChronoRtClock.h            <- shared by both RT threads
               wheel_device.{h,cpp}       <- CANONICAL evdev/FFB layer
               driver_source.cpp
/WIN_D/protek/UnrealEngine/   UE 5.8.2 source build (160 GB)
```

---

## THE ABI WALL — the single most important invariant

Two incompatible C++ runtimes share one process:

- `libChrono_core.so` → **libstdc++**, dynamically, 85 undefined refs into it.
- `UnrealEditor` → **libc++**, *statically* (`libc++.a` from Epic's clang 20.1.8).

2,873 of Chrono's 28,574 exported symbols pass `std::` types. Separately,
Chrono is built `-march=skylake` (`EIGEN_MAX_ALIGN_BYTES=32`) while a stock UE
TU gets 16. Measured: `struct { char; Eigen::Quaterniond; }` is **64 bytes in
one and 48 in the other**, while bare `sizeof(Quaterniond)` is 32 in *both* —
so a spot check passes and the heap corrupts silently.

**Rules:**
1. Unreal code includes **`chrono_shim.h` only**. Never a Chrono or Eigen header.
2. Only PODs cross: fixed-width ints, doubles, opaque `void*` handles, structs
   of those with explicit padding.
3. No allocation crosses — the shim frees what the shim allocated.
4. No exceptions escape — every entry point wraps in `catch (...)`.
5. No pointers returned into shim-owned memory.

Enforced by a linker version script (`shim/chrono_shim.map`) exporting exactly
16 `Chs_*` symbols (7 through ABI 1, 4 vehicle/FFB in ABI 2, 2 terrain in
ABI 3, 2 tyre standstill in ABI 4, 1 parking hold in ABI 5). Verify with:
`readelf -d …/libUnrealEditor-ChronoBridge.so | grep NEEDED` → must list
`libchrono_shim.so` and **not** `libChrono_core.so`.

Bump `CHRONO_SHIM_ABI_VERSION` whenever a struct in the header changes; the UE
module checks it at load and refuses to run on mismatch.

**Do NOT set `CXX_VISIBILITY_PRESET hidden` alongside the version script.**
Visibility is applied by the compiler; a version script cannot promote a
symbol the compiler already hid. Doing both exports nothing at all.

**RPATH not RUNPATH.** The shim links `-Wl,--disable-new-dtags` with
`INSTALL_RPATH "$ORIGIN"`. RUNPATH is not inherited by transitive deps, so
Chrono's own `.so` files (which carry no rpath) would fail to resolve.

---

## Commands

```bash
# Chrono container + shim
bash docker/01-build-image.sh        # builds chrono:10.0.0 and -builder
bash docker/02-verify.sh             # step-cost benchmark
bash docker/03-export-sdk.sh         # -> chrono-sdk/
bash docker/04-build-shim.sh         # -> chrono-sdk/lib/libchrono_shim.so

# Unreal
cd SimA
/WIN_D/protek/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh \
    SimAEditor Linux Development -project="$PWD/SimA.uproject"
/WIN_D/protek/UnrealEngine/Engine/Binaries/Linux/UnrealEditor "$PWD/SimA.uproject"

# Vehicle model
cd vehicle
python3 gen_innova.py                # regenerate the 14 JSON files (innova_TireANCF: Q1 rig only)
bash run.sh probe|assemble|ackermann|tire|perf [args] [solver]
bash run.sh ancfland|ancfpark|ancfroll <tyre> ...   # Q1 offline tyre rigs; RIG_* env, see Q1
bash dbg.sh <mode>                   # same under gdb (installs gdb in-container)

# Vehicle THROUGH the shim (pure-C client, proves nothing C++ crosses)
bash docker/05-verify-vehicle-shim.sh assemble|sweep|clutch|neutral|perf [steps]
bash docker/05-verify-vehicle-shim.sh determ [script] [passes]   # M6 determinism
bash docker/05-verify-vehicle-shim.sh drift  [script]            # zero-steer drift
bash docker/05-verify-vehicle-shim.sh terrain                    # the hill, ABI 3
bash docker/05-verify-vehicle-shim.sh creep                      # standstill creep
bash docker/05-verify-vehicle-shim.sh judge [script]             # end-to-end verdict
bash docker/05-verify-vehicle-shim.sh judge park_bay             # parking, judged (task 3)
#   ... also park_bay_short (must be PARK_OUTSIDE), park_parallel; PARK_TRACE=1 traces
bash docker/05-verify-vehicle-shim.sh verdict course             # THE TEST: whole course,
#   driven and judged, twice in two processes, digests compared (task 4) -> PASS 4/4
bash docker/05-verify-vehicle-shim.sh judge course_cone|course_nostop   # must FAIL
bash docker/05-verify-vehicle-shim.sh clutch|diff                # asserted: WORKS / OPEN
bash docker/05-verify-vehicle-shim.sh solvers                    # QR/MINRES/PSOR statics+turn
bash docker/05-verify-vehicle-shim.sh hold                       # braked car at rest, asserted
bash docker/05-verify-vehicle-shim.sh attitude                   # design ride height + 53%, asserted
bash docker/05-verify-vehicle-shim.sh batch "hold" "hill 0.12" "verdict course"   # ONE build,
#   many modes, one log; each spec headed '########## <spec>'. A re-baseline in one command.
bash docker/05-verify-vehicle-shim.sh holdtrace <grade> <hold> [land]  # braked spawn, traced
BRAKE_MODEL=simple python3 vehicle/gen_innova.py   # the chattering pre-task-15 brake
CHS_NEUTRAL=hold bash docker/05-verify-vehicle-shim.sh <mode>   # the pre-task-10 neutral
CHS_IDLE_GOVERNOR=0 bash docker/05-verify-vehicle-shim.sh <mode> # the pre-task-8 engine
CHS_SOLVER=minres bash docker/05-verify-vehicle-shim.sh <mode>   # the pre-task-13 car
#   also CHS_SOLVER_ITERS, CHS_SOLVER_STATS=1, CHS_OPEN_DIFF=1 (diagnostic)
bash referee/run.sh selftest                                     # 98 assertions, ~1 s
bash docker/05-verify-vehicle-shim.sh rollacct [grade] [rr] [tr_ds]  # free-roll account, asserted
bash docker/05-verify-vehicle-shim.sh rollback                   # rollback floor vs model noise, asserted
bash docker/05-verify-vehicle-shim.sh judge course_rollback      # a bad hill start: must FAIL (rollback)
bash docker/05-verify-vehicle-shim.sh hill [grade] [b_begin] [b_end]
bash docker/05-verify-vehicle-shim.sh standstill                 # Dahl sigma sweep on the hill
bash docker/05-verify-vehicle-shim.sh mzlow [b_begin] [b_end]    # Mz at parking speed;
#   its parking-torque rows (task 16) are asserted
CHS_PARKING_TORQUE=0 bash docker/05-verify-vehicle-shim.sh <mode>  # the pre-task-16 wheel
bash docker/05-verify-vehicle-shim.sh blendhill [grade] [b_end]  # blend vs hill start
bash mirror/run.sh selftest                                      # 53 assertions, ~1 s
bash ring/run.sh               # the snapshot ring under a real race, ~10 s (task 5)
bash mirror/run.sh render      # in-editor: markers placed by geometry, found in pixels
bash cockpit/run.sh render     # in-editor: the rim's angle (-ChronoLookWheel), no HUD
#   panel over a mirror, no void, the glance (task 17); ~10 min.
bash cockpit/run.sh frametime  # circle.txt, 110 s: frame time and its game/render/
#   RHI/GPU split (task 9, 18). `profile`: 60 s plus one frame's GPU passes
# -ChronoMaxFPS=<n> overrides the 60 Hz cap (0 = none); -ChronoProfileGPU
TYRE_MODEL=fiala python3 vehicle/gen_innova.py    # the rejected tyre experiment
CHS_TIRE_ENVELOPE=1 bash docker/05-verify-vehicle-shim.sh verdict course  # tyre envelope (Q1)
bash vehicle/calibrate_steering.sh [ACK:PIN ...]   # refit the steering constants

# Driver input sources (host-native, no container, no Chrono, no Unreal)
cd driver
bash run.sh selftest                 # 66 assertions, ~1 s
bash run.sh dump <builtin|file> [hz] # print a script as physics will see it
bash run.sh list                     # built-in script names

# Drive the car in the editor: WASD steer/throttle/brake, LeftShift clutch,
# LeftCtrl fine, Q centre, 0-5 + R gears, Z/C (or ,/.) glance at the left/right
# door mirror while held (task 17). Or run a script instead:
#   UnrealEditor SimA.uproject -ChronoScript=slalom
# A scripted drive is HELD until frames are steady (the session start, task 5).
# -ChronoShot=<s> screenshots and quits; add -ChronoShotUI to include the HUD
# (Saved/Screenshots/*/ChronoHUD.png). Offscreen, as mirror/run.sh does:
#   ... -game -RenderOffscreen -unattended -ResX=1280 -ResY=720

# Course (host-native; derived from the vehicle, never hand-edited)
cd course
bash run.sh gen                      # regenerate simA_course.json + report
bash run.sh plot [cols]              # ASCII top-down layout + feasibility
bash run.sh check                    # fails if the JSON was hand-edited

# Wheel (host-native, no container, no Chrono)
cd wheel
bash run.sh decode                   # offline: needs no hardware and no root
bash run.sh probe|monitor [s]|ffb [s]|loop [s]
bash run.sh virtual [s]              # uinput fake G923; needs 04-uinput-access
sudo bash scripts/04-uinput-access.sh   # once, to make /dev/uinput usable
```

**UE logs are in `SimA/Saved/Logs/SimA.log`, not stdout.** Headless
`-nullrhi -unattended` runs take >20 min on the first asset scan and often
never reach `Tick`; use a normal GUI launch and read the log file.

Container caveats: run as `--user "$(id -u):$(id -g)"` with `CCACHE_DISABLE=1`
and `HOME=/tmp`, and stage sources on **tmpfs** — `/WIN_D` is NTFS and cannot
hold SELinux labels, so a `:z` bind mount from there is denied under Enforcing.

---

## Measured performance (i7-9700, single thread, 1 ms step)

| Workload | mean | p99 | max |
|---|---|---|---|
| Rigid SMC, 40 free bodies | 7.4 µs | 9.7 | 39.2 |
| **Full Innova, default solver** | 382.5 µs | 515.4 | 619.2 |
| **Full Innova, MINRES** | **185.8 µs** | 254.0 | 355.1 |
| Full Innova, SPARSE_QR | 1218.1 µs | 1740.2 | 2287.3 |
| TMeasy, one tyre | 44.2 µs | 80.7 | 153.4 |
| ANCF-4, one tyre (M4: no contact, unconverged; see Q1) | 49,709 µs | 58,245 | 60,302 |
| ANCF-4, one tyre, in contact, sparse LU, **0.1 ms step** (Q1; by machine load) | 0.29–0.44 s | — | — |
| **Innova via `Chs_Step`, MINRES** | **179.4 µs** | 195.4 | 320.6 |
| + `Chs_GetFeedback` + `Chs_GetSnapshot` | 179.6 µs | — | — |
| `wheel_poll`, empty evdev queue | 0.227 µs | — | — |
| wheel thread work per 1 ms tick | 1.31 µs | — | 16.8 |

The ABI wall costs **0.2 µs per step** (179.4 → 179.6 with feedback and a
5-body snapshot on top). Crossing it is not a performance concern.

| Innova via `Chs_Step`, PSOR 50 (task 13) | 136.2 µs | 216.3 | 340.4 |
| **+ BrakeShafts, four more clutches (task 15)** | **147.5 µs** | 224.4 | 398.9 |
| same run, MINRES 60 (pre-task-13) | 192.1 µs | 316.0 | 440.3 |
| same car at its design ride height (task 14) | 136.1 µs | 164.9 | 350.1 |

**Use `ChSolver::Type::PSOR`, 50 iterations, no warm start** (since task 13).
MINRES was chosen here for speed, and it cannot open a clutch: it is a linear
solver, and a clutch is a box-constrained multiplier. PSOR is cheaper still,
and it matches the exact direct solve to <1%. See "Task 13" in the log. Body count is the
wrong intuition; closed kinematic loops are what cost.

1 kHz thread timing (from `ChronoPhysicsThread.cpp`):

| strategy | mean late | max |
|---|---|---|
| plain sleep, SCHED_OTHER | 85.8 µs | 126.2 |
| plain sleep, SCHED_FIFO | 78.3 µs | 125.1 |
| **hybrid spin, SCHED_FIFO** | **0.1 µs** | **28.5** |

The lateness was **C-state exit latency**, not scheduling (C6 = 85 µs,
C7s = 124 µs on this CPU). Fix is sleep to `deadline − 200 µs` then busy-spin.

---

## Milestone 3 design notes

- `TChronoStateRing<64>` is a seqlock: writer never blocks, reader retries
  (bounded to 8 attempts, then reuses the last frame). Verified 924 M reads,
  **zero torn snapshots**. `Head` is on its own cache line.
  **Task 5:** now 256 deep (`FChronoRing`), and a consumer that needs EVERY
  sample must read by absolute index (`ReadAt`): `Read(back)` counts from
  the head as it is at each call, so a loop the producer publishes into
  skips one. `ring/run.sh` stress-tests both.
- **Interpolate, don't take the latest sample.** 1000/60 = 16.667 is not an
  integer, so staleness walks 0.667 / 0.333 / 0.000 ms and produces a periodic
  stutter. Measured per-frame motion variation: **2.82% latest-sample vs 0.00%
  interpolated.** Render 2 ms behind; clamp α rather than extrapolate.
- `GameSimClock` advances by real `DeltaTime` and is slew-corrected 2%/frame
  toward the producer. **Never assign it from `sim_time`** — that reintroduces
  the exact stutter interpolation removes.
- Physics thread touches no `UObject`/`AActor`/`FString`. Owned by a
  `UWorldSubsystem` so its lifetime tracks the `UWorld`.
- `TPri_TimeCritical` is *not* `SCHED_FIFO`; RT priority is requested via
  `pthread_setschedparam` inside `Init()`, on the thread itself.

---

## Milestone 4: the vehicle

**Toyota Kijang Innova Reborn 2.4 G MT (AN140, IMV).** Chosen because its
topology maps onto Chrono's best-exercised templates — `ChDoubleWishbone`
front, `ChSolidThreeLinkAxle` rear (a three-link approximation of the real
four-link + Panhard).

**Step 0's claim that the IMV platform has extensive SAE documentation was
wrong.** Toyota publishes no hardpoints, K&C data or bushing rates. ~90% of the
model is derived. `gen_innova.py` tags every value `[SPEC]` / `[DERIV]` /
`[EST]` and recomputes on change — **edit the generator, never the JSON.**

Verified: static equilibrium 17,659.8 N vs 17,658.0 N expected (**0.01%**);
Ackermann 106% → 88% → 79%, full lock 34.6° (design 35°).
**Task 14:** that check verified the TOTAL, not the split, nor where the car
sat. It sat 4.4° nose-up with the rear 289 mm into bump. Now 17,657.7 N, 53.0%
front, level at design ride height, full lock 35.36°.

Still `[EST]` and worth revisiting before treating output as measurement:
hardpoints (refine with `ChSuspensionTestRig`), gear ratios, CG height 0.62 m,
and two empirically fitted constants — `ACK_SCALE` and `PINION_CAL` (now 1.0
and 0.954; see "The fix broke Ackermann" in the log, and task 14).

### Chrono JSON traps — these segfault, they do not error

Readers use `d["Key"].GetDouble()` with no `HasMember` guard.

| Subsystem | Required form |
|---|---|
| RigidChassis | `"Void": false` on every component |
| ShaftsDriveline2WD | `"Motor Block"`, `"Axle"`, `"Driveshaft"`, `"Differential Box"` |
| WheeledVehicle | `"Suspension Indexes": [1]` in the **vehicle** file, not the driveline |
| EngineShafts | bare array, x in **rpm** (Chrono applies `CH_RPM_TO_RAD_S`) |
| TMeasyTire | unit suffixes are part of the key: `"Unloaded Radius [m]"` |

**Always run `bash run.sh probe` first** — it loads each subsystem JSON
individually so a crash names the guilty file.

### Conventions

- **Chrono**: ISO 8855 — X forward, **Y left**, Z up, metres, right-handed.
- **Unreal**: X forward, **Y right**, Z up, centimetres, **left-handed**.
- Conversion lives in `ChronoInterpolator.h`. The handedness flip negates Y for
  vectors *and* the corresponding quaternion components — getting it wrong
  gives a car that steers the wrong way.
- Chrono 10 renamed `ChVehicleModelData.h` → `ChVehicleDataPath.h`
  (`GetVehicleDataFile`, `SetVehicleDataPath`). `WheeledVehicle.h` lives under
  `wheeled_vehicle/vehicle/`. Link via `${CHRONO_TARGETS}`; the exported targets
  are `Chrono::Chrono_core` etc. — `Chrono::core` is a build-tree alias only.
- Measure a road-wheel steer angle from the spindle's **spin axis** (local Y),
  never its local X: the spindle rotates about Y as the wheel rolls.

---

## The engineering log

Everything from Milestone 5 on, and every task's full account, is in
`ENGINEERING_LOG.md`, not here: this file is loaded into every session and
must stay well under Claude Code's 150k-character limit. **A finished task's
write-up goes there**; the backlog entry here keeps the result, the baseline
numbers and the verify command. Its sections, in order:

- Milestone 5: the wheel and force feedback
- Milestone 6 — in progress
- Milestone 6.1: the two input paths
- Milestone 6.2: the course
- Milestone 6.2b: terrain, and ABI 3
- Milestone 6.3: the referee
- Milestone 6.4: mirrors
- Milestone 6.5: content, generated rather than authored
- The hill start: what actually fixed it
- First actual run — what rendering revealed
- The mirror `rc=-2`: two bugs wearing one symptom
- The brake on a slope: a parking hold, and what it revealed
- Task 1: the aligning moment below 1 m/s
- Task 2: the mirror, end to end
- Task 3: parking completion
- Task 4: the verdict
- Task 13: the clutch, opened
- Task 15: the brake that could not hold
- Task 5: the HUD, and what seeing the car found
- Task 7: the rollback rule
- Task 11: the regulation, read
- Task 6: the free-roll account
- Task 8: an engine that idles, and stalls
- Task 10: a true neutral
- Task 14: the car sat on the wrong springs
- Q1: ANCF, rFactor 2, and the parking torque
- Task 16: the parking torque
- Task 9: the cockpit
- Task 17: the glance
- The scene-only screenshot, made so
- Task 18: the frame budget

## Milestone 6 — what cannot be validated without hardware

The physics of the hill start is verifiable (traction-limited: 1st-gear
tractive force ≈ 1.00× vehicle weight, but the rear tyres cap near 7,100 N of
17,700 N available). What is **not** verifiable is whether the exercise is
*difficult in the right way* — that is clutch-pedal ergonomics, and it needs a
real clutch pedal. Do not tune the difficulty blind; leave it and note it.
