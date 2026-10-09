# SIM A — engineering log

The long-form record: what was measured, what was tried and rejected, and why
each decision went the way it did. It was the second half of `CLAUDE.md` until
2026-10-08, when that file passed Claude Code's 150k-character limit for
auto-loaded instructions. Moved verbatim, in its original order, so "above" and
"below" inside it still mean what they meant.

`CLAUDE.md` is the brief every session loads (backlog, constraints, commands);
this file is read on demand. To find a section: `grep -n '^## ' ENGINEERING_LOG.md`,
then read from that line.

---

## Milestone 5: the wheel and force feedback

**The G923 has still never been attached to this machine** (re-probed
2026-09-03: only a Dell KB216 and an HP mouse enumerate; no `046d` device).
Everything below was therefore built to be verifiable without it, and the parts
that genuinely need hardware are listed at the end as still open.

### Kernel support is per-PID and the two paths differ

Probed with `modinfo -F alias`, not read off a wiki:

| PID | Module | FF capability |
|---|---|---|
| `046d:c24f` | `hid-logitech` (lg4ff) | `FF_CONSTANT`, `FF_AUTOCENTER` |
| `046d:c262` (G920), `046d:c26e` (G923 Xbox) | `hid-logitech-hidpp` | + `FF_SPRING`, `FF_DAMPER` |
| `046d:c266` (G923 PS/PC) | **neither** | none — would enumerate as a bare HID pad |

The G923 PS SKU is expected to appear as `c24f` in its G29 compatibility mode.
Because three outcomes are possible and two of them expose different effect
sets, **capabilities are queried at runtime with `EVIOCGBIT(EV_FF)`**; nothing
functional is inferred from the product ID. `wheel_ffb_init` fails with a
readable message rather than silently producing no force.

### Chrono traps found here — both silent, both fixed in the shim

- **Neutral sets the gearbox ratio to `1e20`.** `ChTransmission::SetGear(0)`
  takes the `OnNeutralShift` path and assigns `CHRONO_NEUTRAL_GEAR_RATIO`,
  `#define`d as `1e20`. MINRES stops converging and returns garbage reactions
  for the *whole* coupled system, tyre contacts included. Measured: a
  stationary car went from z = **+0.264 m to −2.017 m in one second** — it
  falls through the ground. An H-pattern shifter crosses neutral on every
  change, so this path is constant.
  → The shim never passes neutral to the transmission. It holds the gearbox in
  its last gear and forces `m_clutch = 1.0`. Physically identical (no torque
  path either way), numerically sane. Regression test: `05-... neutral`.
  **Superseded by task 10:** under PSOR neutral IS passed through, and the
  1e20 is harmless; this workaround survives only for linear solvers.
  **Correction (task 4):** not physically identical — under MINRES the clutch
  cannot open (task 13), so "forced open" disconnects nothing: in neutral the
  engine stays geared to the wheels through the held gear.
- **A manual gearbox initialises in 1st, not neutral.** Caching the requested
  gear starting from 0 means the first `SetGear(0)` is skipped and the car sits
  in gear while reporting neutral. Seed the cache from `GetCurrentGear()`.
- `GetVehicleDataFile` **concatenates without a separator** — the data path
  needs a trailing `/` or you get `…/vehicleinnova/…` and a "could not open"
  that reads like a missing file.
- Clutch polarity **verified in Chrono's source**, not assumed:
  `ChManualTransmissionShafts::Synchronize` does
  `m_clutch->SetModulation(1.0 - driver_inputs.m_clutch)`, so `1.0` = pedal
  down = disengaged. Same sense as the pedal axis, so it passes straight
  through with no inversion. **The polarity is right, and under MINRES it
  does not matter (task 4):** the modulation has no effect at all — every
  pedal position gives the same result in `clutch` mode (task 13).

### Where force feedback comes from

Tyre self-aligning moment `Mz`, summed over both front tyres and divided by the
17.5 steering ratio, then reduced by a `[EST]` 0.7 hydraulic-assist fraction and
damped against steering rate. **Not** synthesised from lateral acceleration or
steer angle — the whole value of the signal is the part a synthetic model
misses.

Measured, 8 m/s, MINRES, quasi-static ramp:

| steer | road angle | column torque | lateral accel |
|---|---|---|---|
| 0.20 | 7.1° | −3.52 N·m | 0.29 g |
| **0.40** | **13.0°** | **−5.06 N·m (peak)** | **0.51 g** |
| 0.60 | 19.0° | −1.71 N·m | 0.68 g |

The torque **peaks and then falls by two thirds while the wheel keeps turning
further** — pneumatic trail collapsing as the rear of the contact patch starts
to slide. That is the wheel going light before the front washes out, and it is
the reflex the exercise has to teach. Peak rim torque **1.52 N·m against the
G923's 2.3 N·m**, so there is headroom for kerbs.

Beyond steer ≈ 0.70 the numbers go erratic (`Mz_FR` swinging ±150 N·m). The car
is past the friction circle and sliding; the quasi-static sweep stops measuring
anything there. The harness flags the region rather than averaging through it.

**Re-measured by task 14**, with the CG at its real height and the flag
MEASURED (`ay = v × yaw rate`; it was the kinematic `v² tan δ / L`, a
statement about the steering wheel): column torque peaks at steer 0.60,
**9.88 N·m (rim 2.97)**; the car's own lateral acceleration keeps rising to
**0.77 g at steer 0.80**, by which point the torque has fallen 17%. The cue
arrives before the grip goes, and the numbers stay smooth past it. The rim
peak is now above the G923's 2.3 N·m (task 12, item 6).

### Steering range: an unavoidable 1.35× compression

Innova full lock is 34.6° × 17.5 = 605° per side, **1211° lock-to-lock (3.4
turns)**. A G923 turns **900° (2.5)**. The axis is normalised against the
device's own reported range, so device full lock always maps to car full lock;
what is lost is the *ratio* — 450° of hand movement to full lock instead of
605°, a rack 1.35× quicker than the real car's.

That is the right trade and it is deliberate: parallel parking and the tight
slalom gates need full lock to be **reachable**. A compressed hand-over-hand
feel is a fidelity loss; an unreachable steering lock is a broken exercise.
1211° cannot be had from a 900° wheel.

### Threading

A **separate `FChronoWheelThread` at SCHED_FIFO 79**, one below physics, with
single-slot seqlocks in each direction (`TChronoSeqSlot`, not the ring — nothing
here needs history, only the newest value).

Reading the wheel is cheap enough to have shared the physics thread (0.227 µs
for an empty drain; 1.31 µs mean for the whole tick). **Writing** is the reason
for the split: every FFB update is an `EVIOCSFF` ioctl descending into
`ff-core` → `hid-logitech*` → a USB interrupt-OUT submission, and its cost is
unmeasured because there is no wheel. An unmeasured syscall does not belong on
a thread that must not miss 1 ms. Cost of the split is ≤1 ms of latency each
way, against ~10 ms before force feedback feels disconnected.

FFB is **computed at 1 kHz and transmitted at 250 Hz** (divisor 4) with a
64/32767 deadband. A full-speed HID endpoint cannot absorb 1000 updates a
second; commanding faster only queues work in the driver.

### Verified without hardware

- `wheel/run.sh decode` — 17 assertions on axis normalisation, pedal inversion,
  gate→gear (including the 6th gate resolving to neutral on a 5-speed), shift
  transitions, and frame accounting. All pass.
- `wheel/run.sh probe` — correctly rejects all 21 evdev nodes on this machine.
- `docker/05-verify-vehicle-shim.sh` — assemble / sweep / clutch / neutral /
  perf. `test_shim.c` is compiled as **C** and its `NEEDED` list is exactly
  `libchrono_shim.so`, `libm`, `libc`.
- `readelf -d libUnrealEditor-ChronoBridge.so` lists `libchrono_shim.so` and
  neither `libChrono_core.so` nor `libstdc++`.

### Still open — needs the wheel plugged in

1. **FFB sign.** Whether a positive `FF_CONSTANT` level turns the wheel
   clockwise is a device convention that cannot be checked without one.
   `run.sh ffb` applies a slow triangle for exactly this; if it pulls the wrong
   way, negate in `wheel_ffb_set`.
2. **Shifter gate codes.** `BTN_TRIGGER_HAPPY1..7` is the default table and is
   *unverified*. `run.sh monitor` prints the raw bitmask to confirm it.
3. **`EVIOCSFF` cost.** Decides whether the separate thread stays necessary.
   `wheel_ffb_stats` already accumulates it.
4. **`/dev/uinput` is root-only**, so `virtual_g923` — and with it the whole
   kernel-side FFB path — has not been run. `sudo bash
   scripts/04-uinput-access.sh` installs a udev rule granting the `input`
   group access; the user is already in that group.
5. **±45 N·m of `Mz` on each front tyre when straight ahead**, equal and
   opposite so they cancel at the rack (column torque 0.017 N·m — correct).
   Probably static toe-in. Worth understanding before trusting per-tyre `Mz`
   for anything other than the sum.

---

## Milestone 6 — in progress

Mirrors, the SIM A course, and referee triggers. **Nothing here needs the
wheel** (see the hardware timeline above): mirrors are planar
`USceneCaptureComponent2D` reflections, the course is geometry plus trigger
volumes, and the referee is rules over vehicle state.

### Sequencing

| step | content | state |
|---|---|---|
| **6.1** | input paths: keyboard + scripted driver, determinism proven | **done** |
| 6.2 | SIM A course geometry, derived from the vehicle | **done** |
| 6.3 | referee rules over the physics stream | **done** |
| 6.2b | terrain patches through the shim (ABI 3) | **done** |
| 6.4 | planar mirrors (`USceneCaptureComponent2D`) | **done, MEASURED** — sub-pixel in-editor (task 2) |
| 6.5 | Content: level, chassis mesh, materials | **done, and SEEN RUNNING** |

`SimA/Content` holds two GENERATED assets, `/Game/Maps/SimACourse` and
`/Game/Materials/M_Mirror` (`scripts/22-build-content.py`). There is still no
chassis mesh and no `UInputMappingContext`, which is why keyboard input polls
`IsInputKeyDown` rather than using Enhanced Input.

---

## Milestone 6.1: the two input paths

Both live in `driver_source.{h,cpp}` — pure C11, no UE and no Chrono, the same
arrangement as `wheel_device`. The plugin holds the only copy;
`driver/run.sh selftest` compiles *that* copy and asserts 38 properties in
about a second. Nothing here needs the wheel or the editor.

`driver_source.h` is in `Public/` (unlike `wheel_device.h`) only because
`FChronoPhysicsThread` holds a `DrvKeyboard` and a `DrvScript` **by value**, and
C++ needs complete types for that. It is safe to expose for the same reason
`chrono_shim.h` is: doubles and fixed-width integers, no layout that `-march`
or a C++ runtime can disagree about.

### Three sources, split by who owns the clock

| source | lives on | clock | reproducible |
|---|---|---|---|
| wheel | wheel thread → seqlock | the device's | no |
| keyboard | game thread publishes *intent*, physics integrates | render, then ours | no, by nature |
| **script** | **physics thread** | **`step_index × dt`** | **yes** |

Precedence is script > wheel > keyboard, selected per step in
`ChronoPhysicsThread::Run`. The single sign flip into Chrono's positive-left
convention still happens at exactly one line, downstream of all three.

### Keyboard: intent crosses, axes do not

A key is a boolean sampled once per rendered frame. Using it directly as a
steering position commands **605° per millisecond** — and the shim's FFB term
differentiates `m_steering`, so it would see a rate ~16,000× anything a hand
can produce. So the game thread publishes a `DRV_KEY_*` bitmask at 60 Hz and
the axes are integrated at 1 kHz on the physics side. Verified: 60 s of 60 Hz
key churn, worst per-step axis jump 0.008 = exactly `rate × dt`.

**Steering does not self-centre, deliberately.** Every SIM A exercise is a slow
manoeuvre holding a fixed lock for seconds; a rack that unwinds on key release
makes parallel parking impossible to perform, because a key cannot be held at
40% of lock. `Q` is an explicit "straighten up". Rates are all `[EST]` except
steering, which is derived: 605° of rim at full lock, ~720°/s hand-over-hand →
1.2 axis-units/s.

`DRV_RATE_CLUTCH_DN = 2.0` (0.5 s release, 2 s with the fine modifier) is the
one that decides whether a hill start is hard. It is chosen to be *usable*, not
correct — that needs a real clutch pedal, and CLAUDE.md's rule against tuning
hill-start difficulty blind applies.

Keys: `WASD`/arrows, `LeftShift` clutch, `LeftCtrl` fine (¼ rate), `Q` centre,
`0`–`5` and `R` gears. Polled through `APlayerController::IsInputKeyDown`
rather than Enhanced Input **because Enhanced Input needs content assets and
`SimA/Content` is empty**. Migrate once there is a pawn; the physics side will
not notice, since a bitmask crosses either way.

### Script: keyed to the step index, not to a seqlock

The obvious design — publish the script into the same seqlock the wheel uses —
is wrong. The wheel and physics threads keep independent 1 kHz deadline clocks
with up to 28.5 µs of relative jitter, so physics occasionally reads a value
twice or skips one. Invisible to a driver; fatal to a test, because a changed
verdict could be the rule change or could be the scheduler.

So the script is evaluated **on the physics thread** at
`t = (Steps − Epoch) × StepSeconds` — multiplied, not accumulated, so there is
no drift. Armed with `-ChronoScript=<name|path>`, and **only before `Launch()`**:
`drv_script_load` can `realloc` a table the 1 kHz thread scans every step, so
arming before the thread exists is what removes the need for a lock, exactly
as with the raw `Wheel` pointer.

The four analog channels **interpolate linearly; gear steps.** Not cosmetic: a
step change between two 1 ms samples is a physically impossible input that
slams an unbounded rate into the FFB damping term and a slip transient into
TMeasy. Built-ins: `idle`, `pullaway`, `slalom`, `upshift` — `upshift` crosses
neutral, because an H-pattern shifter does so on every change and that is the
path that used to drop the car through the ground.

Past the last keyframe the script **holds** its final values rather than
zeroing, because zeroing would change the trajectory a referee rule is about to
judge.

### Fixed along the way

`UChronoWorldSubsystem` did not override `DoesSupportWorldType`, whose default
is `Game || Editor || PIE` (`WorldSubsystem.cpp:29`). The editor preview world
was therefore already running a Chrono world plus two SCHED_FIFO threads, and
pressing Play created a **second** set — two Innovas at ~180 µs a step. Nothing
noticed while both were simulating a stationary car. Now gated to `Game || PIE`.

### Measured

| | |
|---|---|
| `drv_keyboard_step` | 0.011 µs |
| `drv_script_eval`, 13 keyframes | 0.007 µs |
| worst per-step axis jump, 60 s churn | 0.008 (= `rate × dt`, the bound) |
| 12,000 script samples, two passes | bit-identical |

Both are four orders of magnitude under the 1000 µs period and two under the
180 µs vehicle step. The input layer is not a budget question.

### Determinism — measured, both ways

`docker/05-verify-vehicle-shim.sh determ [script] [passes]`. Result:
**12,500 steps bit-identical**, in-process *and* across two separate processes.

Two failure modes needed two different tests, and the second is the one worth
remembering:

- **In-process** — solver state surviving `Chs_DestroyWorld`, an unreset
  accumulator, a static cache. Two passes in one process catch this.
- **Across processes** — if anything inside Chrono iterated a container ordered
  by *pointer value*, ASLR would change the summation order of a floating-point
  reduction. The sim would repeat perfectly within one process and quietly
  differ between runs — the worst possible shape for a regression suite, and
  invisible to the in-process check. The shell wrapper runs the binary twice
  and compares a printed digest. It matches.

The hash covers `bodies` **only**. `ChsSnapshot::last_step_us` is a wall-clock
measurement and would make the test fail every time for a reason unrelated to
the simulation.

`test_shim` stays a pure-C binary: `driver_source.cpp` is compiled with
`gcc -x c`, because linking it with g++ would drag libstdc++ in and destroy the
`NEEDED` check the harness exists for.

### The car did not track straight — root cause found and fixed

Found by the determinism run, not looked for: with **zero steering input**, the
car curved at ~900 m radius (+0.55 m over 30.8 m). Lateral growth was parabolic
in distance, so a constant curvature, not a settle offset.

`docker/05-verify-vehicle-shim.sh drift [script]` instruments the mechanism
rather than the symptom — road-wheel angle per side, chassis roll, front load
split. It needed **no ABI change**: `road_steer_rad[2]`, `tire_fz_n[2]` and the
chassis quaternion were already across the wall.

**Root cause: a bump-steer error.** `tie_ch` copied `TIEROD_Z` from `tie_up`,
putting the tie rod's inner and outer ends at the *same height*. For zero bump
steer the inner pivot must sit on the line joining the wishbone **inner** pivots
at the same height fraction the outer end sits between the ball joints:

```
t      = (UBJ_z − TIEROD_Z)/(UBJ_z − LBJ_z)   = 80.6%
RACK_Z = UCA_z + t·(LCA_z − UCA_z)            = −0.0769   (was −0.100)
```

23 mm out. Measured consequence: front toe swung from 0.097° unloaded to
**1.3° at static ride height** — a 13× change over a few centimetres. A real
front end has a fraction of a degree over its *whole* travel. `RACK_Z` is now
`[DERIV]`, not `[EST]`.

| | drift over ~31 m | radius | yaw | loaded toe |
|---|---|---|---|---|
| before | 0.552 m | 908 m | 1.94° | ~1.3° |
| after tie-rod fix | 0.291 m | 1836 m | 1.06° | ~0.15° |
| after steering refit | 0.336 m | 1591 m | 1.19° | ~0.10° |
| **task 14, design ride height** | **0.086 m** (32.7 m) | **~6200 m** | 0.29° | 0.10° |

### The fix broke Ackermann, and that is the real lesson

`ACK_SCALE = 0.18` and `PINION_CAL = 0.92` were **fitted against the wrong
geometry**. Correcting it collapsed Ackermann from ~90% to **3.6%** and dropped
full lock to 32.1°. An empirically fitted constant encodes a compensation for a
specific error, so fixing the error invalidates the constant — silently, and in
the wrong direction. Hand-fitting once and writing the number down is how that
trap gets set.

`vehicle/calibrate_steering.sh` makes refitting a command. It sweeps
`ACK_SCALE:PINION_CAL` pairs, building all variants **in one container run**
(image start and cmake dominate). Both constants must move together: `ACK_SCALE`
moves the steering-arm ball joint, which changes the linkage ratio as well as
the Ackermann. `gen_innova.py` reads both from the environment for the sweep.

Refitted to **`ACK_SCALE = 1.0`, `PINION_CAL = 0.98`**, anchored on the one
*published* number rather than a design intent. Toyota's 5.4 m is kerb-to-kerb,
so `R_rear = √(5.4² − L²) − track/2 = 3.877 m` and `δ_avg = 35.35°`, where
δ_avg is the **Ackermann average**, `cot δ_avg = (cot δ_in + cot δ_out)/2` — not
the arithmetic mean, which differs by ~1° at 66–89% Ackermann.

| | before | after |
|---|---|---|
| Ackermann | 106 → 88 → 79% | 66 → 89% |
| full lock (Ackermann avg) | 34.6° | 35.22° |
| implied kerb-to-kerb vs 5.4 m | +1.7% | **+0.3%** |
| static equilibrium vs 17,658 N | 0.01% | 0.06% |

**Refitted again by task 14**, and it is the same lesson a second time. That
fit was made with the front 84 mm into bump, because the springs were wrong.
At the design ride height 0.98 gives 36.74°. **`PINION_CAL = 0.954`**: inner
40.72°, outer 31.12°, Ackermann average **35.36°**, kerb-to-kerb **5.398 m
(−0.04%)**, Ackermann 44% (steer 0.4) → 88% (full lock). `gen_course.py`'s
`DELTA_MAX_DEG` and the pilot's `PARK_DELTA_MAX` carry the new figure.

### The residual is real physics, confirmed by experiment

`driver/scripts/coast.txt` accelerates then declutches — removing drive torque
while speed, geometry and tyre loads carry over. At the moment the clutch goes
in:

- **roll flips sign**, −0.284° → +0.24°
- **yaw reverses and unwinds**, 1.089° → 0.882 → 0.691 → … → 0.256°
- **y stops growing**, asymptoting at 0.545 m

So the curvature is entirely drive-torque driven. `innova_Driveline.json` sets
Motor Block direction `[1,0,0]`, so the driveshaft spins about X and its
reaction on the chassis is a **roll** moment; roll gives roll steer; roll steer
curves the car. A real solid-axle RWD car does exactly this. The dominant
*error* is fixed; what is left is the model behaving like the vehicle it models.

### RESOLVED-AS-CONSTRAINT: the car creeps backwards on flat ground

> **OVERTURNED by task 13.** The creep was the ENGINE. Its losses map is
> −20 N·m at 0 rpm, which drives it backwards, and the welded clutch connected
> it to the wheels in every case below. That is why "neutral vs 1st-open vs
> 1st-engaged" came out byte-identical. With a working clutch, neutral and
> pedal-down stand still (−0.006 m in 20 s), and only in gear with the pedal
> up does the car still creep (task 8). The genuine force-element tyre drift
> is ~4 mm/s. The reasoning below is kept as a record of how a welded clutch
> read as a tyre property.

`docker/05-verify-vehicle-shim.sh creep`. On flat terrain, from **rest**, with
no throttle: the car settles to a steady **−0.17 m/s** and travels 3.45 m
backwards in 20 s.

Eliminated by experiment, in this order:

| suspect | test | result |
|---|---|---|
| driveline / clutch / gear | neutral vs 1st-open vs 1st-engaged | **byte-identical** — driveline irrelevant |
| rolling resistance | `ROLL_RES=0` regenerate | slightly **worse** (−0.197) — it was opposing |
| spawn drop transient | settle 1/10/30 s, spawn 0.45 vs 0.26 | unchanged |
| the tyre model | swap TMeasy → `RigidTire` | **creep vanishes**: 0.010 m in 20 s |

**It is NOT TMeasy-specific.** That was the first conclusion and it was wrong;
the corrected picture, from testing a third model:

| tyre model | kind | creep over 20 s |
|---|---|---|
| TMeasy | force element | −3.45 m |
| **Fiala** | force element | **−3.66 m** (slightly worse) |
| RigidTire | real collision geometry | −0.010 m (still) |

Both *force-element* models creep; only the one with genuine rigid contact does
not. So this is a property of how force-element tyres work, not a defect in one
of them, and **swapping tyre models does not fix it.**

The mechanism follows from what a force-element tyre is. It generates
longitudinal force from **slip**, and slip requires relative motion — so it
cannot hold a static force at zero speed without explicit stiction handling.
Chrono documents TMeasy's as "simplified":

> Known differences to the commercial version:
> - No parking slip calculations
> - No dynamic parking torque
> - Simplified stand still handling
> - Standstill algorithm based on Dahl Friction Model with Damping

Its parameters — `m_begin_start_transition`, `m_end_start_transition`,
`m_vnum` — are private with no setters. Fiala appears to have no equivalent at
all, which is consistent with it being marginally worse.

Something applies a small steady longitudinal force at the contact patch (the
leading candidate is the suspension's anti-dive/anti-squat reaction to the
static vertical load, which is real and does not vanish at rest), the tyre can
only balance it by slipping, and slipping *is* the car moving. RigidTire holds
it because rigid frictional contact provides static friction. Measured: the
wheels free-roll *with* the car — `ω·R = −0.171` against `v = −0.173` — so this
is rolling, not sliding.

**The brake does not save it, and is non-monotonic:** 0.5 gives −0.62 m,
**1.0 gives −1.54 m**. Locked wheels hand the problem straight to the
standstill algorithm.

#### Why this matters more here than in most vehicle sims

"No parking slip calculations, no dynamic parking torque" is a limitation in
exactly the regime **this entire project lives in**. The SIM A test is parallel
parking, bay parking and a hill start: low speed, large steer, near standstill.

**Consequences, and they block work:**
- The **hill-start rollback rule cannot be written** against this baseline. It
  is a 0.30 m displacement threshold on precisely the quantity that drifts
  0.17 m/s unprompted — a spurious fail in under two seconds.
- Final parked positions will drift if the referee judges them after a settle.
  Judge at the moment of stopping, not after.

### Measured ON THE SLOPE — and it changes the answer

Every measurement above was on FLAT ground. A hill start is a different regime:
at 12%, gravity supplies `m·g·sin θ = 2101 N`, which might simply swamp
whatever drives a 0.17 m/s creep. `05-verify-vehicle-shim.sh hill [grade]`
spawns the car directly on a slope through the origin and measures three
things.

| | HOLD, brake 1.0, 4 s | free ROLL vs `g·sin θ` | LAUNCH, 1st, 60% |
|---|---|---|---|
| **TMeasy** | **fails** — slides 0.246 m | **−52%** (−49% with rolling resistance nulled) | fails; engine back-driven to −1390 rpm |
| **RigidTire** | holds — 0.012 m | **−99.8%** — does not roll at all | unstable: 6.6 m/s reported, 0.03 m travelled |

**Neither model is usable for a rollback rule, for opposite reasons.** TMeasy
will not hold the car with the brake fully applied and eats half the gravity
force once rolling. RigidTire holds perfectly and then welds the car to the
slope — a car in neutral with no brake on a 12% gradient must roll back, and it
does not.

The `−49%` is the important number. Rolling resistance was ruled out by
measurement (`ROLL_RES=0` moves it by 3 points, and 0.012 × 17,658 N = 212 N
against 2101 N is the ~10% it should be worth).

### Read the source: WHICH parameters, and an earlier claim corrected

An earlier note in this file named `m_begin_start_transition`,
`m_end_start_transition` and `m_vnum` as the parameters to patch. **That was
wrong**, and acting on it would have fixed nothing:

- `m_begin/end_start_transition` feed a **time**-based fade (`m_time`, 0.1 s →
  0.25 s) that ramps tyre forces in at the start of a simulation.
- `m_vnum` (0.01) is a floor on the slip denominator, `vta = R·|ω| + m_vnum`.

The parameters that actually govern this are in `ChTMeasyTire::CombinedCoulombForces`:

```
sigma0 = 100000.0   // bristle stiffness, Dahl friction model
sigma1 =   5000.0   // bristle damping
F = sigma0*z + sigma1*dz/dt
```

with a blend `m_frblend_begin = 1.0`, `m_frblend_end = 3.0` m/s. **Below
1 m/s the tyre force is 100% the Dahl bristle model.**

The mechanism is then arithmetic: the damping term is viscous in the SLIP
velocity, so `σ₁ = 5000 N·s/m` × 0.1 m/s of slip = **500 N per tyre, ~2000 N
across four**, against the 2101 N gravity component of a 12% grade. A freely
rolling wheel must have *some* slip in order to angularly accelerate, so it is
taxed nearly the entire force that should be moving it. That is the −49%.

Chrono's own comment on the model:

> *"At tire stand still it acts like a spring which enables holding of a
> vehicle on a slope without creeping (**hopefully**). ... This model is
> **experimental and needs some testing**."*

**Also found, and separately important:** `Mz` is only computed when
`vel.x >= m_frblend_begin`, so **there is no aligning moment at all below
1 m/s** — no steering feel during any parking manoeuvre.

It is also the band every PARKING manoeuvre lives in, so this is not only a
hill-start problem. It is a general low-speed fidelity defect affecting most of
the SIM A test.

**FIXED by task 1** — `CHS_TIRE_BLEND_BEGIN = 0.20` moves the gate below the
manoeuvring band; column torque at 0.21 m/s goes −0.079 → −3.256 N·m. See
"Task 1: the aligning moment below 1 m/s".

### So: which of the three options

1. **Patch Chrono** — the only route to an accurate rollback rule, and the
   source now says exactly what to expose: `sigma0` / `sigma1` and the blend
   range. Keeps TMeasy's `Mz` and its load-index-derived parameters. **Chosen.**
2. **A standstill clamp in the shim** — addresses HOLD and does nothing for the
   −49% roll deficit, and it operates in the same velocity band as the quantity
   being measured. Structurally wrong for this rule.
3. **A RigidTire scenario** — **rejected by measurement, not by preference.**
   It is the best of the three at holding and the worst at rolling, and rolling
   is what the rule measures. A rule that can never fire is not a rule. It also
   costs all force feedback and looked numerically unstable under drive torque.

`TYRE_MODEL=tmeasy|fiala|rigid python3 gen_innova.py` regenerates any of the
three if the experiment needs repeating; the default is `tmeasy`.

### The patch, and why it is carried as a patch

`docker/patches/0001-tmeasy-standstill-setters.patch` — 46 lines, one hunk,
**purely additive**: inline setters and getters beside the existing
`SetFrictionCoefficient`, which already touches `m_par`. A build with the patch
and no caller behaves identically to an unpatched one, so nothing else in the
project shifts.

Nothing is upstreamed and nothing needs to be — it is our image. But it is a
**patch file applied in the Dockerfile after the clone**, not a vendored copy
of the edited source. On a Chrono version bump a patch that no longer applies
**fails the build and names the hunk**, where a vendored file would silently
keep a stale edit or silently lose the change. The build also greps for
`SetDahlParameters` afterwards, so a silently-skipped patch cannot pass.

`patch` was added to the builder's package list. Exposed through the shim at
**ABI 4** as `Chs_SetTireStandstill` / `Chs_GetTireStandstill`, so tuning is a
runtime parameter and a sweep runs in one container invocation:
`05-verify-vehicle-shim.sh standstill [grade]`.

Against an unpatched Chrono the shim **will not compile**, deliberately:
silently ignoring the call would leave the hill start quietly broken.

### Also found: an inconsistency in the snapshot

`Chs_GetSnapshot` fills the chassis `angular_velocity` from
`rf.GetAngVelLocal()` but the spindles' from `GetSpindleAngVel`, which
`ChWheeledVehicle.h` documents as *"expressed in the global reference frame"*.
Two different frames in one array. Nothing depends on the spindle value yet, so
it has not been changed — but anything that starts using it must not assume it
matches slot 0.

**Now used, by task 7.** The rollback rule reads the spindles' spin, and
`ref_wheel_speed` handles the two frames explicitly: it projects each
spindle's world-frame angular velocity onto the body's lateral axis, then
subtracts the body's own local rate about that axis. The `creep` harness had
assumed the spindles' vector was local and got the sign of rolling backwards
(v = +ω·R, not −). Its columns matched in magnitude, which hid it.

## Milestone 6.2: the course

`course/gen_course.py` → `simA_course.json`. Same contract as `gen_innova.py`:
**edit the generator, never the JSON** (`run.sh check` fails if you do). Vehicle
dimensions are **parsed out of `gen_innova.py`**, not copied — a stale wheelbase
in a course generator would score the car against the wrong geometry and
nothing would catch it.

Frame is Chrono ISO 8855 (X forward, Y left, metres), i.e. the frame physics
already publishes, so the referee needs no conversion. Indonesia drives on the
left, so kerbside exercises are at +Y.

### The regulation problem, and how it is handled

**I could not verify the SIM A course dimensions against a primary source.**
Rather than assert them — the exact mistake Step 0 made about IMV
documentation — every course dimension is tagged **`[REG?]`** and printed next
to the **minimum this car can physically achieve**, which is derived and is not
in doubt:

| dimension | bound | derived limit | configured `[REG?]` | margin |
|---|---|---|---|---|
| slalom cone spacing | min | 5.43 m | 8.00 m | +2.57 |
| parallel slot length | min | 6.54 m (1.38× car) | 7.10 m (1.5×) | +0.57 |
| bay aisle width | min | 3.05 m | 6.00 m | +2.95 |
| hill grade | **max** | 0.423 (traction) | 0.12 | +0.30 |

So the feasibility verdicts are trustworthy statements about **this car** and
provisional statements about **the test**. Swapping in real figures is a
one-line edit and a rerun.

**Task 11 swapped them in**, from Perkapolri 9/2012 Lampiran A (see "Task 11:
the regulation, read"): slalom 7.10 m (margin +1.75), parallel 7.10 m
(+0.54), bay 2.745 × 7.10 m with a 6.00 m `[EST]` aisle (+2.94), hill 15° =
26.8% (within traction). It was more than a one-line edit: the hill gained a
bridge and a descent, and three pilot assumptions broke.

### A cross-check that fell out for free

At the measured 34.6° full lock, `R_rear = L/tan δ = 3.986 m`, which implies a
kerb-to-kerb radius of `√(L² + (R_rear + track/2)²) = 5.494 m` against Toyota's
published **5.4 m — 1.7% off**. Lock and turning circle agree.

Worth knowing: `gen_innova.py` computes `delta_avg = atan(WHEELBASE/TURN_RADIUS)`
= 27.0°, which reads the published 5.4 m as a *rear-axle-centre* radius. The
kerb-to-kerb reading gives ~35°, matching the independently-set `DELTA_MAX`.
The two differ by 8° of road-wheel angle; the course uses the kerb-to-kerb
reading because it is the one consistent with the measurement.

### Parking minima are integrated, not solved

Both manoeuvres are integrated with a kinematic bicycle model outward from the
parked pose — the same path reversed, which removes any search over start
positions. Published closed forms exist but each carries assumptions about
which corner binds, and those cannot be checked here.

The bay aisle took **two wrong answers**, both worth keeping:

1. A closed form from the swept radii gave **3.3 m** — it tracked the ICR and
   the outer corner but missed that the car is still partly *in* the bay while
   its nose is already across the aisle.
2. Integrating a bare 90° arc gave **2.4 m**, and that arc never finishes: a
   quarter circle of radius R translates the rear axle centre laterally by
   exactly R, leaving the centreline 0.29 m past the bay mouth with half the
   car still in the bay's depth.

The real single-cut manoeuvre has a straight segment, and the terminating
condition both versions lacked is that **every body corner must end at y ≤ 0**,
genuinely in the aisle. That gives 3.05 m — and that figure assumes **zero
clearance from the parked row**, so it is a geometric floor, not a design
number. Published ~6 m aisles carry margin this does not.

### `run.sh plot` earned its keep immediately

The ASCII top-down view caught two things a JSON diff would not: the start pose
was cropped off the left edge (extents were computed from cones/lines/gates
only), and the feasibility table applied `set ≥ limit` uniformly, reporting a
12% hill against a 42% traction limit as **IMPOSSIBLE**. Three of those bounds
are floors; the hill grade is a ceiling. The direction is now explicit per row.

### Blocking 6.3

- **The hill does not exist in physics.** `Chs_CreateVehicle` hardcodes one
  flat 300×300 m `RigidTerrain` patch. The ramp is geometry with nothing behind
  it — drive at it today and you drive through flat ground. Needs a new shim
  entry point and **ABI 3**.
- **The car does not track straight** (see above). Over one 8 m slalom gap the
  drift is only ~0.05 m, which the 2.57 m spacing margin absorbs — but a
  pass/fail threshold set before the cause is understood will be measuring the
  drift, not the driver.

---

## Milestone 6.2b: terrain, and ABI 3

The hill start needs a real gradient — the exercise *is* holding a car against
gravity with the clutch, and the traction analysis saying it is grip-limited
rather than torque-limited means nothing on flat ground.

### Why patches and not a mesh

`RigidTerrain::AddPatch` takes a box, an OBJ mesh, or a BMP heightmap. Mesh and
heightmap both take a **filename**, which would put file discovery, image
decoding and mesh loading behind the ABI wall for a shape that is three
rectangles. The box form takes numbers, and numbers are what a POD ABI is good
at. A ramp is a box rotated about Y.

```c
ChsWorld Chs_CreateVehicleEx(const char* data_path, double init_z,
                             const ChsTerrainPatch* patches, int32_t n_patches);
double   Chs_GetTerrainHeight(ChsWorld world, double x, double y);
```

`patches` is caller-owned and copied during the call; nothing is retained
(ABI rules 3 and 5). `Chs_CreateVehicle` is now a **wrapper** over this with
`n_patches = 0`, so ABI 2 behaviour cannot drift from ABI 3 — confirmed by the
determinism digest being unchanged across the whole change (`9e1623bf645620a4`;
re-baselined to `c829c405af66692a` by task 1 — see that section).

`Chs_GetTerrainHeight` exists for the referee: "rolled back 0.30 m" on a slope
is a distance along the **road**, and a rule using world Z or world X would be
measuring the gradient instead of the driver.

### Two silent geometry traps

Both are invisible in a JSON diff, so both are stated in the header and
asserted by `05-verify-vehicle-shim.sh terrain`:

1. The `ChCoordsys` origin is the centre of the patch's **driving surface**,
   not of the box. Chrono extrudes thickness downward from it.
2. `length` runs along the patch's **own inclined X**. A ramp over horizontal
   run *r* at gradient *g* is `r·√(1+g²)`, not *r* — using *r* makes the hill
   quietly short and leaves a step at the crest. 20 m at 12% is **20.1435 m**.

Chrono is right-handed with +Y left, so a rotation about +Y takes +X toward −Z:
**a climb needs a negative pitch** (−6.8428° here).

Patches must also **tile without overlapping**. The first version had one flat
patch spanning the whole course with the ramp on top; the flat was still
present under the ramp, giving duplicate contacts at the toe.

Verified: height profile flat → exact 12% → flat at 2.400 m, and the car
climbed to a road height of 2.400 m at x=124.11 with a 0.285 m ride height,
crossing the seam at x=104.1 without incident.

### The test failed twice first, and both failures were the test

Worth recording because both produced confident, wrong verdicts:

1. **Use-after-free.** The climb section destroyed the world, then called
   `Chs_GetTerrainHeight(w, …)` on the freed handle to compute a respawn
   height. Result: "the car fell through the hill." The terrain was fine.
2. **The drift, biting exactly as predicted.** With steering pinned at 0 the
   car had drifted `y = x²/2R ≈ 4.7 m` by x=122, and the ramp is 8 m wide —
   half-width 4 m. It drove off the *side*. The approach patch is 40 m wide,
   which is why nothing went wrong before x=104.

The second is CLAUDE.md's own warning landing: a test built on a car that does
not track straight measures the drift, not the thing under test. The climb test
now runs a proportional lane-keeper on lateral offset and heading, because a
driver steers.

---

## Milestone 6.3: the referee

`referee.{h,cpp}` in the plugin, pure C, exercised by `referee/run.sh selftest`
(31 assertions then, 59 since task 3). The course reaches it as `simA_referee.txt`,
a keyword-per-line projection of the same generator run that writes the JSON.

### Cones are not physics, deliberately

They could have been rigid bodies knocked over on contact. Three reasons they
are not:

1. `ChsSnapshot` carries 16 bodies and the vehicle uses 5. A full course has
   more cones than the remaining 11, so they could not all cross the wall.
2. Growing the snapshot grows the 64-slot ring with it — ~428 KB republished
   every millisecond, on a machine with 256 KB of L2 per core.
3. A 0.5 kg cone against 1800 kg changes the trajectory by nothing measurable.

So Chrono simulates the car and the referee is exact geometry over its pose. A
struck cone falling over is a renderer's problem.

### It drains the ring; it does not sample it

The interpolated pose the renderer draws is a function of frame timing, so a
verdict derived from it would depend on framerate. `ref_update` is fed once per
**physics** sample. On the game thread that is ~17 evaluations per frame at
60 Hz against a 64-slot ring — 3.7× margin — and if the game thread ever does
fall behind, `REF_SAMPLE_GAP` reports how many samples were missed rather than
scoring a trajectory with holes and presenting it as sound.

### Rules, and how each is defined

| rule | tested against | note |
|---|---|---|
| cone strike | body **outline** vs cone radius | outline, not interior: a cone driven over was struck when the bodywork reached it |
| line cross | body edges vs segment | proper intersection only; collinear is not a crossing |
| gate pass/skip | **travelled segment** vs gate segment | tests "changed sides" and "within the gate" in one operation |
| stall | rpm **and speed**, with hysteresis | see below |
| park (task 3) | the pose **at the stop**, once the stop has lasted 1 s | body centre in a zone; every corner ≥ 5 cm inside; heading ±5° |

Gates track the reference point, not the outline — a car whose nose has crossed
has not been through. Every object latches, so one contact fires once rather
than once per millisecond.

### Two bugs the harness caught in code written the same hour

1. **The body box was 2.75 m too far forward.** The chassis reference frame is
   at the FRONT AXLE (`gen_innova.py` puts the front hardpoints at x=0 and the
   rear axle at x=−wheelbase), so the box runs `+front_overhang` to
   `−(wheelbase + rear_overhang)` — not the body-centre convention I first
   wrote. It would have scored strikes the car never made and missed real ones.
2. **The gate test did not implement what its own comment described** — it had
   become a proximity check that would fire on the infinite line.

### The stall rule needed a conjunct, and finding out why was the point

The first end-to-end judged run reported a stall on a launch that then pulled
away cleanly to 6.4 m/s. Measured: engaging the clutch drags this engine to
**330–380 rpm and holds it there for over a second** on a successful launch,
and a run that genuinely bogs sits in the *same* band. **rpm alone does not
discriminate on this vehicle**, so no threshold or time window separates them.

Speed does: a car doing 6 m/s has not stalled whatever the tachometer says.
`REF_STALL_MAX_SPEED` is load-bearing, not a fudge, and there is a regression
test pinning it.

**Why, found by task 4:** every launch starts from 0 rpm because the clutch
cannot slip — under MINRES it is welded shut (task 13). The conjunct stays
right; the "idle governor" framing of task 8 does not.

**The underlying oddity is a VEHICLE question and is left open:** a 2.4 diesel
with 343 N·m from 1400 rpm should not be dragged below a third of its idle
speed by a 30% throttle launch, and the engine model has **no idle governor**
to recover once it is. Do not tune the referee threshold to compensate.
**Closed on the vehicle side by tasks 13 and 8**: a working clutch, an idle
governor, and an engine that really dies under 300 rpm. The whole course now
never takes the engine under 654 rpm while the clutch carries torque. The
conjunct stays, because it is still right.

### Determinism, end to end

`05-verify-vehicle-shim.sh judge [script]` runs script → shim → Chrono →
referee → verdict, headless. Two separate processes produce **identical event
logs**. That is the whole of 6.1 paying off: a changed verdict means a changed
rule, never a changed scheduler.

Judged runs are **closed-loop on lane position** (`drv_lane_keep`). The car
does not track straight and never will; open-loop it is 4.7 m off by x=122,
which would score the drift instead of the driver. Determinism survives because
the controller is a pure function of already-deterministic state.

### ~~Not implemented: hill-start rollback~~ — implemented by task 7

Blocked by the TMeasy standstill creep, not by effort. It is a 0.30 m
displacement threshold on a quantity that drifts 0.17 m/s unprompted, and
brakes make it worse. See the constraint above; the tyre-model experiment
(Fiala or Pac02) is the prerequisite. **Resolved otherwise**: the creep was
the welded clutch and the engine (tasks 13, 8), the account closes (task 6),
the threshold is the regulation's zero (task 11), and the rule counts wheel
travel (task 7).

---

## Milestone 6.4: mirrors

`mirror_math.{h,cpp}` (pure C, 53 host assertions since task 2, no GPU) plus
`UChronoMirrorComponent` driving a `USceneCaptureComponent2D`.

### Planar reflection is not an approximation here

The no-ray-tracing constraint costs nothing for this. For a **flat** mirror the
set of rays leaving the glass toward the eye is exactly the set reaching the
reflected eye position, so rendering from the reflected eye is the right answer
rather than an estimate of it. Ray tracing would compute the same image more
slowly. The approximation only appears for curved glass, and these exercises
are performed with flat mirrors.

### Three parts, all required

1. **Virtual eye** — `E' = E − 2((E−P₀)·N)N`.
2. **Clip at the glass** — everything *behind* the mirror in real space is *in
   front of* the virtual camera. Without clipping at the glass, the inside of
   the car's own bodywork renders into the mirror. **Done by the near plane
   since task 2**: Unreal's capture clip plane was inert all along (needs
   `r.AllowGlobalClipPlane`), and the corrected basis makes the glass parallel
   to the image plane, so a near plane just past it is an exact clip.
3. **Off-axis frustum** — the eye is never on a door mirror's axis. Measured on
   the door-mirror geometry: it subtends 13.4°, but a symmetric frustum
   containing it needs 50.4° — **3.8× the horizontal pixels**. That is why this
   does not use `UPlanarReflectionComponent`, which renders a symmetric frustum
   and resolves through screen-space UVs. On a GPU with headroom that is the
   better trade; on an RX 550 driving three mirrors it is not.

Cost control is a divisor, not a resolution knob: three mirrors are three extra
scene traversals and draw-call cost dominates here. A door mirror at 30 Hz
against a 60 Hz main view is a far better trade than dropping the main view.

### Winding is viewer-relative, and it is checked at runtime

`mm_offaxis_projection` requires `cross(pb−pa, pc−pa)` to face the eye. Stated
as a formula rather than as "left" and "right" because **handedness makes those
words differ between Chrono and Unreal**. Supplying corners in world-axis order
instead is the obvious mistake and it is not small — every frustum extent comes
out negated and the mirror renders inside out — so it gets its own return code
(`−2`, distinct from `−1`) and the component reports it rather than trusting it.

The test suite caught this in its own fixtures first: every projection assertion
failed until the corners were rewound.

### Verified, and what is not

Verified without a GPU: reflection is an involution, points on the plane are
fixed, distance flips sign; the mirror's **four corners map exactly to the NDC
edges** and its centre to (0,0) — so the frustum is the mirror aperture and
nothing outside it is rendered; an on-axis eye gives symmetric extents and an
off-axis eye asymmetric ones; degenerate, reversed and behind-the-mirror inputs
are rejected rather than approximated.

~~**Not verified: the sign of the off-axis skew**~~ — **SUPERSEDED by task 2.**
This paragraph treated the skew as a one-bit unknown to settle by eye
(`bFlipOffAxisSkew`). It was not one: the component composed the Unreal
matrix by hand, in a basis the tests above never saw, and no sign could have
fixed it. The composition now lives in `mm_ue_mirror_capture` and is tested
like everything else here. See "Task 2: the mirror, end to end".

---

## Milestone 6.5: content, generated rather than authored

`SimA/Content` was empty and had become the critical path — it blocked seeing
the mirrors, confirming their two unverified signs, and driving the course.

### Engine primitives, not authored meshes

`/Engine/BasicShapes/{Cone,Cube,Cylinder,Plane}` are real static meshes
referenceable by path, so **nothing needed authoring to get a visible scene**.
A driving-test cone really is a cone and a painted boundary really is a thin
box, so this is not only a stopgap.

- `AChronoCourseActor` — cones, boundary lines, gates, ground and ramp.
- `AChronoVehicleActor` — chassis box, four cylinder wheels, three mirrors, a
  camera. Driven from `GetFrameState()`, owning no physics.
- `AChronoGameMode` — spawns both plus a sun and sky.

### The course actor reads the referee's own files

`simA_referee.txt` and `simA_terrain.txt`, the exact files fed to
`ref_load_course` and `Chs_CreateVehicleEx`. **One source, three consumers.**
Authoring the visuals separately would create a second source of truth, and the
failure mode is the worst kind: a car that visibly misses a cone the referee
scores as struck, with nothing in either system wrong on its own.

The same reasoning is why `20-make-level.py` bakes the *actors* into a level
but never their geometry — regenerate the course and reopen the level, and the
new numbers are picked up.

### Unreal collision is OFF on everything

Chrono owns the physics. Leaving Chaos collision enabled on the course would
let a second physics engine push the car around behind Chrono's back — two
engines disagreeing about one scene, which is the failure the ABI wall exists
to keep out of this project.

### The mirror material is unlit, and that is not laziness

The render target already holds a fully lit image; it came out of a real scene
capture. Feeding it to Base Color would light it a **second** time, from the
lights near the glass, and a mirror would darken in shadow and bloom in
sunlight while the world it reflects did not.

There is also **no screen-space UV math**, which is a dividend of the off-axis
projection: the capture frustum *is* the mirror rectangle, so the target maps
1:1 onto the glass. UE's `UPlanarReflectionComponent` needs a screen-space
projection precisely because its frustum is not the mirror.

Materials cannot be authored at runtime — `UMaterialInstanceDynamic` retargets
parameters but cannot build a graph — so `21-make-mirror-material.py` creates
`/Game/Materials/M_Mirror` once, and the component makes a dynamic instance and
points `MirrorTexture` at its own render target. The parameter name is
duplicated in the script and in `ChronoMirrorComponent.cpp`; a mismatch is
silent and shows as a mirror stuck on the default texture, so both sides say so.

If the material is missing the capture still runs and `GetMirrorTarget()` still
works — there is just nothing showing it.

### The GameMode is idempotent, and that is why

`20-make-level.py` bakes a course and a vehicle into `/Game/Maps/SimACourse`
so they can be refined in the editor. A GameMode that spawned unconditionally
would then produce a **second** course and a second car on that map — two
vehicles reading the same physics stream and drawn on top of each other, which
reads as a rendering bug rather than a duplicate actor.

So it looks for existing actors instead of taking a config flag. The right
thing happens on a bare map and on an authored one, with no setting to get
wrong when you switch between them.

### Running with no level at all

`DefaultEngine.ini` now points at `/Game/Maps/SimACourse`, but nothing depends
on that: with it set to `/Engine/Maps/Entry` (the engine's minimal map) the
GameMode builds the whole scene itself, so the project ran before any asset
existed and still would. `scripts/22-build-content.py` runs both generators in ONE editor
launch, because the initial asset scan dominates a commandlet's cost and paying
it twice is waste.

### Three unverified signs — how each was resolved (task 2)

The three were exposed as flags rather than guessed, the M5 force-feedback
handling. Two turned out not to be one-bit questions at all:

| flag | resolution |
|---|---|
| `bFlipOffAxisSkew` | **deleted** — the basis was wrong, not the sign; no setting could work |
| `bFlipMirrorU` | **`false`**, from a measured UV read-off plus the capture's derived image axes |
| mirror placement / `DriverEyeChrono` | aimed by the bisector law; seat moved to the RIGHT (RHD) |

---

## The hill start: what actually fixed it

Three rounds of this investigation blamed the tyre model. **It was ours.**

### Root cause: the M5 neutral workaround, not TMeasy

The shim never passes neutral to the transmission (Chrono sets a 1e20 ratio and
MINRES diverges); it holds the last gear and forces the clutch open. Torque is
broken at the clutch — but the **inertia** between clutch and wheels is still
geared to them, and reflected inertia goes as the SQUARE of the ratio.

Held in 1st (4.313 × 3.909 = 16.9:1) the driveline is worth **2047 kg
equivalent against an 1800 kg car**. A car the driver has put in neutral drags
more than its own mass in phantom rotating inertia.

Predicting free roll as `m_eq/(M + m_eq)` and measuring all five gears:

| gear | equiv mass | predicted | measured |
|---|---|---|---|
| 1st | 2047 kg | −53.2% | **−52.4%** |
| 2nd | 726 kg | −28.7% | −33.0% |
| 3rd | 388 kg | −17.7% | −20.5% |
| 5th | 251 kg | −12.3% | −14.6% |

Measurement tracks prediction across the whole range, which is what identifies
the cause. **No tyre parameter moves it** — σ₁ from 5000 to 1 changed the
deficit by 2 points, σ₀ across 1e5→1e8 by 1.7.

### The fix, and what it bought

One line: in neutral, hold the gearbox's **highest** gear rather than its last.
Smallest ratio, least reflected inertia.

| | before | after |
|---|---|---|
| free roll vs `g·sin θ` | −52.4% | **−14.6%** |
| launch on a 12% slope | fails; engine back-driven to −1390 rpm | **pulls away at 4.9 m/s** |

**The hill start works now.** It is a mitigation, not a fix: the real one is a
true neutral, which means patching `CHRONO_NEUTRAL_GEAR_RATIO` from `1e20` to
something finite (1e4–1e6 decouples the input inertia just as well without
destroying conditioning). That would also delete the clutch-forcing workaround.
**Done by task 10, without the patch**: under PSOR the 1e20 is harmless.

### Being straight about the Chrono patch

`0001-tmeasy-standstill-setters.patch` **did not fix this.** It is additive and
harmless, it is what made the σ₀/σ₁ sweeps possible — and those sweeps are how
the Dahl parameters were *ruled out* — and reading that source is where the
"no `Mz` below 1 m/s" finding came from. But it solved nothing on its own, and
the container rebuild it cost was not what unblocked the hill start.

### Still open: the brake will not hold on the slope

> **RESOLVED by task 15.** The brake was `ChBrakeSimple`, whose sticking is
> dead code in Chrono 10: at rest it chattered, and nothing could settle.
> `BrakeShafts` holds the car on 12–45% by itself. The narrowing below was
> right that it was not the Dahl σ's or driveline inertia; it was the brake.

`HOLD` remains ≈ −0.28 m over 4 s with brake 1.0 on a 12% grade. Narrowed by
measurement to **not** driveline inertia and **not** the Dahl σ's. Brakes are
sized to lock at 1 g and holding needs 0.12 g, so either the brake is not
delivering its torque or the locked-wheel contact is not delivering μ·N. That
is the remaining blocker for the rollback rule.

---

## First actual run — what rendering revealed

Run with `-game -ChronoShot=<s>`, which screenshots itself and quits.
Everything before this was "compiles and passes host tests"; six runs turned
that into a picture.

**`ShaderCompileWorker` was never built.** Only `SimAEditor` ever was, so the
first launch stalled on a dialog. `Build.sh ShaderCompileWorker Linux
Development` — 8 minutes, one time.

### The physics survives contact with the renderer

Measured in the real editor with the scene drawing:

```
physics: 41344 steps, step 230.4/637.8 us (mean/max),
         late 0.7/49.8 us, missed 0, rt=yes
```

**Zero missed ticks, 0.7 us mean lateness, SCHED_FIFO granted.** Milestone 3's
timing holds under full rendering load, which no host test could have shown.

**Re-measured by task 5** with the car as it is now (PSOR 50, BrakeShafts,
the governor), 110 s of steady play from the session start, the car driving
(`circle.txt`): step **205–220 us mean, 1043 us max**, lateness 0.8–1.1 us
mean, 334 us max, **2 steps over budget in ~109,000**, `rt=yes`. The two are
isolated, 60 s apart, and 2–4% over. A fixed-step clock catches up on the
next step, so neither the referee nor the rendered motion can see them.

### Four rendering bugs, in the order they were peeled off

1. **Black screen — Lumen.** UE5 defaults `DynamicGlobalIlluminationMethod` to
   Lumen, and Step 0 established this GPU has no ray tracing. The scene
   rendered black while the log cheerfully reported 23 course meshes built.
   Fixed in `DefaultEngine.ini`: GI and reflections off. The Step 0 hardware
   constraint arriving three milestones later in a form nobody predicted.
2. **Static mobility.** Components created in `OnConstruction` default to
   Static; with `AllowStaticLighting=False` and no lightmaps they receive no
   light at all. The chassis was visible only because it is created at
   `BeginPlay`. Now forced Movable before registering.
3. **World-grid checker.** Making the dynamic instance from the mesh's OWN
   material inherits the engine primitives' checker and has no colour
   parameter. Load `BasicShapeMaterial` explicitly instead.
4. **Auto-exposure.** Metered on the car -- the one bright object -- and
   crushed everything else to black. Fixed exposure on the camera.

### Two ordering bugs the log caught

- **Properties set after `RegisterComponent()` do nothing.** Registering into a
  world that has begun play runs `BeginPlay` immediately, and the mirror's
  `BeginPlay` sizes the glass. All three mirrors logged the 25x6 default; the
  door mirrors were meant to be 16x9.
- **The chassis box swallowed the wheels.** Spanning z = 0 to 1.4 m put the
  underside on the ground with the 0.34 m wheels inside it. They were built and
  placed correctly all along.

### What the frame shows

Ground plane in correct perspective, the Innova chassis with a shadow, the
**hill at the horizon as a raised platform**, the slalom cones as an orange
dotted line down the centreline, and the white boundary lines -- all where
`gen_course.py` puts them. Course, terrain and referee agree because they read
the same files.

### Still open after six runs

- ~~Mirror winding returns `rc=-2`~~ — **fixed, and it was two bugs.** See
  below. ~~No reflection has been seen yet~~ — **resolved by task 2**: the
  first suspect was right (single-sided glass facing away, `face . normal =
  -1.00`), and three more faults sat behind it. See "Task 2: the mirror, end
  to end".
- Wheels are black-on-shadow and effectively invisible. Cosmetic.
- The referee logs `SAMPLE_GAP` continuously while the framerate is low, which
  is the mechanism working correctly -- and a reminder that a verdict taken
  during shader compilation is not sound.

---

## The mirror `rc=-2`: two bugs wearing one symptom

### 1. Wound for the wrong eye

`mm_offaxis_projection` needs `cross(pb-pa, pc-pa)` to face the eye it is
given — and for a mirror that eye is the **REFLECTED** one, because the virtual
camera sits behind the glass. The reflected eye is on the opposite side of the
plane from the driver *by construction*, so corners wound to face the driver —
the intuitive choice, and the one the component made — face exactly the wrong
way, always.

The winding now lives in **`mm_mirror_corners`**, in the tested library rather
than inline in the component, because the rule is counter-intuitive enough that
it needs a test standing next to it.

**The test that was missing:** every earlier projection assertion fed
`mm_offaxis_projection` an eye already on the correct side, so none exercised
what the component actually does — *reflect first, then project*. All 24
assertions passed while three mirrors reported `rc = -2` on every frame of
every run. The new `test_full_mirror_pipeline` covers the whole path and
asserts the old winding is still rejected rather than silently accepted.

### 2. The door mirrors pointed outboard

Fixing the winding flipped *which* mirrors failed — Interior and Left started
working and **Right broke**, which is what exposed the second bug. The yaw sign
must angle each door mirror **IN toward the centreline**: the driver sits on
one side and must be able to see the mirror on the other.

Measured, `dot(normal, eye − mirror)` with the driver's eye at its `[EST]`
position:

| | outboard yaw | inboard yaw |
|---|---|---|
| right mirror | **−15.4** (driver behind the glass) | +97.0 |
| left mirror | +16.7 | +64.9 |

The interior mirror is unaffected — it is already on the centreline, which is
why it was never the one that revealed this.

All three now initialise with **no warnings**.

---

## The brake on a slope: a parking hold, and what it revealed

> **Superseded by task 15.** The hold described here was compensating for a
> brake that could not stick. It is now planar, load-limited, and OFF by
> default. See "Task 15: the brake that could not hold".

**Measured problem.** Brake fully applied, 12% grade: the car slid a steady
0.06 m/s, 0.246 m over 4 s. Constant velocity means the tyres produced exactly
the 2101 N gravity component — but only *while sliding*.

**Ruled out by measurement, not by argument:**

| suspect | test | result |
|---|---|---|
| brake torque | sized to lock at 1 g; holding needs 0.12 g | ~10× oversized, not the limit |
| driveline inertia | the neutral-gear fix | fixed ROLL, left HOLD unchanged |
| Dahl damping σ₁ | 5000 → 1 | hold moves < 15% |
| Dahl stiffness σ₀ | 1e5 → 1e8 | hold moves < 15% |

The σ sweeps now **read the values back** through `Chs_GetTireStandstill` and
print them, because a silently-failed `dynamic_pointer_cast` would make every
row identical — which is exactly what a real null result looks like. The
read-back confirms the sweep was real. So this is a genuine null: the Dahl
bristle model is not what limits the hold.

### The fix: `Chs_SetParkingHold` (ABI 5)

While the brake is applied **and** the car is nearly stopped, a force is
applied to the chassis opposing its motion, sized to arrest it in one step and
**clamped to the friction circle** (`µ·m·g`). The clamp is what keeps it
honest: it can never hold more than the tyres physically could, so a slope
steeper than the tyres can hold still slides.

**Why this is safe for the rollback rule when a general standstill clamp is
not:** it is conditioned on the BRAKE, and rollback is measured after the brake
is RELEASED. The two never overlap. A speed-only clamp would sit in exactly the
band the rule measures, which is why that option was rejected.

| | before | after |
|---|---|---|
| HOLD, brake 1.0, 4 s | −0.246 m | **−0.005 m** |

### What it revealed, and why the rule is still blocked

ROLL went from −14.6% to **−45.5%**. The hold is inactive during that phase, so
it did not *cause* this — it *revealed* it. Previously the car entered the roll
phase already sliding at 0.06 m/s, which masked the tyre's behaviour from rest.
Now it starts genuinely stopped, and from rest the Dahl bristle **sticks**.

So the three states are:

- moving → tyre produces force correctly
- braked and stopped → held (by the parking hold)
- unbraked from rest on a slope → **sticks, and should not**

The last one is what the rollback rule measures. **Do not implement the 0.30 m
rollback threshold until breakaway-from-rest is understood.** The next
experiment is to instrument per-tyre longitudinal force during the ROLL phase
(`ReportTireForce().force.x`, an ABI 6 field) and watch it against the bristle
deflection — that distinguishes "stuck bristle" from "clamped at the friction
circle" and nothing else does.

---

---

## Task 1: the aligning moment below 1 m/s

`ChTMeasyTire::Advance`, unchanged Chrono source:

```cpp
double Mz = 0;
if (m_data.vel.x() >= m_frblend_begin) {   // stock 1.0 m/s
    Mz = AlignmentTorque(Fy);
}
```

Below `m_frblend_begin` the aligning moment is **identically zero**. Both
parking exercises and the tight end of the slalom are performed entirely under
1 m/s, so the wheel was dead through most of the test. `sweep` never showed it
because `sweep` measures at 8 m/s — eight times above the gate.

Adopted: **`CHS_TIRE_BLEND_BEGIN = 0.20`**, `blend_end` left at Chrono's 3.0.
Applied in `Chs_CreateVehicleEx`, so the harnesses and the plugin get the same
vehicle. No ABI bump — no struct changed.

### What it bought

`05-verify-vehicle-shim.sh mzlow` holds a 0.60 parking lock and uses a
proportional controller to hold a speed, then averages column torque over 1 s.

| v (m/s) | stock 1.0/3.0 | adopted 0.20/3.0 | |
|---|---|---|---|
| 0.09 | −0.028 | −0.028 | still gated, and rightly |
| 0.21 | −0.079 | **−3.256** | 41× |
| 0.39 | −0.127 | **−3.525** | 28× |
| 0.65 | −0.181 | **−3.622** | 20× |

The residual below the gate is not Mz — Mz is provably zero there. It is the
contact-patch-to-wheel-centre moment transfer and camber leakage in
`ReportTireForce`'s global-frame moment. It carries no steering information.

**0.20 and not 0.10.** At 0.10 the two front tyres straddle the gate during a
0.09 m/s crawl: one contributes Mz and the other does not, so the rack gets
half a signal with a 1.9 N·m p2p wobble on it. 0.20 keeps both tyres on the
same side of the gate throughout the manoeuvring band.

### The regressions, which is where the work was

CLAUDE.md's own brief warned this constant is shared with the Dahl standstill
blend. It is:

- **creep** — unchanged to three decimals across blend_begin 1.0 / 0.30 / 0.10.
  Worth noting *why*: the creep runs at ~0.17 m/s, which is inside the Dahl
  band at 0.30 and outside it at 0.10, and the numbers are identical either
  way. That independently confirms the existing conclusion that the creep is a
  force-element-tyre property rather than a Dahl artefact.
- **sweep** — peak column torque 7.044 → 7.046 N·m at steer 0.45, trail
  collapse still present, friction-circle onset still steer 0.70. Unaffected
  because it runs at 7–8 m/s.
- **hill 0.12** — HOLD −0.0043 m (stock −0.0048), still holds. ROLL −50.4%
  against stock −45.5%. LAUNCH still pulls away up the slope.

The ROLL row is worse on the distance integral and **better on the thing task
6 is about**: speed after 2 s of free roll from rest goes −0.092 m/s (stock, i.e.
stuck) to **−0.867 m/s**. Both are far from the −2.338 gravity alone gives, so
neither supports a rollback rule — task 7 stays blocked.

**Do not read the hill numbers tightly.** Sweeping blend_begin 1.0 → 0.10 at
blend_end 3.0 gives roll errors of −45.5 / −69.2 / −29.0 / −97.4 / −50.4 /
−46.4%. The sim is deterministic, so that spread is real parameter sensitivity,
not run noise — but it means the hill start cannot resolve a 5-point
difference and no tuning should ever be justified by one.

### REJECTED: blend_end 8.0, and it was tempting

`begin..end` sets how much of the Dahl force survives at a given speed, and
σ₁'s viscous term is what damps TMeasy's chatter. So the ripple that lowering
`begin` lets through can in principle be bought back by raising `end` — without
re-gating Mz. Measured, and it works:

| v (m/s) | col p2p, end=3.0 | end=5.0 | end=8.0 |
|---|---|---|---|
| 0.99 | 2.210 | 0.077 | 0.109 |
| 1.82 | 7.589 | 2.672 | **0.037** |
| 2.86 | 8.441 | 4.524 | 1.674 |
| 4.86 | 5.154 | 5.200 | 4.004 |

It also made the hill sweep *repeatable* — −33.0 to −36.1% across all six
blend_begin values instead of −29 to −97 — which is exactly what task 7 needs.

**And it stops the car launching.** Same reason σ₁ taxes free roll: a driving
wheel needs slip, and the Dahl damping term is viscous in slip, so extending
the Dahl window to 8 m/s taxes drive force across the entire launch range.
Measured on a 12% grade, 1st, 60% throttle: the car never pulls away and ends
**11.2 m further down the hill after 8 s**, against +2.25 m for the adopted
setting. Not adoptable at any ripple benefit.

### Three things this turned up that are NOT this task

**1. A pre-existing mid-band chatter, and it is worst at stock.** Column torque
at 2.86 m/s swings **8.44 N·m p2p about a −4.13 N·m mean** — through zero — with
Chrono's stock blend range. Ruled out as a harness artefact by instrumenting
the controller: pedal p2p ≤ 0.012 and speed p2p ≤ 0.010 m/s in every row, so
the swing is the tyre's. This was always there, is untouched by task 1, and is
a live force-feedback defect: 1–3 m/s is slalom and parking transit speed.
`blend_end` fixes it and costs the hill start, so it needs a different fix.

**2. `hill_measure` disagreed with `hill` by 29 points, and the sweep was the
one that was wrong.** `hill_measure`'s `hold_gear` defaulted to 1, from when
the shim held the *last* gear through neutral. It now always jumps to the
highest, so `hold_gear` cannot hold anything — it only changes which gear the
2 s settle runs in, and the subsequent shift to top puts a transient into the
phase being measured. `blendhill` initially reported a beautiful flat null
(−16% everywhere) that was entirely this artefact. Fixed: `hill_measure` now
passes `hold_gear = 0`, which is `do_hill`'s protocol, and the two agree
(−35.4% both). **A sweep whose rows disagree with the single-shot report is
worse than no sweep — it reads exactly like a null result.**

**3. `driveline` mode's premise is void** (deleted by task 10, with `hold_gear`). It sweeps `hold_gear` to attribute
the roll deficit to reflected driveline inertia. Since the top-gear neutral fix
that sweep cannot vary what it claims to: it now reports −16.2 / −15.6 / −27.3
/ −70.0 / −35.4%, which is neither monotone in the ratio nor close to its own
printed prediction. Its conclusion text is stale. Left alone — it is task 10's
territory, and a true neutral deletes the workaround it was written to probe.

### Harness

```bash
bash docker/05-verify-vehicle-shim.sh mzlow [blend_begin] [blend_end]
bash docker/05-verify-vehicle-shim.sh blendhill [grade] [blend_end]
bash docker/05-verify-vehicle-shim.sh hill  [grade] [blend_begin] [blend_end]
bash docker/05-verify-vehicle-shim.sh sweep [blend_begin] [blend_end]
bash docker/05-verify-vehicle-shim.sh creep [blend_begin] [blend_end]
```

With no blend arguments every mode uses the shim default. **Pass `1.0 3.0`
explicitly to measure stock Chrono** — a non-positive argument now means
"leave the shim default", not "leave Chrono's". `assemble` prints the range in
use so it is visible without going looking for it.

Determinism digest re-baselined **`9e1623bf645620a4` → `c829c405af66692a`**
(`determ pullaway`, in-process and cross-process). Expected: the tyre
parameters changed, so the trajectory did.

> **Task 13 correction.** All of the above was measured with a spool rear axle
> (the welded differential). The spool made the car understeer at full lock,
> which loaded the front tyres' slip and so their `Mz`. With an open
> differential, blend 0.20 at 0.30 m/s gives **−0.330 N·m** where this table
> says −3.256. That is still 8× the gated −0.040, so the gate fix stands, but
> parking-speed feel is weak. TMeasy has no parking (bore) torque, which is
> what a real car's wheel mostly feels at this speed. The 8 m/s curve did not
> move (peak 7.046 → 7.033 N·m).

---

## Task 2: the mirror, end to end

Two sessions. The first (2026-09-05) made the glass show its render target and
hit its usage limit mid-diagnosis of the frustum; its work was never committed
on its own and reached the repo only in the squashed `masukan pertama`
commit, which is why none of it was recorded here. The second (2026-09-27)
finished the diagnosis, fixed the frustum, corrected one of the first
session's conclusions, and built a measurement.

### Four faults between "black" and "a reflection"

"The mirror is black" was one symptom of four independent faults, any one of
which produces it alone:

| fault | found by | fix |
|---|---|---|
| glass culled — a +90° pitch points the Plane's face AWAY from the driver | `face . normal = -1.00`, logged at BeginPlay | two-sided `M_Mirror` |
| `bFlipMirrorU` set a `FlipU` parameter the material never had — silent | reading `21-make-mirror-material.py` | real `FlipU`/`FlipV`/`SwapUV`/`UVDebug`; the texture bind is read back and logged |
| interior mirror saw only sky: zero pitch, eye 15 cm below it, cone +14.7°..+21.8° | its dumped target, 100% black | mirrors AIMED by the bisector law, `N = normalize(normalize(E−M) + normalize(A−M))` |
| image turned 90° on the glass — the stood-up Plane's U runs DOWN it | dumped target against the same frame's glass | `bSwapMirrorUV = true` |

What separated them was `DumpTargetToDisk` on every `-ChronoShot`, into
`Saved/MirrorDumps/`. A scene in the PNG means everything upstream of the glass
works; three earlier render attempts had been spent unable to tell a black
capture from a good capture the glass was not showing.

A FIFTH was hiding behind the frustum bug, and fixing the frustum exposed it —
see "The clip plane was never on" below.

### The frustum: the wrong basis, not the wrong sign

With an image finally visible, the runtime corner check logged the glass
corners **1.8 to 10.8 NDC units** off the edges — with EITHER skew sign. The
component combined Kooima extents from `mm_offaxis_extents` with a look-at
rotation toward the glass centre, and those live in different bases:

- Kooima measures the extents in the mirror's screen basis `(vr, vu, vn)` with
  `vr = pb − pa = −right`. That basis is LEFT-handed — a reflection flips
  handedness — and **no rotation can reproduce it**.
- An Unreal capture renders in `inverse(its rotation) × axis-swap` and uses
  `CustomProjectionMatrix` exactly as given — read in `SceneCaptureRendering.cpp`
  (UE 5.8.2, around line 1180), not assumed.

Established on the host before anything changed: a Python model of the
component reproduced all 24 logged corner coordinates (3 mirrors × 2 skew
settings × 4 corners) to **0.017 NDC**. In that model no combination of skew
signs, even independent per axis, did better than 1.8 / 4.3 / 10.8. With the
capture oriented in the glass's own basis the corners came out exact to
**1e-14** — but only with OPPOSITE signs on the two off-centre terms relative to
Kooima's extents, which is why one shared `bFlipOffAxisSkew` could never have
been right.

**Fix.** `mm_ue_mirror_capture` does the whole capture in Unreal's conventions:
the reflected eye; the basis `forward = normal`, `up = up`,
`right = up × normal` (Unreal's `Y = Z × X`, exactly what
`FRotationMatrix::MakeFromXZ` builds); the extents measured in THAT basis; and
the projection in `FMatrix` layout. Both off-centre terms then take the
textbook sign. The component copies the result; no hand-built matrix is left.

Forward along the glass normal matters beyond the corners. The image plane is
then parallel to the glass, so the glass reaches the target by a pure scale and
offset and the render target sits on the glass's UVs exactly, across the whole
surface. A look-at basis that got the four corners right would still warp
everything between them.

**Why it survived a whole milestone:** `mirror/run.sh` tested
`mm_offaxis_projection`, which the component never called. The tested code and
the shipped code diverged at exactly the line nobody could check. Now the
shipped composition IS the tested function, with 19 new assertions: each corner
on ITS OWN NDC corner (not merely `|ndc| = 1` — a mirrored frustum also puts
four corners on the edges); the aimed-at point landing dead centre, which
checks the bisector law and the projection against each other; the law of
reflection for an arbitrary world point; and a Kooima cross-check that pins the
sign asymmetry. Mutation-tested: negating the x term, reversing the handedness,
sharing one sign, or skipping the eye reflection each fails 3 to 15 of them.

### The clip plane was never on

With the frustum fixed, the first in-editor run passed the runtime corner
check on all three mirrors (`worst error 0.00e+00`) — and every render target
was 100% black, all nine dumps byte-identical. The corrected frustum had made
it worse, which is the clue: the glass lies exactly ON the aperture, so with a
correct frustum it covers the capture's entire view, and it is two-sided and
shows the capture's own target. A feedback loop, black forever. The wrong
frustum had put most of the view beside the glass, which is why images were
seen before.

The glass should have been cut away by the capture's clip plane, and
`bEnableClipPlane` had been `true` since Milestone 6.4. It never did anything.
Unreal's own comment on it: *"The global clip plane must be enabled in the
renderer project settings for this to work"* — `r.AllowGlobalClipPlane`,
default 0, read-only at runtime (`BasePassRendering.cpp:120`), and not set in
`DefaultEngine.ini`. The third silent no-op in this component, after the culled
glass and the `FlipU` parameter that did not exist. The "bodywork" the first
session saw in the left door mirror was most likely the chassis box's front
face, seen from the reflected eye in front of the car, with nothing clipping it.

**Not fixed by enabling the cvar**: that forces a full engine shader recompile
and, by its own description, ~15% more base-pass time — on an RX 550 driving
three captures. The corrected basis gives the clip for free instead: the
capture looks along the glass normal, so the whole glass is at one view depth,
and a near plane `kNearPastGlass` = 0.5 cm past it removes the glass and
everything between the reflected eye and it, exactly. `mm_ue_mirror_capture`
places it; five host assertions pin it (the glass and a point between eye and
glass clipped, a point just past the glass and the aimed-at point kept); and
the runtime check now also reports the glass's reversed-Z depth, which must be
> 1, and says `GLASS NOT CLIPPED` otherwise. `bEnableClipPlane` is now `false`,
with the reason beside it.

### `bFlipMirrorU` had been set on a wrong premise

The first session read the glass's UV layout off a frame with `-ChronoMirrorUV`
(swap on, both flips off): the glass's +Y edge — the driver's screen-left on
the interior mirror — sampled u = 1, and its top edge v = 1. Both readings are
sound. It then set BOTH flips true, reasoning that `pa`, on the +Y edge, is
target u = 0. That holds in Kooima's basis only. The capture's image +x is the
glass's +Y, so u = 1 belongs on the +Y edge: the raw U mapping was already
right, and with U flipped the reflection is left-right reversed. The V flip is
right — target row 0 is the top, the glass's +Z edge — and the frame agreed
(road below sky once it was set).

### The driver sits on the right

`DriverEyeChrono` was `(−0.55, +0.38, 1.15)`. Chrono's +Y is LEFT, so that was a
left-hand-drive seat. Indonesia drives on the left and the Innova sold there is
right-hand drive, so it is now `−0.38`. `gen_innova.py`'s chassis
`Driver Position` had the same error and is fixed the same way. The
regenerated JSON differs in that one number, and Chrono reads it for its
driver's-eye camera, not for dynamics — confirmed: `determ pullaway` still
gives `c829c405af66692a` with it (see below). The mirrors are aimed FROM the
eye, so the seat change re-angles every glass.

### The measurement: `mirror/run.sh render`

`-ChronoMirrorProbe` floats two unlit markers (magenta, green) behind the car,
each on the ray from the reflected eye through a chosen off-centre glass point
`(S, T)`, 4 m past the glass. By the law of reflection the driver sees each
marker AT that glass point, so its place is predicted twice: in the target at
`u = (S+1)/2, v = (1−T)/2`, and on screen at the glass point projected through
the player's camera. `LogProbeExpectations` logs both from the frame that is
dumped; `probe.py` finds the blobs and reports the error in pixels.

- The layout is asymmetric in both axes, so a mirrored, flipped or transposed
  image moves every marker — a wrong U flip by about 280 px.
- Each capture hides the other mirrors' markers; the rear views overlap.
- On screen, the search is confined to the looked-at glass's own bounding box
  (logged as `glass_box`): in the interior-mirror view the left door mirror
  sits ~26.6° off axis against a ~27.6° half-FOV, so its glass — carrying its
  own markers in the same colours — can share the frame.
- Markers are drawn with `M_Mirror` sampling a 4×4 target cleared to their
  colour: no asset of their own, and unlit, because a lit sphere's shadowed
  half drops out of a colour threshold and drags the centroid toward the sun.
- `probe.py` was itself checked on synthetic frames using all five PNG row
  filters: 0.1 px error on correct markers, and a FAIL on one moved 40 px.

**Results — `bash mirror/run.sh render`, 2026-09-28, three runs, one per mirror:**

| check | result |
|---|---|
| runtime corner check, Unreal's own matrices | worst error **0.00e+00**, all 3 mirrors; glass clipped at ndc z 1.005–1.012 |
| markers in the render targets | **0.06–0.17 px** from prediction, all 6 |
| markers on the glass, in the screenshot | **0.14–0.24 px** from prediction, all 6 |

That covers the frustum, the capture's orientation, the near-plane clip, the
glass's UV swap and both flips — `FlipU = false`, `FlipV = true`, measured,
not argued.

**The screen check first failed by 11–21 px, and it was the measurement, not
the mirror.** The whole rendered glass was translated by the same amount as its
markers, while the same markers sat within 0.2 px in their targets — so the
prediction was looking through a different camera. It was: in `UWorld::Tick`
(`LevelTick.cpp`, UE 5.8.2) timers fire after `TG_PostPhysics` has moved the car
but before `UpdateCameraManager` refreshes the view, so `ProjectWorldLocationToScreen`
put this frame's glass through last frame's camera — and the car was creeping
(no brake in these runs). `LogProbeExpectations` now calls
`PlayerCameraManager->UpdateCamera(0)` first; the error dropped to 0.24 px. Worth
remembering for any future logic that projects to screen from a timer.

### Kept, and removed

Kept, all opt-in or once per run: `-ChronoMirrorUV`, `-ChronoMirrorProbe`, the
target dumps, the corner-NDC log line (now with the glass depth), the
glass-facing log line, and the texture-bind read-back. Removed:
`bFlipOffAxisSkew` and `-ChronoMirrorSkew`. Switched off, with the reason beside
it: `bEnableClipPlane`.

### Rebuilding the toolchain did not move the physics

The Chrono image had to be rebuilt from scratch for this (the old one was gone
with the move). The new one is Chrono 10.0.0 with GCC 16.2.1 and glibc 2.43 —
the old image's GCC was never recorded, so it is now:
`determ pullaway` still gives **`c829c405af66692a`**, bit-identical in-process
and across processes, WITH the seat change in the vehicle JSON. That one run
proves both that the rebuilt toolchain reproduces the old trajectory and that
`Driver Position` touches no dynamics.

---

## Task 3: parking completion

### The rule

A park is judged on a STOP, and the judgement has three parts:

| | decision | why |
|---|---|---|
| **when** | a stop (`|v|` < 0.05 m/s, after having moved > 0.15) that LASTS 1.0 s; judged pose = the one recorded AT the stop, and the events are stamped with the stop time | the design note: a force-element tyre drifts at rest, so a settled pose measures the tyre, not the driver. The dwell keeps a pause to change gear from being scored as a finished park |
| **where** | every zone whose interior holds the **body centre** | a car waiting beside a space, overlapping it, is not in it |
| **what** | every body corner ≥ **5 cm** inside every zone edge (half a painted line: a corner on the paint is on the line), and heading within **±5°** of the zone's heading | exact for the whole body: a rectangle is inside a convex polygon iff its corners are — so the loader rejects non-convex zones and normalises the winding |

Events: `PARK_OK` (detail = worst-corner clearance), `PARK_OUTSIDE` (that
clearance; negative = over the edge), `PARK_ANGLE` (heading error, degrees).
Every failed criterion gets its own event, and each new attempt is judged
again: deciding WHICH attempt counts is task 4's state machine, not this rule.
All four thresholds are `[EST]`, for task 11 to replace from a primary source.

The zone's heading is part of the exercise, not of the rectangle: the bay is
reversed into, so a car driven in nose first is 180° out and fails, which a
direction-agnostic "axis" test would have passed.

28 host assertions, then six mutants — judging the current pose instead of the
stop pose, no dwell, no winding normalisation, body-overlap instead of centre,
no paint margin, axis instead of heading — each caught by exactly the test
written for that decision.

### The course leaked lines between exercises

Planning the end-to-end run found it: every correct reverse into the bay
crossed the PARALLEL exercise's kerb. The kerb ran 20 m past its slot, to
x = 89.1, straight across the bay's aisle 1 m in front of the bay mouth. And the
bay's aisle edge started 15 m before the bay, at the very end of the parallel
slot, where the parallel manoeuvre's front swings out. A correct manoeuvre in
one exercise scored a fault in the other. Both were caught by checking the
planned swept paths against every line in a Python prototype before any C was
written. `gen_course.py` now ends the kerb 8 m past the slot and starts the
aisle edge 5 m before the bay, and asserts they cannot overlap.

### The editor's referee was fed world-X velocity as "speed"

`ChronoWorldSubsystem` set `V.speed = linear_velocity.x`, and the snapshot's
velocity is in WORLD axes (`rf.GetPosDt()`). The referee means signed speed
along the body X axis — exactly what Chrono's `ChChassis::GetSpeed()` computes,
`dot(v, body X)`, read in the Chrono 10 source. Reversing into the bay the car
moves along world Y, so the editor would have seen it as stopped while it
moved, and the stall rule's speed conjunct was equally blind. Now projected on
body X from the snapshot quaternion. Compiled; not separately measured in the
editor — the headless harness used `GetSpeed()` all along, which is why its
judged runs were right.

### The pilot — why a script could not do this

`judge` steers with a lane keeper, which holds y = 0 and cannot park; an
open-loop steering script tuned to this car would stop parking the moment any
dynamics changed and then report the script, not the rule. So `judge park_*`
runs a closed-loop pilot in `test_shim.c`: a kinematic plan built from the
course's own `ZONE` records (drive OUT of the space from the parked pose,
then run the path backwards — the geometry `gen_course.py` uses for
feasibility), followed by pure pursuit about the rear axle in either
direction, a PI speed loop, and a hard stop onto the end. The spawn is fixed at
the origin, so the referee sees the car through a rigid transform from the
settled spawn onto the manoeuvre's start; the ground there is flat and uniform,
and the course the referee judges against is the real one.

Three pilot lessons, each measured, each worth keeping for task 4:

1. **A P-only speed loop settled at 0.45 m/s** — inside the stall rule's band.
   Reverse at 0.45 m/s turns this engine ~250 rpm, and the referee scored two
   `STALL`s, correctly: no real diesel crawls there. PI control plus a brisk
   launch (past 0.5 m/s inside the 0.75 s window) removed them.
2. **Coasting clutch-in for a slow final approach stopped the car short**: tyre
   scrub at 95% lock bled the speed and, clutch in, it could not restart. The
   pilot now cruises, then brakes onto the end (clutch in) when the predicted
   stopping distance reaches what is left — about 3 cm at 0.6 m/s.
3. **Lookahead 1.2 → 0.8 m halved the tracking error** at 95% lock (8.6 → 4.3
   cm; final heading 4.4° → 2.4°), and the parallel's planned rear gap 0.10 →
   0.15 m turned a 6 mm pass into 10.6 cm.

### Results

`bash docker/05-verify-vehicle-shim.sh judge <run>`:

| run | verdict | detail | stop vs plan | path deviation |
|---|---|---|---|---|
| `park_bay` | **PARK_OK**, zone 1 | clearance 0.121 m | 2.8 cm, −0.58° | ≤ 2.3 cm |
| `park_bay_short` (stops 0.30 m early) | **PARK_OUTSIDE**, zone 1 | −0.188 m, nose over the mouth | 4.3 cm, −0.74° | ≤ 2.3 cm |
| `park_parallel` | **PARK_OK**, zone 0 | clearance 0.106 m | 10.6 cm, −2.43° | ≤ 4.3 cm |

No other event fires on any of them: no line, no cone, no stall. Deterministic
like the rest of `judge`: identical numbers from separate processes.
(Re-measured by task 4 after the pilot's launch floor: every clearance moved
≤ 2.5 mm, no verdict changed — see "Task 4: the verdict".)

## Task 4: the verdict

### The state machine

| | GATED (slalom) | PARK (parallel, bay) | HILL |
|---|---|---|---|
| starts (`EX_START`) | its entry gate is crossed | same | same |
| passes (`EX_PASS`, detail = seconds) | its exit gate | a `PARK_OK` in its zone | its exit gate, **after a confirmed stop** on the way |
| fails (`EX_FAIL`, detail = reason) | first fault | first fault, or `REF_PARK_MAX_ATTEMPTS` (3) judged stops without a `PARK_OK` | first fault, or the exit gate with no stop |

Common to all: a fault is a cone, a line, a missed gate or a stall attributed
to the exercise. Starting an exercise fails a still-running one as
`abandoned` and any earlier still-armed one as `skipped`. A fault that belongs
to no exercise — a stall between two of them — is a *general* fault. The test
is **FAIL** if any exercise failed or any general fault occurred, **PASS** if
every exercise passed, otherwise **INCOMPLETE**; **SUSPECT** is added if the
referee missed physics samples.

The decisions worth knowing, each pinned by its own test and mutant:

1. **A fault fails an exercise that has not started yet.** A cone knocked
   before its exercise is still a knocked cone. This is what made the course's
   object placement a correctness property rather than a layout detail — see
   the fourth leak below.
2. **The verdict sees every event, including those the log drops.**
   `verdict_consume` runs in `emit()` *before* the capacity check; a full log
   must never change a result.
3. **A missed gate is its own event, `GATE_MISS`, and the sequence
   resynchronises.** Before, one missed gate made every later gate "out of
   sequence" for the rest of the run.
4. **The running exercise's exit is checked before a new entry on the same
   gate.** A gate may end one exercise and start the next.
5. **An attempt is a stop, not an event.** One stop that is both outside and
   crooked is ONE failed attempt.
6. **The hill's stop reuses the parking stop detector** — the same 0.05 m/s,
   1.0 s dwell — so "the car stopped on the slope" means exactly what "the car
   stopped in the bay" means.

"First fault fails the exercise" and the 3 attempts are `[REG?]`: the real
scoring may count points. `faults` is kept per exercise even after it fails,
so a points scheme is a change to `ref_verdict` alone.

### The course: exercises, entry gates, and the fourth leak

`gen_course.py` emits `EXERCISE id kind entry exit zone` records and tags every
cone, line and gate with its exercise. The old free-standing start gate is
the slalom's entry; the parallel, bay and hill exercises gained entry gates,
and gates are asserted to be in driving order, because the verdict tells a
skipped exercise from one still to come by gate sequence.

Driving the whole course found a **fourth line leak**: the kerb started
20 m before its slot, inside the slalom — the weave around the last cone swung
the body across it. Under decision 1 above, a clean slalom would have failed
the parallel park before it began. Rather than a fourth pairwise assert, the
generator now asserts the general rule: **every cone and line lies past its own
exercise's entry gate.** Mutating the kerb back fires it:
`parallel object at x=42.0 is before its entry gate`.

### The whole test, driven: `judge course`

The pilot now drives six legs, all read off the course: slalom → parallel
set-up (fwd), into the slot (rev), out and to the bay set-up (fwd), into the
bay (rev), out and up the hill to 1 m past its stop line (fwd), hill start
over the crest (fwd). Checked in a Python prototype against every line, cone,
gate and the two parked cars before any C was written.

**The slalom is arcs, not a sine.** A cosine weave of amplitude 1.5 m needs 92%
of lock at its peaks and clears the cones by 0.12 m at best. The path of least
peak curvature through the same peaks is a chain of equal arcs,
`R = (s²/4 + A²)/(2A)` = 6.08 m, 69% of lock — and while the car is on the arc
past a cone, the turn centre lies beyond that cone, so every point of the body
stays at least `R − w/2` from it: **0.41 m of clearance by construction**, and
≥ 0.38 m under simulated pure pursuit at every speed and lookahead tried.

**The pilot is task 3's, shared.** `pilot_step` is task 3's loop moved out of
`do_park` — verified byte-identical on all three parking runs before anything
was added — plus a cruise with a deceleration down to task 3's approach speed,
a hill start (rev against the brake, clutch up, brake off at the bite), and
one fix:

**A decisive launch.** The first full run FAILED — a `STALL` 0.75 s into the
reverse out of the first stop. The engine sits at ~0 rpm at every standstill
(task 13: the clutch cannot open), and the PI loop's ramped target launched
timidly at full lock: 0.48–0.52 m/s for 0.75 s, engine 214 rpm — a real stall
by the rule's definition, the kind a hesitant driver would also score. Now the
throttle is held ≥ 0.45 until the car is past `REF_STALL_MAX_SPEED`, within the
first second of a leg. It changes task 3's parking runs by at most 2.5 mm of
clearance, and none of their verdicts: parallel `PARK_OK` +0.1056 → +0.1033 m,
bay `PARK_OK` +0.1211 → +0.1235 m, short bay `PARK_OUTSIDE` −0.1875 → −0.1900 m.

### Results

`bash docker/05-verify-vehicle-shim.sh verdict course` — two processes:

| leg | stopped at | vs plan end | heading | max path deviation |
|---|---|---|---|---|
| A slalom → parallel set-up | 40.85 s | 0.4 cm | +0.07° | 8.0 cm |
| B into the slot | 53.34 s | 11.9 cm | −2.68° | 4.0 cm |
| C out → bay set-up | 79.95 s | 0.3 cm | +0.08° | 6.7 cm |
| D into the bay | 97.52 s | 3.9 cm | −0.82° | 2.4 cm |
| E out → up the hill, past the line | 130.09 s | 0.6 cm | +0.23° | 5.5 cm |
| F hill start, over the crest | 142.99 s | 0.4 cm | +0.14° | 1.3 cm |

| exercise | result | took |
|---|---|---|
| 1 slalom | PASSED, 5 gates in order | 20.17 s |
| 2 parallel | PASSED, `PARK_OK` clearance 0.104 m | 21.57 s |
| 3 bay | PASSED, `PARK_OK` clearance 0.119 m | 26.83 s |
| 4 hill | PASSED, stop confirmed | 15.63 s |

**VERDICT: PASS, 4 of 4** — no cone, no line, no stall, no general fault;
`VERDICT_DIGEST 00bd5229f776be69` from both processes. The digest covers every
event and every exercise result field by field (the structs have padding).

The negative cases, each failing only what it should:

| run | what it does | verdict |
|---|---|---|
| `course_cone` | weave 1.0 m: the body edge passes inside every cone | slalom **FAILED (cone)** after 1.94 s, `faults 5`; the rest pass → **FAIL, 3 of 4** |
| `course_nostop` | drives up and over the hill without stopping | hill **FAILED (no stop on the hill)** at the crest; the rest pass → **FAIL, 3 of 4** |

`judge park_*` still runs, and now prints a verdict too: **INCOMPLETE**, by
design — a parking-only run starts at the manoeuvre, never crosses the
exercise's entry gate, and so its `PARK_OK` is not credited (host test 14).

### The editor's referee was told the clutch was always engaged

`ChronoWorldSubsystem` set `V.clutch = 0.0`, a constant, so the stall rule
read every stop in gear as "drive connected": a driver waiting at the hill's
stop line with the pedal down and the engine under 400 rpm would have scored
a `STALL`. Harmless while the referee only logged; with a verdict it fails the
test. The physics thread now publishes the commanded pedal (`StatClutch`,
beside rpm and gear, with the same one-frame staleness against 0.75 s of
hysteresis). The subsystem also exposes `GetRefereeVerdict`,
`GetExerciseResult` and `GetCourse` for task 5's HUD, and logs the running
verdict once per exercise that ends.

### Found: the clutch is welded shut (task 13)

The first full run's stall led here. With the clutch pedal down and 50%
throttle the engine did not rev — it sat at 50–100 rpm, locked to the braked
wheels. The long-standing `clutch` harness mode had been printing the answer
all along, without asserting it: every pedal position, 1.0 included, gives
the identical `1.468 m/s, 746 rpm, moving`. `ChShaftsClutch` is a boxed
constraint and MINRES is a linear solver (task 13 has the table). A new
`diff` mode measured the same for the rear differential: a spool.
`CHS_SOLVER=psor|apgd|bb` swaps the solver for diagnosis — unset, production
is unchanged (the determinism digest is still `c829c405af66692a`).

What a solver switch would cost, `perf 5000`, same machine:

| solver | mean | p99 | max | clutch at pedal 1.0 |
|---|---|---|---|---|
| MINRES, 60 iterations (production) | 185.4 µs | 235.0 | 315.8 | welded |
| **PSOR, 50 iterations** | **135.1 µs** | 202.0 | 323.5 | **held; engine free-revs to 3523 rpm** |
| PSOR, 150 iterations | 311.4 µs | 436.9 | 504.9 | held |

PSOR at 50 iterations is *cheaper* than production and opens both clutches.
What it has not been checked for is accuracy on the suspension's closed
loops, which is what MINRES was chosen for in M4. First look, `assemble`:
chassis z 0.2761 vs 0.2733 m, front axle load 8091.7 vs 8147.1 N (−0.7%).
Two things a working clutch exposes at once: in neutral the freed engine
idles BACKWARDS at −98 rpm, because the losses map is −20 N·m at 0 rpm and
only reaches zero at −100 — and the losses map is mis-scaled besides (below).

### Also found, not fixed

- **The engine's losses map is mis-scaled.** `gen_innova.py` wraps the Losses
  Map's x-values in `rpm2rads()` — directly under a comment saying Chrono
  converts rpm itself and that pre-converting "would scale the whole curve by
  9.55". The JSON has breakpoints at 146.6 … 460.8, which Chrono reads as rpm:
  the −60 N·m peak loss arrives at 461 rpm instead of 4400. Task 8's.
- **~~The chassis sits ~5° nose-up at rest.~~ Fixed by task 14.** The pilot's trace reads pitch
  −4.3 to −5.3° standing on flat ground (−11.3° on the 6.84° ramp); in
  `euler_zyx`'s convention negative is nose-up. Over the wheelbase that is
  ~0.24 m of front-to-rear height difference. Not investigated; it moves the
  driver's eye, the mirrors and weight transfer, so it belongs to the vehicle
  model's `[EST]` list (M4).
- **~~46/54~~ — corrected by task 13 to 51.5/48.5**, from a settled,
  unbraked measurement; the 46.1% below was `assemble` sampling mid-bounce.
- **~~The simulated weight distribution is 46/54~~ — 53.0% front since task 14.**
  `gen_innova.py` sets `WDIST_F = 0.53` and places the chassis CG to reach it;
  `assemble` measures 8147 N on the front tyres, 46.1% of 17,658 N. M4's
  equilibrium check (0.01%) verified the TOTAL, not the split. Meanwhile
  `gen_course.py` COPIES `WDIST_R = 0.47` into the hill-traction analysis
  instead of parsing it — the stale-constant trap its own header warns about.

## Task 13: the clutch, opened

### Why a solver can weld a clutch

Chrono solves each step as a KKT system: velocities, plus a multiplier λ per
constraint (the force or torque it takes). A joint is an EQUALITY, λ free. A
clutch is not: `ChShaftsClutch` holds its shafts together only while the
torque stays in a box, `|λ| ≤ modulation × limit`, and slips beyond it. That
is a variational inequality. PSOR, APGD, BB and ADMM project λ onto its box on
every iteration. MINRES, GMRES and the direct factorisations solve a LINEAR
system: they drop the box, and the clutch becomes a rigid joint. Chrono's
`ChSolver.h` files MINRES under "iterative linear solvers". So the shim's
M4 choice of MINRES welded both of this car's clutches: the pedal's, and
`ShaftsDriveline2WD`'s differential lock, which sits at modulation 0 (open)
and so became a spool.

### Choosing the solver: two rounds of `solvers`, and a false alarm

`bash docker/05-verify-vehicle-shim.sh solvers` configures the shim through
its diagnostic variables and runs statics, a steady turn and a launch per
solver. SPARSE_QR (`CHS_SOLVER=qr`) solves the joints exactly, so it is the
reference.

**Round 1 looked like PSOR failing.** Against QR, PSOR-50 turned 15% harder
and rolled **twice** as much. It was not PSOR. Round 2 removed the
differential lock from the LINEAR solvers too (`CHS_OPEN_DIFF`, found by its
shafts: the clutch between the driven axle's two axle shafts):

| steady turn, steer 0.3, 5 m/s | yaw | roll | column |
|---|---|---|---|
| QR, diff locked (the old car) | 14.457 °/s | 0.703° | −4.169 N·m |
| **QR, diff open** | **16.652** | **1.465** | **−2.298** |
| MINRES, diff open | 16.680 | 1.462 | −2.294 |
| **PSOR 1000 (converged)** | **16.652** | **1.465** | **−2.298** |
| PSOR 50 | 16.617 | 1.456 | −2.298 |

The exact solve with an open differential reproduces converged PSOR to four
figures. The whole difference was the spool, which fights the turn and jacks
the rear axle through the three-link geometry. PSOR-50's own error is 0.2–0.6%.
At rest, unbraked, PSOR-50 equals PSOR-1000 to the last digit. It sits 2.3 mm
higher than QR because in neutral its engine is disconnected (QR's is welded
on), not because of convergence.

**Warm start diverges.** Warm-started PSOR runs a residual of 1–3 (cold:
0.02–0.09) and gets the turn wrong by up to 50%. Production turns it off
explicitly.

**Round 1's braked statics were garbage, and that is task 15.** QR, MINRES and
PSOR all gave ~0.11 m ride height and 74% front load. The parking hold froze
the chassis at the bottom of its first spring compression.

### The switch, and the re-baseline

`chrono_shim.cpp`: `PSOR`, `SetMaxIterations(50)`, `EnableWarmStart(false)`.
`CHS_SOLVER=minres` reproduces the old car EXACTLY: `determ pullaway` gives
the old `c829c405af66692a`. So every "before" below is the same code with the
old solver, not a remembered number.

| measurement | before (MINRES) | after (PSOR 50) |
|---|---|---|
| `clutch`, pedal 1.0 | drives off, 1.468 m/s | **held**, engine free at 3523 rpm — now asserted |
| `diff`, inner/outer vs ground 0.714 | 1.002, LOCKED | **0.7141, OPEN** — now asserted |
| `perf 5000` mean / p99 / max | 192.1 / 316.0 / 440.3 µs | **136.2 / 216.3 / 340.4 µs** |
| `determ pullaway` | `c829c405af66692a` | **`562578ed9e0a6d62`**, cross-process identical |
| `creep`, neutral, 20 s | −3.82 m | **−0.006 m** |
| `creep`, 1st, pedal down | −3.36 m | **−0.001 m** |
| `creep`, 1st, pedal up | −3.36 m (identical: welded) | −3.52 m (the engine, task 8) |
| `hill` HOLD / ROLL | −0.004 m / −50.4% | −0.019 m / **−36.7%** |
| `hill` LAUNCH (60%, clutch dumped rolling back) | pulled away, oscillating | oscillates, does not pull away |
| `mzlow` blend 0.20, column @0.30 m/s | −3.256 N·m | −0.330 N·m (spool understeer gone) |
| `sweep` peak column torque | 7.046 @ 0.45 | 7.033 @ 0.45; trail collapse, grip at 0.70 |
| `drift pullaway`, lateral over 32.7 m | 0.333 m | 0.337 m |
| `assemble` z / front load / engine | 0.2733 / 8147 N / −7 rpm | 0.2761 / 8092 N / **−98 rpm** |
| `neutral`, `terrain` | pass, climbs | pass, climbs |
| `verdict course` | PASS 4/4 | **PASS 4/4**, `7ba5b428b8a58ce5`, both processes |
| `course_cone` / `course_nostop` | FAIL (cone) / FAIL (no stop) | same |
| parallel / bay `PARK_OK`, short bay | 0.1033 / 0.1235 / −0.1900 | 0.1093 / 0.1211 / −0.1876 |

**The `hill` LAUNCH row is not a regression metric.** It dumps the clutch at
60% throttle while the car rolls back at 1.3–1.5 m/s: wheelspin at the
friction limit. It oscillated ±1–3 m/s under MINRES as well, and at every PSOR
iteration count, converged included. A proper hill start is what the course's
leg F does: rev against the brake, let the clutch up, release the brake at the
bite. It passes.

### What it re-explains

- **Flat-ground creep** (M6, "RESOLVED-AS-CONSTRAINT"): the engine's losses
  torque, connected by the welded clutch. Overturned.
- **"Engine bogs to 330–380 rpm" (task 8, the stall conjunct)**: 1st gear at
  walking pace, no clutch to slip.
- **The hill-start "reflected inertia" fix**: the reflected mass was the
  engine's. With the clutch open, holding top gear still saves the input
  shaft's ~751 kg-in-1st.
- **Task 1's parking-speed `Mz` gain**: mostly spool understeer.

### Exposed, not fixed

- **The engine idles backwards and pushes the car backwards in gear** (losses
  −20 N·m at 0 rpm, x-axis in rad/s, no governor): task 8, rewritten.
- **The parking hold opposes vertical motion**: task 15, new.
- **Task 14's split was wrong** (46% was mid-bounce; settled: 51.5%). Its
  4.30° nose-up attitude is confirmed. (Both fixed by task 14.)

### ~~Not measured~~ — measured by task 5

The editor under rendering has not been run with PSOR. Headless it is cheaper
than MINRES on every statistic (mean 136 vs 192 µs, max 340 vs 440), and M3's
1 ms budget under rendering was met with MINRES at 230 / 638 µs. So it is
expected to hold, but it is not yet measured. **Task 5 measured it**: 205–220
µs mean, 1043 µs max, 2 steps over budget in ~109,000.

## Task 15: the brake that could not hold

### The brief was right, and it was a symptom

The brief asked for the parking hold to act only in the road plane. That is
done. But the harness came first (`hold`, asserted), and its first check asks
whether a braked wheel at rest is still. It was not.

### Chrono 10's `ChBrakeSimple` cannot stick

Read in the builder image (`/src/chrono`), not assumed. `ChBrakeSimple` drives
a `ChLinkLockBrake`. Its `UpdateForces` applies `−T·sign(ω)` and sets
`must_stick` whenever that sign flips — and **nothing in the file ever reads
`must_stick`**, so the lock constraint (E3) is never activated. The other
route, `ChBrake::EnableLocking` (lock the hub at pedal > 0.99), defaults false,
and the JSON reader never sets it. The class's own header calls itself
OBSOLETE and points to a `ChShaftsClutch` between shafts.

An explicit Coulomb torque at rest flips every step, so the wheel **chatters**:
0.36 rad/s peak, 19 sign flips in 20 steps. A tyre on a chattering wheel never
settles. That is why "the brake will not hold on the slope", and the parking
hold was built to paper over it.

### `BrakeShafts`

`ChBrakeShafts` is a `ChShaftsClutch` between the wheel's axle shaft and a
brake shaft fixed to the upright, with torque limit = max braking torque and
modulation = pedal. Under a VI solver it is rigid below the limit and slips at
it, which is what pad friction does. Under MINRES it would have welded every
wheel: task 13 is what makes it usable. `gen_innova.py BRAKE_MODEL=shafts` is
the default (`simple` reproduces the old brake). Its JSON needs
`"Shaft Inertia"`, read with no `HasMember` guard (0.4, Chrono's generic).
Measured stuck at rest: 0.004 rad/s, no sign flips. Cost: +11 µs/step for the
four clutches.

### Four designs, measured

| design | wheel at rest | braked spawn vs free | 12%, brake held | PSOR 50…1000 |
|---|---|---|---|---|
| `BrakeSimple` + 3-D hold (was) | chatters | **frozen: 0.113 m, 74% front** | held, at −30° pitch | **slides 1.26 m at 200** |
| `BrakeSimple` + planar hold (the brief) | chatters | −1.6 mm | held at −23°; brake alone slides 3 m | holds |
| `BrakeShafts` + planar hold | stuck | +17.9 mm (it pins the COM) | held; still moving 0.7 mm / 2 s | holds |
| **`BrakeShafts`, hold off (adopted)** | **stuck** | **−3.7 mm** | **0.8 mm, then still** | **1.0–1.4 mm** |

The planar projection fixed the heave freeze and the iteration fragility. The
fragility was a race: the 3-D speed gate included the landing bounce, so the
hold engaged only after the car had begun to slide, sometimes too late.
Acting at the centre of mass still fights the body's pitch settle, though,
and with a brake that holds, the hold has no job left. It stays in the shim,
planar and load-limited, OFF by default: `Chs_SetParkingHold(w, 1)` for a
`BRAKE_MODEL=simple` vehicle, which needs it.

### A spawn is a crash test

The first steep-grade rows slid 10 m (25%) and 24 m (45%). They measured a
level car dropped onto a steep ramp: at 25% the rear axle falls 0.69 m, and
`holdtrace` shows the car tumbling (pitch −38°, then +74°). A braking test
must stop the car the way a driver does. `hold_ramp` drives over the crest of
a descending ramp at walking pace and brakes on the grade. On that protocol
the brake alone holds 25% (2.3 mm) and 45% (4.0 mm), and at 120% (past
μ = 0.9) the car slides away, as it must.

### Braking legitimately moves a settled car

Braking a settled car on flat ground raises it 3.6 mm (0.08°, 0.6% of front
load) and moves it 1 mm. Under static load the suspension geometry makes
~400 N of fore-aft force at the contact patches. A free wheel rolls it away;
locked, the tyres carry it (1 mm of pre-slide × 4 × 10⁵ N/m), and
anti-dive/anti-squat turns that into a few mm of jacking. A real car does the
same. The harness bounds (5 mm, 0.2°, 1.5%) sit just above this and 25× under
the freeze.

### Re-baselined (task 13 → task 15)

| measurement | task 13 | task 15 |
|---|---|---|
| `hold` | — | **ALL CHECKS PASSED** |
| `hill` HOLD / ROLL | −0.019 m (3-D hold) / −36.7% | −0.007 m, brake alone / **−16.7%** |
| `hill` LAUNCH (clutch dumped rolling back) | oscillates | rolls back; engine back-driven to −2310 rpm (zero torque below −100 rpm: task 8) |
| `creep`, braked rows | pedal 0.5 moved 0.107 m | all still |
| `determ pullaway` | `562578ed9e0a6d62` | **`935a951af232e086`** |
| `verdict course` | PASS, `7ba5b428b8a58ce5` | **PASS, `0975eb5c2454d8af`**, both processes |
| `course_cone` / `course_nostop` | FAIL / FAIL | same |
| parallel / bay / short bay | 0.1093 / 0.1211 / −0.1876 | 0.1126 / 0.1186 / −0.1929 |
| `perf` mean / p99 / max | 136.2 / 216.3 / 340.4 µs | 147.5 / 224.4 / 398.9 µs |

Unchanged: `clutch` WORKS, `diff` OPEN, `neutral`, `terrain`, `sweep` peak
7.033 N·m, `mzlow` −0.331, `drift` 0.338 m, PSOR's `solvers` rows. The brakes
change nothing while driving.

### What it means for tasks 6 and 7

From a physically held start, free roll falls **16.7%** short of `g·sin θ`.
Rolling resistance is worth ~10%, and the masses turning with the wheels in
neutral (~195 kg equivalent) ~10% more. Task 6's "the tyre sticks from rest"
may simply be gone; task 6 now has to confirm that account term by term,
and task 7 waits on that.

### Not done

- ~~The editor under rendering is still unmeasured with PSOR~~, now plus four
  clutches. Headless max is 399 µs against a 1000 µs budget. **Measured by
  task 5**: see "Task 5: the HUD, and what seeing the car found".
- `hill` keeps its spawn-onto-the-slope protocol: it is task 6's instrument,
  and changing it belongs with that task. `hold_ramp` is the better protocol.

## Task 5: the HUD, and what seeing the car found

### The HUD

`AChronoHUD` (`AChronoGameMode::HUDClass`) draws on the canvas every frame
from what the subsystem already publishes. A canvas HUD is code, like the
keyboard's `IsInputKeyDown` polling: UMG widgets would be content assets, and
`SimA/Content` holds only generated ones.

| panel | shows |
|---|---|
| bottom left | speed (km/h), gear (N/R/1–5), revs to the 4400 rpm redline, clutch / brake / throttle bars and a wheel bar as commanded (the stats gained `Throttle`, `Brake`, `Steering`), and **ENGINE STALLED — clutch down or neutral to restart** at 0 rpm (task 8's stall) |
| top left | the verdict (PASS / FAIL / "n of 4 passed", "starting..." before the session), and each exercise: armed, in progress, passed with its time, or FAILED with the reason, plus the hill's recorded rollback |
| top right | the referee's calls in words, for 8 s each: exercise started / passed / FAILED -- reason, cone struck, line crossed, gate missed, engine stalled, parked with its clearance, not in the space, rolled back on the hill |
| bottom right | the loop's step mean/max, lateness, missed ticks, RT, and the input source |

Exercise names come from the course's data: the gated exercise is the
slalom, the hill kind is the hill, and a parking zone's heading tells the
parallel (0°) from the bay (−90°). There is no copied table.

Verified from rendered frames. `-ChronoShotUI` saves the viewport as drawn,
HUD included (`FScreenshotRequest`, `Saved/Screenshots/*/ChronoHUD.png`). The
default `HighResShot` renders the 3-D scene only, which is what
`mirror/run.sh render` measures, and that path is unchanged. **Corrected by
task 9:** `HighResShot` includes the canvas HUD too; it leaves out only Slate.
No measurement was affected, since no panel covers a marker. Since task 17's
pass the `-ChronoShot` timer hides the HUD before a `HighResShot`, so the claim
now holds ("The scene-only screenshot, made so").

### Seeing it found three defects, none of them the HUD's

**1. The loading screen ate the scripted drive.** The first frame showed a
black screen, "script finished", and a referee that had seen nothing. The
physics thread starts with the world. That run's first frame came **103,552
steps** later (asset loading and shader compilation), and a 12 s script had
long since run out. The car had driven on at the script's last throttle,
off the terrain, and the camera followed it into empty space.

Fix: a **session start**. It begins when frames have been steady, under 40 ms
for 0.5 s; 40 ms is inside the ring's history, so no sample can be lost
between two drains. At that moment a scripted drive is released (it was held
at t = 0, which in every built-in script is pedal down, engine idling), the
referee is re-initialised and judges from the next sample, and the timing
stats restart. Keyboard and wheel runs get the same referee start. The log
says `Session started at physics step N`.

**2. The referee's drain skipped one sample whenever the physics thread
published mid-drain.** 135 one-sample `SAMPLE_GAP`s in two minutes of steady
play, with 28 ms frames against the ring's history, so the drain never fell
behind. `Read(back)` counts back from the head *as it is at that call*. A loop
of them that the producer publishes into shifts by one and skips a sample.
This was in the drain from the start, so every editor verdict with a sample
gap was flagged SUSPECT for this. Fixed with `ReadAt(absolute index)`, which
refuses a sample that is gone or going. It also refuses the oldest slot:
between the writer finishing a slot and publishing the new head, that slot
looks valid but holds the NEXT lap's sample. The drain walks absolute
indices, and reads only the samples it has not judged (it used to copy all
64 slots every frame). The ring also went from 64 to **256** deep (a quarter
of a second, ~435 KB; the writer still writes one slot a step).

`ring/run.sh`, a new host harness, compiles the canonical `ChronoStateRing.h`
against the shim's C header and races a producer against a consumer. Every
snapshot's fields encode its index, so tearing and misindexing are
detectable:

| consumer | read | skipped | wrong index | torn |
|---|---|---|---|---|
| keeping up, `ReadAt` | 149,906 | **0** | 0 | 0 |
| keeping up, the old `Read(back)` loop | 148,767 | **1,172** | — | 0 |
| falling far behind, `ReadAt` | 37,993 | 111,214 (reported as gaps) | **0** | **0** |

In the editor, the same two minutes of play: **1 `SAMPLE_GAP`** (was 135),
and that one is the screenshot freezing the game thread for 0.5 s.

**3. There was no sky.** Above the horizon the view was black, so every
screenshot looked broken. `AChronoGameMode` now spawns an `ASkyAtmosphere`
(engine code, no asset) and flags the sun as its light. It shows in the
mirrors too, as it should. `mirror/run.sh render` still passes: markers in
the render targets 0.06–0.22 px from prediction (unchanged), on the glass
0.93–1.71 px (task 2: 0.14–0.24), inside the 6 px tolerance. Likely cause:
the markers used to sit on black and now sit on blue sky, so anti-aliased
edge pixels cross the colour threshold differently. Not separated.

### The 1 kHz loop under rendering, finally measured

Tasks 13 and 15 left this open. 110 s of steady play, from the session start,
with the car driving a slow circle at idle (`driver/scripts/circle.txt`,
built to stay on the flat patch), offscreen at 1280×720:

| | M3 (MINRES, 41k steps) | task 5 (PSOR 50, BrakeShafts, governor, ~109k steps) |
|---|---|---|
| step mean / max | 230.4 / 637.8 µs | **205–220 / 1043 µs** |
| lateness mean / max | 0.7 / 49.8 µs | 0.8–1.1 / 334 µs |
| steps over budget | 0 | **2**: 1018 and 1043 µs, 60 s apart |
| SCHED_FIFO | yes | yes |

The two over-budget steps are isolated and 2–4% over. The clock is fixed-step
(`Steps × StepSeconds`), so the next step starts at once and catches up. The
referee's samples stay consecutive, and the renderer's clock is slewed. The
mean uses 21% of the budget.

### Not done

- The course's ground patch is 40 m wide, and the world beyond it renders as
  the atmosphere's dark ground. A visual ground plane belongs with task 9.
- `DriverEyeChrono` (1.48 m up) and the chassis box are unchanged: task 9.

## Task 7: the rollback rule

### What the regulation asks, and what the referee can measure

Perkapolri 9/2012 Lampiran A (task 11): stopped on the ascent, the car pulls
away with "tidak ada reaksi kendaraan mundur", and fails on "mundur pada saat
berhenti ditanjakan". Zero rollback. A referee cannot test "zero" on a
simulated car; it needs a DETECTION FLOOR above the model's own motion of a
held car, and below any rollback an examiner would see.

### The first floor was inside the model's noise

The first version integrated the BODY's signed speed from the stop, which is
distance along the road with the slope included. The pilot's clean clutch start
measured 2.7 cm, and a trace showed none of it came from the start. It all
happened in the half-second after the braked stop, with the wheels locked. The
`rollback` harness then measured it: braked to a stop on 15° from 0.5, 1 and 2
m/s, the body settles back **5.8, 5.8 and 7.6 cm**. The mechanism is the Dahl
bristles (σ₀ = 10⁵ N/m). Braking deflects them forward toward saturation
(μFz/σ₀ ≈ 4 cm), and gravity then swings them back to their holding
deflection. A 5 cm floor on body travel would have failed a perfectly held
car. A floor above 8 cm would have missed rollbacks an examiner can see.

### The fix is what the words say: the car ROLLING

The regulation's fault is the vehicle rolling back. The settle happens with
the wheels locked. So the rule counts **wheel travel**: the rear spindles'
spin relative to the body, times the rolling radius. `ref_wheel_speed` in the
referee computes it, so the editor and the harness get the same number. The
rolling radius joins the course's CAR record (0.3331 m, TMeasy's
`(2R0 + r_stat)/3`, derived by `gen_course.py` from the tyre size). A
five-field CAR record is now refused rather than guessed.

| on the course's 15° | body travel | **wheel travel (judged)** |
|---|---|---|
| braked stop from 0.5 / 1 / 2 m/s, held 3 s | 5.8 / 5.8 / 7.6 cm | **1.2 / 1.2 / 1.3 cm** |
| the pilot's clutch start (`verdict course`) | 2.7 cm | **0.8 cm** → PASS |
| `course_rollback`: brake released at half the holding capacity, 0.5 s to react | — | **12.1 cm** → `ROLLBACK`, hill FAILED |

The locked wheels still turn 1.2 cm against the body. That is probably the
rear axle tube pitching on its links as the body settles; it has not been
separated. The floor (5 cm) clears it by 4×. The bad start needed the
reaction time. With half the bite alone, the pilot's clutch loop caught the
car in 3.6 cm, faster than a person could.

### The rule

- A stop inside a HILL exercise, the referee's usual moving→stopped
  transition, starts the watch. The wheels' travel is integrated from that
  moment. The dwell is not waited for, so a car that stops and rolls straight
  back is still caught.
- Backward travel past `REF_ROLLBACK_FLOOR` emits `ROLLBACK`, once per stop
  (`object_id` = the exercise, detail = the distance), and fails the
  exercise (`REF_FAIL_ROLLBACK`).
- The worst excursion is kept in `RefExerciseResult.rollback`, and keeps
  being recorded after the fault ends the exercise. The whole excursion is
  the measurement.
- Watching ends when the car has rolled 0.5 m forward, or another exercise
  starts. Reversing in the parking exercises is never watched.

### Verified

13 new host tests (98 in all). They cover a clean start; a 0.10 m rollback
(fails, fired as it crossed the floor, full 0.10 m recorded); 0.02 m (passes,
recorded); parking reverses (ignored); a new stop after creeping forward
(caught); the settle, body back 0.08 m with the wheels locked (not a
rollback); `ref_wheel_speed` level, yawed 90°, pitched with locked wheels,
yawed and pitched; and a five-field CAR refused. Five mutants each fail at
least one test: the verdict ignoring `ROLLBACK`, unsigned distance, watching
outside the hill, no floor, and body speed instead of wheel speed. The last
fails exactly the settle test.

| run | result |
|---|---|
| `verdict course` | **PASS 4/4**, `317eca4c2c8d9b64`, both processes; hill rollback −0.0083 m |
| `judge course_rollback` | **FAIL**: `ROLLBACK` at 146.822 s, hill FAILED (rollback), −0.1213 m |
| `course_cone` / `course_nostop` | FAIL (cone) / FAIL (no stop) |
| parks | PARK_OK 0.129 / 0.454, short bay PARK_OUTSIDE −0.321 |
| `rollback` | wheels' worst settle 1.28 cm < half the 5 cm floor |

## Task 11: the regulation, read

### Where the numbers are, and are not

| document | status | what it says about the course |
|---|---|---|
| Perkapolri No. 9/2012 tentang SIM, Lampiran A | superseded, **published** (JDIH BPK) | every figure below, verbatim |
| Perpol No. 5/2021, Pasal 18 ayat (6) | in force | "Materi yang diujikan dan ketentuan pelaksanaan ujian praktik ditetapkan dengan Keputusan Kakorlantas Polri" |
| Perpol No. 2/2023 (amends 5/2021) | in force | same ayat (6); adds (7): other test sites must meet a Kakorlantas standard |
| Keputusan Kakorlantas Polri | in force | **not found** in JDIH BPK or peraturan.go.id |
| detik.com, Oct 2022 | reporting | the 2012 items, citing Perkapolri 9/2012 (but "2 kali panjang" for the slalom, against the text's 1.5) |
| Antara, Aug 2023 | reporting | zig-zag and figure-8 replaced by an "S" track, lanes 1.5 → 2.5 × car width; class not named. The figure-8 was only ever a motorcycle item, so this is read as SIM C. Inference, not text |

Two first attempts failed usefully. The Korlantas copy of Perpol 5/2021
serves a broken certificate chain; the pasal.id copy showed it had no
dimensions at all. The search engine's own summary had quoted the 2012
figures as if they were Perpol 5/2021's. A summary is not a source; reading
the text is what showed the delegation.

### The figures, and how they were read

| item | Lampiran A says | course |
|---|---|---|
| slalom (3.b) | 9 cones + start + finish, "jarak antara patok ... 1,5 kali panjang kendaraan", forward AND reverse | 9 cones at **7.10 m**, forward only |
| parallel (3.c.5) | "panjangnya 1,5 kali panjang kendaraan ... dan lebar 1,5 meter", reversed in two movements | **7.10 m** long, **2.745 m** wide: "1,5 meter" cannot hold a 1.83 m car, read as 1.5 × width like the bay clause |
| bay, "parkir seri" (3.c.3) | "1,5 lebar kendaraan dan panjang 1,5 kali kendaraan", reversed in once | **2.745 × 7.10 m**; aisle not given → 6.00 m `[EST]` |
| hill (1.h, 3.d) | "sudut kemiringan 15°", "tinggi ... 2,5 M", bridge "1,5 kali panjang kendaraan", then down; stop lines "sekurang-kurangnya ... ditengah-tengah panjang jalan" | **15°** up to **2.5 m**, **7.10 m** bridge, 15° down, stop line halfway up |
| hill rules (3.d.2, 3.d.7) | "tidak ada reaksi kendaraan mundur"; fails on "mati mesin, mundur pada saat berhenti ditanjakan" | rollback allowed: **0** (task 7) |
| scoring | a stage fails on 2 consecutive cone touches; measured to the outer body, not the mirrors | the referee fails an exercise on its first fault |

### Departures from the regulation (printed by `course/run.sh gen`)

- **Not built**: 3.a (50 m forward and reverse through a lane car-width +
  60 cm wide); the slalom's reverse pass; the stops on the bridge (foot
  brake) and the descent (hand brake, which the ABI does not have).
- **Read, not quoted**: the parallel width above; the hill road's width is
  given twice (1.5 × length in 1.h(3), 1.5 × width in 3.d(5)). The terrain is
  8 m wide with no edge lines.
- **The figures may be superseded** by the unpublished Kakorlantas decree.

### What the regulation broke

- **A plan that truncated SILENTLY.** `ParkPlan` held 8192 points (82 m) and
  `plan_extend` stopped appending at the limit without a word. The 9-cone
  slalom made leg A ~95 m, the car "arrived" at a truncated end, and leg B
  started 15 m from its own path. The symptom was a kerb crossing 16 s later.
  Now 32768 points, and a full plan fails the run.
- **The hill-start bite** was a fixed 10% of clutch capacity, right for 12%
  (7% holds it). 15° needs 15%, so it would have rolled back. It is now
  computed from the slope the car stands on (×1.3), which is what a driver
  feels.
- **The parallel set-up** sat at y = −0.8 on the road. The wider space moved
  the parked cars' corners to y = 0.255, onto the car's flank. It is now
  placed off the space's outer edge, with the same measured 1.37 m.
- `park_bay_short` stopped 0.30 m early, which a 7.10 m bay absorbs. It now
  stops with the nose 0.30 m past the mouth: PARK_OUTSIDE −0.321 m.
- `terrain` checked the old hill's hard-coded profile. It now derives the
  expected surface from each patch's own numbers, and drives over the whole
  hill at a held 3 m/s: 85% throttle would have flown the new crest.

### Results

| run | result |
|---|---|
| `verdict course` | **PASS 4/4**, `2b9c9e72f728f6b9`, both processes; slalom 34.5 s, no cone |
| legs A–F, stop vs plan | 0.7 / 9.0 / 0.7 / 1.4 / 1.7 / 0.7 cm; lowest rpm with the clutch carrying torque 635 |
| parallel / bay (course) | PARK_OK 0.130 / 0.454 m |
| `judge park_parallel` / `park_bay` / `park_bay_short` | PARK_OK 0.129 / PARK_OK 0.454 / **PARK_OUTSIDE −0.321** |
| `course_cone` / `course_nostop` | FAIL (all 9 cones) / FAIL (no stop on the hill) |
| `terrain` | profile exact to the patches' own numbers; climbed to 2.500 m and came down |
| host suites | referee 85, driver 46, mirror 53, wheel decode, course check: all pass |

## Task 6: the free-roll account

### The question had changed

Task 6 was "why the tyre sticks from rest": free roll on 12% ran 45–50% short
of `g·sin θ`. Tasks 13, 15 and 10 removed three causes that were not the
tyre: the welded clutch, the chattering brake, and the neutral that geared
the input shaft to the wheels. The deficit came down to −15.4%, which looked
like plain physics. The job became checking that account term by term.

### `rollacct`: measure the terms, then move them one at a time

- **Protocol**: the car is stopped on the slope the way a driver stops there.
  Up a 12% ramp in 1st at idle, brake, pedal down, neutral, brake off. A level
  car spawned onto a slope is a drop test (task 15).
- **Measurement**: `v(t)` fitted by least squares over 0.5 < |v| < 2.0 m/s.
  That window is past the Dahl standstill core and has TMeasy's rolling
  resistance (`rr·Fz·R0·tanh(ω)`, read in `ChTMeasyTire.cpp`) saturated.
- **Account**: `a = (M g sin θ − F_rr) / (M + Σ I (n/R_eff)²)`, with
  `R_eff = (2 R0 + r_stat)/3`, TMeasy's own rolling radius, 0.3318 m here.

| term | value |
|---|---|
| gravity along the slope | 2103.9 N |
| rolling resistance, `rr Fz R0/R_eff` | 213.3 N |
| 4 × (wheel 0.62 + tyre 0.62 + axle shaft 0.35) + spindles, at 1:1 | 6.62 kg·m² → 60.1 kg |
| differential carrier, 1:1 | 0.60 → 5.5 kg |
| driveline driveshaft, at the final drive 3.909:1 | 0.50 → 69.4 kg |
| **predicted, neutral** | **0.9770 m/s²** (−16.4% of `g sin θ`) |
| **measured** | **0.9752 m/s² (−0.19%)** |

| experiment | predicted | measured | error |
|---|---|---|---|
| neutral | 0.9770 | 0.9752 | −0.19% |
| pedal down in 5th (+29 kg of input shaft) | 0.9625 | 0.9602 | −0.24% |
| ... in 3rd (+86 kg) | 0.9355 | 0.9324 | −0.33% |
| ... in 1st (+775 kg) | 0.6977 | 0.6905 | −1.04% |
| `ROLL_RES=0`, neutral | 1.0873 | 1.0885 | +0.12% |
| `ROLL_RES=0`, pedal down in 1st | 0.7765 | 0.7704 | −0.78% |
| at PSOR 1000 / at a 0.5 ms step, neutral | 0.9770 | 0.9752 / 0.9752 | −0.19% / −0.18% |

The distance-from-rest figure `hill` reports (−15.4%) reads a little better
than the fitted −16.6%. Rolling resistance ramps in with `tanh(ω)` over the
first few centimetres, so a 2 s distance from rest undercounts it.

### The transmission's output shaft is not inertia

The first account counted every shaft, including the transmission's output
shaft (0.5 kg·m², another 69 kg). That fits every row about 3% worse (rms
2.93% against 0.52%). `ChWheeledVehicle::Synchronize` shows why: each step
the transmission's output shaft is SET to the driveline's speed, and only the
gearbox's reaction torque is passed back. In neutral the 1e20 ratio leaves
that shaft free, so it passes nothing at all.

Trying to prove it by scaling that inertia found something worse. At
`TR_DS_INERTIA` 1.0, 1.5 and 5.0 the simulation goes **NaN**. Sweeping it:
0.6 and 0.7 match 0.5 within 1% (`clutch`); 0.8 is already 6% off; at 0.9
everything is NaN. It is the stability parameter of Chrono's explicit
powertrain-driveline exchange. The stock 0.5 has a 1.6× margin. **Do not
raise it**; the knob stays to show why.

### The flat-ground creep: bounded, not explained

Task 14 found a neutral car on flat ground rolling back at 5.5 mm/s and
blamed the spawn drop: lateral scrub held by the Dahl bristles. **Refuted**:

| after | creep |
|---|---|
| a spawn, 1 s settle (`creep`) | −0.099 m in 20 s, 5.5 mm/s |
| a 15 m straight drive and a braked stop | −0.105 m, 5.9 mm/s |
| a weave (lateral slip both ways), then the same stop | −0.098 m, 5.5 mm/s |
| the straight drive, PSOR 1000 | −0.102 m, 5.9 mm/s |
| the straight drive, 0.5 ms step | −0.118 m, 6.8 mm/s |
| the straight drive, `ROLL_RES=0` | −0.43 m, still accelerating at ~0.002 m/s² |

It does not depend on history, solver convergence or, to first order, the
step. The `ROLL_RES=0` row sizes it: a steady **~4 N** push, 0.02% of the
car's weight. With rolling resistance in, TMeasy's `tanh(ω)` makes rolling
resistance effectively viscous near zero speed, about 640 N per m/s, and it
balances 4 N at ~6 mm/s. The mechanism is not identified. For the rollback
rule it is bounded: at most 3.5 cm over a 5 s hill start against a 0.30 m
threshold, and on a slope gravity is 500 times larger.

### Harness

- `rollacct [grade] [rr] [tr_ds]`, asserted: neutral within 1%, every row
  within 2%, and A must beat B. Pass the values a regenerated car was built
  with, so the printed account is that car's.
- `SHIM_DT` overrides the harness's 1 ms step, for step-size experiments
  only.
- `gen_innova.py` gains `TR_DS_INERTIA`, an experiment knob, default 0.5.

Nothing in the vehicle changed. `determ pullaway` (`08eb1d195ace64ff`) and
`verdict course` (`1e8c06f3ddc60388`) are bit-identical.

## Task 8: an engine that idles, and stalls

### The three defects the brief listed

1. **The losses map's x-values were in rad/s.** `gen_innova.py` wrapped them
   in `rpm2rads()`, directly under a comment warning that Chrono converts rpm
   itself. Chrono read 146.6 … 460.8 "rpm", so the −60 N·m peak arrived at 461
   rpm instead of 4400.
2. **−20 N·m at 0 rpm**, zero only at −100. A declutched engine was driven
   backwards and idled at −98 rpm. In gear with the pedal up it drove the car
   backwards at 0.18 m/s.
3. **No idle governor.** Chrono's `EngineShafts` has none: torque is
   `curve(ω) × throttle` (`ChShaftsThermalEngine::ComputeTorque`), and a
   second thermal engine applies the losses at throttle 1, always.

### What was done

- **Losses** in rpm, odd-symmetric through zero (they oppose rotation either
  way and vanish at rest), with a 100 rpm ramp through the sign change.
  Magnitudes unchanged `[EST]`: FMEP ~1.2 → 2.6 bar.
- **Governor, in the shim**: fuel = max(pedal, (idle − rpm) / 200 rpm), the
  minimum-speed half of a diesel's min-max governor. It is proportional, so
  it droops about 17 rpm unloaded. The loop's time constant is ~40 ms against
  a 1 ms step. The idle speed is an extra key in the engine's own JSON,
  `"Idle Speed [rpm]"`, which Chrono ignores and the shim reads with
  `ReadFileJSON`: one source, `gen_innova.py`'s `IDLE_RPM`.
- **Stall.** The governor exposed that the engine could not stall. The torque
  map gives 250 N·m at 0 rpm (Chrono has no firing speed), so held on the
  brakes in gear it pushed on at 0 rpm and turned the rear wheels through
  their 1,923 N·m of brake. Now the engine dies below **300 rpm** once it has
  run. It then gets no fuel, only its losses, until the pedal has been down,
  or neutral selected, for **1 s**. That is the proxy for the key, since the
  ABI has no ignition.
- `CHS_IDLE_GOVERNOR=0` turns both off: the pre-task-8 engine, same code.

### Measured

| | before | after |
|---|---|---|
| engine declutched at rest (`assemble`) | −98 rpm | **783 rpm** (17 rpm of droop, as predicted) |
| `creep`, 1st, pedal up | backwards, 0.18 m/s | **forward, 1.60 m/s**; idle through 1st predicts 1.59 |
| `creep`, 1st, pedal up, brake 1.0 | still (the engine at 0 rpm) | 0.137 m lurch as the flywheel unloads, **stalls, held** |
| `clutch`, pedal down, 50% throttle | free, 3523 rpm | free, 4279 rpm (the losses now act at their own rpm) |
| `hill 0.12` LAUNCH, clutch dumped rolling back | rolls back, engine back-driven | **pulls away**, +11.6 m at 4 s |
| `solvers` launch, 3 s | 4.18 m, 2.91 m/s | 9.30 m, 5.72 m/s |

**The brief's own verify criterion was wrong.** "creep (1st, pedal up, must
stop moving)" was written against an engine with no governor, which is item 3
of the same brief. A governed diesel in 1st with the pedal up idles forward.
`creep` now asserts that, at the predicted speed, and asserts the stall.

### The pilot drives on the clutch

The car now idles along at 1.6 m/s in 1st or reverse, so a 0.6 m/s parking
crawl needs the clutch. That is the technique the SIM A test examines. Below
idle speed the pilot takes the throttle off, lets the governor hold idle, and
runs a PI loop on the clutch's CAPACITY: `(1 − pedal) × 600 N·m` passes
while it slips. Above idle speed it lifts the pedal over ~0.3 s and drives on
the throttle. The hill start follows the same method: brake held, pedal
lifted to 10% capacity (holding 12% needs 7%), brake off, hand over to the
clutch loop at the same capacity. The throttle floor, `PILOT_TH_LAUNCH`,
existed only because every launch dragged an ungoverned engine from ~0 rpm.
It is gone.

A traced bay park: 0.600 m/s held on a steady ~0.99 pedal, throttle 0, no
brake, engine 778 rpm. The launch dips it to 728. Over the whole course the
lowest engine speed with the clutch carrying any torque is **654 rpm**. The
course loop's tracker had only counted pedal < 0.5, and a crawl never gets
there.

`mzlow` had silently lost its lowest probes. Its throttle loop could not go
under idle speed, so 0.15, 0.30, 0.50 and 0.80 m/s all ran at 1.16 m/s (the
idle speed at 60% lock), and the table read −4.85 N·m four times over.
Caught because four different speeds gave the same number. It now crawls on
the clutch and holds every probe.

### Re-baselined (task 10 → task 8)

| measurement | task 10 | task 8 |
|---|---|---|
| `verdict course` | PASS, `0b19f42c2e895875` | **PASS, `1e8c06f3ddc60388`**, both processes |
| legs A–F, stop vs plan | 1.4 / 9.0 / 1.3 / 1.5 / 0.8 / 1.1 cm | 0.6 / 9.0 / 0.6 / 1.5 / 1.5 / 0.7 cm |
| parallel / bay / short bay | 0.1290 / 0.1167 / −0.1937 | 0.1289 / 0.1165 / −0.1936 |
| `course_cone` / `course_nostop` | FAIL (cone) / FAIL (no stop) | same |
| `determ pullaway` / `upshift` | `eae78a8ba9b6fbe8` / `fb0f5c765db41301` | `08eb1d195ace64ff` / `df10ad4d7fd55f06`, cross-process identical |
| `mzlow`, blend 0.20, col @ 0.30 / 5.0 m/s | −0.631 / −5.008 N·m | −0.694 / −5.098 N·m |
| `sweep` torque peak / grip peak | 9.883 @ 0.60 / 0.80, 0.77 g, 17% | 9.981 @ 0.60 / 0.80, 0.79 g, 28% |
| `drift pullaway`, lateral at 32.7 m | 0.086 m | 0.17 m: the script now makes more torque (below) |
| `hold`, `attitude`, `diff`, `neutral`, `terrain` | pass | pass |
| `perf` mean / p99 / max | 136.3 / 170.5 / 312.0 µs | 133.4 / 165.7 / 260.7 µs |

**The drift row is not a geometry change.** `pullaway` is 35% throttle in 1st
for 12 s. With the losses at their real rpm the engine nets more torque: 8.7
m/s at the end where it was 6.5, and 66 m covered where it was 33. The M6
mechanism is roll steer from drive torque, and the body now rolls 0.75°
under power where it rolled 0.3–0.5°.

### Not done

- **The pedal map is linear**: capacity = `(1 − pedal) × 600 N·m`, so a crawl
  lives in the last 1–7% of travel. A real clutch engages over a band in the
  middle of its travel. That is clutch ergonomics (task 12, item 5) and is
  not tuned blind. It will bite the KEYBOARD first, though: its clutch key
  crosses 7% of travel in 35 ms.
- **A dead engine reads 0 rpm**; no explicit flag crosses the ABI. The HUD
  (task 5) may want one, which would be ABI 6.
- The scripted lane-keeper runs (`judge slalom|pullaway|upshift`) steer along
  the centreline through the slalom cones and never could pass. With and
  without the governor they FAIL, with no STALL. They are event-flow and
  determinism checks.

## Task 10: a true neutral

### Why PSOR can take the 1e20 that broke MINRES

Chrono's neutral, read in `ChTransmission.cpp`: `SetGear(0)` sets the
`ChShaftsGearbox` ratio to `CHRONO_NEUTRAL_GEAR_RATIO`, `#define`d `1e20`, and
`ChManualTransmissionShafts::Synchronize` opens the clutch in gear 0. The
gearbox row is `t·(ω_in − ω_casing) − (ω_out − ω_casing) = 0`. With t = 1e20
it pins the input shaft to the gearbox casing and leaves the driveshaft free.
That is a neutral.

MINRES is a Krylov method over the whole KKT system, so a row scaled 1e20
puts a 1e40 spread into the operator it iterates on. It stopped converging
and dropped the car through the ground (M5). PSOR relaxes one row at a time
and projects each exactly: that row pins the input shaft, and no other row
ever sees its scale. The brief proposed patching the constant to 1e4–1e6.
Under PSOR that is unnecessary: no patch and no image rebuild.

### The shim

`Chs_SetDriverInput` passes gear 0 to the transmission under a VI solver.
Under `CHS_SOLVER=minres|qr|lu` it keeps M5's workaround (top gear held,
clutch forced open), because a linear solver still cannot take the ratio.
`CHS_NEUTRAL=hold` restores the workaround under any solver, so the
comparison below is the same code both ways.

### Measured

| | `CHS_NEUTRAL=hold` | true neutral |
|---|---|---|
| `hill 0.12` free ROLL vs `g·sin θ` | −16.6% | **−15.4%** |
| select neutral at 4.0 m/s in 1st | **kicked to 4.26 m/s** | 4.005 m/s: coasts |
| engine in neutral, pedal UP, 40% throttle | 3409 rpm, free | same |
| `hold`, PSOR 50 … 1000 on 12% | −2.0 … −0.4 mm | identical |
| `solvers` statics, PSOR 50 | 0.32557 m, 53.00% | identical |

**The ROLL gain is the input shaft.** 0.3 kg·m² geared to the wheels through
5th and the final drive (3.28:1) is `0.3 × 3.28² / R²` ≈ 30 kg of car, which
predicts ~1.5 points of `g·sin θ`. Measured: 1.2.

**The kick was a defect in the workaround that nobody had seen.** Selecting
neutral made the shim shift to 5th. The gearbox constraint then forced the
input shaft, spinning at 1st-gear speed, down to 5th-gear speed in one step,
and its angular momentum went into the car: +0.26 m/s, every time a driver
went to neutral at 4 m/s in 1st. The opposite case is shared by both
versions and remains: taking a gear while moving spins the input shaft up
to speed in one step, where a real synchromesh takes a fraction of a second.

### The residual statistic is not a convergence measure in neutral

`CHS_SOLVER_STATS` prints PSOR's largest constraint violation. In neutral
that is the gearbox row: 1e20 × the input shaft's lag behind the casing,
about **1e13–1e15** at 50 iterations, from a lag of order 1e-6 rad/s. It is
not divergence. The evidence that everything else converges as before is
physical: `hold`'s PSOR 50…1000 sweep and the statics are identical to the
digit, and every run that never selects neutral is **bit-identical**:
`determ pullaway` `eae78a8ba9b6fbe8`, the course verdict `0b19f42c2e895875`,
`course_cone`, `course_nostop` and all three parks. Read the stats in gear.

### Harness

- `neutral` now also drives: 1st to 4 m/s, neutral with the pedal UP and
  40% throttle, then 2nd. Asserted: the engine revs free of the wheels (3409
  rpm, where 2nd would hold 1025), the car coasts (4.005 → 3.848 m/s), and 2nd
  drives it again (3.36 → 7.29 m/s).
- `driveline` and `hill_measure`'s `hold_gear` are **deleted**. They swept
  the gear the workaround held, and a true neutral holds none.

### Re-baselined (task 14 → task 10)

| measurement | task 14 | task 10 |
|---|---|---|
| `neutral` | PASSED (stationary only) | **PASSED**, rolling checks asserted |
| `hill 0.12` HOLD / ROLL | −0.0018 m / −16.6% | −0.0018 m / **−15.4%** |
| `determ upshift` (crosses neutral) | — | `fb0f5c765db41301`, cross-process identical |
| `determ pullaway`, `verdict course`, cone, nostop, parks | | **bit-identical** |
| `attitude`, `hold`, `clutch`, `diff`, `terrain`, `drift`, `sweep`, `mzlow` | | identical |
| `creep`, neutral rows | −0.0987 m in 20 s | −0.0989 m |
| `perf` mean / p99 / max | 136.1 / 164.9 / 350.1 µs | 136.3 / 170.5 / 312.0 µs |

## Task 14: the car sat on the wrong springs

### Measured first: `attitude`

Every suspension frame puts the wheel centre at z = 0 (front at the chassis
origin, rear at x = −wheelbase), so a spindle's z in the CHASSIS frame is its
bump travel away from design. Settled, unbraked, in neutral, all four should
read zero. Before: **front +84 mm, rear +289 mm, pitch −4.37° (nose up),
front 51.2%.** The whole car sagged, and the rear by 29 cm.

### Four errors, each read in Chrono's source

| error in `gen_innova.py` | what it did | fix |
|---|---|---|
| the front spring sat at `y = LBJ_y × 0.70`: 70% of the way out from the CAR's centreline | a lever ratio is measured from the ARM's pivot (y = 0.31): real motion ratio **0.46, not 0.70**, so the front was 2.3× too soft in wheel terms and short of preload | the spring at 70% of the arm's span from its pivot; motion ratios MEASURED on the hardpoints (front 0.700, rear 0.995) |
| the rear "Spring Curve Data" was written as the force, `f = −k·d` | `NonlinearSpringForce` computes `F = P − map(len − rest)`: negated twice, **the rear spring pulled** when compressed, and the table clamps outside ±0.12 m. The body sat on the three-link geometry | linear `Spring Coefficient`, like the front |
| preload from the FULL corner load | the tyre carries the unsprung weight, not the spring; the unsprung mass was a free 42 kg a corner | preload from the SPRUNG corner load; unsprung derived from the component masses (63.2 kg a front corner, 195 kg the rear axle) |
| `CG_Z = 0.62  # m above ground` written into the chassis frame, whose origin is at wheel-centre height | the assembled CG **0.82 m** up at design height, every load transfer a third too big | solve for the body CG that puts the ASSEMBLY at 0.62 m, as x already was |

Also: Chrono's own progressive bump/rebound stops (`Minimum/Maximum Length`,
ADAMS rubber, read in `ChSpringDamper`) now sit 90 mm of bump and 100 mm of
rebound from design `[EST]`. The mass literals in the JSON were replaced by
the constants the CG sums use, so the two cannot drift apart again. The rear
damper's bump:rebound ratio is 2:1, the reverse of the usual. It is noted and
unchanged: no measurement here depends on it.

### Result, asserted

`attitude`: pitch **−0.048°**, roll 0.000°, FL/FR **−3.9 mm**, RL/RR **+0.1
mm**, front **52.99%**. The exact QR solve: 0.32593 m, −0.044°, 52.98%. PSOR 50
matches PSOR 1000 to the last digit at rest.

The 3.9 mm comes from the spawn, not the design. `attitude` repeats the
settle from several heights. From 0.36 m the front sits at 0.0 mm, from 0.33 m
at +0.6 mm, and a 12 s settle from 0.45 m changes nothing. It is state the
tyres hold, not a slow settle: the contact patch scrubs sideways as the
suspension compresses, the Dahl bristles keep it, and the wishbones turn it
into jacking. The assertion bounds are |pitch| < 0.3°, |bump| < 5 mm and
53 ± 0.5%.

### What else moved

- **Steering.** `PINION_CAL` 0.98 → 0.954 (see "The fix broke Ackermann").
  The same lesson twice: a constant fitted in a wrong state encodes that
  state.
- **Zero-steer drift fell 4×**: 0.086 m over 32.7 m, radius ~6200 m (was
  0.338 m, 1590 m). The body still rolls about 0.5° under power, so the
  steer per degree of roll is what fell. The rear three-link, whose link
  angles depend on ride height, is the likely place. Not measured
  separately.
- **The sweep's flag.** `sweep` marked "past grip" when the KINEMATIC
  `v² tan δ / L` passed 0.85 g, which says what the steering wheel asks for,
  not what the tyres give. With the CG at its real height that flag landed on
  the torque peak and reported "NO COLLAPSE". It now measures `v × yaw rate`:
  the torque peaks at 0.60 (9.88 N·m) and has fallen 17% when the car's grip
  peaks at 0.80 (0.77 g). The verdict is now "does the torque fall BEFORE the
  grip peaks".
- `gen_course.py` now PARSES `WDIST_F`. It copied `WDIST_R`, the stale
  constant its own header warns about.
- `05-verify-vehicle-shim.sh batch "<spec>" ...`: one build, many modes, one
  log. This re-baseline is one command.

### Re-baselined (task 15 → task 14)

| measurement | task 15 | task 14 |
|---|---|---|
| `attitude` (pitch / front / rear bump / front load) | −4.37° / +84 / +289 mm / 51.2% | **−0.05° / −3.9 / +0.1 mm / 53.0%** |
| full lock, Ackermann avg | 35.21° (measured 84 mm into bump) | **35.36°**, `PINION_CAL` 0.954 |
| `drift pullaway` | 0.338 m over 32.8 m | **0.086 m** over 32.7 m |
| `sweep` torque peak / grip peak | 7.033 N·m @ 0.45 / "0.70" (kinematic) | **9.883 @ 0.60 / 0.80, 0.77 g**, 17% fallen |
| `mzlow`, blend 0.20, col @ 0.30 / 5.0 m/s | −0.331 / −4.444 N·m | −0.631 / −5.008 N·m |
| `hill` HOLD / ROLL | −0.0071 m / −16.7% | −0.0018 m / −16.6% |
| `creep`, neutral after a 1 s settle | still (the 10/30 s settles rolled +3.5 mm/s) | −0.099 m in 20 s, 5.5 mm/s (the drop, see task 6) |
| `hold` | ALL PASSED | ALL PASSED (12%: −1.2 mm over 20 s) |
| `clutch` / `diff` / `neutral` / `terrain` | WORKS / OPEN / pass / climbs | same |
| `determ pullaway` | `935a951af232e086` | **`eae78a8ba9b6fbe8`**, cross-process identical |
| `verdict course` | PASS, `0975eb5c2454d8af` | **PASS, `0b19f42c2e895875`**, both processes |
| `course_cone` / `course_nostop` | FAIL (cone) / FAIL (no stop) | same |
| parallel / bay / short bay | 0.1126 / 0.1186 / −0.1929 | 0.1290 / 0.1167 / −0.1937 |
| `perf` mean / p99 / max | 147.5 / 224.4 / 398.9 µs | 136.1 / 164.9 / 350.1 µs |
| `vehicle/run.sh assemble`, sum Fz | 17,659.8 N (M4) | 17,657.7 N of 17,658.0 |

`creep` still exits 1, for the reason it has since task 13: in 1st with the
pedal up, the engine's losses drive the car backwards (task 8).

### Not done

- The editor's `DriverEyeChrono` z = 1.15 was placed by eye against the
  sagging car. At design ride height it is 1.48 m above the ground, high for
  a seated MPV driver. The mirrors are aimed from it, so it moves with task
  9's cockpit, not here.

## Q1: ANCF, rFactor 2, and the parking torque

Answered 2026-10-02. The brief, as asked (it was under "Open questions" in
`CLAUDE.md`):

The user's question, rephrased to be answerable (the original is below it):

> Can Chrono's `ANCFTire`, run OFFLINE, give this car better steering force
> feedback in the 1 kHz loop? One tyre only: the Innova's 205/65 R16. No tyre
> pipeline for other cars.
>
> Start from what the repo already has. ANCF was ruled out of the loop at
> 49,709 µs per step for one tyre, 1,125× TMeasy (Hard constraints;
> `DoTire` in `vehicle/test_innova.cpp`; the M4 artifact). That was a TIMING
> measurement on Chrono's shipped Polaris ANCF-4 tyre, because no ply or
> material data is published for a 205/65 R16, so there is no Innova ANCF tyre
> yet. Force feedback is the front tyres' TMeasy `Mz` through the rack (M5).
> TMeasy has no parking (bore) torque, which is most of what a real wheel
> feels at parking speed: column torque is only −0.69 N·m at 0.30 m/s
> (`mzlow`, task 8).
>
> My understanding, to be checked rather than assumed: rFactor 2's tyre model
> (ISI; I believe Terence Groening's work) simulates the tyre's construction
> offline, in the tool that makes the tyre file, and the game runs a cheaper
> model on the precomputed results. Is that what gives rFactor 2 its steering
> feel? It is closed source, so cite what is published, and say what is known
> and what is guessed.
>
> Answer:
> 1. What does published material say about how rFactor 2 builds and uses its
>    tyre data?
> 2. Which way of bringing ANCF in fits this project, and why:
>    (a) tabulate ANCF's steady-state forces and moments on `ChTireTestRig` and
>    interpolate them at 1 kHz in a custom tyre inside the shim;
>    (b) fit TMeasy's (or a Pacejka model's) parameters to ANCF rig runs
>    offline, keeping a standard real-time tyre;
>    (c) keep TMeasy for the car, and use an ANCF-derived table only for the
>    torque sent to the wheel.
>    For each: what happens to the handling, the referee's verdicts and
>    determinism, and whether what the wheel says still matches what the car
>    does.
> 3. Does ANCF on the rig produce a parking torque when the wheel is turned at
>    or near standstill? Measure it before building on it.
> 4. How would a 205/65 R16 ANCF tyre be parameterised with no published
>    construction data, and how would we know it is closer to the real tyre
>    than TMeasy is?
> 5. What a table needs: its inputs (slip angle, slip ratio, load, camber,
>    speed, turn rate at standstill), ranges and resolution from standstill to
>    slalom speed (the pilot's course run is the reference), the offline CPU
>    time at ~50 ms per step, its memory, and the cost of one lookup per step.
> 6. The measurement that would count as done: the table against ANCF at
>    points between the grid, and `sweep` / `mzlow` before and after.
>
> Constraint: do not tune the FFB gain; task 12, item 6, needs the wheel.

Original wording:

> Does the repo contains discussion of chrono::vehicle::ANCFTire? I just
> remembered that using ANCF just during simulation development, while during
> simulation running, the game engine simply interpolate from the calculated
> ANCF solutions, I think that is the approach of rFactor 2 finite element
> tire model as implemented by Terence Groening, but, since the source code of
> rFactor 2 is not open source, how could we use project chrono ANCFTire to
> provide better force feedback data to steering wheel but in a way that it
> could be done in 1000hz maybe like using development phase ANCF
> calculations, then for the playtime the engine is simply interpolate, is
> that how rFactor 2 could provide realsitic steering wheel? but, I dont plan
> to have many unique car tire data, I just want the innova only for this
> project.

**The short answer.**

- **rFactor 2 does not interpolate precomputed forces.** ISI's own 2013 TGM
  guide (quoted in item 1) describes a structural model of the carcass,
  solved offline into a lookup table of carcass properties. At runtime it
  runs a brush contact patch on a rigid ring, and the bristles STICK. Its
  forces and moments, parking torque included, come from that live brush.
- **ANCF as Chrono 10 ships gives no parking torque.** A parked ANCF tyre
  turned on Chrono's contact resists with a torque proportional to the steer
  RATE: 8.4–9.4 N·m at 20°/s, 17.4–17.7 N·m at 40°/s. It is gone within
  10 ms of the wheel stopping. Chrono's SMC friction is viscous by
  construction (`ChContactSMC.cpp`: "TODO: implement proper MultiStep
  mode").
- **Given a friction that sticks, the same tyre does show one.** With a
  bristle law written here (through Chrono's own hook) it resists with
  50–58 N·m while turning at 2 kN, and holds about half of that when the
  wheel stops. Its size is the sliding-patch estimate μ·Fz·r̄, which needs
  no ANCF. Its stiffness is set by the guessed bristle, not by ANCF's carcass.
- **Every ANCF number this repo had came from a tyre that never touched the
  road.** A Chrono 10 `ChSystem` has no collision system until one is set.
  In contact and converged, the shipped tyre needs a 0.1 ms step at
  0.29–0.44 s a step: **2,900–4,400 s of CPU per simulated second**, 60–90×
  what the question assumed. A steady-state table over the course's envelope
  would take about a month of all eight cores, and would still say nothing
  at standstill, where the course spends 11% of its time.
- **Recommendation: option (c), narrowed.** TMeasy keeps driving the car,
  and its rolling Mz keeps driving the wheel. Add a small parking-torque
  STATE model to the column torque only: twist, saturation, and relaxation
  with rolling. Anchor its three parameters on physics and on one
  measurement of the real car (item 4), not on an unvalidated ANCF
  construction. Proposed as task 16, `parking-torque`.

### 1. What is published about rFactor 2's tyre model

The one primary source is ISI's own **"TGM Tyre Tool Quick Start", July 2013**
(build 240), hosted by Studio 397
(`studio-397.com/wp-content/uploads/2016/12/rF2_TGM_TyreTool_QuickStart_V3.pdf`).
Read in full here. What it says, quoted:

- The runtime model: *"The TGM is in essence a brush model, using a 6-DOF rigid
  ring to simulate frequency modes ... the tyre surface essentially made up of
  a bunch of bristles, each capable of providing surface interactions with the
  ground."* Bristles have a static and a sliding friction: *"Static covers
  bristles that are static relative to the track surface ... during normal
  cornering, even toward the peak slip angle, various parts of the tyre contact
  patch are likely to be static."*
- The offline model: *"a highly sophisticated quasi-static model is tested over
  a wide range of conditions to supply adequate information to generate a
  detailed lookup table."* It is a cross-section of 31–49 `[Node]`s (position,
  thickness, plies, materials), swept into `NumSections` (~100) round the
  circumference, 2 layers.
- **What the table holds**: *"a representation of testing results ... used to
  define the physical properties of the tyre, such as carcass rigidities,
  growth rates, etc, and how these change over a wide range of conditions."*
  The conditions are gauge pressure × carcass temperature × rotation speed²
  × *"6 standard predefined tests"*: at least 48 tests, 72 by default, *"4
  hours"* single-threaded.
- The runtime model's own bristle stiffness and damping
  (`TemporaryBristleSpring/Damper`) are tuned *"so that the real time model
  approximates the stiffness as seen in the QSA model."*

Studio 397's blog (2017-12-25, "point-based accelerations for TGM (QSA)
analysis") adds that the QSA's centrifugal loading was made local, prompted by
manufacturer data *"measured at multiple loads"*, and calls it *"the biggest
change in rF2 since the introduction of the contact patch model"*. "TGM" is
commonly expanded as "Terence Groening Model" (community sources only). In a
2021 interview Groening calls it *"the complex tire model in rFactor 2 even if
was perhaps too difficult to mod"*, and says nothing about its internals.

**So the premise needs correcting.** rFactor 2 does NOT run its offline model
and interpolate the resulting forces. It tabulates the CARCASS (stiffness and
growth against pressure, temperature and speed) and computes every force and
moment at runtime, in a brush contact patch whose bristles STICK. The offline
model supplies structure; friction is simulated live. Whatever parking torque
rF2 has comes from that live brush with static friction, not from a table.

**Known vs guessed.**
- Known (the 2013 guide): brush + 6-DOF rigid ring at runtime; static and
  sliding bristles; a quasi-static structural model offline; a lookup of
  carcass properties over pressure, temperature and rotation speed; ~4 h per
  tyre in 2013.
- NOT published, so guessed by anyone who states it: whether the QSA is a
  finite-element formulation (the guide speaks of nodes, sections, plies and
  "meshes", not of elements); the six predefined tests; the runtime rate (no
  primary source found); how many bristles the runtime patch has; and any
  validation of rF2's steering torque against a real car.
- Not read, though they exist: a Politecnico di Milano thesis on optimising
  the TGM for a professional simulator (politesi.polimi.it, HTTP 403 from
  here), and a TU Delft thesis comparing steering forces in a race car and a
  race simulator (repository.tudelft.nl, TLS refused from here; which
  simulator it used was not established). Both are worth reading before
  anything is built on this section.

### 2. Which way of bringing ANCF in

Two facts decide it before any ANCF number does. First, the quantity missing
from the wheel is the PARKING torque, and it is not a function of the
instantaneous slip state. At standstill slip angle and slip ratio are
undefined, and the patch's twist torque depends on how far the wheel has been
turned since it last rolled: it winds up, saturates, holds when the steering
stops, and unwinds over a rolling distance about the patch length. That is
hysteresis with a state, which no steady-state table can hold. Second, no
Chrono 10 real-time tyre has a parking or turn-slip term to fit (TMeasy's
header: "No parking slip calculations / No dynamic parking torque"; Pac02 has
no turn-slip factors).

| | (a) ANCF table drives a custom tyre | (b) fit TMeasy/Pacejka to ANCF | (c) TMeasy drives the car, ANCF only for the wheel |
|---|---|---|---|
| handling | replaced wholesale by an unvalidated ANCF construction | changed by refitted parameters | **unchanged** |
| referee verdicts | every baseline moves; the pilot was tuned on TMeasy; re-validate the course | baselines move | **unchanged: the FFB is an output** |
| determinism | kept (a lookup is a pure function), every digest changes | kept, digests change | **kept, digests unchanged** |
| parking torque | NO: a steady table indexed by slip is singular at standstill; it still needs a standstill model, the thing missing | NO: neither model has the term to fit | **yes, if what it adds is a state model, not a table** |
| does the wheel match the car? | yes, one model | yes, one model | in the parking band yes (below); in rolling only if TMeasy's own Mz is kept there |
| cost | lookup ≤ 2.4 µs a step (measured) + a tyre to write and validate | none at runtime | a few flops a step |

Why (c) does not break "the wheel says what the car does" in the parking
band: the car's steering is POSITION-commanded (the rack follows the
driver's input), so in the real car too the bore torque reaches only the
rack and the hands, and the car's motion at a standstill is held by the very
friction that makes the torque. Above walking pace it would: a rolling-Mz
table from another model would put the "wheel goes light" cue at that
model's slip angle while the car's front grip still follows TMeasy (task 14:
torque peak at steer 0.60, grip peak at 0.80). So the added term must fade
out as the wheel rolls, and TMeasy's Mz stays in charge of rolling feel.

**Recommendation: (c), narrowed to the parking band, as a state model.** A
twist state per front tyre, ψ̇ = (steer rate) − (|v|/σ)·ψ, a torque
M = sat(k_ψ ψ, M_max(Fz)) with Coulomb-style saturation and hysteresis, added
to the column torque and fading with rolling distance. Three parameters per
load: torsional stiffness k_ψ, saturation M_max, relaxation length σ. This
is the rFactor 2 split in miniature: structure measured offline, friction
simulated live. What ANCF could contribute is those three numbers, and only
if it produces them physically: item 3.

### 3. Does ANCF on the rig produce a parking torque? Measured

Measuring it took three fixes before the first number, and each is worth
keeping.

**The rig could not ask the question.** `ChTireTestRig`'s slip-angle joint
(`m_slip_lock`, initialised at the rig origin) turns the wheel about a
vertical axis through the RIG's origin, and the spindle sits 0.3 m beside it
(`3 * dim`). Rolling, that is a small lateral offset. Parked, it tows the
patch 5.2 cm along the road per 10° of "steer": TMeasy on it read Fx from
−3.1 to +3.9 kN, a locked wheel being dragged. So `ancfpark` builds the rig a
parking-torque test needs (`ParkRig` in `test_innova.cpp`): the wheel locked
to a steering motor whose axis is vertical through the wheel centre (zero
scrub radius, caster and inclination), on a carrier that slides vertically
and is the load. The torque the motor needs is the parking torque, and it
matches the tyre's own reported Mz to the digit.

**No ANCF tyre in this repo had ever touched the road.** A Chrono 10
`ChSystem` is built with `collision_system(nullptr)` (`ChSystem.cpp`), and
nothing generates a contact until `SetCollisionSystemType` is called. A
force-element tyre never notices, since it queries the terrain itself. M4's
`tire` mode never set one, so its ANCF tyre floated. Its MINRES/100 solve was
not a solution either: on the parking rig without contact it let a 4.6 kN
load sink at 5 mm/s against a damper that allows 0.23 m/s, and with contact
on it stops at a residual of 0.2–0.3 and goes to NaN. The 49,709 µs/step in
the Hard constraints is the cost of that floating, unconverged tyre.

**With contact on, the shipped tyre would barely sit under a 4.68 kN
corner.** `ancfland` lowers a tyre onto the road and lets it settle:

| configuration (Polaris ANCF-4, 3,134 coordinates) | result |
|---|---|
| MINRES 100 or 300 its, steps 1 ms … 50 µs | residual 0.07–0.3, never converges; NaN within 20 ms at 1 ms |
| SPARSE_QR, 1 ms | 9.4 s per step |
| SPARSE_LU, 1 ms / 0.5 ms / 0.25 ms; explicit or stiff contact; node radius 1 or 2 cm; 1.5 or 4.68 kN | lands, then **diverges 50–80 ms after first contact**: Fz jumps to ~300 kN in one step |
| SPARSE_LU, 0.1 ms, 4.68 kN | carries 2.77 kN at 0.10 s with 15 contacts, then diverges at 0.16 s |
| SPARSE_LU, 0.1 ms, 4.68 kN, node spheres 2 cm | diverges |
| SPARSE_LU, 0.1 ms, 4.68 kN, **contact modulus 20 MPa** (`RIG_CONTACT_E`) | stable, but at 0.5 s still sinking 1.9 cm/s on 34 mm; the contact count flickers 12 → 0 |
| SPARSE_LU, 0.1 ms, **2 kN** (the tyre's design load) | stable: 9.3 mm, secant 215 kN/m |
| **Chrono's own FEA-tyre set-up**: HHT + Newton, triangle-mesh contact, 0.1 ms, 2 kN | **settles**: 8.3 mm, secant 239–242 kN/m, 36 contacts |

The mechanism of the divergence is the contact model. The ANCF tyres' JSON
gives the contact a Young's modulus of 2 MPa. Under Hertz contact on 1 cm
node spheres, 4.68 kN over ~15 contacts sinks each sphere about 11 mm,
deeper than its radius, so node centres reach the road surface and the
sphere-box contact fails. The shell elements suffer too: their EAS
sub-iteration (`ChElementShellANCF_3423`, tolerance a fixed 1e-5, cap 100)
stalls near |HE| ≈ 1e-4 and printed over a million non-convergence lines in
60 ms of one run. Chrono's own FEA-tyre test (`demo_VEH_Test_TireFEA`) runs
this Polaris tyre at 200 kg, 0.1 ms steps, HHT with Newton iterations and
Pardiso: a 2 kN tyre, not a 4.7 kN one. So the clean comparison below is at
2 kN, on that set-up.

**Why the stock contact cannot hold a torque.** Chrono 10's SMC contact keeps
no tangential history: `ChContactSMC.cpp` computes the tangential
"displacement" as |v_t|·dt for both `OneStep` and `MultiStep` ("TODO:
implement proper MultiStep mode"), so the friction force is
(k_t·dt + g_t)·|v_t|: viscous, capped at μ·Fn. A node at rest relative to the
road carries no tangential force at all, which is exactly the adhesion zone
a parking torque lives in. ANCF-tyre work aimed at transients has paired
the shell with a bristle-type friction law: Yamashita, Sugiyama et al.
combine ANCF tyres with LuGre friction for transient braking and cornering
(J. Comput. Nonlinear Dyn., 2015 and 2016; titles found, papers not read
here). `BristleContactSMC` in `test_innova.cpp` is that idea, through
Chrono's own hook (`SetContactForceTorqueAlgorithm`, no patch): an
elastic-plastic bristle per contact, k_t per contact, capped at μ·Fn.
`RIG_STICK=<k_t>` turns it on.

**The measurement.** `ancfpark`: the wheel turned 0 → +4° at a constant rate,
held 0.3 s, through 0 to −4°, held, back to 0. The torque is the steering
motor's reaction. Polaris ANCF-4 at 2 kN on Chrono's set-up unless stated:

| contact, steer rate | while turning, 0.2° → 3.8° | 10–30 ms after stopping | rest of the 0.3 s hold |
|---|---|---|---|
| TMeasy, the car's own (4.68 kN, 1 ms) | **0.000 N·m** | 0.000 | 0.000 |
| Chrono SMC, 20°/s | **8.4–9.4 N·m**, flat with angle | 0.31 | 0.04–0.38 |
| Chrono SMC, 40°/s | **17.4–17.7 N·m**, 2.0× | 0.18 | −0.09 … 0.28 |
| bristle, k_t 3e5 N/m, 20°/s | **50–58 N·m**, from 0.2° on | 31.8–33.6 | **23–31 N·m, held**: 24.0 at the end, 44% |

The rest of the profile, both 20°/s runs (through 0 to −4°, held, back to 0):

| phase | Chrono SMC | bristle |
|---|---|---|
| turning back, +4° → −4° | −9.0 … −9.4 N·m, flat | **−41 N·m 0.2° into the reversal**, −48 at 0.6°, then −47 … −64 |
| held at −4°, 0.3 s | −0.34 → 0.00 | **−44 → −32** |
| turning back to 0° | +8.8 … +9.1 | +37 → +65 |
| back at 0°, 0.1 s later | +0.05 | **+38: torque left in the patch at zero steer** |
| peak, either direction | 9.8 | 63 / 70 |

Both runs are bit-reproducible: the morning's re-run after a reboot
repeated the night's settle and first hold to the last printed digit.

What it says:

1. **Chrono's contact gives a viscous drag, not a parking torque.** The
   torque is flat in angle, doubles with the rate, and vanishes the moment
   the wheel stops. It is about a sixth of the 54 N·m the same patch gives
   when it can stick.
2. **A friction that sticks gives the right shape.** Torque from the first
   fraction of a degree, saturating; held when the wheel stops (44% after
   0.3 s at +4°, 57% at −4°); reversing within 0.2° of reverse travel, a
   hysteresis loop (+53 N·m at +3.4° on the way up, −48 N·m at +3.4° coming
   back); and 38 N·m left wound into the patch with the wheel back at 0°.
   Its level matches the sliding-patch estimate: 54 N·m at 1.85 kN is
   μ·Fz·r̄ with r̄ ≈ 3.2 cm for this tyre's 36-point patch.
3. **But what ANCF should have added is not resolved.** The torque saturates
   by 0.2° of twist. The wind-up is set by k_t, a guess per contact on a
   mesh 5 cm coarse, not by the carcass's torsional compliance. With a
   physical tread stiffness the bristles would be stiffer still, and the
   explicit contact would need a smaller step.
4. **At the Innova's 4.68 kN it does not work.** With the contact modulus
   raised to 20 MPa so that the tyre sits at all, the patch rests on 12
   contacts that flicker to 0 as the stiff contact chatters on the coarse
   mesh. That resets the bristles: 9.9–14.6 N·m while turning, nothing held.
   Chrono's contact on the same tyre: 3.4–5.9 N·m.

So ANCF does not give the wheel what it is missing without a new friction
law. Given one, the number it produces is the one the patch's geometry gives
directly. ANCF is not what stands between this car and a parking torque; a
parking-torque model is.

### 4. A 205/65 R16 ANCF tyre with no construction data

What can be had, and what it is worth:

| input | source | standing |
|---|---|---|
| rim radius 0.2032, section height 0.133, width 0.205 m | the size code | exact |
| inflation | the door placard (230 kPa used, `[EST]` until read off the car) | measurable in a minute |
| cross-section shape | Chrono's HMMWV profile, scaled | a guess at a shape |
| plies, angles, layer thicknesses, moduli | none published for any 205/65 R16 | **guessed** |
| mesh | 60 × 12 shells (3.5 cm round the patch) | a cost/accuracy choice |

`gen_innova.py` now writes exactly that guess, `innova_TireANCF.json`
(offline rig only, never the car): Chrono's HMMWV ANCF-4 plies on this
size's cross-section. Even Chrono's own shipped ANCF tyres are not measured
tyres. The Polaris file is the HMMWV cross-section on a smaller rim with
every modulus divided by about 10, and every section in both is ONE
orthotropic layer at ply angle 0, a smeared stand-in for a ply stack.

Measured, the guess and TMeasy disagree, and nothing here says which is
nearer. TMeasy, synthesised from load index 95, sits 14.8 mm under 4.68 kN:
**317 kN/m** secant (and 317 at 3 kN: linear). The `innova` ANCF guess, on
Chrono's own FEA-tyre set-up, settles 10.1 mm under 4.68 kN on 52 contacts:
**464 kN/m, 1.46× TMeasy**. Its finer mesh (4,694 coordinates) costs 0.96 s
a step, **~9,600 s of CPU per simulated second**. On the linearised set-up,
with the contact stiffened to 20 MPa so that it would sit at all, it carried
4.87 kN on 4.3 mm mid-bounce and diverged at 0.13 s.

**How would we know it is closer to the real tyre than TMeasy?** Only by
measuring the real tyre. TMeasy's parameters come from the load index through
TMeasy's own synthesis; an ANCF tyre built from guessed plies is a different
guess with more parameters, not a better one. A guessed construction can be
pinned only to whole-tyre measurements, and the cheap ones are all on the
car itself:

1. **vertical stiffness**: corner load over deflection, jack and ruler;
2. **footprint length and width** under the static load: an ink print;
3. **the parking torque itself**: the steering-wheel torque needed to turn
   the parked car's wheels, engine off (no assist) and on, read with a
   torque wrench or a spring balance on the rim. That is the one number the
   whole question is about, and it costs an afternoon, not a tyre rig.

Without 3, a parking-torque term should be anchored on the physics that
does not need construction data: a sliding patch resists turning with
μ·Fz·r̄, r̄ the mean distance from the patch centre. For a 12.7 × 16 cm patch
at 4.68 kN that is **220–233 N·m** (μ 0.85–0.9). The saturation is
geometric; what ANCF would add is the torsional STIFFNESS of the carcass
before saturation, and the relaxation with rolling, and item 3 shows this
mesh does not resolve them.

### 5. What a table would need

**The envelope, measured** on the reference run (`CHS_TIRE_ENVELOPE=1 bash
docker/05-verify-vehicle-shim.sh verdict course`: the pilot's whole test,
164,205 steps, verdict PASS `317eca4c2c8d9b64`, unchanged by the diagnostic):

| input | front tyres | rear tyres |
|---|---|---|
| speed along the wheel | −1.08 … +2.78 m/s; **no step above 2.8 m/s** | −0.85 … +2.39 m/s |
| time by speed (FL) | <0.05 m/s 7.3% · 0.05–0.2 1.7% · 0.2–0.5 2.0% · 0.5–1 37.1% · 1–2 18.8% · 2–4 33.2% | |
| slip angle, \|v\| > 0.5 m/s | p1/p99 −3.0/+3.3°, p99.9 6.9°, max 19° | within ±1.5° |
| slip angle, all steps | ±90°: below walking pace it is atan(Vy/\|Vx\|), meaningless | same |
| slip ratio, \|v\| > 0.5 m/s | p1/p99 ±0.37, max ±1.0 | p1/p99 −0.31/+0.33 |
| camber | −3.4 … +3.1° | ±0.13° |
| load Fz | p1 2.7–3.4 kN, p50 4.68 kN, p99 6.1 kN, p99.9 6.8 kN (max 11.6 kN: landings) | p50 4.15 kN, p99 5.9 kN |
| road-wheel steer rate | p50 3–4°/s, p99 165–175°/s, max 360–500°/s | — |
| ... at \|v\| < 0.1 m/s | p99 5–7°/s, p99.9 22–25°/s, max 100–110°/s | — |

Three things follow. The whole SIM A test is driven below 2.8 m/s, so a table
for this project needs no high-speed rows. But **11% of the time is below
0.5 m/s, where slip angle and slip ratio stop being defined** (the ±90°
row): a steady-state table indexed by slip cannot address the parking regime
at all. That regime needs a state, the patch's twist since it last rolled,
not a table row. And the slip-ratio spread at 0.5–1 m/s (p99 ±0.37 on
UNDRIVEN front wheels) is larger than braking explains; it is probably the
wheel-speed chatter task 1 found in TMeasy's blend band (finding 1), not the
driver. Not separated.

**A grid shaped to that envelope**, steady-state, five inputs:

| axis | nodes | values |
|---|---|---|
| slip angle | 19 | 0, ±0.5, ±1, ±2, ±3, ±4, ±6, ±8, ±11, ±15° |
| slip ratio | 11 | 0, ±0.02, ±0.05, ±0.1, ±0.2, ±0.4 |
| load | 5 | 2.5 … 6.5 kN |
| camber | 3 | −3, 0, +3° |
| speed | 4 | 0.5, 1, 2, 3 m/s (nothing slower is a steady state) |

12,540 nodes, six outputs each: **0.30 MB** as float, inside L2. Read by 5-D
multilinear interpolation (32 corners), measured with `bench_tyre_table.c`
on this machine while seven ANCF jobs held the other cores: **1.3 µs per 1
ms step for four tyres, 0.13% of the budget**. A dense 41 × 31 × 7 × 5 × 8
grid (8.5 MB) costs 1.2 µs read smoothly and 2.35 µs read at random. The
lookup is never the problem.

**Generating it is.** A converged ANCF step with contact costs 0.29–0.44 s at
the 0.1 ms the shipped tyre needs, by integrator and by how many runs share
the machine (0.29–0.30 s with three, 0.33–0.44 s with four to seven):
**~2,900–4,400 s of CPU per simulated second**, against the ~50 s the
question assumed (50 ms × 1,000 steps). The finer 205/65 R16 mesh costs
0.96 s a step, ~9,600 s per simulated second. Each table row, slip angle swept
quasi-statically through ±15° over ~10 relaxation lengths (~5 m) at fixed
slip ratio, load, camber and speed, is 1.7–10 s of simulated time (10 s at
0.5 m/s) plus a settle: ~5 s on average. 660 rows × 5 s × 4,000 s/s ≈
**1.3 × 10⁷ CPU-seconds: ~150 days on one core, about a month on all
eight**, before a single point is validated, and with no row that says
anything about standstill. On the question's 50 ms assumption it would have
been ~2 days.

A parking-torque model needs a few dozen runs instead: twist sweeps at three
loads and two rates, plus rolling-relaxation runs, ~20 simulated seconds,
~80,000 CPU-seconds, **a few hours on eight cores**. Its inputs are the ones
the envelope says matter at standstill: road-wheel steer rate (p99.9 22–25°/s,
max 110°/s below 0.1 m/s), load (2.7–6.8 kN), and distance rolled.

### 6. What would count as done

For the recommended route, a parking-torque state model added to the column
torque, "done" is four measurements, none of which is a compile:

1. **The source, first.** Either an ANCF run that settles, converges (a direct
   solve, or an iterative one shown to meet its tolerance), uses a contact
   friction that can stick, resolves the twist before saturation, and passes
   the hold test: torque kept through a hold, hysteresis on reversal. Item 3
   gets all but the resolution. Or the real car's standstill steering torque
   (item 4, measurement 3). Without one of them the model's three numbers are
   the μ·Fz·r̄ estimate and a guessed stiffness, and are labelled `[EST]`
   exactly like that.
2. **The model against its source between the grid.** Fitted at loads
   3, 4.7 and 6 kN, checked at 3.8 and 5.4 kN and at a steer rate not used
   in the fit: peak and held torque within 10% of the source, hysteresis
   loop area within 20%.
3. **`mzlow` before and after**, at 0.0 / 0.05 / 0.15 / 0.30 / 0.80 m/s:
   column torque at standstill goes from ~0 to the model's value, and falls
   toward TMeasy's own Mz as the wheel rolls (by 0.80 m/s within 10% of the
   before value).
4. **What must NOT move.** `sweep` at 8 m/s bit-identical (the term has
   faded out), and `verdict course` digest unchanged, since the FFB is an
   output and the car must not notice (`317eca4c2c8d9b64` today). Plus the
   1 kHz cost: a few flops, so `perf` within noise.

For option (a) it would be the table against ANCF at hold-out points midway
between grid nodes (error < 5% of the local value in Fy and Mz), then a full
re-baseline of every harness, which is why (a) is the expensive one.

The FFB gain stays untouched (task 12, item 6): the parking torque will put
the rim far above the G923's 2.3 N·m (≈220 N·m a tyre, two tyres, /17.5,
×0.3 assist ≈ 7.5 N·m), and how to map that onto the wheel is a hardware
decision.

### Harness

```bash
bash vehicle/run.sh ancfland <tyre> [dt] [minres|qr|lu] [seconds]   # does it settle? cost/step
bash vehicle/run.sh ancfpark <tyre> [deg/s] [amp deg] [dt] [solver] # the parking torque
bash vehicle/run.sh ancfroll <tyre> [m/s] [slip deg] [dt] [solver]  # ChTireTestRig, rolling
#   <tyre>: tmeasy | innova (gen_innova.py's 205/65 R16 guess) | polaris | hmmwv | a JSON path
#   env (forwarded into the container by run.sh): RIG_LOAD, RIG_STICK=<k_t> (a
#   contact that sticks), RIG_CONTACT_E, RIG_NODE_R, RIG_INTEGRATOR=hht,
#   RIG_SURFACE=mesh, RIG_STIFF, RIG_HOLD, RIG_SETTLE_T, RIG_TAIL, ... (header)
#   ANCF needs `lu` and a 0.1 ms step at most; append 2>/dev/null to drop the
#   shell elements' EAS chatter. Expect hours per simulated second. The
#   reference parking run, item 3's bristle row:
#   RIG_LOAD=2000 RIG_INTEGRATOR=hht RIG_SURFACE=mesh RIG_STICK=3e5 \
#     RIG_SETTLE_T=0.3 RIG_HOLD=0.3 RIG_TAIL=0.1 \
#     bash vehicle/run.sh ancfpark polaris 20 4 1e-4 lu 2>/dev/null
CHS_TIRE_ENVELOPE=1 bash docker/05-verify-vehicle-shim.sh verdict course   # item 5's envelope
gcc -O2 -march=skylake vehicle/bench_tyre_table.c -lm && ./a.out [5 axis sizes]  # lookup cost
```

### Corrections this forced on earlier records

- **Hard constraints, "ANCF tyres cannot run in real time"**: the conclusion
  stands; the 49,709 µs/step measured a tyre with no contact and an
  unconverged solve. A contacting, converged step costs 0.29–0.44 s at
  0.1 ms steps: 3,000–4,400× slower than real time.
- **M4's `tire` mode** now sets a collision system. Its ANCF row will read
  differently if re-run, and should be read with the above.
- `ChTireTestRig` is fine for ROLLING tests and wrong for parked steering
  (its slip axis is 0.3 m off the wheel). Use `ancfpark`'s rig for those.

## Task 16: the parking torque

### The model

Turn a parked wheel and the tread does not slide at once. Static friction
holds it while the patch and carcass twist, and the moment builds until the
patch slides. Stop turning and the twist stays wound in. Roll, and fresh,
untwisted tread enters the patch. TMeasy has none of it (Q1: 0.000 N·m on the
parking rig). `ParkingTorque` in `chrono_shim.cpp`, per front tyre:

```
dψ/dt = dδ/dt − (|v|/σ)·ψ          δ: road-wheel angle relative to the chassis
|K·ψ| ≤ M_max = μ·Fz·r̄             elastic-perfectly-plastic: the patch slides
M     = −w(|v|)·K·ψ                added to the front tyres' Mz sum, before ratio and assist
```

| parameter | value at 4.68 kN | source |
|---|---|---|
| K, twist stiffness | 3,720 N·m/rad | `[DERIV]`: brush model on TMeasy's own cornering stiffness, `K = C_Fα (a² + b²)/(3a)`, C_Fα 69.2 kN/rad (reproduced from `GuessPassCar70Par`) |
| patch length 2a | 14.1 cm, live | TMeasy's own `P_len = 2√(R0·depth)`, from its deflection each step |
| patch width 2b | 16 cm | `[EST]`, ~0.78 × the 205 mm section |
| r̄, mean lever of a sliding patch | 5.76 cm, live | closed form for a uniformly loaded rectangle |
| μ | 0.85 | the tyre JSON's own "Coefficient of Friction" |
| M_max | 229 N·m per tyre | Q1's estimate was 220–233 |
| saturation | 3.5° of road wheel, ~62° at the rim | K and M_max |
| σ, relaxation length | a = 7 cm, live | `[EST]`: a rigid-carcass brush relaxes in ~0.45a; the carcass lengthens it |
| w(\|v\|), fade | 1 below 0.20 m/s, 0 above 3.0 | TMeasy's own weight on its Dahl standstill force, `1 − SineStep(v, begin, end)`, read live from the tyre |

The fade is the weight TMeasy gives its standstill bristle force, so the
twist moment lives in exactly that bristle regime. Above `blend_end`,
TMeasy's slip-based `Mz` has the wheel alone. It uses |v|, not TMeasy's
signed `vel.x`: rolling relaxes the twist whichever way the car moves.

K and the patch width are written by `gen_innova.py` into the tyre JSON
("Parking Torque"), beside TMeasy's keys; Chrono ignores the block. This is
the same arrangement as task 8's idle speed. No struct changed, so there is
no ABI bump. `CHS_PARKING_TORQUE=0` reproduces the old wheel with the same code.

**Force feedback only.** The steering is position-commanded, so in the real
car too this torque reaches only the rack and the driver's hands.

### The stiffness, and the one that was not used

TMeasy implies two stiffnesses for its patch, 10× apart:

- **From its cornering stiffness** (the rolling tyre), through the brush
  model: 3,720 N·m/rad. The patch slides after 3.5° of road wheel.
- **From its Dahl standstill bristle**, σ₀ = 10⁵ N/m spread over the same
  patch: ~350 N·m/rad. The patch would not slide until 37°, past full lock,
  so the torque would grow like a plain spring across the whole range.

The first is used. It makes the parked tyre twist with the stiffness the
rolling tyre corners with. Two searches found no published twist stiffness
for this size. Two titles look worth reading, neither read here: Chalmers,
"Tyre measurements and modelling for parking manoeuvres"
(research.chalmers.se/publication/508403), and TU Dresden, "Simulation of the
Steering System Power Demand during the Concept Phase Focusing on Tire
Modelling at Standstill". What settles it is the real car's standstill rim
torque against steer angle (Q1, item 4.3).

### Measured: `mzlow 0.20`, the parking rows

Each row holds a speed, turns the wheel 0 → 0.60 at 0.30/s (~10.6°/s of road
wheel, ~184°/s at the rim), holds 5 s, turns back, holds 0.5 s. Each speed
runs twice, with the model off and on. Column torque in N·m (rim = ×0.3):

| v (m/s) | model | turning | held 0.5 s | steady 4–5 s | turning back | left at 0 |
|---|---|---|---|---|---|---|
| 0 | off | −0.298 | −0.176 | −0.159 | 0.145 | 0.293 |
| 0 | **on** | **−26.492** | −26.282 | −26.168 | 26.331 | 26.419 |
| 0.05 | off | −0.003 | −0.001 | −0.001 | 0.003 | 0.000 |
| 0.05 | on | −26.187 | −18.783 | −0.963 | 26.176 | 18.983 |
| 0.15 | off | −0.005 | −0.004 | −0.004 | 0.003 | 0.000 |
| 0.15 | on | −26.158 | −9.565 | −0.022 | 26.146 | 10.019 |
| 0.30 | off | −0.341 | −0.439 | −0.653 | −0.153 | 0.015 |
| 0.30 | on | −17.580 | −2.930 | −0.669 | 16.399 | 2.454 |
| 0.80 | off | −0.614 | −0.672 | −1.008 | −0.104 | −0.015 |
| 0.80 | on | −6.632 | −0.702 | −1.018 | 5.808 | 0.003 |
| **−0.30** (reverse) | off | **0.000** | 0.001 | −0.004 | −0.004 | −0.001 |
| −0.30 | on | −16.797 | −2.337 | 0.011 | 16.797 | 2.488 |

Asserted, all PASS:
- At a standstill the old wheel turns freely (−0.30 N·m). The new one
  resists, −26.5 N·m (2·M_max/17.5 = 26.2 predicted), holds 99% of that when
  the wheel stops, and reverses with the turn.
- The share while turning falls with speed: −26.2 / −26.2 / −26.2 / −17.2 /
  −6.0. At 0.05 and 0.15 m/s the patch still saturates. While turning, the
  share is K·σ·δ̇/v, and saturation happens where that exceeds M_max.
- A steady lock gives TMeasy's value back within 10% (the brief's criterion):
  0.30 m/s −0.669 vs −0.653, 0.80 m/s −1.018 vs −1.008.

"Left at 0" is the twist wound the other way. A driver who centres a parked
wheel finds it pulled back toward where it came from, as on a real car.

The old steady-lock table, off → on: −0.004 → −0.023 (0.15 m/s), −0.694 →
−0.710, −0.960 → −0.973, −1.141 → −1.148, −1.154 → −1.156, and identical
from 2.0 m/s up. The peak-to-peak and largest-step columns are unchanged at
every speed: the model adds no buzz, and it does not touch task 1's mid-band
chatter (p2p 1.7–10.6 N·m at 1.2–3 m/s) either. The small steady offset, at
most −0.02 N·m at the column and 0.006 at the rim, falls roughly as 1/v.
That fits a slow drift of the road-wheel angle as the car settles into its
circle, which a twist state correctly turns into torque. Not chased: it is
under 3% of TMeasy's own value and at the FFB deadband.

### Found: TMeasy has no aligning moment in reverse

`ChTMeasyTire::Advance` computes `Mz` only when `m_data.vel.x() >=
m_frblend_begin`, and `vel.x` is SIGNED along the wheel heading. Reversing at
any speed, `Mz` is identically zero. The −0.30 m/s rows show it: 0.000
turning, −0.004 holding. Both parking exercises are reversed in, so before
this task the wheel was dead through every parking manoeuvre in the test, not
only at the standstills. A real tyre in reverse does make an aligning
moment: its trail and the caster both act the other way, so the wheel does
not self-centre and feels light. Zero is not a crazy answer, but it is not
the real one. The parking torque now gives the reverse manoeuvres their feel:
−16.8 N·m turning at 0.30 m/s, the same as forward.

### What must not move

| check | parking torque off | on |
|---|---|---|
| `sweep` (43 lines of output) | | **identical** (diffed) |
| `determ pullaway` | `08eb1d195ace64ff` | **`08eb1d195ace64ff`**, both processes |
| `verdict course` | PASS, `317eca4c2c8d9b64` | **PASS 4/4, `317eca4c2c8d9b64`**, both processes |
| `perf 5000` mean / p99 / max | 141.3 / 215.0 / 337.4 µs | 141.1 / 205.8 / 350.3 µs |
| `mzlow`'s existing table, model off | −0.694 @ 0.30, −5.098 @ 5.0 | (task 8's values, so the harness refactor is exact) |

The editor has it too: `docker/04-build-shim.sh`, then the incremental
Unreal build, which copies the shim into the plugin. The plugin's NEEDED list
still names `libchrono_shim.so` and no Chrono or libstdc++ library.

### Not done

- **The rim torque is out of the G923's range.** −26.5 N·m at the column ×
  0.3 is 7.9 N·m at the rim against the wheel's 2.3. Q1 predicted ~7.5. How
  to map it, by gain or by a soft clip, is task 12, item 6. Not tuned here.
- **The assist is constant.** A hydraulic rack has no assist with the engine
  stalled, so a stalled car's parking torque would reach the rim undivided.
  The 0.7 fraction is `[EST]` FFB shaping; real parking efforts suggest more
  assist at a standstill. Both belong with the wheel in hand.
- **Q1 item 6's steps 1 and 2** (validating against a source between grid
  points) need the real car's standstill torque, or an ANCF run with a
  friction that sticks and resolves the twist. Neither exists, so K and the
  patch width stay `[DERIV]`/`[EST]`.
- **Path-curvature turn slip** is left out: δ is measured against the
  chassis, so a steady circle winds no twist. TMeasy leaves it out while
  rolling too.

## Task 9: the cockpit

### What it is

The car was a solid box on four cylinders, with the driver's eye inside it.
It is now a shell to sit in, still built from the engine's Cube and
Cylinder. Footprint, seat and steering range are read from the generators'
own files, never copied:

- **The body** (`AChronoVehicleActor::BuildShell`, 42 parts): panels with
  real openings for the windscreen, side windows and tailgate glass, plus
  pillars, roof, headliner, door cards, a dashboard, and the binnacle over
  the instruments on the right. The footprint is the course's CAR record,
  the outline the referee scores, so the body drawn is the body judged. The
  stations (beltline 1.10 m, cowl, header, pillars) are `[EST]`, chosen by
  what the driver sees from the seat.
- **The seat** is set once, in `gen_innova.py` (`EYE_X/Y/H`), and read from
  `innova_Chassis.json`'s "Driver Position": x −1.35, y −0.38 (right-hand
  drive), **1.36 m** above the ground. The editor had it at 1.48 m and the
  JSON at 1.30.
- **A steering wheel**: a 24-segment rim, 0.19 m radius, with one pale
  segment at the top, three spokes and a hub, on a column raised 28°. It
  rolls by the input × the pinion's maximum, 612.5° from
  `innova_Steering.json`, so full lock is 1.7 turns each way.
- **Wheels** gained a silver rim and a dark spoke. A near-black cylinder in
  the car's shadow had been invisible.
- **The mirrors moved into the cockpit.** The interior mirror hangs under the
  header, the door mirrors stand on the front door corners, and each has a
  housing behind its glass. `-ChronoLookMirror` no longer hides the body.
- **Scenery.** Past the course's patches the view fell into a black void,
  across the windscreen and in every mirror. A 4 km plane 2 cm under the
  course won the depth test against it everywhere (seen with the plane
  coloured red; 1 m under, it did not). Why was not isolated; the plane is
  now cut around the patches' footprints into 10 boxes, so nothing overlaps
  anything. It is drawn, not simulated: a car that left the course would
  still fall.

### Measured: `cockpit/run.sh render`

Two offscreen runs of `driver/scripts/cockpit.txt`, which holds the car
still with the wheel at steer 0.2.

**The wheel** (`-ChronoLookWheel`, a camera on the column axis 0.6 m behind
the rim, on the driver's side). At the screenshot the vehicle logs the input,
the angle it commands, and where the rim's centre and its unturned 12 and 3
o'clock land on screen. The column's frame does not turn with the rim.
`probe.py wheel` walks the rim's circle every 0.5°, finds the bright arc,
and takes its circular mean:

| | |
|---|---|
| view | centre (640.0, 360.0), rim radius 213.3 px; 3 o'clock is clockwise of 12 on screen, so the view is the driver's |
| stripe | one arc, 15.5° wide (one segment of 24 is 15°), brightness 181 against the rim's 29 |
| expected | 0.2 × 612.5° = **+122.50°** clockwise as the driver sees it |
| **found** | **+122.50°**, error +0.00° (tolerance 3°) |

The zero is real: the lit arc is symmetric about the prediction at the
probe's 0.5° step. The frame agrees by eye too. The stripe sits at four
o'clock, and the spokes, built at 3, 6 and 9 o'clock, stand at about 1, 7 and
10. Four mutants of the logged expectation against the same frame all FAIL:
sign flipped (error 115° after wrapping), a 360° full lock (+50.5°), a 3.5°
offset, and 3 o'clock moved to the left (the view from the dashboard's side).
The rim's angle is linear in the input by construction (one product), so one
input settles both sign and scale.

**The driver's view, HUD and all** (`-ChronoShotUI`). The HUD's test panel
sat top left, squarely over the interior mirror. The glass went into the
cockpit after the HUD was laid out, and nothing compared the two. The panel
now sits on the dashboard, bottom right, above the timing line, in a part of
the view the car already blocks. Rather than lay it out by eye, the HUD now
measures it: every frame, `AChronoHUD::CheckMirrorsClear` projects each
glass's corners through the frame's view, measures how much of the box the
panels cover, and logs `HUD over mirrors` whenever a mirror goes off screen,
clear or covered.

| check | result |
|---|---|
| interior mirror, glass (109,73)–(394,161) px | **0.0%** under a panel; the old position covered 85.4% of that box |
| right door mirror, (991,421)–(1110,484) | **0.0%** |
| left door mirror | off screen; see "Not done" |
| black (max channel < 16) in each mirror's target | **0.00%**, all three |
| black in rows 361–400, just below the horizon | **0.00%** |

The horizon is the centre row because the view is level (the car sits at
−0.05°, task 14). The ground ran out just below it: 129 m ahead is 0.6°
down, and the patch's side, 20 m off at the frame's edge, about 3°. The same
criterion finds 9.33% black in the dashboard rows, so it is not blind.

### The mirrors, with the body in place: `mirror/run.sh render`

**ALL MIRRORS MEASURED CORRECT**, on the final build:

| | in the target | on the glass, in the look view |
|---|---|---|
| interior | 0.02, 0.10 px | 0.60, 0.71 px |
| left door | 0.03, 0.08 | 1.07, 1.09 |
| right door | 0.05, 0.10 | 0.86, 0.90 |

All three frustums are the aperture (corner error 0, glass clipped). The run
before the HUD moved gave the same targets and glass errors within 0.06 px
of these. Task 2's glass errors were within 0.24 px with the mirrors in
their old places and no body. These are within 1.09 px, against a 6 px
tolerance. They are systematic and mirror left to right (−1.1 px left, +0.9
right, −0.6 interior, in x). That is about one render-target texel as magnified on screen: a door
mirror's 512 texels span ~550 px in its look view. Not chased.

### What must not move

| check | result |
|---|---|
| `determ pullaway` | **`08eb1d195ace64ff`**, both processes. Only "Driver Position" changed in the vehicle JSON, which is camera-only (task 2) |
| the 1 kHz loop under cockpit rendering, 110 s of `circle.txt` | mean 176.7–179.6 µs per step, max 611; **0 missed in ~103,000 steps**; lateness 0.4–0.7 µs mean; one `SAMPLE_GAP`, the screenshot's. Task 5 recorded 205–220 µs and 2 over budget |

### Frame time, like for like

The pause recorded the box car at 25.4 ms on `circle.txt` and the cockpit at
27.7–28.3 on `cockpit.txt`: a different script, on a different day. Measured
again today, same script, 110 s each (`cockpit/run.sh frametime`, windows of
5 s after the session start):

| build | frame mean | 5 s windows | max |
|---|---|---|---|
| box car: the plugin at `1628304^`, plus this task's frame-time log line | **27.84 ms** | 27.45–28.29 | 35.48 |
| cockpit | **28.48 ms** | 28.14–28.75 | 36.02 |
| cockpit, no screensaver | 28.46 ms | 28.17–28.77 | 36.52 |
| box car, 2026-09-29 (the pause's baseline) | 25.4 ms | 25.2–26.7 | — |

**The cockpit costs 0.6 ms a frame (2%).** The box car was built by checking
out the plugin at `1628304^` and keeping only `ChronoWorldSubsystem.{h,cpp}`
from HEAD (their only change in that commit is the frame-time line). It was
then built, run, and the work restored. An xfce screensaver started between
the runs. It was left running rather than unblanked, because unblanking would
have unlocked an unattended desktop, and the cockpit was run again under it:
0.02 ms difference. **The unchanged box car is 2.4 ms slower today than on
2026-09-29**, four times what the cockpit costs, and that drift is most of
what the pause saw. Cause not identified here; task 18 found the frame
GPU-bound and a lit display the leading suspect ("The day-to-day drift,
revisited"). Every window is under the session start's 40 ms.

### Corrections this forced on earlier records

- **`HighResShot` includes the canvas HUD.** Task 5 recorded that it "renders
  the 3-D scene only". The look-mirror frames show the panels. `AHUD` draws
  into the viewport, and `HighResShot` leaves out only Slate. Nothing measured
  was wrong: no panel covers a marker or the rim. The comment in
  `ChronoGameMode.cpp` is corrected, and task 5's paragraph is annotated.

### Harness

`cockpit/run.sh render` (the two runs above, ~7 min) and `frametime` (~3
min), measured by `cockpit/probe.py wheel|seat|frames`. The probe borrows
`mirror/probe.py`'s pure-Python PNG decoder, loaded by path because both files
are called `probe.py`. New in the plugin:

- `-ChronoLookWheel`, the camera down the column;
- `AChronoVehicleActor::LogWheelExpectation`, called from the `-ChronoShot`
  timer beside the mirrors' probe, which, like that probe, refreshes the
  camera first;
- the HUD's `HUD over mirrors` line.

### Not done

- **The left door mirror cannot be seen without a head turn, and nothing
  turns the head.** (Done by task 17: a glance.) From the right-hand seat it is 55° left of straight ahead
  (eye (−1.35, −0.38), glass (−0.38, +1.00)), past the 45° half-FOV of the
  forward view. That is real: a driver of this car turns their eyes to it.
  But **both parking spaces are on the left** (zones at y 0.26–3.0 and
  4.0–11.1, Chrono's +y), and both are reversed into, so the left mirror is the
  one a parking driver needs most. A glance control (a key, and on the G923 a
  button or the D-pad) is a task of its own; it touches input, the camera and
  the HUD check above.
- **60 Hz.** (Done by task 18: 16.67 ms, the GPU at 8.8.) 28.5 ms is 35 frames a second, against the 60 Hz render target
  in `CLAUDE.md`'s hard constraints. The box car was no better. Where the time
  goes (GPU, the render thread, the three scene captures) is unmeasured. It
  predates this task.
- The body is cubes. It is a placeholder for a real mesh, which this project
  does not have, and every station in it is `[EST]`.

## Task 17: the glance

### The problem

From the right-hand seat the left door mirror is 55° left of straight ahead
(eye (−1.35, −0.38), glass (−0.38, +1.00)), past the forward view's 45°
half-FOV, and nothing could turn the head. Both parking spaces are on the
left and both are reversed into, so it was the mirror a parking driver needs
most.

### What it is

A **glance axis**, −1 (the left door mirror) … 0 (ahead) … +1 (the right),
in `DrvInput` beside the controls. It is the head, not the car, and never
reaches Chrono. It comes from three sources, the same way the controls do:

- **Keys, held** (Z/C for the free left hand when the arrows steer, comma and
  period for the free right hand when WASD does). The game thread publishes
  them in the key mask (`DRV_KEY_GLANCE_L/R`), and `drv_keyboard_step`
  integrates them at 1 kHz. Unlike the rack, the head **returns by itself**
  on release: a mirror check is a glance, and a driver who must press a key
  to look ahead again has been made to stare at the mirror.
- **A script's optional 7th column**, interpolated like a pedal. Six-field
  lines load unchanged as glance 0. The loader now rejects an eighth field,
  and a fractional gear such as `1.5`, by line number; both were silently
  read before.
- **The wheel's D-pad**, `ABS_HAT0X` → `WheelState.dpad_x`, integrated on the
  physics thread by the same `drv_glance_step`. This is the HID hat a
  Logitech wheel reports under Linux, and an **unverified default for the
  G923**, like the shifter gate codes: task 12, item 7.

The rate is 4 /s (`DRV_RATE_GLANCE`, `[EST]`), 0.25 s from ahead to a mirror.
The fine modifier does not slow it, because it is for limbs on controls.
`drv_glance_ease` (smoothstep on |g|) shapes it for the camera, so the head
starts and stops without a jolt from any source. Through the ease the left
mirror's 55° peaks near 330°/s, inside what a head does.

The camera turns by a slerp from straight ahead to the glass centre, by the
eased glance, with no roll. **The eye does not move**: each mirror is exact
for that one point, so what the glass shows is unchanged and only its place
in the view moves. That is also why the glance needs no new visibility
check. The rays from the eye to the glass are the ones `mirror/run.sh
render` already found unobstructed (its markers on the left glass, seen
from this eye).

### Measured

- **`driver/run.sh selftest`, 66 assertions (20 new):** the rate (−0.4 after
  0.1 s), the clamp, no control moved, the return on release with no step
  over rate × dt, both keys meaning ahead, the fine modifier ignored, the
  D-pad path, the ease (fixed ends, odd, monotonic, flat at both ends: slope
  0.03 against a linear 1), six- and seven-field lines mixed, interpolation,
  hold past the end, no built-in ever glancing, and the eighth field and
  fractional gear rejected.
- **`wheel/run.sh decode`:** D-pad left, right and released map to −1/+1/0,
  and the rim does not move.
- **`cockpit/run.sh render`, the glance run** (`driver/scripts/glance.txt`,
  turned left at t = 1.0–1.25 s and held): glance −1.0000, eased −1.0000,
  toward `MirrorLeft`. **The left glass lands at (615,342)–(667,379) px,
  centred 1.1 px from the frame's centre, 0.0% under a HUD panel.**
- `determ pullaway` **`08eb1d195ace64ff`** and `verdict course` **PASS 4/4,
  `317eca4c2c8d9b64`**, both processes: the new channel moves nothing in the
  car.

### Found on the way: the HUD's report was stale at the shot

The first glance run failed with the glass 134.5 px from centre. The glance
was fine. The HUD logs its mirror report only when a mirror goes off screen,
clear or covered, and the last such change came **mid-turn**: the right
mirror left the screen as the head swept left. The shot timer now asks the
HUD for a report in the frame it captures (`ReportMirrorsNextFrame`, logged
"at the shot"), and the probe prefers that line.

## The scene-only screenshot, made so

Task 5 recorded that `HighResShot` "renders the 3-D scene only", and task 9
found that wrong: the canvas HUD is drawn into the viewport, and `HighResShot`
leaves out only Slate. No measurement was affected, since no panel covered a
marker. The probes still depended on the panels staying where they were, so
the claim is now made true rather than annotated. The `-ChronoShot` timer
hides the HUD (`bShowHUD = false`) before a `HighResShot`, and
`-ChronoShotUI` remains the path that keeps it. Task 5's paragraph is
annotated to say so.

## Task 18: the frame budget

### Measured first: the frame is GPU-bound, and TSR is most of it

The health line now carries `stat unit`'s split, averaged over its 5 s
window: game, render and RHI thread time (`GGameThreadTime` and friends) and
GPU time (`RHIGetGPUFrameCycles`). `-ChronoProfileGPU` runs UE's `ProfileGPU`
5 s before the shot, and `cockpit/probe.py gpu` prints the passes. As found,
60 s of `circle.txt`:

| | frame | game | render | RHI | **GPU** |
|---|---|---|---|---|---|
| as found (TSR) | 25.75 ms | 1.80 | 6.74 | 2.11 | **24.09** |

One profiled frame, 19.75 ms of GPU work:

| pass | ms |
|---|---|
| **TSR**, UE5's default anti-aliasing at Epic, with a **2560×1440** history for a 1280×720 view | **10.89** |
| interior mirror capture (512×128), of which the translucency lighting volume 0.75 | 2.31 |
| deferred lighting | 1.73 |
| screen-space AO | 1.55 |
| motion blur | 0.80 |
| translucency lighting volume, main view | 0.79 |

TSR costs more than the whole rest of the frame. The translucency lighting
volume lights translucent materials, and this scene has none:
`BasicShapeMaterial` and the unlit `M_Mirror` are both opaque. Motion blur is
wrong for a driving test anyway, since it smears the mirrors and the lines
the driver is judged against.

### The change: three renderer settings and a frame cap

In `DefaultEngine.ini`: `r.AntiAliasingMethod=2` (TAA, with no upscaling and
nothing here needing any), `r.TranslucencyLightingVolume=0` (the 5.7 name;
`r.TranslucentLightingVolume` is deprecated), and
`r.DefaultFeature.MotionBlur=False`. Same run:

| | frame | game | render | RHI | **GPU** |
|---|---|---|---|---|---|
| TAA, no volume, no blur, uncapped | 9.90 ms | 1.82 | 5.76 | 1.54 | **8.49** |
| the same, capped at 60 (`frametime`, 110 s) | **16.67 ms** (windows 16.67–16.68, max 19.8) | 1.71 | 5.39 | 1.45 | **8.78** |

The profiled frame falls from 19.75 to 7.46 ms, and the mirror capture from
2.31 to 1.44. **The 60 Hz render target is met with the GPU at about half the
budget.**

**The cap is needed.** Uncapped, the game ran ~100 frames a second for a 60 Hz
panel, and the frames nobody sees cost the 1 kHz loop: 10 missed deadlines in
113,000 steps, worst step 2.25 ms. `t.MaxFPS=60` in an ini **did not work**.
The cvar was set (the log says so) and frames stayed at 10 ms, because
`UGameUserSettings` re-applies its own `FrameRateLimit` (0, unlimited)
through `UEngine::SetMaxFPS`, which writes with the cvar's last set reason,
i.e. the ini's own priority. A saved `GameUserSettings.ini` on this machine
already holds `FrameRateLimit=0`, so a project default would lose here too.
So `AChronoGameMode::StartPlay` sets the limit on the user settings object
and applies it. `-ChronoMaxFPS=<n>` overrides it, and 0 means uncapped.

### The 1 kHz loop at 60 Hz: a few misses, characterised

With the GPU no longer the bottleneck, the CPU-side threads stop waiting on
it. The physics loop, 112,000 steps a run:

| config | runs: missed (worst step µs) |
|---|---|
| TSR, ~37 fps (task 9 build and this one) | **0** (611), **0** (472) |
| TAA, uncapped ~100 fps | 10 (2252), 4 (1193) |
| TAA, 60 fps, thread free to migrate | 6 (2516), 0 (434) |
| TAA, 60 fps, pinned onto CPU 3 (the GPU interrupt's) | 5 (1549) |
| TAA, 60 fps, pinned to CPU 6 | 1 (1245) |
| TAA, 60 fps, kept off CPU 3 | 3 (1163), 0 (517) |

What was ruled out, each by a run:
- **The frame rate itself**: the cap halves the misses but does not end
  them.
- **Page faults** (an RT thread faulting waits on the process's memory-map
  lock): the physics thread's `minflt` stayed flat at 92 through the run.
- **The GPU interrupt.** `irqbalance` routes every amdgpu interrupt (11.3 M)
  and its IRQ-work interrupts (4.9 M) to CPU 3, and the thread migrates
  between ticks (seen on CPUs 1, 3 and 4 in three seconds). Kept off CPU 3
  it still missed 3 and 0, as much as when it was left free.

What remains is ~0.2 involuntary context switches a second on a `SCHED_FIFO`
80 thread: something of higher priority preempts it. That source is not
identified. **0–6 misses per 112,000 steps** (≤ 0.005%), a worst step of 2.5
ms, and a loop on absolute deadlines that catches up within a few steps:
invisible behind the renderer's 2 ms interpolation delay and below anything
a hand on the wheel could feel. Task 5 recorded and accepted 2 in 109,000.
A trap for anyone pinning by hand: **each run starts two physics threads**.
The first belongs to the startup world, lives 0.7 s (704 steps), and exits
when `SimACourse` loads. Pin the second.

### The day-to-day drift, revisited

Task 9 recorded the unchanged box car at 25.4 ms on 2026-09-29 and 27.8 ms on
2026-10-08, cause unknown. Every 28.5 ms run that day had the displays on;
after the host's reset at ~17:16 the session sat at LightDM's greeter with
both displays DPMS-off, and the same TSR scene measured 25.75 ms. A
GPU-bound frame paying ~2.8 ms for a lit display (scanout, or the
compositor's own GPU work) is the leading hypothesis. It is untested: with
no desktop session there was nothing that could wake a display. It matters
far less now: the frame waits on the 60 Hz cap, with the GPU at ~8.8 ms.

### What it changed downstream

`mirror/run.sh render` on the final build: **ALL MIRRORS MEASURED CORRECT**,
the targets unchanged (≤ 0.10 px), and on the glass **0.21–0.64 px**, down from
task 9's 0.60–1.09 px. Task 9 left a systematic, left/right-mirrored ~1 px
glass offset unchased; it went away when TSR did, so TSR's reprojection is the
likely cause. The frames now carry no HUD.

### Harness

`cockpit/run.sh profile` runs the split and the GPU profile, and `frametime`
now prints the split and names the bound. The health line format gains `=
game … render … rhi … gpu …`.

### Not done

- **On screen.** Every number here is offscreen (`-RenderOffscreen`). A
  window on the 60 Hz panel adds presentation and, if wanted, vsync; the cap
  makes the two close, but no on-screen frame has been measured, and the
  display-on hypothesis above is the reason to.
- **Screen-space AO, 1.55 ms**, is kept. It is the depth cue that makes the
  cabin read as a cabin, and the budget has room for it.
