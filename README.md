# SIM A Driving Licence Simulator

High-fidelity simulation of the Indonesian **SIM A** practical driving test
(slalom, parallel parking, bay parking, hill start), built on
[Project Chrono](https://projectchrono.org) multibody dynamics rendered through
Unreal Engine 5.8.

Chrono steps a full Toyota Kijang Innova multibody model at **1 kHz on a
SCHED_FIFO thread**; Unreal draws it at 60 Hz by interpolating a lock-free
snapshot ring. The two never share a C++ type — see *The ABI wall* below.

---

## What state this is in

| | |
|---|---|
| Physics, 1 kHz, real-time | **working** — under full rendering (task 5): 205–220 µs mean per step, 2 of ~109,000 steps over the 1 ms budget, 0.8–1.1 µs mean lateness |
| Innova vehicle model | **working**: sits level at its design ride height with the designed 53% front (`attitude`, asserted since task 14); turning circle −0.04% vs published |
| Course, terrain, referee | **working** — generated from dimensions, deterministic end-to-end; parking judged (`judge park_bay`, `park_parallel`) |
| The whole test, judged | **working** — a pilot drives all four exercises in Chrono and the referee returns **PASS, 4 of 4**, identically from two processes (`verdict course`) |
| Clutch and rear differential | **working** since task 13 (PSOR solver); both asserted by `clutch` / `diff` |
| Keyboard + scripted driver | **working** — bit-identical trajectories across processes |
| Rendering | **working, at 60 Hz** (task 18: the GPU needs 8.8 ms of the 16.7) — course, scenery, hill, cones and sky; since task 9 the car is a cockpit you sit in: windows, dashboard, a steering wheel that turns with the input, and a glance at either door mirror (task 17), measured from rendered frames (`cockpit/run.sh render`) |
| HUD | **working** since task 5: speed, gear, revs, pedals, the test's progress and the referee's calls |
| Mirrors | **working** — measured in-editor, seen from the seat through the car's windows: markers placed by geometry land within 0.10 px in the render targets and 0.64 px on the glass (`mirror/run.sh render`) |
| Steering wheel / pedals | **code complete, never run** — no G923 has ever been attached |
| Hill-start rollback rule | **working** since task 7: no rollback allowed (the regulation), wheel travel judged against a 5 cm detection floor |
| Steering feel when parked | **working** since task 16: a parking (bore) torque model in the force feedback, −26.5 N·m at the column turning at a standstill, fading as the car rolls; TMeasy has none, and none at all in reverse (`ENGINEERING_LOG.md`, task 16 and Q1) |

This is a research/engineering project, not a finished product.
`ENGINEERING_LOG.md` is the engineering log and the authoritative record of
what was measured and why each decision was made. `CLAUDE.md` is the brief
distilled from it: the backlog, the constraints and the commands.

---

## Getting started on a fresh Fedora machine

Tested on Fedora 44. Roughly **5 hours** end to end, most of it Unreal: its
dependency download (tens of GB) and a compile measured at 3.4 hours on an
8-core i7-9700 with 16 GiB.

### 0. Prerequisites

```bash
sudo dnf install -y git git-lfs docker docker-buildx cmake gcc gcc-c++ make \
                    python3 binutils
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"     # log out and back in for this to take
```

`docker-buildx` is not optional. The Chrono image needs BuildKit (its
Dockerfile uses `RUN --mount=type=cache`), and Fedora's `docker` package
(`moby-engine`) does not pull it in. Without it, step 3 stops with "BuildKit is
enabled but the buildx component is missing". (This machine runs Docker CE
from docker.com's own repository, which bundles it.)

To clone Unreal Engine you also need **an Epic Games account linked to
GitHub**, because Epic's repository stays private until you accept their EULA
at <https://www.unrealengine.com/ue-on-github>. You also need **an SSH key on
that GitHub account**: `scripts/10-clone-unreal.sh` clones over SSH and
checks `ssh -T git@github.com` first.

Disk: budget **~240 GB**. `scripts/10-clone-unreal.sh` refuses to start with
less than **220 GiB** free where Unreal goes. The Chrono images take another
~11 GB, plus Docker's build cache, and the exported SDK 1.2 GB.

### 1. Clone

```bash
git clone https://github.com/dawud-tan/SimA-simulation.git
cd SimA-simulation
```

Everything it needs is in the clone: the generated vehicle and course files,
and the two content assets (`SimA/Content`). Only build output is ignored.

### 2. Host tuning (optional but recommended)

The 1 kHz thread wants real-time scheduling and a performance governor.
Without these it still runs, but wakeup jitter goes from 0.1 µs to ~86 µs.

```bash
sudo bash scripts/00-remediate-host.sh    # must be root; refuses otherwise
```

Read it before running it. It was written for the machine this was built on
(a Dell OptiPlex 7070), and it does more than tune:

1. It moves Docker's data root to `/home/docker-data`, because `/` there had
   5.9 GiB free. It copies and never deletes `/var/lib/docker`. If your `/`
   has room, run it as `sudo KEEP_DOCKER_ROOT=1 bash scripts/00-remediate-host.sh`.
2. It installs a toolchain: `ninja-build ccache mold lld clang git-lfs`, plus
   Vulkan/GL/SDL development packages, and registers git-lfs's hooks.
3. It enlarges zram and switches it to zstd.
4. It grants real-time priority (`/etc/security/limits.d`).
5. It pins the CPU governor to `performance`.
6. It grants evdev access for the wheel.

### 3. Build the Chrono container (~45 min)

```bash
bash docker/01-build-image.sh         # builds chrono:10.0.0 and -builder
bash docker/02-verify.sh              # step-cost benchmark, optional
```

This clones Chrono 10.0.0, applies `docker/patches/*.patch`, and builds it
`-march=skylake`. The patch is **purely additive** (it exposes TMeasy
standstill parameters); the build fails loudly if it no longer applies.

### 4. Export the SDK and build the ABI shim (~3 min)

```bash
bash docker/03-export-sdk.sh          # -> chrono-sdk/  (1.2 GB, gitignored)
bash docker/04-build-shim.sh          # -> chrono-sdk/lib/libchrono_shim.so
```

### 5. Run the host test suite (~5 seconds, no container, no GPU)

Do this before building Unreal. If any of it fails, Unreal will not help.

```bash
bash driver/run.sh  selftest    # 66 assertions — input integration, scripts, glance
bash referee/run.sh selftest    # 98 assertions — the SIM A rules and the verdict
bash mirror/run.sh  selftest    # 53 assertions — planar mirror geometry
bash ring/run.sh               # the snapshot ring under a real race (~10 s)
bash wheel/run.sh   decode      # 21 assertions — G923 decode, needs no wheel
bash course/run.sh  check       # course JSON matches its generator
```

### 6. Verify the physics through the ABI wall (~2 min each)

```bash
bash docker/05-verify-vehicle-shim.sh assemble   # builds the Innova
bash docker/05-verify-vehicle-shim.sh determ     # bit-identical trajectories
bash docker/05-verify-vehicle-shim.sh judge      # script -> physics -> verdict
bash docker/05-verify-vehicle-shim.sh verdict course   # the whole test -> PASS 4/4
bash docker/05-verify-vehicle-shim.sh attitude   # level, design ride height, 53% front
# Several at once, one build and one log:
bash docker/05-verify-vehicle-shim.sh batch "attitude" "hold" "clutch" "verdict course"
```

`determ` is the important one: it runs the same scripted drive twice in one
process **and** in two separate processes, and compares a digest. If those
match, every referee verdict is reproducible. `verdict course` is the same
check on the verdict itself: the whole course driven twice, in two processes,
with every event and exercise result compared.

### 7. Build Unreal Engine (~3.5 hours to compile, one time)

```bash
bash scripts/10-clone-unreal.sh       # needs the Epic/GitHub link + SSH key
bash scripts/11-build-unreal.sh       # measured: 3.4 h (12,164 s) here
```

Unreal throttles its build by RAM, not cores. With 16 GiB and no limit, its
link steps get OOM-killed hours in. `11-build-unreal.sh` installs
`scripts/BuildConfiguration.xml` (6 processes, 3 GiB per action) into
`~/.config/Unreal Engine/UnrealBuildTool/` when you have no config there.

Both default to `/WIN_D/protek/UnrealEngine`. Override with `DEST=`:

```bash
DEST=$HOME/UnrealEngine bash scripts/10-clone-unreal.sh
DEST=$HOME/UnrealEngine bash scripts/11-build-unreal.sh
export UE=$HOME/UnrealEngine        # the steps below use $UE
```

### 8. Build the project

From the repository's root, as every step here is:

```bash
export UE=${UE:-/WIN_D/protek/UnrealEngine}
$UE/Engine/Build/BatchFiles/Linux/Build.sh SimAEditor Linux Development \
    -project="$PWD/SimA/SimA.uproject"

# REQUIRED, and easy to miss: without it the editor stalls on a dialog
# reading "Unable to launch ShaderCompileWorker".
$UE/Engine/Build/BatchFiles/Linux/Build.sh ShaderCompileWorker Linux Development
```

### 9. Regenerate the content assets (only after changing their scripts)

`/Game/Maps/SimACourse` and `/Game/Materials/M_Mirror` come with the clone.
After editing `scripts/20-make-level.py` or `21-make-mirror-material.py`:

```bash
$UE/Engine/Binaries/Linux/UnrealEditor "$PWD/SimA/SimA.uproject" \
    -run=pythonscript -script="$PWD/scripts/22-build-content.py"
```

`AChronoGameMode` builds the scene procedurally. The level is what lets you
refine mirror placement in the editor.

### 10. Run it

```bash
$UE/Engine/Binaries/Linux/UnrealEditor "$PWD/SimA/SimA.uproject"
```

Press Play. Controls:

| key | |
|---|---|
| `W` / `S` | throttle / brake |
| `A` / `D` | steer left / right |
| `Left Shift` | clutch (held = disengaged) |
| `Left Ctrl` | fine modifier — quarter rate on every axis |
| `Q` | straighten the steering (there is no auto-centre, deliberately) |
| `0`–`5`, `R` | gears |
| `Z` / `C` (or `,` / `.`) | glance at the left / right door mirror while held |

Or drive it from a script instead of a keyboard:

```bash
$UE/Engine/Binaries/Linux/UnrealEditor "$PWD/SimA/SimA.uproject" -game -ChronoScript=slalom
$UE/Engine/Binaries/Linux/UnrealEditor "$PWD/SimA/SimA.uproject" -game -ChronoShot=30  # screenshot, quit
```

The rendered result is measured, not eyeballed, by two harnesses that run the
game offscreen and read the frames it saves:

```bash
bash mirror/run.sh render     # each mirror shows what the geometry says (~6 min)
bash cockpit/run.sh render    # the steering wheel's angle, no HUD panel over a
                              # mirror, no void past the course, the glance (~10 min)
bash cockpit/run.sh frametime # frame time and where it goes, 110 s of driving (~3 min)
bash cockpit/run.sh profile   # the same, plus one frame's GPU time pass by pass
```

Frames are capped at 60 Hz, the display's rate; `-ChronoMaxFPS=<n>` changes
that (0 = uncapped).

---

## The ABI wall

The single most important invariant in this codebase. Two incompatible C++
runtimes live in one process:

- `libChrono_core.so` links **libstdc++** dynamically
- `UnrealEditor` links **libc++** statically

and Chrono is built `-march=skylake` (`EIGEN_MAX_ALIGN_BYTES=32`) while a stock
Unreal translation unit gets 16. Measured: `struct { char; Eigen::Quaterniond; }`
is **64 bytes in one and 48 in the other**, while bare `sizeof(Quaterniond)` is
32 in *both* — so a spot check passes and the heap corrupts silently.

Unreal therefore includes **`chrono_shim.h` only**, which is pure C: fixed-width
integers, doubles, opaque handles. Verify the wall is intact with:

```bash
readelf -d SimA/Plugins/ChronoBridge/Binaries/Linux/libUnrealEditor-ChronoBridge.so \
  | grep NEEDED
```

It must list `libchrono_shim.so` and **must not** list `libChrono_core.so` or
`libstdc++`.

---

## Layout

```
docker/     Chrono container, ABI shim build, and the physics verification
            harness (05-verify-vehicle-shim.sh has 11 modes)
docker/patches/  local Chrono patches, applied during the image build
shim/       chrono_shim.{h,cpp,map} — the ABI wall itself
vehicle/    gen_innova.py -> innova/*.json, plus the vehicle test harness
            (test_innova.cpp: also the offline ANCF tyre rigs, ancfland/park/roll)
course/     gen_course.py -> course JSON + terrain + referee projections
driver/     keyboard + scripted driver harness and example scripts
referee/    SIM A rule engine harness
mirror/     planar mirror geometry harness
cockpit/    measures the cockpit in rendered frames (wheel, seat view, frame time)
wheel/      G923 evdev/force-feedback harness (runs with no wheel attached)
scripts/    host tuning, Unreal clone/build, content generation
SimA/       the Unreal project; all logic lives in Plugins/ChronoBridge
```

**Generated files are owned by their generator.** Edit `gen_innova.py`, never
`innova/*.json`. Edit `gen_course.py`, never `simA_course.json`.
`course/run.sh check` fails the build if you forget.

---

## Known open items

These are measured and characterised, not vague. `ENGINEERING_LOG.md` has the numbers.

1. ~~**Hill-start rollback rule is not implemented.**~~ **Done by task 7**:
   the regulation allows none, and the referee counts the WHEELS rolling back
   past a 5 cm floor. It counts wheels rather than the body because a braked
   car settles back 6–8 cm on its tyres with the wheels locked.
2. ~~**No aligning moment below 1 m/s.**~~ **Fixed by task 1**: the shim moves
   TMeasy's blend start from 1.0 to 0.20 m/s, and column torque at a parking
   lock and 0.21 m/s went from −0.079 to −3.256 N·m.
3. ~~**Mirrors produce no visible reflection.**~~ **Fixed by task 2**, and
   measured: `bash mirror/run.sh render` puts markers where the geometry says
   each mirror must show them and finds them within 0.24 px. See
   `ENGINEERING_LOG.md`, "Task 2: the mirror, end to end".
4. ~~**The clutch is welded shut, and so is the rear differential.**~~
   **Fixed by task 13**: production now uses PSOR, a solver that enforces a
   clutch's torque limit (MINRES could not). It is also cheaper, 136 µs/step vs 192.
   It removed the old flat-ground creep, which was the engine's own losses
   torque reaching the wheels. ~~The engine idled *backwards* when declutched
   and had no idle governor.~~ **Fixed by task 8**: it idles at 783 rpm, idles
   the car forward in gear, and stalls under 300 rpm; the test pilot crawls on
   the clutch like a driver.
   ~~The brake could not hold on a slope.~~ **Fixed by task 15**: Chrono's
   `ChBrakeSimple` cannot stick (dead code), so braked wheels chattered. The
   car now uses `BrakeShafts`, which holds a car stopped on grades up to 45%
   by itself. The parking-hold workaround is off by default.
5. **The car does not track perfectly straight**: ~6200 m radius under power
   since task 14 put it at its design ride height (it was ~1590 m with the
   rear sagging 29 cm). This is *real physics* (driveline torque reacts as a
   roll moment, roll gives roll steer on a solid axle), not a defect. Scripted
   reference runs close the loop with `drv_lane_keep`.
   ~~The car sat 4.4° nose-up with the rear 29 cm into bump.~~ **Fixed by task
   14**: four derivation errors in the springs and the CG height.
6. **Course dimensions are the regulation's, as last published**: Perkapolri
   9/2012, Lampiran A, read in its own text (task 11). The current regulation,
   Perpol 5/2021 as amended in 2023, delegates them to a Korlantas decree that is
   not published, so they may be superseded. The slalom's reverse pass, the
   narrow-lane item and the bridge/descent stops are not built.
7. **No steering wheel has ever been attached.** Five hardware unknowns are
   task 12 in `CLAUDE.md`'s backlog, and at the end of the Milestone 5
   section in `ENGINEERING_LOG.md`.
8. ~~**The left door mirror needs a head turn, and nothing turns the head.**~~
   **Done by task 17**: hold `Z` (or `,`) to glance at it, `C` (or `.`) for
   the right one; on the wheel, the D-pad.
9. ~~**35 frames a second, not 60.**~~ **Done by task 18**: TSR, UE5's default
   anti-aliasing, was 10.9 ms of a GPU-bound frame. With TAA and two unused
   features off, the GPU needs 8.8 ms and frames are capped at 60 Hz. The
   1 kHz loop misses 0–6 deadlines in 112,000 steps at that rate, by
   something unidentified (see the log).

---

## Hardware this was built against

Verified by probing, not from spec sheets:

- **No hardware ray tracing.** Radeon RX 550 (Polaris/GCN4) and Intel UHD 630
  expose neither `VK_KHR_ray_query` nor `VK_KHR_ray_tracing_pipeline`. Lumen is
  disabled in `DefaultEngine.ini`; leaving it on renders the scene **black**.
- **60 Hz render target.** EDID caps the panel on the AMD card at 60 Hz.
- **ANCF tyres cannot run in real time.** In contact and converged, Chrono's
  shipped ANCF-4 tyre needs a 0.1 ms step at 0.29–0.44 s per step,
  3,000–4,400× slower than real time (`ENGINEERING_LOG.md`, Q1; the older 49,709 µs/step
  figure timed a tyre that never touched the road). TMeasy runs in the loop.
- 8C/8T i7-9700, no SMT, no AVX-512, 16 GiB, mechanical disk.

If your machine differs, the constraints in `CLAUDE.md` should be re-measured
rather than assumed.
