#!/usr/bin/env bash
# Milestone 5/6: verify the vehicle entry points through the ABI wall.
#
# test_shim.c is compiled as C and linked ONLY against libchrono_shim.so. If it
# runs, no C++ runtime detail is crossing the boundary.
#
# Milestone 6 adds the `determ` mode, which also pulls in driver_source -- the
# keyboard/script layer from the plugin. That file is written as C and is
# compiled here with `gcc -x c`, deliberately: linking it with g++ would drag
# libstdc++ into the test binary and destroy the NEEDED check below, which is
# the whole point of the harness.
#
#   bash docker/05-verify-vehicle-shim.sh <mode> [arg2 [arg3 [arg4]]]
#   bash docker/05-verify-vehicle-shim.sh batch "hold" "hill 0.12" "verdict course" ...
#
# `batch` builds ONCE and runs each quoted spec in turn, every one exactly as
# its single-mode form would (determ and verdict still run in two processes).
# A re-baseline is ~20 modes and the image start plus cmake dominate each, so
# twenty separate runs cost several times longer for nothing. A failing spec
# does not stop the ones after it; the batch exits non-zero if any failed, and
# each is headed `########## <spec>` so one log can be split.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${IMAGE:-chrono:10.0.0-builder}"

# One job line, MODE ARG2 ARG3 ARG4 tab-separated, with the per-mode defaults.
job_line() {
    local MODE="${1:-assemble}" ARG2 ARG3 ARG4
    # ARG2 means different things per mode, so its DEFAULT has to as well.
    # It previously defaulted to 3000 for everything, which handed the terrain
    # mode a step count as a filename.
    case "$MODE" in
        terrain)       ARG2="${2:-/work/course/simA_terrain.txt}" ;;
        determ|drift)  ARG2="${2:-pullaway}" ;;
        hill|standstill|blendhill|rollacct) ARG2="${2:-0.12}" ;;
        judge)         ARG2="${2:-slalom}" ;;
        verdict)       ARG2="${2:-course}" ;;
        # mzlow/creep/sweep take a TMeasy blend_begin; 0 means the SHIM default
        # (CHS_TIRE_BLEND_BEGIN, 0.20), not Chrono's stock 1.0 -- pass 1.0 3.0
        # explicitly to measure stock.
        mzlow|creep|sweep) ARG2="${2:-0}" ;;
        *)             ARG2="${2:-3000}" ;;
    esac
    # ARG3 is `determ` passes, and for `hill` the blend_begin regression argument.
    case "$MODE" in
        hill)          ARG3="${3:-0}" ;;
        # mzlow's second argument is blend_end; Chrono's stock value is 3.0.
        mzlow|sweep|creep|blendhill) ARG3="${3:-3.0}" ;;
        # rollacct: the rolling-resistance coefficient and the transmission's
        # output-shaft inertia the car was GENERATED with (ROLL_RES,
        # TR_DS_INERTIA), so its printed account is that car's.
        rollacct)      ARG3="${3:-0.012}" ;;
        *)             ARG3="${3:-2}" ;;
    esac
    # ARG4 is `hill`'s blend_end, the fourth argument of the mz-below-1ms sweep.
    case "$MODE" in
        rollacct)  ARG4="${4:-0.5}" ;;
        *)         ARG4="${4:-3.0}" ;;
    esac
    printf '%s\t%s\t%s\t%s\n' "$MODE" "$ARG2" "$ARG3" "$ARG4"
}

# /WIN_D is NTFS: no SELinux labels, so a :z bind mount from there is denied
# under Enforcing. Stage on tmpfs, as with 04-build-shim.sh.
STAGE=$(mktemp -d /tmp/chrono-vshim.XXXXXX)
trap 'rm -rf "$STAGE"' EXIT
cp -r shim "$STAGE/"
mkdir -p "$STAGE/vehicle" "$STAGE/driver" "$STAGE/course"
cp -r vehicle/innova "$STAGE/vehicle/"
[ -f course/simA_terrain.txt ] && cp course/simA_terrain.txt "$STAGE/course/"
[ -f course/simA_referee.txt ] && cp course/simA_referee.txt "$STAGE/course/"

# The driver layer's canonical home is the plugin; stage that copy, never a
# duplicate. Same rule as wheel/run.sh and driver/run.sh.
PLUGIN="SimA/Plugins/ChronoBridge/Source/ChronoBridge"
cp "$PLUGIN/Public/driver_source.h"     "$STAGE/driver/"
cp "$PLUGIN/Private/driver_source.cpp"  "$STAGE/driver/"
cp "$PLUGIN/Private/course_terrain.h"   "$STAGE/driver/"
cp "$PLUGIN/Private/course_terrain.cpp" "$STAGE/driver/"
cp "$PLUGIN/Public/referee.h"           "$STAGE/driver/"
cp "$PLUGIN/Private/referee.cpp"        "$STAGE/driver/"
[ -d driver/scripts ] && cp -r driver/scripts "$STAGE/driver/"
true

if [ "${1:-}" = "batch" ]; then
    shift
    [ $# -gt 0 ] || { echo "batch: give one quoted spec per mode, e.g. \"hill 0.12\"" >&2; exit 2; }
    : > "$STAGE/jobs.txt"
    for spec in "$@"; do
        read -r -a parts <<< "$spec"
        job_line "${parts[@]}" >> "$STAGE/jobs.txt"
    done
else
    job_line "$@" > "$STAGE/jobs.txt"
fi

# One job, inside the container, after the build.
cat > "$STAGE/run_one.sh" <<'EOF'
MODE="$1"; ARG2="$2"; ARG3="$3"; ARG4="$4"
if [ "$MODE" = determ ]; then
    ./build/test_shim determ /work/vehicle "$ARG2" "$ARG3" | tee /tmp/run_a.txt
    echo
    echo '--- second process, fresh address space ---'
    ./build/test_shim determ /work/vehicle "$ARG2" "$ARG3" > /tmp/run_b.txt
    A=$(grep '^DIGEST ' /tmp/run_a.txt)
    B=$(grep '^DIGEST ' /tmp/run_b.txt)
    echo "  process A: $A"
    echo "  process B: $B"
    if [ "$A" = "$B" ]; then
        echo '  CROSS-PROCESS: digests match. Trajectory is reproducible'
        echo '  across address-space layouts, so referee verdicts derived'
        echo '  from it are comparable between runs.'
    else
        echo '  CROSS-PROCESS: DIGESTS DIFFER.'
        echo '  The sim repeats within a process but not between them --'
        echo '  look for a pointer-ordered container in the solve path.'
        diff <(grep '^  chassis ' /tmp/run_a.txt) <(grep '^  chassis ' /tmp/run_b.txt) || true
        exit 1
    fi
# `verdict` is `judge` run twice, in two processes, comparing the VERDICT
# DIGEST -- task 4's acceptance test: the same drive must produce the same
# verdict, event for event, whatever the address-space layout.
elif [ "$MODE" = verdict ]; then
    ./build/test_shim judge /work/vehicle "$ARG2" | tee /tmp/v_a.txt
    echo
    echo '--- second process, fresh address space ---'
    ./build/test_shim judge /work/vehicle "$ARG2" > /tmp/v_b.txt
    A=$(grep '^VERDICT_DIGEST ' /tmp/v_a.txt)
    B=$(grep '^VERDICT_DIGEST ' /tmp/v_b.txt)
    echo "  process A: $A"
    echo "  process B: $B"
    if [ -n "$A" ] && [ "$A" = "$B" ]; then
        echo '  VERDICT REPRODUCED: same events, same results, same verdict.'
    else
        echo '  VERDICTS DIFFER between two runs of the same drive.'
        diff /tmp/v_a.txt /tmp/v_b.txt || true
        exit 1
    fi
else
    ./build/test_shim "$MODE" /work/vehicle "$ARG2" "$ARG3" "$ARG4"
fi
EOF

# PARK_TRACE=1 prints the parking pilot's speed/pedal trace (judge park_*).
# CHS_SOLVER=minres|psor|apgd|bb|qr|lu swaps the solver -- diagnostic only;
# CHS_SOLVER=minres reproduces the pre-task-13 car (clutches welded). Also
# CHS_SOLVER_ITERS, CHS_SOLVER_STATS=1, CHS_OPEN_DIFF=1, CHS_NEUTRAL=hold (the
# pre-task-10 neutral), CHS_IDLE_GOVERNOR=0 (the pre-task-8 engine),
# CHS_TIRE_ENVELOPE=1 (the tyres' operating envelope, printed at exit: Q1),
# CHS_PARKING_TORQUE=0 (the wheel before task 16); see
# chrono_shim.cpp.
docker run --rm --user "$(id -u):$(id -g)" -e CCACHE_DISABLE=1 -e HOME=/tmp \
       ${PARK_TRACE:+-e PARK_TRACE="$PARK_TRACE"} \
       ${CHS_SOLVER:+-e CHS_SOLVER="$CHS_SOLVER"} \
       ${CHS_SOLVER_ITERS:+-e CHS_SOLVER_ITERS="$CHS_SOLVER_ITERS"} \
       ${CHS_SOLVER_STATS:+-e CHS_SOLVER_STATS="$CHS_SOLVER_STATS"} \
       ${CHS_OPEN_DIFF:+-e CHS_OPEN_DIFF="$CHS_OPEN_DIFF"} \
       ${CHS_NEUTRAL:+-e CHS_NEUTRAL="$CHS_NEUTRAL"} \
       ${CHS_IDLE_GOVERNOR:+-e CHS_IDLE_GOVERNOR="$CHS_IDLE_GOVERNOR"} \
       ${CHS_TIRE_ENVELOPE:+-e CHS_TIRE_ENVELOPE="$CHS_TIRE_ENVELOPE"} \
       ${CHS_PARKING_TORQUE:+-e CHS_PARKING_TORQUE="$CHS_PARKING_TORQUE"} \
       ${SHIM_DT:+-e SHIM_DT="$SHIM_DT"} \
       -v "$STAGE:/work:z" -w /work "$IMAGE" bash -lc '
    set -e
    cmake -S shim -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build build --parallel 4 >/dev/null
    # In the deployed plugin the Chrono .so files sit BESIDE libchrono_shim.so,
    # which is what its $ORIGIN RPATH resolves to. Inside the container they
    # live in /opt/chrono, so the harness points the linker and loader there
    # explicitly. This is a test-rig detail, not a production path.
    CHLIB=$(dirname $(find /opt/chrono -name "libChrono_core.so*" | head -1))
    echo "chrono libs: $CHLIB"

    gcc -std=c11 -D_GNU_SOURCE -O2 -Wall -Wextra -x c \
        -c driver/driver_source.cpp -Idriver -o build/driver_source.o
    gcc -std=c11 -D_GNU_SOURCE -O2 -Wall -Wextra -x c \
        -c driver/course_terrain.cpp -Idriver -Ishim -o build/course_terrain.o
    gcc -std=c11 -D_GNU_SOURCE -O2 -Wall -Wextra -x c \
        -c driver/referee.cpp -Idriver -o build/referee.o

    gcc -std=c11 -D_GNU_SOURCE -O2 -Wall -Wextra -o build/test_shim \
        shim/test_shim.c build/driver_source.o build/course_terrain.o \
        build/referee.o \
        -Ishim -Idriver -Lbuild -lchrono_shim -Wl,-rpath,/work/build \
        -Wl,-rpath-link,$CHLIB -lm
    export LD_LIBRARY_PATH=$CHLIB
    echo "--- test_shim links (must NOT list libChrono_core or libstdc++) ---"
    readelf -d build/test_shim | grep NEEDED
    echo

    set +e
    rc=0
    n=$(wc -l < /work/jobs.txt)
    while IFS=$(printf "\t") read -r M A2 A3 A4; do
        [ "$n" -gt 1 ] && echo "########## $M $A2 $A3 $A4"
        bash /work/run_one.sh "$M" "$A2" "$A3" "$A4" < /dev/null
        r=$?
        if [ $r -ne 0 ]; then
            rc=$r
            [ "$n" -gt 1 ] && echo "########## $M $A2 FAILED (exit $r)"
        fi
    done < /work/jobs.txt
    exit $rc
'
