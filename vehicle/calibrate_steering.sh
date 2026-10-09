#!/usr/bin/env bash
# Sweeps ACK_SCALE and reports the resulting Ackermann and full lock.
#
# WHY THIS EXISTS: ACK_SCALE and PINION_CAL are empirical fudge factors that
# compensate for the difference between the textbook Ackermann construction and
# the assembled 3D linkage. They are fitted to a SPECIFIC geometry, so any
# hardpoint change silently invalidates them -- which is exactly what happened
# when the tierod inner end was moved to its zero-bump-steer height: bump steer
# improved 9x and Ackermann collapsed from ~90% to ~4%, because 0.18 had been
# fitted against the old, wrong geometry.
#
# Hand-fitting them once and writing the number down is how that trap was set.
# This makes refitting a command.
#
# All variants are built ONCE and evaluated in a SINGLE container invocation:
# the image start and the cmake build dominate, so N runs of run.sh would take
# N times longer for no reason.
set -euo pipefail
cd "$(dirname "$0")"
IMAGE="${IMAGE:-chrono:10.0.0-builder}"
# Candidates are ACK_SCALE:PINION_CAL pairs. They must be fitted TOGETHER:
# ACK_SCALE moves the steering-arm ball joint, which changes the linkage
# ratio as well as the Ackermann, so refitting one alone moves full lock.
CANDIDATES="${*:-1.0:0.92 1.0:1.00 1.0:1.08 1.2:1.00 1.2:1.08}"

STAGE=$(mktemp -d /tmp/innova-cal.XXXXXX); trap 'rm -rf "$STAGE"' EXIT
cp -r . "$STAGE/src"

i=0
for pair in $CANDIDATES; do
    a="${pair%%:*}"; pin="${pair##*:}"
    ACK_SCALE="$a" PINION_CAL="$pin" python3 gen_innova.py >/dev/null
    cp -r innova "$STAGE/variant_$i"
    echo "$pair" > "$STAGE/variant_$i.ack"
    i=$((i+1))
done
# Leave the tree holding the DEFAULT set, not whichever variant was last.
python3 gen_innova.py >/dev/null

docker run --rm --user "$(id -u):$(id -g)" -e CCACHE_DISABLE=1 -e HOME=/tmp \
    -v "$STAGE:/work:z" -w /work "$IMAGE" bash -lc '
        set -e
        cmake -S src -B build -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_CXX_FLAGS="-march=skylake" \
              -DVEHICLE_DATA_DIR=/work/src/ >/dev/null 2>&1
        cmake --build build --parallel 4 >/dev/null 2>&1
        for v in /work/variant_*.ack; do
            n=$(basename "$v" .ack)
            a=$(cat "$v")
            rm -rf /work/src/innova
            cp -r "/work/$n" /work/src/innova
            echo "########## ACK_SCALE:PINION_CAL = $a"
            ./build/test_innova ackermann 2>/dev/null | \
                grep -E "^   1\\.00|^   0\\.70" | awk "{print \"   \" \$0}"
        done
    '
