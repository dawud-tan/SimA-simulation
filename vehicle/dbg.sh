#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
STAGE=$(mktemp -d /tmp/innova.XXXXXX); trap 'rm -rf "$STAGE" 2>/dev/null || true' EXIT
cp -r . "$STAGE/src"
docker run --rm -v "$STAGE:/work:z" -w /work chrono:10.0.0-builder bash -lc '
    dnf -y install gdb >/dev/null 2>&1
    export CCACHE_DISABLE=1
    cmake -S src -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
          -DCMAKE_CXX_FLAGS="-march=skylake -g" -DVEHICLE_DATA_DIR=/work/src/ >/dev/null
    cmake --build build --parallel 4 >/dev/null 2>&1
    gdb -batch -ex run -ex "bt 12" --args ./build/test_innova '"$*"' 2>&1 | tail -20
'
