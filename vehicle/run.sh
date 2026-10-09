#!/usr/bin/env bash
# Build and run the Innova verification harness inside the Chrono container.
set -euo pipefail
cd "$(dirname "$0")"
IMAGE="${IMAGE:-chrono:10.0.0-builder}"
STAGE=$(mktemp -d /tmp/innova.XXXXXX); trap 'rm -rf "$STAGE"' EXIT
cp -r . "$STAGE/src"
# Every RIG_* variable reaches the container (the Q1 tyre rigs read them).
RIG_ENV=()
while IFS= read -r v; do RIG_ENV+=(-e "$v"); done < <(compgen -e | grep '^RIG_' || true)
docker run --rm --user "$(id -u):$(id -g)" -e CCACHE_DISABLE=1 -e HOME=/tmp "${RIG_ENV[@]}" \
    -v "$STAGE:/work:z" -w /work "$IMAGE" bash -lc '
        set -e
        cmake -S src -B build -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_CXX_FLAGS="-march=skylake" \
              -DVEHICLE_DATA_DIR=/work/src/ >/dev/null 2>&1 || \
        cmake -S src -B build -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_CXX_FLAGS="-march=skylake" -DVEHICLE_DATA_DIR=/work/src/
        cmake --build build --parallel 4 2>&1 | grep -iE "error|warning: unused|Built target" | head -20
        ./build/test_innova '"$*"'
    '
