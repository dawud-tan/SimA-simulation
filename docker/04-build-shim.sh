#!/usr/bin/env bash
# Build libchrono_shim.so inside the Chrono container, then export it to the
# host for the Unreal plugin to link against.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${IMAGE:-chrono:10.0.0-builder}"
OUT="${OUT:-$PWD/chrono-sdk}"

# /WIN_D is NTFS and cannot hold SELinux labels, so bind-mount from tmpfs.
STAGE=$(mktemp -d /tmp/chrono-shim.XXXXXX)
trap 'rm -rf "$STAGE"' EXIT
cp -r shim "$STAGE/"

# --user: the builder stage runs as root, which would leave root-owned build
# artifacts in $STAGE that the cleanup trap cannot remove.
# CCACHE_DISABLE: the image puts ccache on PATH via /usr/lib64/ccache, but its
# cache dir is root-owned and we now run as the host user. One .cpp file gains
# nothing from caching anyway.
docker run --rm --user "$(id -u):$(id -g)" -e CCACHE_DISABLE=1 -e HOME=/tmp \
       -v "$STAGE:/work:z" -w /work "$IMAGE" bash -lc '
    set -e
    cmake -S shim -B build -DCMAKE_BUILD_TYPE=Release >/dev/null
    cmake --build build --parallel 4
    echo
    echo "=== exported dynamic symbols (should be ONLY Chs_*) ==="
    nm -D --defined-only build/libchrono_shim.so | grep " T " | awk "{print \"  \"\$3}"
    echo
    echo "total exported: $(nm -D --defined-only build/libchrono_shim.so | grep -c " T ")"
'

# NOTE: Chrono's own .so files carry no RPATH at all, but they do not need one.
# The shim is built with DT_RPATH (not DT_RUNPATH) of $ORIGIN, and DT_RPATH is
# inherited by transitive dependencies -- so libChrono_vehicle.so finds
# libChrono_core.so through the shim's search path. This is exactly why the
# shim link uses --disable-new-dtags.
cp "$STAGE/build/libchrono_shim.so" "$OUT/lib/"
cp "$STAGE/shim/chrono_shim.h"      "$OUT/include/"
echo
echo "Installed:"
echo "  $OUT/lib/libchrono_shim.so"
echo "  $OUT/include/chrono_shim.h"
