#!/usr/bin/env bash
# Extract the linkable Chrono SDK (headers + .so + CMake config) to the host,
# where the Milestone 2 Unreal plugin will consume it.
set -euo pipefail
cd "$(dirname "$0")"

DEST="${DEST:-../chrono-sdk}"
DOCKER_BUILDKIT=1 docker build \
    --target sdk \
    --output "type=local,dest=${DEST}" .

echo "SDK exported to: $(realpath "$DEST")"
cat "${DEST}/share/BUILDINFO" 2>/dev/null || true
echo
echo "Shared libraries:"
find "$DEST" -name '*.so*' -printf '  %f\n' 2>/dev/null | sort | head -20
