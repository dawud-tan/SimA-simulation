#!/usr/bin/env bash
# Build the Chrono 10 image. Expect 25-50 min cold on this machine (8 cores,
# mechanical disk); a few minutes warm, thanks to the ccache mount.
set -euo pipefail
cd "$(dirname "$0")"

IMAGE="${IMAGE:-chrono:10.0.0}"
JOBS="${JOBS:-6}"

command -v docker >/dev/null || { echo "docker not found"; exit 1; }
docker info >/dev/null 2>&1 || { echo "docker daemon unreachable (in the 'docker' group?)"; exit 1; }

ROOT=$(docker info --format '{{.DockerRootDir}}')
FREE_G=$(df -PBG "$ROOT" 2>/dev/null | awk 'NR==2{gsub("G","",$4);print $4}')
echo "Docker root : $ROOT (${FREE_G} GiB free)"
if [ "${FREE_G:-0}" -lt 25 ]; then
    echo "REFUSING: need >=25 GiB free. Run scripts/00-remediate-host.sh first."
    exit 1
fi

# BuildKit is required for the --mount=type=cache lines in the Dockerfile.
DOCKER_BUILDKIT=1 docker build \
    --target runtime \
    --build-arg BUILD_JOBS="$JOBS" \
    --progress=plain \
    -t "$IMAGE" .

# The runtime image is deliberately slim -- runtime libs only, no compiler.
# Tag the builder stage as well: it carries gcc/cmake/ninja alongside
# /opt/chrono, which is what you need to COMPILE anything against Chrono
# (the verification program, and later the Milestone 2 C shim).
# Every builder layer is already cached from the run above, so this is cheap.
DOCKER_BUILDKIT=1 docker build \
    --target builder \
    --build-arg BUILD_JOBS="$JOBS" \
    -t "${IMAGE}-builder" .

echo
echo "Built $IMAGE          (slim, for running headless physics)"
echo "Built ${IMAGE}-builder  (toolchain, for compiling against Chrono)"
docker run --rm "$IMAGE" cat /opt/chrono/share/BUILDINFO
