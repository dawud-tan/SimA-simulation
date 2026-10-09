#!/usr/bin/env bash
# Compile and run the Milestone 1 verification + 1 kHz benchmark inside the
# image. Prints the per-step cost table that decides the Milestone 4 tire model.
set -euo pipefail
cd "$(dirname "$0")"

# The builder image, not the runtime one -- verification COMPILES a program
# against Chrono, and the runtime image has no compiler by design.
IMAGE="${IMAGE:-chrono:10.0.0-builder}"

# /WIN_D is NTFS. NTFS cannot store SELinux labels -- `chcon` there returns
# EOPNOTSUPP -- so a `:z` bind mount straight out of the project directory is
# denied with SELinux Enforcing. (ntfs3 does not change this; it is an NTFS
# limitation, not a FUSE one.) Stage the sources on tmpfs, which does support
# labelling, and mount that instead.
STAGE=$(mktemp -d /tmp/chrono-verify.XXXXXX)
trap 'rm -rf "$STAGE"' EXIT
cp -r verify "$STAGE/"

docker run --rm \
    -v "$STAGE/verify:/work/verify:z" \
    -w /work \
    "$IMAGE" bash -lc '
        set -e
        cmake -S verify -B /tmp/vbuild -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_CXX_FLAGS="-march=skylake -mtune=skylake" >/dev/null
        cmake --build /tmp/vbuild --parallel 4 >/dev/null
        /tmp/vbuild/verify_chrono
    '
