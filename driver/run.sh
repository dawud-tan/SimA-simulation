#!/usr/bin/env bash
# Milestone 6.1 driver-source harness. Host-native: no container, no Chrono,
# no Unreal, no hardware. Same shape as wheel/run.sh and for the same reason --
# a UE launch takes twenty minutes to reach its first Tick, which is not a
# feedback loop anyone can iterate in.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build

CFLAGS="-std=c11 -O2 -Wall -Wextra -g"
CXXFLAGS="-std=c++17 -O2 -Wall -Wextra -g"

# driver_source's canonical home is the ChronoBridge plugin. This harness
# builds THAT copy, never a duplicate -- located by walking up, the same trick
# ChronoBridge.Build.cs uses for chrono-sdk and wheel/run.sh uses for the
# device layer. A hard-coded ../.. chain breaks silently when the tree moves.
find_src_dir() {
    local d="$PWD"
    while [[ "$d" != "/" ]]; do
        local pub="$d/SimA/Plugins/ChronoBridge/Source/ChronoBridge/Public"
        local pri="$d/SimA/Plugins/ChronoBridge/Source/ChronoBridge/Private"
        # Header is Public/ (FChronoPhysicsThread holds these by value), the
        # implementation is Private/. Emit both include paths.
        [[ -f "$pub/driver_source.h" ]] && { echo "$pub $pri"; return 0; }
        d="$(dirname "$d")"
    done
    echo "driver_source.h not found in any parent of $PWD" >&2
    return 1
}
read -r INC SRC <<<"$(find_src_dir)"

build() {
    # driver_source is C compiled as C++, because that is how the UE module
    # compiles a .cpp in Private/. The harness main stays C. extern "C" in the
    # header is what joins them -- and proves the header really is C-clean.
    g++ $CXXFLAGS -I"$INC" -c "$SRC/driver_source.cpp" -o build/driver_source.o
    gcc $CFLAGS   -I"$INC" -c test_driver.c            -o build/test_driver.o
    g++ $CXXFLAGS -o build/test_driver build/test_driver.o build/driver_source.o -lm
}

case "${1:-help}" in
  build)    build; echo "-> build/test_driver" ;;

  # The whole hardware-independent check: rates, clamping, clutch polarity,
  # gear edge detection, axis continuity under 60 Hz key churn, script purity,
  # loader error reporting, and per-step cost.
  selftest) build; exec ./build/test_driver selftest ;;

  # Print a script exactly as the physics thread will see it. Pipe to a file
  # and diff two revisions to see precisely what an edit changed.
  dump)     build; exec ./build/test_driver dump "${2:-slalom}" "${3:-20}" ;;

  list)     build; exec ./build/test_driver list ;;

  *) sed -n '1,4p;/^  [a-z]*)/p' "$0"; echo
     echo "usage: run.sh build|selftest|dump <name|file.txt> [hz]|list" ;;
esac
