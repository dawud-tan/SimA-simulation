#!/usr/bin/env bash
# The physics -> game snapshot ring, stress-tested on the host (task 5).
# Compiles the plugin's canonical ChronoStateRing.h against the shim's C
# header: no Unreal, no Chrono, no container. ~10 s.
set -euo pipefail
cd "$(dirname "$0")"
PLUGIN=../SimA/Plugins/ChronoBridge/Source/ChronoBridge
mkdir -p build
g++ -std=c++17 -O2 -pthread -I"$PLUGIN/Public" -I../shim -o build/test_ring test_ring.cpp
./build/test_ring
