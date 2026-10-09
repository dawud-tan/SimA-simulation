#!/usr/bin/env bash
# Milestone 6.3 referee harness. Host-native: no container, no Chrono, no
# Unreal, no hardware. Same arrangement as driver/ and wheel/ -- the logic
# lives in the plugin and this builds THAT copy.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build

CFLAGS="-std=c11 -O2 -Wall -Wextra -g"
CXXFLAGS="-std=c++17 -O2 -Wall -Wextra -g"

find_src_dir() {
    local d="$PWD"
    while [[ "$d" != "/" ]]; do
        local pub="$d/SimA/Plugins/ChronoBridge/Source/ChronoBridge/Public"
        local pri="$d/SimA/Plugins/ChronoBridge/Source/ChronoBridge/Private"
        [[ -f "$pub/referee.h" ]] && { echo "$pub $pri"; return 0; }
        d="$(dirname "$d")"
    done
    echo "referee.h not found in any parent of $PWD" >&2
    return 1
}
read -r INC SRC <<<"$(find_src_dir)"

build() {
    g++ $CXXFLAGS -I"$INC" -c "$SRC/referee.cpp" -o build/referee.o
    gcc $CFLAGS   -I"$INC" -c test_referee.c     -o build/test_referee.o
    g++ $CXXFLAGS -o build/test_referee build/test_referee.o build/referee.o -lm
}

case "${1:-help}" in
  build)    build; echo "-> build/test_referee" ;;
  selftest) build; exec ./build/test_referee ;;
  *) echo "usage: run.sh build|selftest" ;;
esac
