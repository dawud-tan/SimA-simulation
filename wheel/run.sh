#!/usr/bin/env bash
# Milestone 5 wheel bring-up. Everything here is host-native: no container,
# no Chrono, no Unreal. The device layer deliberately depends on nothing but
# libc and the Linux uapi headers.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build

CFLAGS="-std=c11 -O2 -Wall -Wextra -g"
CXXFLAGS="-std=c++17 -O2 -Wall -Wextra -g"

# The device layer's canonical home is the ChronoBridge plugin -- there is only
# one copy, and this harness builds that copy rather than a duplicate. Located
# by walking up, the same trick ChronoBridge.Build.cs uses for chrono-sdk: a
# hard-coded ../.. chain breaks silently when the tree moves.
find_device_dir() {
    local d="$PWD"
    while [[ "$d" != "/" ]]; do
        local c="$d/SimA/Plugins/ChronoBridge/Source/ChronoBridge/Private"
        [[ -f "$c/wheel_device.h" ]] && { echo "$c"; return 0; }
        d="$(dirname "$d")"
    done
    echo "wheel_device.h not found in any parent of $PWD" >&2
    return 1
}
DEV="$(find_device_dir)"

build() {
    # wheel_device is C compiled as C++ (that is how the UE module builds it);
    # the harness mains stay C. extern "C" in the header joins them.
    g++ $CXXFLAGS -I"$DEV" -c "$DEV/wheel_device.cpp" -o build/wheel_device.o
    gcc $CFLAGS   -I"$DEV" -c test_wheel.c            -o build/test_wheel.o
    g++ $CXXFLAGS -o build/test_wheel build/test_wheel.o build/wheel_device.o -lm
    gcc $CFLAGS -o build/virtual_g923 virtual_g923.c -lm
}

case "${1:-help}" in
  build)    build; echo "-> build/test_wheel build/virtual_g923" ;;

  # No hardware, no root. Exercises axis normalisation, pedal inversion,
  # gate->gear resolution and poll cost against a socketpair.
  decode)   build; exec ./build/test_wheel decode ;;

  # Enumerate every evdev node; says which driver claimed each and which one
  # scores as the wheel.
  probe)    build; exec ./build/test_wheel probe ;;

  # Live view. Watch the RAW gate bitmask while moving the shifter: that is
  # how the BTN_TRIGGER_HAPPY* mapping gets confirmed on real hardware.
  monitor)  build; exec ./build/test_wheel monitor "${2:-20}" ;;

  # Constant-force triangle sweep. Confirms FFB sign and measures ioctl cost.
  ffb)      build; exec ./build/test_wheel ffb "${2:-8}" ;;

  # The 1 kHz poll+FFB loop in the shape the physics thread will run it.
  loop)     build; exec ./build/test_wheel loop "${2:-5}" ;;

  # Stand up the fake wheel (needs /dev/uinput access).
  virtual)  build; exec ./build/virtual_g923 "${2:-30}" ;;

  # End-to-end: fake wheel + client, in one command.
  selftest)
      build
      ./build/test_wheel decode
      echo
      if [[ ! -w /dev/uinput ]]; then
          echo "SKIP: /dev/uinput not writable -- run scripts/04-uinput-access.sh once (sudo)."
          echo "      The decoder test above is hardware-independent and already covers"
          echo "      normalisation and gear logic; what is skipped is the kernel FFB path."
          exit 0
      fi
      ./build/virtual_g923 12 &
      VPID=$!
      sleep 1
      ./build/test_wheel probe   || true
      ./build/test_wheel monitor 3 || true
      ./build/test_wheel loop    3 || true
      wait $VPID
      ;;

  *) sed -n '1,4p;/^  [a-z]*)/p' "$0"; echo
     echo "usage: run.sh build|decode|probe|monitor [s]|ffb [s]|loop [s]|virtual [s]|selftest" ;;
esac
