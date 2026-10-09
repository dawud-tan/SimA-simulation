#!/usr/bin/env bash
# Task 9 cockpit harness. Like `mirror/run.sh render`, NOT host-native: it
# runs the real game offscreen and measures the frames it saves (probe.py).
# Needs the plugin built (and so chrono-sdk). ~3.5 min per run.
#
#   render     three runs, ~10 min. Two of driver/scripts/cockpit.txt, the
#              car held still with the wheel at steer 0.2:
#                wheel  -ChronoLookWheel: the rim's stripe where the input
#                       puts it, 0.2 x 612.5 = 122.5 deg clockwise;
#                seat   the driver's view, HUD and all: no panel over any
#                       mirror on screen, no void below the horizon or in a
#                       mirror.
#              and one of driver/scripts/glance.txt (task 17):
#                glance the head turned to the left door mirror: its glass
#                       centred in the view and clear of the HUD.
#   frametime  driver/scripts/circle.txt for 110 s, the task 5 run: the
#              rendered frame time after the session start, and where it
#              went -- game, render and RHI thread, GPU (task 18).
#   profile    circle.txt for 60 s with -ChronoProfileGPU: the frame split,
#              and one frame's GPU time pass by pass (task 18).
#
# Mirrors are measured separately, with the body in place, by
# `mirror/run.sh render`.
set -euo pipefail
cd "$(dirname "$0")"
REPO="$(cd .. && pwd)"
SIMA="$REPO/SimA"
UE="${UE:-/WIN_D/protek/UnrealEngine}"
EDITOR="$UE/Engine/Binaries/Linux/UnrealEditor"

# run <name> <shot seconds> <script> [editor args...]: one offscreen run into
# build/<name>, its log, mirror dumps and screenshot collected.
run() {
    local name="$1" secs="$2" script="$3"; shift 3
    local out="build/$name"
    [[ -x "$EDITOR" ]] || { echo "no editor at $EDITOR (set UE=)" >&2; return 1; }
    rm -rf "$out" "$SIMA/Saved/MirrorDumps" "$SIMA/Saved/Screenshots"
    mkdir -p "$out"
    echo "=== $name: rendering"
    timeout 1800 "$EDITOR" "$SIMA/SimA.uproject" -game -ChronoShot="$secs" \
        -ChronoScript="$REPO/driver/scripts/$script" \
        -RenderOffscreen -unattended -ResX=1280 -ResY=720 "$@" \
        > "$out/editor.txt" 2>&1 || true
    cp "$SIMA/Saved/Logs/SimA.log" "$out/" 2>/dev/null || true
    cp -r "$SIMA/Saved/MirrorDumps" "$out/" 2>/dev/null || true
    # `|| true`: under pipefail a run with no screenshot would end the script
    # here instead of being reported by the probe.
    local shot
    shot="$(find "$SIMA/Saved/Screenshots" -name '*.png' 2>/dev/null | head -1 || true)"
    if [[ -n "$shot" ]]; then cp "$shot" "$out/shot.png"; fi
}

render() {
    local rc=0
    run wheel 30 cockpit.txt -ChronoLookWheel
    python3 probe.py wheel --log build/wheel/SimA.log --shot build/wheel/shot.png || rc=1
    run seat 30 cockpit.txt -ChronoShotUI
    python3 probe.py seat --log build/seat/SimA.log --dumps build/seat/MirrorDumps \
        --shot build/seat/shot.png || rc=1
    run glance 30 glance.txt -ChronoShotUI
    python3 probe.py glance --log build/glance/SimA.log || rc=1
    echo; [[ $rc -eq 0 ]] && echo "COCKPIT MEASURED CORRECT" || echo "COCKPIT MEASUREMENT FAILED"
    return $rc
}

frametime() {
    run frametime 110 circle.txt
    python3 probe.py frames --log build/frametime/SimA.log
}

profile() {
    run profile 60 circle.txt -ChronoProfileGPU
    python3 probe.py frames --log build/profile/SimA.log
    python3 probe.py gpu --log build/profile/SimA.log
}

case "${1:-help}" in
  render)    render ;;
  frametime) frametime ;;
  profile)   profile ;;
  *) echo "usage: run.sh render|frametime|profile" ;;
esac
