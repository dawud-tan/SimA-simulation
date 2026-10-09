#!/usr/bin/env bash
# Milestone 6.4 mirror-geometry harness. Host-native: no GPU, no editor, no
# content. The logic lives in the plugin; this builds THAT copy.
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
        [[ -f "$pub/mirror_math.h" ]] && { echo "$pub $pri"; return 0; }
        d="$(dirname "$d")"
    done
    echo "mirror_math.h not found in any parent of $PWD" >&2
    return 1
}
read -r INC SRC <<<"$(find_src_dir)"

build() {
    g++ $CXXFLAGS -I"$INC" -c "$SRC/mirror_math.cpp" -o build/mirror_math.o
    gcc $CFLAGS   -I"$INC" -c test_mirror.c          -o build/test_mirror.o
    g++ $CXXFLAGS -o build/test_mirror build/test_mirror.o build/mirror_math.o -lm
}

# The end-to-end measurement. NOT host-native, unlike everything above: it
# runs the real editor once per mirror, each with -ChronoMirrorProbe and the
# view aimed at that mirror, then measures the frames with probe.py. Needs the
# plugin built (and so chrono-sdk). ~1 min per mirror once shaders are cached.
render() {
    local ue="${UE:-/WIN_D/protek/UnrealEngine}"
    local sima; sima="$(cd .. && pwd)/SimA"
    local editor="$ue/Engine/Binaries/Linux/UnrealEditor"
    [[ -x "$editor" ]] || { echo "no editor at $editor (set UE=)" >&2; return 1; }
    local rc=0 i shot out
    for i in 0 1 2; do
        out="build/render_$i"
        rm -rf "$out" "$sima/Saved/MirrorDumps" "$sima/Saved/Screenshots"
        mkdir -p "$out"
        echo "=== mirror $i: rendering"
        # -ChronoShot=10: past the startup transient and any late shader
        # compile, which would otherwise draw the glass with a fallback
        # material and measure that instead.
        timeout 1800 "$editor" "$sima/SimA.uproject" -game -ChronoShot=10 \
            -ChronoLookMirror="$i" -ChronoMirrorProbe \
            -RenderOffscreen -unattended -ResX=1280 -ResY=720 \
            > "$out/editor.txt" 2>&1 || true
        cp "$sima/Saved/Logs/SimA.log" "$out/" 2>/dev/null || true
        cp -r "$sima/Saved/MirrorDumps" "$out/" 2>/dev/null || true
        # `|| true`: under pipefail a run that produced no screenshot would
        # otherwise end the whole script here instead of being reported.
        shot="$(ls "$sima"/Saved/Screenshots/*/HighresScreenshot*.png 2>/dev/null | head -1 || true)"
        if [[ -n "$shot" ]]; then cp "$shot" "$out/shot.png"; fi
        python3 probe.py --log "$out/SimA.log" --dumps "$out/MirrorDumps" \
            ${shot:+--shot "$out/shot.png"} || rc=1
    done
    echo; [[ $rc -eq 0 ]] && echo "ALL MIRRORS MEASURED CORRECT" || echo "MIRROR MEASUREMENT FAILED"
    return $rc
}

case "${1:-help}" in
  build)    build; echo "-> build/test_mirror" ;;
  selftest) build; exec ./build/test_mirror ;;
  render)   render ;;
  *) echo "usage: run.sh build|selftest|render" ;;
esac
