#!/usr/bin/env bash
# =============================================================================
#  Milestone 2, step 2: compile UnrealEditor (Development) from source.
#
#  Measured: 3.4 hours on this machine (12,164 s, 2026-09-02: 8 cores, no SMT,
#  16 GiB, mechanical disk). Run under tmux/screen so a closed terminal does
#  not kill it.
#
#  Run:  bash scripts/11-build-unreal.sh
# =============================================================================
set -uo pipefail

DEST="${DEST:-/WIN_D/protek/UnrealEngine}"
TARGET="${TARGET:-UnrealEditor}"
CONFIG="${CONFIG:-Development}"

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
no(){ printf '  \033[31m✗\033[0m %s\n' "$*"; }
wa(){ printf '  \033[33m!\033[0m %s\n' "$*"; }

[ "$(id -u)" -ne 0 ] || { no "Do NOT run as root."; exit 1; }
cd "$DEST" 2>/dev/null || { no "$DEST not found. Run 10-clone-unreal.sh first."; exit 1; }
# Engine/Build/, not Build/ -- see note in 10-clone-unreal.sh.
[ -f Engine/Build/OneTimeSetupPerformed ] || { no "Setup.sh has not completed. Run 10-clone-unreal.sh."; exit 1; }

echo "=== Pre-flight ==="
AVAIL=$(df -BG --output=avail "$DEST" | tail -1 | tr -dc '0-9')
echo "  free space   : ${AVAIL} GiB"
[ "${AVAIL:-0}" -ge 120 ] || { no "Need >=120 GiB for build artifacts."; exit 1; }

# UBT throttles by available RAM, not core count. Without this config, 8
# concurrent link actions against 16 GiB invoke the OOM killer hours into the
# build. The one this engine was built with is in the repo; it is installed
# when you have none, and never over one you already have.
UBTCFG="$HOME/.config/Unreal Engine/UnrealBuildTool/BuildConfiguration.xml"
REPOCFG="$(cd "$(dirname "$0")" && pwd)/BuildConfiguration.xml"
if [ -f "$UBTCFG" ]; then
    ok "UBT config present (MaxProcessorCount + MemoryPerActionBytes)"
elif mkdir -p "$(dirname "$UBTCFG")" && cp "$REPOCFG" "$UBTCFG"; then
    ok "UBT config installed from the repo: 6 processes, 3 GiB per action"
else
    wa "UBT config MISSING at $UBTCFG -- risk of OOM during link"
fi

free -h | head -2 | sed 's/^/  /'
echo "  governor     : $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"

# --- Generate project files ------------------------------------------------
echo
echo "=== GenerateProjectFiles.sh ==="
./GenerateProjectFiles.sh || { no "GenerateProjectFiles failed"; exit 1; }
ok "project files generated"

# --- Build -----------------------------------------------------------------
# Linux target name is the platform triple; Build.sh takes
# <Target> <Platform> <Configuration>.
LOG="$DEST/build-${TARGET}-$(date +%Y%m%d-%H%M%S).log"
echo
echo "=== Building $TARGET $CONFIG (Linux) ==="
echo "  log: $LOG"
echo "  started: $(date)"
echo "  This is the 5-9 hour step. Safe to leave unattended."
echo

START=$(date +%s)
# tee into a log so a failure hours in is diagnosable without scrollback.
./Engine/Build/BatchFiles/Linux/Build.sh "$TARGET" Linux "$CONFIG" 2>&1 | tee "$LOG"
RC=${PIPESTATUS[0]}
END=$(date +%s)

echo
printf '  elapsed: %02dh %02dm\n' $(( (END-START)/3600 )) $(( ((END-START)%3600)/60 ))

if [ "$RC" -ne 0 ]; then
    no "Build FAILED (rc=$RC). Last errors:"
    grep -iE 'error|fatal|Killed|out of memory' "$LOG" | tail -20
    echo
    wa "If you see 'Killed' or OOM: lower MaxProcessorCount to 4 in"
    wa "  $UBTCFG"
    wa "then re-run -- UBT resumes from completed actions."
    exit 1
fi

ok "Build succeeded"
BIN="$DEST/Engine/Binaries/Linux/UnrealEditor"
if [ -x "$BIN" ]; then
    ok "Editor binary: $BIN"
    ls -lh "$BIN" | sed 's/^/    /'
    echo
    echo "Launch with:  $BIN"
else
    wa "Expected binary not found at $BIN"
fi
echo "  tree size: $(du -sh "$DEST" 2>/dev/null | cut -f1)"
