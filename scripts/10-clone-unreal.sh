#!/usr/bin/env bash
# =============================================================================
#  Milestone 2, step 1: fetch Unreal Engine 5.8.2 sources + binary dependencies.
#
#  Runs as YOUR user, not root -- the tree must be writable by you.
#
#  Time: dominated by the GitDependencies download (tens of GB). Run it in a
#  terminal you can leave alone, or under tmux/screen.
#
#  Run:  bash scripts/10-clone-unreal.sh
# =============================================================================
set -uo pipefail

DEST="${DEST:-/WIN_D/protek/UnrealEngine}"
TAG=5.8.2-release

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
no(){ printf '  \033[31m✗\033[0m %s\n' "$*"; }
wa(){ printf '  \033[33m!\033[0m %s\n' "$*"; }

[ "$(id -u)" -ne 0 ] || { no "Do NOT run this as root."; exit 1; }

echo "=== Pre-flight ==="
command -v git-lfs >/dev/null && ok "git-lfs present" || { no "git-lfs missing"; exit 1; }
AVAIL=$(df -BG --output=avail "$(dirname "$DEST")" | tail -1 | tr -dc '0-9')
echo "  free at destination: ${AVAIL} GiB"
[ "${AVAIL:-0}" -ge 220 ] || { no "Need >=220 GiB (source ~80, build artifacts ~120)."; exit 1; }
# A stale SSH_AUTH_SOCK (agent died, variable survived in the environment)
# makes ssh try a nonexistent agent before falling back to key files. Clear it
# rather than fail on it -- the key itself is what matters.
if [ -n "${SSH_AUTH_SOCK:-}" ] && [ ! -S "${SSH_AUTH_SOCK}" ]; then
    wa "SSH_AUTH_SOCK points at a dead agent socket; ignoring it"
    unset SSH_AUTH_SOCK
fi

# Note: ssh to github always exits non-zero (no shell access is granted), so
# the greeting text -- not the exit code -- is the success signal.
SSH_OUT=$(timeout 30 ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new \
              -T git@github.com 2>&1)
if grep -q 'successfully authenticated' <<<"$SSH_OUT"; then
    ok "GitHub SSH auth works ($(sed -n 's/^Hi \([^!]*\)!.*/\1/p' <<<"$SSH_OUT"))"
else
    no "GitHub SSH auth failed:"
    sed 's/^/      /' <<<"$SSH_OUT"
    exit 1
fi

# --- Clone ----------------------------------------------------------------
# --depth 1 on the release tag. UE's full history is enormous and irrelevant
# for building a tagged release; GitDependencies does not need it.
if [ -d "$DEST/.git" ]; then
    ok "Repo already present at $DEST -- skipping clone"
else
    echo
    echo "=== Cloning $TAG (shallow) ==="
    git clone --depth 1 --branch "$TAG" \
        git@github.com:EpicGames/UnrealEngine.git "$DEST" || { no "clone failed"; exit 1; }
    ok "cloned"
fi

cd "$DEST" || exit 1
echo "  HEAD: $(git log -1 --format='%h %d')"

# --- Setup ----------------------------------------------------------------
# Setup.sh does three things:
#   1. On Ubuntu/Debian only, apt-installs build-essential. On Fedora this
#      block is skipped silently -- we already have gcc/make, which is all it
#      would have installed.
#   2. SetupToolchain.sh downloads Epic's bundled clang
#      (v26_clang-20.1.8-rockylinux8) from cdn.unrealengine.com. The host
#      compiler is never used to build the engine.
#   3. GitDependencies pulls tens of GB of prebuilt binary blobs.
echo
echo "=== Setup.sh (bundled toolchain + binary dependencies) ==="
echo "    This is the long one. Toolchain: $(python3 Engine/Build/BatchFiles/Linux/GetLinuxSDKVersion.py 2>/dev/null || echo '?')"
echo
./Setup.sh || { no "Setup.sh failed"; exit 1; }

echo
# NOTE: Engine/Build/, not Build/. Setup.sh sets TOP_DIR=$SCRIPT_DIR/../../..
# which from Engine/Build/BatchFiles/Linux resolves to Engine/, so its final
# `touch Build/OneTimeSetupPerformed` lands inside Engine/.
if [ -f Engine/Build/OneTimeSetupPerformed ]; then
    ok "Setup complete"
    echo "  tree size: $(du -sh "$DEST" 2>/dev/null | cut -f1)"
    echo
    echo "Next:  bash scripts/11-build-unreal.sh"
else
    no "Setup did not create Engine/Build/OneTimeSetupPerformed -- check output above."
    exit 1
fi
