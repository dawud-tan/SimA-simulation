#!/usr/bin/env bash
# =============================================================================
#  Step 0 remediation for the SIM A / Chrono + Unreal simulator host.
#  Target: Fedora 44, Dell OptiPlex 7070, SELinux Enforcing.
#
#  Run with:  sudo bash scripts/00-remediate-host.sh
#
#  SAFETY: This script COPIES the Docker data root; it never deletes the
#  original. You delete /var/lib/docker manually, only after verifying your
#  29 volumes and 4 images survived. Every step is idempotent and re-runnable.
# =============================================================================
set -euo pipefail

REAL_USER="${SUDO_USER:-${USER}}"
REAL_HOME="$(getent passwd "$REAL_USER" | cut -d: -f6)"
NEW_DOCKER_ROOT="/home/docker-data"

c_ok()   { printf '\033[32m  ✓ %s\033[0m\n' "$*"; }
c_warn() { printf '\033[33m  ! %s\033[0m\n' "$*"; }
c_err()  { printf '\033[31m  ✗ %s\033[0m\n' "$*"; }
step()   { printf '\n\033[1;36m[%s] %s\033[0m\n' "$1" "$2"; }

[ "$(id -u)" -eq 0 ] || { c_err "Must run as root (use sudo)."; exit 1; }
c_ok "Acting on behalf of user: $REAL_USER ($REAL_HOME)"

# -----------------------------------------------------------------------------
step 1 "Relocate Docker data root off / (5.9 GiB free) to $NEW_DOCKER_ROOT"
# -----------------------------------------------------------------------------
# KEEP_DOCKER_ROOT=1 skips this: it is for a / short of space, as this
# machine's was, and another machine's / may have plenty.
if [ "${KEEP_DOCKER_ROOT:-0}" = 1 ]; then
    c_warn "KEEP_DOCKER_ROOT=1: Docker's data root left where it is."
elif [ "$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo none)" = "$NEW_DOCKER_ROOT" ]; then
    c_ok "Already migrated. Skipping."
else
    # Record what we must not lose.
    PRE_IMAGES=$(docker images -q 2>/dev/null | wc -l)
    PRE_VOLUMES=$(docker volume ls -q 2>/dev/null | wc -l)
    PRE_CONTAINERS=$(docker ps -aq 2>/dev/null | wc -l)
    c_ok "Pre-migration inventory: ${PRE_IMAGES} images, ${PRE_VOLUMES} volumes, ${PRE_CONTAINERS} containers"

    SRC_SIZE_KB=$(du -sx /var/lib/docker 2>/dev/null | cut -f1)
    DST_FREE_KB=$(df -Pk /home | awk 'NR==2{print $4}')
    printf '    source: %s MiB   /home free: %s MiB\n' "$((SRC_SIZE_KB/1024))" "$((DST_FREE_KB/1024))"
    [ "$DST_FREE_KB" -gt "$((SRC_SIZE_KB * 2))" ] || { c_err "Not enough room on /home."; exit 1; }

    systemctl stop docker docker.socket 2>/dev/null || true
    sleep 2
    c_ok "Docker daemon stopped"

    mkdir -p "$NEW_DOCKER_ROOT"
    # -a archive, -H hardlinks (overlayfs relies on them), -A ACLs, -X xattrs
    # (SELinux labels live in xattrs; losing them breaks the daemon).
    rsync -aHAX --info=progress2 /var/lib/docker/ "$NEW_DOCKER_ROOT/"
    c_ok "Data copied (original left intact at /var/lib/docker)"

    if [ -f /etc/docker/daemon.json ]; then
        cp -a /etc/docker/daemon.json "/etc/docker/daemon.json.bak.$(date +%s)"
        c_warn "Existing daemon.json backed up — merge by hand if it had settings."
    fi
    mkdir -p /etc/docker
    printf '{\n  "data-root": "%s"\n}\n' "$NEW_DOCKER_ROOT" > /etc/docker/daemon.json
    c_ok "Wrote /etc/docker/daemon.json"

    # SELinux is Enforcing. Without this equivalence rule the daemon starts but
    # cannot label container filesystems, and every run fails with permission
    # errors that look nothing like an SELinux problem.
    if command -v semanage >/dev/null 2>&1; then
        semanage fcontext -a -e /var/lib/docker "$NEW_DOCKER_ROOT" 2>/dev/null \
          || c_warn "fcontext equivalence already present"
        restorecon -R "$NEW_DOCKER_ROOT"
        c_ok "SELinux context applied to $NEW_DOCKER_ROOT"
    else
        c_err "semanage missing — install policycoreutils-python-utils and re-run."
        exit 1
    fi

    systemctl start docker
    sleep 4

    POST_ROOT=$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo FAILED)
    POST_IMAGES=$(docker images -q 2>/dev/null | wc -l)
    POST_VOLUMES=$(docker volume ls -q 2>/dev/null | wc -l)
    POST_CONTAINERS=$(docker ps -aq 2>/dev/null | wc -l)
    printf '    now: root=%s  images=%s  volumes=%s  containers=%s\n' \
        "$POST_ROOT" "$POST_IMAGES" "$POST_VOLUMES" "$POST_CONTAINERS"

    if [ "$POST_ROOT" = "$NEW_DOCKER_ROOT" ] \
       && [ "$POST_IMAGES" -eq "$PRE_IMAGES" ] \
       && [ "$POST_VOLUMES" -eq "$PRE_VOLUMES" ] \
       && [ "$POST_CONTAINERS" -eq "$PRE_CONTAINERS" ]; then
        c_ok "Migration verified — every image, volume and container accounted for."
        c_warn "Old data still occupies /var/lib/docker. After you are satisfied, run:"
        printf '        sudo rm -rf /var/lib/docker\n'
    else
        c_err "MISMATCH. Do NOT delete /var/lib/docker. To roll back:"
        printf '        sudo rm /etc/docker/daemon.json && sudo systemctl restart docker\n'
        exit 1
    fi
fi

# -----------------------------------------------------------------------------
step 2 "Install missing toolchain"
# -----------------------------------------------------------------------------
# git-lfs is mandatory: Epic's Setup.sh pulls binary dependencies through it
# and fails in a way that does not mention LFS.
# mold/lld matter because UE link steps are the slowest part of the build on
# a mechanical disk, and GNU ld is the worst of the three at it.
dnf install -y \
    ninja-build ccache mold lld clang git-lfs \
    vulkan-tools libglvnd-devel mesa-libGL-devel sdl2-compat-devel \
    python3-devel kernel-tools xrandr
c_ok "Packages installed"
sudo -u "$REAL_USER" git lfs install >/dev/null 2>&1 && c_ok "git-lfs hooks registered for $REAL_USER"

# -----------------------------------------------------------------------------
step 3 "Enlarge zram and switch to zstd"
# -----------------------------------------------------------------------------
# Current: 8 GiB on lzo-rle. The 30.5 GiB disk swap sits on a 7200 rpm platter;
# if a UE link spills there the build effectively stalls. Bigger, better-
# compressed zram absorbs that spill at RAM speed instead.
cat > /etc/systemd/zram-generator.conf <<'EOF'
[zram0]
zram-size = 16384
compression-algorithm = zstd
swap-priority = 100
EOF
systemctl daemon-reload
if swapoff /dev/zram0 2>/dev/null; then
    systemctl restart systemd-zram-setup@zram0.service && swapon /dev/zram0 2>/dev/null || true
    c_ok "zram reconfigured live"
else
    c_warn "zram0 busy — new size applies after next reboot."
fi
zramctl 2>/dev/null || true

# -----------------------------------------------------------------------------
step 4 "Grant realtime scheduling (needed for the 1 kHz physics thread)"
# -----------------------------------------------------------------------------
groupadd -f realtime
usermod -aG realtime "$REAL_USER"
cat > /etc/security/limits.d/99-realtime.conf <<'EOF'
# Allows SCHED_FIFO for the Chrono physics loop. Kept below 99 so it can
# never outrank kernel threads; sched_rt_runtime_us still caps it at 950ms/s.
@realtime   -   rtprio      95
@realtime   -   memlock     unlimited
EOF
c_ok "rtprio 95 granted to group 'realtime' (member: $REAL_USER)"
c_warn "Takes effect on next LOGIN, not immediately. Verify later with: ulimit -r"

# -----------------------------------------------------------------------------
step 5 "Pin CPU governor to performance"
# -----------------------------------------------------------------------------
# powersave on intel_pstate costs build throughput, and frequency transitions
# are a jitter source for a kilohertz control loop.
if command -v cpupower >/dev/null 2>&1; then
    cpupower frequency-set -g performance >/dev/null 2>&1 \
        && c_ok "Governor set to performance (this boot)" \
        || c_warn "Could not set governor"
    printf '    now: %s\n' "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
    # Persist across reboots.
    cat > /etc/systemd/system/cpu-performance.service <<'EOF'
[Unit]
Description=Pin CPU governor to performance
After=multi-user.target
[Service]
Type=oneshot
ExecStart=/usr/bin/cpupower frequency-set -g performance
[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload && systemctl enable cpu-performance.service >/dev/null 2>&1
    c_ok "Persisted via cpu-performance.service"
fi

# -----------------------------------------------------------------------------
step 6 "Grant evdev access for the G923 (when you plug it in)"
# -----------------------------------------------------------------------------
usermod -aG input "$REAL_USER" 2>/dev/null || true
c_ok "User added to 'input' group — lets the physics process read the wheel without root"

printf '\n\033[1;32m=== REMEDIATION COMPLETE ===\033[0m\n'
printf 'Remaining manual actions:\n'
printf '  1. Log out and back in (activates realtime + input group membership).\n'
printf '  2. Verify Docker volumes, then: sudo rm -rf /var/lib/docker\n'
printf '  3. NOT DONE: the ntfs3 remount of /WIN_D. See notes — it was skipped deliberately.\n\n'
