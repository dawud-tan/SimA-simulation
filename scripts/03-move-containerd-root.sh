#!/usr/bin/env bash
# =============================================================================
#  Move containerd's storage root off / to /home.
#
#  WHY THIS IS NEEDED (and why 00-remediate-host.sh was not enough):
#  Docker 29 uses the containerd snapshotter (`driver-type:
#  io.containerd.snapshotter.v1`). Under that driver, IMAGE LAYERS AND BUILD
#  CACHE live in containerd's content store, not in Docker's data-root.
#  Setting `data-root` in /etc/docker/daemon.json moves volumes, networks and
#  metadata -- but not the bulk of the data. `docker info` still cheerfully
#  reports "Docker Root Dir: /home/docker-data" while gigabytes land on /.
#
#  A single Chrono build produced 10.6 GB of build cache and filled a 36 GB
#  root partition to 100%.
#
#  Run:  sudo bash scripts/03-move-containerd-root.sh
# =============================================================================
set -uo pipefail

# By default the old copy is deleted automatically once verification passes.
# Pass --keep-old to leave it in place (it will keep occupying / until removed).
KEEP_OLD=0
[ "${1:-}" = "--keep-old" ] && KEEP_OLD=1

OLD=/var/lib/containerd
NEW=/home/containerd-data
CFG=/etc/containerd/config.toml

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
no(){ printf '  \033[31m✗\033[0m %s\n' "$*"; }
wa(){ printf '  \033[33m!\033[0m %s\n' "$*"; }

[ "$(id -u)" -eq 0 ] || { echo "Must run as root (sudo)."; exit 1; }

if grep -qE '^\s*root\s*=\s*"'"$NEW"'"' "$CFG" 2>/dev/null; then
    ok "Already migrated (config points at $NEW). Nothing to do."; exit 0
fi

PRE_IMAGES=$(docker images -q 2>/dev/null | wc -l)
PRE_VOLUMES=$(docker volume ls -q 2>/dev/null | wc -l)
echo "Before: ${PRE_IMAGES} images, ${PRE_VOLUMES} volumes"
df -h / | tail -1

# Shrink what we have to copy. Build cache is disposable by definition.
echo
echo "Pruning build cache first (disposable, and makes the copy far smaller)..."
docker builder prune -af >/dev/null 2>&1 && ok "build cache pruned"

SRC_KB=$(du -sx "$OLD" 2>/dev/null | cut -f1)
DST_KB=$(df -Pk /home | awk 'NR==2{print $4}')
printf '  source %s MiB   /home free %s MiB\n' "$((SRC_KB/1024))" "$((DST_KB/1024))"
[ "$DST_KB" -gt "$((SRC_KB * 2))" ] || { no "Not enough room on /home."; exit 1; }

echo
echo "Stopping docker + containerd..."
systemctl stop docker.socket docker 2>/dev/null
systemctl stop containerd 2>/dev/null
sleep 3
pgrep -x containerd >/dev/null && { no "containerd still running"; exit 1; }
ok "stopped"

mkdir -p "$NEW"
# NOTE: / and /home are separate filesystems, so `mv` would not be a rename --
# it would copy then unlink, with identical peak disk usage. Copying first and
# deleting after verification is the same operation with a safety checkpoint.
echo "Copying $OLD -> $NEW ..."
# -H matters: the content store is full of hardlinks. Losing them would both
# balloon the size and corrupt layer references.
rsync -aHAX --info=progress2 "$OLD/" "$NEW/" || { no "rsync failed"; exit 1; }
ok "copied"

cp -a "$CFG" "$CFG.bak.$(date +%s)" && ok "config backed up"
# `root` is a top-level key in every containerd config version. Replace the
# commented default if present, otherwise append.
if grep -qE '^\s*#?\s*root\s*=' "$CFG"; then
    sed -i -E 's|^\s*#?\s*root\s*=.*|root = "'"$NEW"'"|' "$CFG"
else
    printf '\nroot = "%s"\n' "$NEW" >> "$CFG"
fi
grep -E '^root' "$CFG" | sed 's/^/    /'

# SELinux is Enforcing. Without the equivalence rule containerd starts but
# cannot label layers, and every container run fails in confusing ways.
if command -v semanage >/dev/null 2>&1; then
    semanage fcontext -a -e "$OLD" "$NEW" 2>/dev/null || wa "equivalence rule already present"
    restorecon -R "$NEW" && ok "SELinux context applied"
else
    no "semanage missing (policycoreutils-python-utils)"; exit 1
fi

echo
echo "Starting containerd + docker..."
systemctl start containerd && sleep 2
systemctl start docker && sleep 4

POST_IMAGES=$(docker images -q 2>/dev/null | wc -l)
POST_VOLUMES=$(docker volume ls -q 2>/dev/null | wc -l)
echo
echo "After:  ${POST_IMAGES} images, ${POST_VOLUMES} volumes"
docker images 2>/dev/null

echo
if [ "$POST_IMAGES" -eq "$PRE_IMAGES" ] && [ "$POST_VOLUMES" -eq "$PRE_VOLUMES" ]; then
    ok "Migration verified."
    if [ "$KEEP_OLD" -eq 1 ]; then
        wa "--keep-old given; $OLD still occupies /. Remove it with: sudo rm -rf $OLD"
    else
        echo "Reclaiming $OLD ..."
        rm -rf "$OLD" && ok "old copy deleted"
        echo
        df -h / | tail -1
    fi
else
    no "Image/volume count changed. Do NOT delete $OLD."
    echo "Roll back:  sudo cp $CFG.bak.* $CFG && sudo systemctl restart containerd docker"
fi
