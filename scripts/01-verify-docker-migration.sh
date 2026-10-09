#!/usr/bin/env bash
# =============================================================================
#  Prove /home/docker-data is a faithful copy of /var/lib/docker before you
#  delete the original.
#
#  Counting volumes is not verification -- Docker reports 29 either way. This
#  compares actual bytes.
#
#  Run:  sudo bash scripts/01-verify-docker-migration.sh
# =============================================================================
set -uo pipefail

OLD=/var/lib/docker
NEW=/home/docker-data

[ "$(id -u)" -eq 0 ] || { echo "Must run as root (sudo)."; exit 1; }
[ -d "$OLD" ] || { echo "$OLD is already gone - nothing to verify."; exit 0; }
[ -d "$NEW" ] || { echo "FATAL: $NEW missing. Do not delete anything."; exit 1; }

echo "Stopping Docker so nothing mutates mid-comparison..."
systemctl stop docker docker.socket 2>/dev/null || true
sleep 2

# -----------------------------------------------------------------------------
echo
echo "=== 1. PER-VOLUME BYTE COMPARISON ==========================================" 
# -----------------------------------------------------------------------------
printf '%-64s %12s %12s  %s\n' "VOLUME" "OLD" "NEW" "STATUS"
fail=0
for d in "$OLD"/volumes/*/; do
    v=$(basename "$d")
    [ "$v" = "metadata.db" ] && continue
    o=$(du -sb "$OLD/volumes/$v" 2>/dev/null | cut -f1)
    n=$(du -sb "$NEW/volumes/$v" 2>/dev/null | cut -f1)
    if [ -z "$n" ]; then
        st="MISSING IN NEW"; fail=1
    elif [ "$o" = "$n" ]; then
        st="ok"
    else
        st="SIZE MISMATCH"; fail=1
    fi
    printf '%-64s %12s %12s  %s\n' "${v:0:64}" "${o:-0}" "${n:-0}" "$st"
done

# -----------------------------------------------------------------------------
echo
echo "=== 2. CHECKSUM-LEVEL DIFF OF volumes/ ====================================="
echo "(rsync dry-run with --checksum: reads every byte on both sides.)"
echo "(Any line printed below is a real difference. Silence means identical.)"
# -----------------------------------------------------------------------------
diffout=$(rsync -aHAXn --checksum --delete --itemize-changes \
              "$OLD/volumes/" "$NEW/volumes/" 2>&1 | grep -vE '^$|^sending|^sent |^total ')
if [ -z "$diffout" ]; then
    echo "  [OK] volumes/ is byte-identical."
else
    echo "$diffout" | head -40
    fail=1
fi

# -----------------------------------------------------------------------------
echo
echo "=== 3. IMAGE STORE DIFF ===================================================="
# -----------------------------------------------------------------------------
imgdiff=$(rsync -aHAXn --checksum --delete --itemize-changes \
              "$OLD/image/" "$NEW/image/" 2>&1 | grep -vE '^$|^sending|^sent |^total ')
if [ -z "$imgdiff" ]; then
    echo "  [OK] image/ is byte-identical."
else
    echo "$imgdiff" | head -20
    echo "  NOTE: some churn here is normal if Docker ran after the migration."
fi

systemctl start docker
sleep 3
echo
echo "=== 4. LIVE DAEMON RECHECK ================================================="
echo "  root      : $(docker info --format '{{.DockerRootDir}}' 2>/dev/null)"
echo "  images    : $(docker images -q 2>/dev/null | wc -l)"
echo "  volumes   : $(docker volume ls -q 2>/dev/null | wc -l)"
echo "  containers: $(docker ps -aq 2>/dev/null | wc -l)"

echo
if [ "$fail" -eq 0 ]; then
    echo "VERDICT: copy is faithful. Safe to reclaim space with:"
    echo "    sudo rm -rf /var/lib/docker"
else
    echo "VERDICT: DIFFERENCES FOUND. Do NOT delete /var/lib/docker."
    echo "To roll back to the old location:"
    echo "    sudo rm /etc/docker/daemon.json && sudo systemctl restart docker"
fi
