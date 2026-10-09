#!/usr/bin/env bash
# =============================================================================
#  Switch /WIN_D from ntfs-3g (FUSE, userspace) to ntfs3 (in-kernel).
#
#  Default mode is CHECK ONLY -- it inspects and reports, changing nothing.
#  Pass --apply to actually remount. On any failure it restores ntfs-3g
#  automatically, so the volume is never left unmounted.
#
#  Run from OUTSIDE /WIN_D:   cd / && sudo bash <path>/02-remount-ntfs3.sh
# =============================================================================
set -uo pipefail

DEV=/dev/sda5
MNT=/WIN_D
UUID=6CC2DA69C2DA3754
APPLY=0
[ "${1:-}" = "--apply" ] && APPLY=1

ok(){ printf '  \033[32m✓\033[0m %s\n' "$*"; }
no(){ printf '  \033[31m✗\033[0m %s\n' "$*"; }
wa(){ printf '  \033[33m!\033[0m %s\n' "$*"; }

[ "$(id -u)" -eq 0 ] || { echo "Must run as root (sudo)."; exit 1; }
case "$PWD" in "$MNT"*) echo "FATAL: cwd is inside $MNT. cd / first."; exit 1;; esac

blockers=0

echo "=== 1. ntfs3 kernel module ==="
if modprobe ntfs3 2>/dev/null && grep -q ntfs3 /proc/filesystems; then
    ok "ntfs3 loaded and registered"
else
    no "ntfs3 unavailable"; blockers=1
fi

echo
echo "=== 2. Processes holding $MNT open ==="
# Anything with an open file, a mapping, or a cwd here will block umount.
holders=$(fuser -m "$MNT" 2>/dev/null | tr ' ' '\n' | grep -E '^[0-9]+$' | sort -u)
if [ -z "$holders" ]; then
    ok "nothing holding the mount"
else
    for p in $holders; do
        wa "pid $p  $(cat /proc/$p/comm 2>/dev/null)  cwd=$(readlink /proc/$p/cwd 2>/dev/null)"
    done
    no "$(echo "$holders" | wc -l) process(es) must be closed before umount"
    blockers=1
fi

echo
echo "=== 3. NTFS volume health ==="
# ntfs3 refuses a read-write mount if the dirty bit is set. Windows Fast
# Startup sets it on EVERY volume at shutdown, which is the usual cause.
flags=$(ntfsinfo -m "$DEV" 2>/dev/null | grep -iE 'volume flags|dirty')
if [ -n "$flags" ]; then
    echo "$flags" | sed 's/^/    /'
    if echo "$flags" | grep -qi dirty; then
        no "volume is DIRTY - ntfs3 will mount read-only"
        wa "fix with: ntfsfix -d $DEV   (ONLY if Windows is fully shut down,"
        wa "          not hibernated, and Fast Startup is disabled)"
        blockers=1
    else
        ok "volume is clean"
    fi
else
    wa "could not read volume flags (ntfsinfo may need the volume unmounted)"
fi

if [ -e /WIN_C/hiberfil.sys ]; then
    sz=$(stat -c%s /WIN_C/hiberfil.sys 2>/dev/null)
    if [ "${sz:-0}" -gt 1048576 ]; then
        no "/WIN_C/hiberfil.sys is $((sz/1024/1024)) MiB - Windows is HIBERNATED"
        wa "Boot Windows, disable Fast Startup, shut down fully, then retry."
        blockers=1
    else
        ok "hiberfil.sys present but empty (Fast Startup appears off)"
    fi
else
    ok "no hiberfil.sys on /WIN_C"
fi

echo
if [ "$APPLY" -eq 0 ]; then
    echo "CHECK-ONLY mode. Blockers: $blockers"
    [ "$blockers" -eq 0 ] && echo "Looks safe. Re-run with --apply to remount." \
                          || echo "Resolve the items above first."
    exit 0
fi

[ "$blockers" -eq 0 ] || { echo "REFUSING to apply with $blockers blocker(s)."; exit 1; }

echo "=== 4. Remounting ==="
cp -a /etc/fstab "/etc/fstab.bak.$(date +%s)" && ok "fstab backed up"

OPTS="rw,uid=1000,gid=1000,dmask=0002,fmask=0002,iocharset=utf8,windows_names"

umount "$MNT" || { no "umount failed"; exit 1; }
ok "unmounted"

if mount -t ntfs3 -o "$OPTS" "$DEV" "$MNT" 2>/tmp/ntfs3.err; then
    ok "mounted with ntfs3"
else
    no "ntfs3 mount failed: $(cat /tmp/ntfs3.err)"
    wa "restoring ntfs-3g..."
    mount -t ntfs-3g -o rw,nls=utf8,dmask=0002,fmask=0002,uid=1000,gid=1000,windows_names \
          "$DEV" "$MNT" && ok "ntfs-3g restored - no harm done"
    exit 1
fi

echo
echo "=== 5. Post-mount verification ==="
T="$MNT/.ntfs3check.$$"
mkdir -p "$T" 2>/dev/null
echo hello > "$T/a.txt" 2>/dev/null && ok "write works" || no "write FAILED"
[ "$(cat "$T/a.txt" 2>/dev/null)" = "hello" ] && ok "read-back works" || no "read FAILED"
touch "$T/aA" 2>/dev/null; [ -e "$T/Aa" ] && no "CASE-INSENSITIVE (bad for UE)" || ok "case-sensitive"
ln -s a.txt "$T/link" 2>/dev/null && ok "symlinks work" || no "symlinks FAILED"
chmod +x "$T/a.txt" 2>/dev/null; [ -x "$T/a.txt" ] && ok "exec bit settable" || wa "exec bit fixed by mask (usually fine)"
rm -rf "$T"

echo
echo "Small-file throughput (2000 creates):"
mkdir -p "$MNT/.iobench"
/usr/bin/time -f "    %e s" bash -c "for i in \$(seq 1 2000); do echo x > $MNT/.iobench/f\$i; done; sync" 2>&1 | tail -1
rm -rf "$MNT/.iobench"

echo
echo "Mount is live but NOT persistent. To make it survive reboot, replace the"
echo "/WIN_D line in /etc/fstab with:"
echo
echo "UUID=$UUID  $MNT  ntfs3  $OPTS,nofail  0 0"
echo
echo "'nofail' matters: if ntfs3 ever refuses the volume, the machine still boots."
echo "Rollback at any time:  sudo umount $MNT && sudo mount $MNT"
