#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Grant the 'input' group access to /dev/uinput, so the Milestone 5 virtual
# wheel (wheel/virtual_g923.c) can run without root.
#
# WHY THIS IS NEEDED: /dev/uinput ships root-only (crw------- root root). The
# user is already in the 'input' group -- which is what makes /dev/input/event*
# readable -- but uinput is not covered by that. Without this, the force
# feedback path cannot be exercised at all on a machine with no wheel attached.
#
# This is a host change, so it lives here with the other remediation scripts
# rather than being done silently from a build step. Run it once.
# ---------------------------------------------------------------------------
set -euo pipefail
[[ $EUID -eq 0 ]] || { echo "run with sudo: sudo bash $0"; exit 1; }

RULE=/etc/udev/rules.d/70-uinput-input-group.rules
cat > "$RULE" <<'RULEEOF'
# Milestone 5: uinput for the virtual G923 test device.
# static_node= is required because /dev/uinput exists before the module loads.
KERNEL=="uinput", SUBSYSTEM=="misc", GROUP="input", MODE="0660", OPTIONS+="static_node=uinput"
RULEEOF
echo "wrote $RULE"

echo "uinput" > /etc/modules-load.d/uinput.conf
modprobe uinput
udevadm control --reload-rules
udevadm trigger --sysname-match=uinput || true

ls -l /dev/uinput
echo
echo "Done. Verify as the normal user:  test -w /dev/uinput && echo ok"
