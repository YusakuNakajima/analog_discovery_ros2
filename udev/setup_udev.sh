#!/usr/bin/env bash
# Install (or remove) the Analog Discovery udev rule on the HOST, then check that
# an attached unit became accessible without root.
#
#   ./udev/setup_udev.sh             # install / update the rule
#   ./udev/setup_udev.sh --uninstall # remove it
#
# Run it as a normal user on the host PC (not inside the container); it asks for
# sudo only for the steps that need it. The container sees the result through
# its /dev mount.
set -euo pipefail

RULE_NAME="99-analog-discovery.rules"
RULE_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/${RULE_NAME}"
RULE_DST="/etc/udev/rules.d/${RULE_NAME}"
VENDOR_ID="1443"

if [[ -f /.dockerenv ]]; then
  echo "This looks like a container. Run this script on the host PC instead." >&2
  exit 1
fi

reload_udev() {
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor="${VENDOR_ID}"
  sudo udevadm settle --timeout=5 || true
}

if [[ "${1:-}" == "--uninstall" ]]; then
  if [[ -e "${RULE_DST}" ]]; then
    sudo rm -f "${RULE_DST}"
    reload_udev
    echo "Removed ${RULE_DST}"
  else
    echo "${RULE_DST} is not installed"
  fi
  exit 0
elif [[ $# -gt 0 ]]; then
  echo "usage: $0 [--uninstall]" >&2
  exit 2
fi

if [[ -f "${RULE_DST}" ]] && cmp -s "${RULE_SRC}" "${RULE_DST}"; then
  echo "${RULE_DST} is already up to date"
else
  sudo install -m 0644 "${RULE_SRC}" "${RULE_DST}"
  echo "Installed ${RULE_DST}"
fi
reload_udev

# Check every attached unit: the rule makes its device node 0666.
found=0
failed=0
for dev in /sys/bus/usb/devices/*; do
  [[ -f "${dev}/idVendor" && -f "${dev}/idProduct" ]] || continue
  [[ "$(cat "${dev}/idVendor")" == "${VENDOR_ID}" ]] || continue
  node="$(printf '/dev/bus/usb/%03d/%03d' "$(cat "${dev}/busnum")" "$(cat "${dev}/devnum")")"
  found=$((found + 1))
  if [[ -r "${node}" && -w "${node}" ]]; then
    echo "OK   ${node} ($(stat -c '%A %U:%G' "${node}"))"
  else
    echo "FAIL ${node} ($(stat -c '%A %U:%G' "${node}")) is not accessible" >&2
    failed=$((failed + 1))
  fi
done

if [[ ${found} -eq 0 ]]; then
  echo "No Digilent device (VID ${VENDOR_ID}) attached; the rule applies when one is plugged in."
elif [[ ${failed} -gt 0 ]]; then
  echo "Unplug and replug the unit, then run this script again." >&2
  exit 1
fi
