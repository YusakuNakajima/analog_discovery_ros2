#!/usr/bin/env bash
# One-step setup of analog_discovery_ros2 INSIDE the OnoLab ROS 2 container:
#   1. install Adept Runtime + WaveForms from the hand-downloaded .debs
#   2. build the package
#   3. check that the host udev rule lets this user open the AD3
#   4. list the attached devices (ad_probe.py; nothing is opened)
#
#   ./setup.sh               # .debs in this directory or ~/onolab_ros2/depends/digilent
#   ./setup.sh --no-verify   # accept .deb versions whose sha256 is not known
#
# The host part (udev rule) is separate: run ./udev/setup_udev.sh on the host.
set -euo pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../../.." && pwd)"  # ros2_ws
if [[ ! -f "${WS_DIR}/src/third_party.repos" ]]; then
  echo "Expected this package at ros2_ws/src/third_party/analog_discovery_ros2 (got ${PKG_DIR})" >&2
  exit 1
fi

echo "=== 1/4 Adept Runtime + WaveForms"
"${PKG_DIR}/scripts/install_waveforms.sh" "$@"

echo "=== 2/4 build"
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
(cd "${WS_DIR}" && colcon build --symlink-install --packages-select analog_discovery_ros2)

echo "=== 3/4 USB access"
nodes=0
blocked=0
for dev in /sys/bus/usb/devices/*; do
  [[ -f "${dev}/idVendor" && "$(cat "${dev}/idVendor")" == "1443" ]] || continue
  node="$(printf '/dev/bus/usb/%03d/%03d' "$(cat "${dev}/busnum")" "$(cat "${dev}/devnum")")"
  nodes=$((nodes + 1))
  if [[ -r "${node}" && -w "${node}" ]]; then
    echo "OK   ${node} $(cat "${dev}/idProduct") serial $(cat "${dev}/serial" 2>/dev/null || echo '?')"
  else
    echo "FAIL ${node} ($(stat -c '%A %U:%G' "${node}")) is not writable by $(id -un)"
    blocked=$((blocked + 1))
  fi
done
if [[ ${nodes} -eq 0 ]]; then
  echo "No Digilent device (VID 1443) attached; plug in the AD3 and re-run."
elif [[ ${blocked} -gt 0 ]]; then
  echo "On the HOST run: ${PKG_DIR#"${HOME}"/}/udev/setup_udev.sh  (path relative to the repository)"
  exit 1
fi

echo "=== 4/4 devices"
set +u
source "${WS_DIR}/install/setup.bash"
set -u
ros2 run analog_discovery_ros2 ad_probe.py
echo "Setup done. Next: ros2 run analog_discovery_ros2 ad_selftest.py"
