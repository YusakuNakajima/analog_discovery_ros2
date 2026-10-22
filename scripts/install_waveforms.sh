#!/usr/bin/env bash
# Install Digilent Adept Runtime + WaveForms (which provides libdwf) INSIDE the
# container, from .deb files downloaded by hand.
#
#   ./scripts/install_waveforms.sh [DIR]     # DIR defaults to ~/onolab_ros2/depends/digilent
#   ./scripts/install_waveforms.sh --no-verify [DIR]
#
# files.digilent.com is behind a Cloudflare challenge, so curl/wget cannot fetch
# the packages: download both in a browser from
#   https://digilent.com/reference/software/adept/runtime
#   https://digilent.com/reference/software/waveforms/waveforms-3/start
# (Linux, Ubuntu x86_64 .deb) into DIR. The container keeps them after a
# reinstall; re-run this script after recreating the container.
#
# Known packages are checked against the sha256 values published in the Arch
# AUR PKGBUILDs (digilent.adept.runtime 2.30.1, digilent.waveforms 3.25.1,
# looked up 2026-10-22). Other versions need --no-verify.
set -euo pipefail

declare -A KNOWN_SHA256=(
  [digilent.adept.runtime_2.30.1_amd64.deb]=e5e51d2640c2ff34ef3b436f3bdf37838120b15160d73dfbab82e90773b6b372
  [digilent.waveforms_3.25.1_amd64.deb]=d2979aab726c9202a48a1c5d2b314531513171c0b62fa2f2a2edcd29202727d3
)

verify=1
if [[ "${1:-}" == "--no-verify" ]]; then
  verify=0
  shift
fi
DIR="${1:-${HOME}/onolab_ros2/depends/digilent}"

if [[ ! -f /.dockerenv ]]; then
  echo "Run this inside the OnoLab ROS 2 container (the host only needs udev/setup_udev.sh)." >&2
  exit 1
fi

shopt -s nullglob
adept=("${DIR}"/digilent.adept.runtime_*.deb)
waveforms=("${DIR}"/digilent.waveforms_*.deb)
if [[ ${#adept[@]} -ne 1 || ${#waveforms[@]} -ne 1 ]]; then
  echo "Need exactly one digilent.adept.runtime_*.deb and one digilent.waveforms_*.deb in ${DIR}" >&2
  echo "found: ${adept[*]:-none} / ${waveforms[*]:-none}" >&2
  exit 1
fi

for deb in "${adept[0]}" "${waveforms[0]}"; do
  name="$(basename "${deb}")"
  actual="$(sha256sum "${deb}" | cut -d' ' -f1)"
  expected="${KNOWN_SHA256[${name}]:-}"
  if [[ -n "${expected}" && "${actual}" == "${expected}" ]]; then
    echo "OK   ${name} sha256 matches"
  elif [[ -n "${expected}" ]]; then
    echo "FAIL ${name} sha256 ${actual} != ${expected}" >&2
    exit 1
  elif [[ ${verify} -eq 1 ]]; then
    echo "${name} is not a known version (sha256 ${actual}); re-run with --no-verify to install it" >&2
    exit 1
  else
    echo "WARN ${name} not verified (sha256 ${actual})"
  fi
done

# Adept first: WaveForms depends on it. apt resolves the remaining dependencies
# (libusb, avahi, Qt, xdg-utils for the postinst).
sudo apt-get update
sudo apt-get install -y "${adept[0]}"
sudo apt-get install -y "${waveforms[0]}"
python3 -c "import ctypes; ctypes.cdll.LoadLibrary('libdwf.so'); print('libdwf.so loads')"
