#!/usr/bin/env bash
# Install Digilent Adept Runtime + WaveForms (which provides libdwf) INSIDE the
# container.
#
#   ./scripts/install_waveforms.sh [DIR]     # DIR: see below
#   ./scripts/install_waveforms.sh --no-verify [DIR]
#
# Without DIR, the verified versions below are downloaded into vendor/digilent
# (git-ignored) from this repository's GitHub Release, which mirrors Digilent's
# unmodified packages (their license allows that; see vendor/digilent/LICENSE).
# files.digilent.com itself is behind a Cloudflare challenge that curl/wget
# cannot pass. Files already in vendor/digilent are reused.
# With DIR, the .debs in DIR are used and nothing is downloaded; get other
# versions in a browser from
#   https://digilent.com/reference/software/adept/runtime
#   https://digilent.com/reference/software/waveforms/waveforms-3/start
# (Linux, Ubuntu x86_64 .deb).
# Re-run after recreating the container; it does nothing when already installed.
#
# Known packages are checked against the sha256 values published in the Arch
# AUR PKGBUILDs (digilent.adept.runtime 2.30.1, digilent.waveforms 3.25.1,
# looked up 2026-10-22). Other versions need --no-verify.
set -euo pipefail

declare -A KNOWN_SHA256=(
  [digilent.adept.runtime_2.30.1_amd64.deb]=e5e51d2640c2ff34ef3b436f3bdf37838120b15160d73dfbab82e90773b6b372
  [digilent.waveforms_3.25.1_amd64.deb]=d2979aab726c9202a48a1c5d2b314531513171c0b62fa2f2a2edcd29202727d3
)
RELEASE_URL="https://github.com/YusakuNakajima/analog_discovery_ros2/releases/download/digilent-debs-adept2.30.1-waveforms3.25.1"

verify=1
if [[ "${1:-}" == "--no-verify" ]]; then
  verify=0
  shift
fi
if [[ ! -f /.dockerenv ]]; then
  echo "Run this inside the OnoLab ROS 2 container (the host only needs udev/setup_udev.sh)." >&2
  exit 1
fi

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${1:-}" ]]; then
  DIR="$1"
else
  DIR="${PKG_DIR}/vendor/digilent"
  for name in "${!KNOWN_SHA256[@]}"; do
    if [[ -f "${DIR}/${name}" ]] &&
       [[ "$(sha256sum "${DIR}/${name}" | cut -d' ' -f1)" == "${KNOWN_SHA256[${name}]}" ]]; then
      continue
    fi
    echo "Downloading ${name} from the GitHub Release"
    # Download to a temporary name so an interrupted download never looks complete.
    curl --fail --location --silent --show-error --retry 3 \
      --output "${DIR}/${name}.part" "${RELEASE_URL}/${name}"
    mv "${DIR}/${name}.part" "${DIR}/${name}"
  done
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

installed() {  # fully installed ("ii") at the version of the given .deb
  [[ "$(dpkg-query -W -f='${db:Status-Abbrev} ${Version}' "$1" 2>/dev/null)" == \
     "ii  $(dpkg-deb -f "$2" Version)" ]]
}
if installed digilent.adept.runtime "${adept[0]}" && installed digilent.waveforms "${waveforms[0]}"; then
  echo "Adept Runtime and WaveForms are already installed at these versions"
  python3 -c "import ctypes; ctypes.cdll.LoadLibrary('libdwf.so'); print('libdwf.so loads')"
  exit 0
fi

# Adept's postinst runs "udevadm control --reload-rules" and "udevadm trigger".
# There is no udevd in the container, so the first fails and aborts the install;
# and in a privileged container "udevadm trigger" would write to the HOST's
# /sys and re-trigger every device on the host. Both are skipped with a no-op
# udevadm on PATH: the rule that matters is installed on the host by
# udev/setup_udev.sh.
# WaveForms' postinst registers desktop menu entries and icons with xdg-*,
# which fails in the container (no menu directories) and is only GUI
# integration; libdwf does not need it. Those are no-ops too.
SHIM_DIR="$(mktemp -d)"
trap 'rm -rf "${SHIM_DIR}"' EXIT
for cmd in udevadm xdg-mime xdg-desktop-menu xdg-icon-resource; do
  printf '#!/bin/sh\necho "install_waveforms.sh: skipped %s $* (not needed in the container)"\n' \
    "${cmd}" > "${SHIM_DIR}/${cmd}"
  chmod +x "${SHIM_DIR}/${cmd}"
done
# apt-get replaces PATH for dpkg with DPkg::Path, so the shims must be put there
# too; otherwise the postinst finds no udevadm (exit 127) or, where one exists,
# runs the real one.
SHIM_PATH="${SHIM_DIR}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
as_root() {
  sudo env PATH="${SHIM_PATH}" DEBIAN_FRONTEND=noninteractive "$@"
}
apt_install() {
  as_root apt-get -o "DPkg::Path=${SHIM_PATH}" install -y "$@"
}

# Finish a previous attempt that stopped in the postinst, then Adept first
# (WaveForms depends on it). apt resolves the remaining dependencies
# (libusb, avahi, Qt, xdg-utils for the postinst).
as_root dpkg --configure -a
as_root apt-get update
apt_install "${adept[0]}"
apt_install "${waveforms[0]}"
python3 -c "import ctypes; ctypes.cdll.LoadLibrary('libdwf.so'); print('libdwf.so loads')"
