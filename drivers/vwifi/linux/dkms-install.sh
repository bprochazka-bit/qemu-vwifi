#!/bin/bash
# Stage the vwifi driver into /usr/src and register it with DKMS.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
#   sudo ./dkms-install.sh          # stage, add, build, install
#   sudo ./dkms-install.sh remove   # unregister and clean /usr/src
#        ./dkms-install.sh stage DIR  # stage only, into DIR (no root)
#
# The "stage" action exists so dkms-selftest.sh can build the exact tree
# DKMS would build without needing dkms or root. Keeping one copy of the
# file list is the point: a staged tree that differs from what the
# selftest checks is a selftest that proves nothing.
#
# Why a script rather than "dkms add ." straight from the repo:
#
# DKMS copies exactly one directory into /usr/src, and the driver's
# sources are not self-contained — they include abi/vwifi_abi.h, which
# lives at the top of the repository and is shared verbatim with the
# QEMU device and the Windows driver. A plain `dkms add` would stage a
# tree that cannot compile.
#
# So this script stages the sources AND drops a copy of the ABI header
# beside them. That copy is a build artifact of packaging, in the same
# way the headers integrate.sh puts in a QEMU tree are: generated, never
# edited, replaced wholesale on the next install. The one authoritative
# copy stays in abi/.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ABI_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)/abi"

# dkms.conf is the single source of truth for the version.
PACKAGE_NAME="$(sed -n 's/^PACKAGE_NAME="\(.*\)"$/\1/p' "${SCRIPT_DIR}/dkms.conf")"
PACKAGE_VERSION="$(sed -n 's/^PACKAGE_VERSION="\(.*\)"$/\1/p' "${SCRIPT_DIR}/dkms.conf")"

if [ -z "${PACKAGE_NAME}" ] || [ -z "${PACKAGE_VERSION}" ]; then
    echo "ERROR: could not read PACKAGE_NAME/PACKAGE_VERSION from dkms.conf" >&2
    exit 1
fi

DEST="/usr/src/${PACKAGE_NAME}-${PACKAGE_VERSION}"
ACTION="${1:-install}"

need_root() {
    if [ "$(id -u)" -ne 0 ]; then
        echo "ERROR: must run as root (try: sudo $0 ${ACTION})" >&2
        exit 1
    fi
}

have_dkms() {
    if ! command -v dkms >/dev/null 2>&1; then
        echo "ERROR: dkms is not installed." >&2
        echo "       Debian/Ubuntu: apt install dkms" >&2
        echo "       Fedora:        dnf install dkms" >&2
        exit 1
    fi
}

# Remove any registration for this name/version, whatever state it is in.
dkms_purge() {
    if dkms status -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}" 2>/dev/null | grep -q .; then
        echo "   removing existing DKMS registration ${PACKAGE_NAME}/${PACKAGE_VERSION}"
        dkms remove -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}" --all || true
    fi
}

case "${ACTION}" in
remove)
    need_root
    have_dkms
    dkms_purge
    rm -rf "${DEST}"
    echo "=== ${PACKAGE_NAME} ${PACKAGE_VERSION} removed ==="
    exit 0
    ;;
stage)
    if [ $# -lt 2 ]; then
        echo "Usage: $0 stage <directory>" >&2
        exit 1
    fi
    DEST="$2"
    ;;
install)
    need_root
    have_dkms
    ;;
*)
    echo "Usage: $0 [install|remove|stage <dir>]" >&2
    exit 1
    ;;
esac

if [ ! -f "${ABI_DIR}/vwifi_abi.h" ] || [ ! -f "${ABI_DIR}/vwifi.h" ]; then
    echo "ERROR: shared ABI headers not found under ${ABI_DIR}." >&2
    echo "       Run this from a checkout of the repository — the driver" >&2
    echo "       cannot be staged without the shared ABI header." >&2
    exit 1
fi

# Catch the version drifting between the package and the module. These
# are two files that must agree and nothing else enforces it.
HDR_VERSION="$(sed -n 's/^#define VWIFI_DRV_VERSION[[:space:]]*"\(.*\)"$/\1/p' \
                "${SCRIPT_DIR}/vwifi_drv.h")"
if [ -n "${HDR_VERSION}" ] && [ "${HDR_VERSION}" != "${PACKAGE_VERSION}" ]; then
    echo "ERROR: version mismatch." >&2
    echo "       dkms.conf:    ${PACKAGE_VERSION}" >&2
    echo "       vwifi_drv.h:  ${HDR_VERSION}" >&2
    echo "       Update both, then re-run." >&2
    exit 1
fi

echo "=== Staging ${PACKAGE_NAME} ${PACKAGE_VERSION} into ${DEST} ==="

if [ "${ACTION}" = "install" ]; then
    dkms_purge
    rm -rf "${DEST}"
fi
mkdir -p "${DEST}"

# Enumerated, not globbed. A *.c glob also picks up vwifi.mod.c if the
# source directory has been built in place — staging that MODPOST
# artifact produces a tree that fails to build, with an error that
# points nowhere near the actual cause.
SOURCES=(
    vwifi_main.c
    vwifi_cfg80211.c
    vwifi_net.c
    vwifi_monitor.c
    vwifi_drv.h
    Makefile
    dkms.conf
)

for f in "${SOURCES[@]}"; do
    if [ ! -f "${SCRIPT_DIR}/${f}" ]; then
        echo "ERROR: missing source file ${f}" >&2
        exit 1
    fi
    install -m 0644 "${SCRIPT_DIR}/${f}" "${DEST}/"
done

[ -f "${SCRIPT_DIR}/README.md" ] && \
    install -m 0644 "${SCRIPT_DIR}/README.md" "${DEST}/"

# The staged copies of the shared contracts. Generated, not source.
#   vwifi_abi.h  device <-> driver
#   vwifi.h      medium wire protocol, for the channel-flag definitions
#                struct vwifi_channel is specified in terms of
install -m 0644 "${ABI_DIR}/vwifi_abi.h" "${DEST}/"
install -m 0644 "${ABI_DIR}/vwifi.h"     "${DEST}/"
cat > "${DEST}/ABI-HEADER-IS-A-COPY" <<EOF
vwifi_abi.h and vwifi.h in this directory are COPIES, staged by
dkms-install.sh from the repository's abi/. They are regenerated on
every install.

Do not edit it here. Edits belong in the repository, where the QEMU
device and the Windows driver compile against the same file. A local
edit here would put this module out of sync with the device it talks
to, which does not fail loudly — it corrupts memory across the
guest/host boundary.
EOF

echo "   staged $(ls -1 "${DEST}" | wc -l) files"

if [ "${ACTION}" = "stage" ]; then
    exit 0
fi

echo "=== dkms add / build / install ==="
dkms add     -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}"
dkms build   -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}"
dkms install -m "${PACKAGE_NAME}" -v "${PACKAGE_VERSION}"

echo ""
echo "=== Done ==="
dkms status -m "${PACKAGE_NAME}"
echo ""
echo "The module rebuilds automatically on kernel upgrade (AUTOINSTALL)."
echo "Load it now with:  modprobe ${PACKAGE_NAME}"
