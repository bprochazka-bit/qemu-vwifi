#!/bin/bash
# integrate.sh – Patch the vwifi-virt device into a QEMU source tree
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Usage:
#   ./scripts/integrate.sh /path/to/qemu-source
#
# What this script does:
#   1. Copies source files into hw/net/vwifi-virt/ inside the QEMU tree
#   2. Copies the shared abi/ headers in alongside them
#   3. Adds a subdir('vwifi-virt') entry to hw/net/meson.build
#   4. Adds an entry to hw/net/Kconfig
#
# The script is idempotent: running it twice will not duplicate entries.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${PROJECT_ROOT}/src"
# Two shared contracts, one copy of each, both at the top of the repo:
#   vwifi_abi.h  device <-> guest driver (shared verbatim with drivers/)
#   vwifi.h      medium wire protocol (shared with the hub and every peer)
ABI_DIR="$(cd "${PROJECT_ROOT}/../.." && pwd)/abi"

if [ $# -ne 1 ]; then
    echo "Usage: $0 /path/to/qemu-source-tree" >&2
    exit 1
fi

QEMU_DIR="$1"

if [ ! -f "${QEMU_DIR}/meson.build" ]; then
    echo "ERROR: ${QEMU_DIR}/meson.build not found." >&2
    echo "       Are you sure this is a QEMU source tree?" >&2
    exit 1
fi

if [ ! -d "${QEMU_DIR}/hw/net" ]; then
    echo "ERROR: ${QEMU_DIR}/hw/net/ not found." >&2
    exit 1
fi

echo "=== Integrating vwifi-virt into QEMU tree at ${QEMU_DIR} ==="

# ---- Step 1: Create target directory and copy sources ----
TARGET_DIR="${QEMU_DIR}/hw/net/vwifi-virt"
mkdir -p "${TARGET_DIR}"
cp -v "${SRC_DIR}/vwifi_qemu.c"    "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_device.c"  "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_device.h"  "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_backend.h" "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_crypto.c"  "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_crypto.h"  "${TARGET_DIR}/"
cp -v "${SRC_DIR}/meson.build"     "${TARGET_DIR}/"
cp -v "${SRC_DIR}/Kconfig"         "${TARGET_DIR}/"

# ---- Step 2: Copy the shared ABI headers ----
cp -v "${ABI_DIR}/vwifi_abi.h"     "${TARGET_DIR}/"
cp -v "${ABI_DIR}/vwifi.h"         "${TARGET_DIR}/"
echo "   Sources copied to ${TARGET_DIR}/"

# ---- Step 3: Patch hw/net/meson.build ----
NET_MESON="${QEMU_DIR}/hw/net/meson.build"
if grep -q "subdir('vwifi-virt')" "${NET_MESON}" 2>/dev/null; then
    echo "   hw/net/meson.build already has vwifi-virt entry – skipping"
else
    echo "" >> "${NET_MESON}"
    echo "# Virtual Wi-Fi adapter for Windows/Linux guests (vwifi-virt)" >> "${NET_MESON}"
    echo "subdir('vwifi-virt')" >> "${NET_MESON}"
    echo "   Patched ${NET_MESON}"
fi

# ---- Step 4: Patch hw/net/Kconfig ----
NET_KCONFIG="${QEMU_DIR}/hw/net/Kconfig"
# Match the line we actually append, not the Kconfig symbol name: the
# appended line names the lowercase path, so a symbol-name grep never
# matches and the entry is duplicated on every run.
if grep -qx "source vwifi-virt/Kconfig" "${NET_KCONFIG}" 2>/dev/null; then
    echo "   hw/net/Kconfig already has VWIFI_VIRT entry – skipping"
else
    echo "" >> "${NET_KCONFIG}"
    echo "source vwifi-virt/Kconfig" >> "${NET_KCONFIG}"
    echo "   Patched ${NET_KCONFIG}"
fi

echo ""
echo "=== Integration complete ==="
echo ""
echo "Next steps:"
echo "  1. cd ${QEMU_DIR}"
echo "  2. mkdir -p build && cd build"
echo "  3. ../configure --target-list=x86_64-softmmu"
echo "  4. make -j\$(nproc)"
echo "  5. Run a guest with:"
echo "       -chardev socket,id=medium,path=/tmp/vwifi.sock,server=off \\"
echo "       -device vwifi-virt,chardev=medium,node_id=win-guest"
echo ""
