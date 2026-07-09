#!/bin/bash
# integrate.sh – Patch the vwifi-ath9k device into a QEMU source tree
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Usage:
#   ./scripts/integrate.sh /path/to/qemu-source
#
# What this script does:
#   1. Copies source files into hw/net/vwifi-ath9k/ inside the QEMU tree
#   2. Adds a subdir('vwifi-ath9k') entry to hw/net/meson.build
#   3. Adds an entry to hw/net/Kconfig
#
# The script is idempotent: running it twice will not duplicate entries.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SRC_DIR="${PROJECT_ROOT}/src"

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

echo "=== Integrating vwifi-ath9k into QEMU tree at ${QEMU_DIR} ==="

# ---- Step 1: Create target directory and copy sources ----
TARGET_DIR="${QEMU_DIR}/hw/net/vwifi-ath9k"
mkdir -p "${TARGET_DIR}"
cp -v "${SRC_DIR}/vwifi_ath9k_pci.c"    "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_ath9k_dma.h"    "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_ath9k_regs.h"   "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_ath9k_eeprom.h" "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_ath9k_crypto.h" "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_ath9k_wep.h"    "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi_ath9k_tkip.h"   "${TARGET_DIR}/"
cp -v "${SRC_DIR}/vwifi.h"              "${TARGET_DIR}/"
cp -v "${SRC_DIR}/meson.build"          "${TARGET_DIR}/"
cp -v "${SRC_DIR}/Kconfig"              "${TARGET_DIR}/"
echo "   Sources copied to ${TARGET_DIR}/"

# ---- Step 2: Patch hw/net/meson.build ----
NET_MESON="${QEMU_DIR}/hw/net/meson.build"
if grep -q "subdir('vwifi-ath9k')" "${NET_MESON}" 2>/dev/null; then
    echo "   hw/net/meson.build already has vwifi-ath9k entry – skipping"
else
    echo "" >> "${NET_MESON}"
    echo "# Virtual Atheros AR9285 (vwifi-ath9k)" >> "${NET_MESON}"
    echo "subdir('vwifi-ath9k')" >> "${NET_MESON}"
    echo "   Patched ${NET_MESON}"
fi

# ---- Step 3: Patch hw/net/Kconfig ----
NET_KCONFIG="${QEMU_DIR}/hw/net/Kconfig"
if grep -q "VWIFI_ATH9K" "${NET_KCONFIG}" 2>/dev/null; then
    echo "   hw/net/Kconfig already has VWIFI_ATH9K entry – skipping"
else
    echo "" >> "${NET_KCONFIG}"
    echo "source vwifi-ath9k/Kconfig" >> "${NET_KCONFIG}"
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
echo "  5. Test with: ./qemu-system-x86_64 -device vwifi-ath9k -nographic"
echo ""
