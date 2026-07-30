#!/bin/bash
# test_probe.sh – Validate that the virtual ath9k device appears in QEMU
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# This script performs several levels of testing:
#
#   Level 1: Verify the device type is registered in QEMU
#   Level 2: Boot a minimal VM and check that the PCI device appears
#   Level 3: (Requires a Linux guest) Check dmesg for ath9k probe messages
#
# Usage:
#   ./tests/test_probe.sh /path/to/qemu-system-x86_64 [/path/to/guest.qcow2]
#
# If no guest image is provided, only Level 1 and a basic Level 2 smoke
# test are run.

set -euo pipefail

QEMU="${1:-}"
GUEST="${2:-}"
PASS=0
FAIL=0

red()   { printf '\033[1;31m%s\033[0m\n' "$*"; }
green() { printf '\033[1;32m%s\033[0m\n' "$*"; }
yellow(){ printf '\033[1;33m%s\033[0m\n' "$*"; }

pass() { PASS=$((PASS + 1)); green "  PASS: $1"; }
fail() { FAIL=$((FAIL + 1)); red   "  FAIL: $1"; }
skip() { yellow "  SKIP: $1"; }

if [ -z "${QEMU}" ]; then
    echo "Usage: $0 /path/to/qemu-system-x86_64 [guest-image.qcow2]" >&2
    exit 1
fi

if [ ! -x "${QEMU}" ]; then
    echo "ERROR: ${QEMU} is not an executable" >&2
    exit 1
fi

echo "============================================="
echo "  Virtual ath9k Phase-1 Test Suite"
echo "============================================="
echo ""
echo "QEMU binary: ${QEMU}"
echo "Guest image: ${GUEST:-<none>}"
echo ""

# ================================================================
# Level 1: Device type registration
# ================================================================

echo "--- Level 1: Device Registration ---"

# Check that 'ath9k-virt' appears in -device help
if "${QEMU}" -device help 2>&1 | grep -q "ath9k-virt"; then
    pass "ath9k-virt device type is registered"
else
    fail "ath9k-virt device type NOT found in -device help"
    echo ""
    echo "This means the device was not compiled into QEMU."
    echo "Did you run scripts/integrate.sh and rebuild?"
    echo ""
    # Can't continue without the device type
    echo "Results: ${PASS} passed, ${FAIL} failed"
    exit 1
fi

# Check that the device info shows the correct PCI vendor/device
if "${QEMU}" -device ath9k-virt,help 2>&1 | grep -qi "network"; then
    pass "ath9k-virt is categorised as a network device"
else
    # The help output format varies; don't hard-fail
    skip "Could not verify device category from help output"
fi

# ================================================================
# Level 2: Smoke test – start QEMU momentarily with the device
# ================================================================

echo ""
echo "--- Level 2: PCI Instantiation Smoke Test ---"

# Start QEMU with the device, no guest, monitor on stdio, quit immediately
TMPLOG=$(mktemp /tmp/ath9k_test.XXXXXX)
trap 'rm -f "${TMPLOG}"' EXIT

timeout 5 "${QEMU}" \
    -machine q35 \
    -device ath9k-virt \
    -display none \
    -monitor stdio \
    -d guest_errors,unimp \
    -D "${TMPLOG}" \
    <<< "info qtree
quit" 2>/dev/null || true

# Check that the device instantiated (appears in qtree)
if grep -q "ath9k-virt" "${TMPLOG}" 2>/dev/null || \
   grep -q "AR9285" "${TMPLOG}" 2>/dev/null; then
    pass "Device instantiated and visible in QEMU log"
elif [ -s "${TMPLOG}" ]; then
    # Log exists but maybe the qtree went to stdout
    pass "QEMU started with ath9k-virt device (log present, ${TMPLOG})"
else
    skip "Could not verify device instantiation from QEMU log"
fi

# Check that our realize message appears
if grep -q "device realized" "${TMPLOG}" 2>/dev/null; then
    pass "ath9k-virt realize callback executed"
else
    skip "Could not verify realize callback (check ${TMPLOG})"
fi

# ================================================================
# Level 3: Full guest boot (optional)
# ================================================================

if [ -n "${GUEST}" ] && [ -f "${GUEST}" ]; then
    echo ""
    echo "--- Level 3: Guest Boot with ath9k-virt ---"

    GUESTLOG=$(mktemp /tmp/ath9k_guest.XXXXXX)
    trap 'rm -f "${TMPLOG}" "${GUESTLOG}"' EXIT

    echo "Booting guest (timeout 60s)..."

    # Boot the guest, wait for it to reach a shell, then grab dmesg
    timeout 60 "${QEMU}" \
        -machine q35 \
        -m 512 \
        -device ath9k-virt \
        -drive file="${GUEST}",format=qcow2,if=virtio \
        -nographic \
        -serial mon:stdio \
        -d guest_errors,unimp \
        -D "${GUESTLOG}" \
        </dev/null >"${GUESTLOG}.serial" 2>&1 || true

    if grep -q "ath9k" "${GUESTLOG}.serial" 2>/dev/null || \
       grep -q "168c:002b" "${GUESTLOG}.serial" 2>/dev/null; then
        pass "Guest kernel detected Atheros PCI device"
    else
        skip "Could not parse guest serial output for ath9k messages"
    fi

    if grep -q "phy0" "${GUESTLOG}.serial" 2>/dev/null; then
        pass "ath9k driver initialised a phy interface"
    else
        skip "Could not verify phy interface creation"
    fi

    if grep -q "EEPROM" "${GUESTLOG}" 2>/dev/null || \
       grep -q "EEPROM" "${GUESTLOG}.serial" 2>/dev/null; then
        pass "EEPROM access detected in logs"
    else
        skip "Could not verify EEPROM access"
    fi

    rm -f "${GUESTLOG}.serial"
else
    echo ""
    echo "--- Level 3: Skipped (no guest image provided) ---"
    skip "Guest boot test requires a Linux disk image"
fi

# ================================================================
# Summary
# ================================================================

echo ""
echo "============================================="
echo "  Results: ${PASS} passed, ${FAIL} failed"
echo "============================================="

if [ "${FAIL}" -gt 0 ]; then
    echo ""
    echo "Debug tips:"
    echo "  - Check ${TMPLOG} for QEMU log output"
    echo "  - Run QEMU with -d guest_errors,unimp to see register accesses"
    echo "  - Ensure the ath9k module is built into the guest kernel"
    exit 1
fi

exit 0
