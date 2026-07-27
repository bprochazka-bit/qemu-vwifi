#!/bin/sh
# vwifi-virt — device test runner
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Builds and runs every test against the portable device core.
# No QEMU, no libvfio-user, no Windows — just the device logic
# driven through the mock backend.
#
# Usage: ./run_tests.sh

set -e
cd "$(dirname "$0")"

CC=${CC:-cc}
CFLAGS="-Wall -Wextra -O2 -I../src -I../../../abi -I."
SRC="../src/vwifi_device.c ../src/vwifi_crypto.c mock_backend.c"
TESTS="crypto medium_proto smoke_get_caps monitor_mode scan connect wpa2 softap"

mkdir -p build
fail=0

for t in $TESTS; do
    printf '=== %s ===\n' "$t"
    if ! $CC $CFLAGS $SRC "$t.c" -o "build/$t" 2>&1; then
        printf '  BUILD FAILED\n'
        fail=1
        continue
    fi
    if ! "./build/$t"; then
        printf '  RUN FAILED\n'
        fail=1
    fi
done

echo
if [ "$fail" -eq 0 ]; then
    echo "All tests passed."
else
    echo "Some tests FAILED."
fi
exit $fail
