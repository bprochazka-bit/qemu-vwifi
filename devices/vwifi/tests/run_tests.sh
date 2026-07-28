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
TESTS="crypto medium_proto smoke_get_caps sta_mac monitor_mode scan connect wpa2 softap"

mkdir -p build
fail=0

# ---------------------------------------------------------------
# Warning gate — QEMU's flags, applied to the sources QEMU compiles.
#
# The device sources are built twice: here with plain -Wall -Wextra as
# part of each test binary, and inside a QEMU tree with -Werror and a
# much longer warning list. The second set used to be discovered only
# by building QEMU, which made every one of them a slow round trip
# through integrate -> configure -> build. This gate front-runs that:
# same flags, same sources, seconds instead of minutes.
#
# It deliberately covers only ../src/, not the test programs. The tests
# are not compiled into QEMU and are not held to its house style.
#
# Kept in sync by hand with QEMU's build; if QEMU adds a warning that
# fires here, add it to this list rather than working around it.
# ---------------------------------------------------------------
QEMU_WARNS="-Wall -Werror -std=gnu11 \
-Wempty-body -Wendif-labels -Wexpansion-to-defined \
-Wformat-overflow=2 -Wformat-security -Wformat-y2k -Wignored-qualifiers \
-Wimplicit-fallthrough=2 -Winit-self -Wmissing-format-attribute \
-Wmissing-prototypes -Wnested-externs -Wold-style-declaration \
-Wold-style-definition -Wredundant-decls -Wshadow=local -Wstrict-prototypes \
-Wtype-limits -Wundef -Wvla -Wwrite-strings \
-Wno-missing-include-dirs -Wno-psabi -Wno-shift-negative-value"

printf '=== warning gate (QEMU flags) ===\n'
for f in ../src/vwifi_device.c ../src/vwifi_crypto.c; do
    if $CC $QEMU_WARNS -O0 -I../src -I../../../abi -c "$f" \
            -o "build/$(basename "$f").warncheck.o" 2>&1; then
        printf '  %-24s OK\n' "$(basename "$f")"
    else
        printf '  %-24s FAILED\n' "$(basename "$f")"
        printf '  ^ this would also fail inside a QEMU tree.\n'
        fail=1
    fi
done
echo

# vwifi_qemu.c is not checked here: it includes QEMU headers and can
# only be compiled inside a QEMU tree.

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
