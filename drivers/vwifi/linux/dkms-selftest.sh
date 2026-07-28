#!/bin/bash
# Build the vwifi driver the way DKMS builds it, without dkms or root.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
#   ./dkms-selftest.sh                 # against the running kernel
#   KDIR=/path/to/tree ./dkms-selftest.sh
#
# Why this exists: `make` in the source directory and `dkms build` are
# not the same build. DKMS copies a subset of the tree into a staging
# directory and runs
#
#     make -jN KERNELRELEASE=<version> KDIR=<tree>
#
# from there -- a different working directory, a different file set, and
# KERNELRELEASE set on the command line. Every one of those has broken
# the DKMS path while a plain `make` stayed green: a missing staged
# header, and a Makefile whose kbuild guard triggered on KERNELRELEASE
# and left make with no targets at all.
#
# So this reproduces that invocation exactly, using dkms-install.sh's own
# staging so there is one file list rather than two.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KDIR="${KDIR:-/lib/modules/$(uname -r)/build}"
KVER="${KVER:-$(uname -r)}"

if [ ! -d "${KDIR}" ]; then
    echo "ERROR: no kernel build tree at ${KDIR}" >&2
    echo "       Install the headers for ${KVER}, or set KDIR=/path/to/tree" >&2
    exit 1
fi

STAGE="$(mktemp -d)"
trap 'rm -rf "${STAGE}"' EXIT

echo "=== staging as DKMS would ==="
"${SCRIPT_DIR}/dkms-install.sh" stage "${STAGE}"

echo "=== building as DKMS would: KERNELRELEASE=${KVER} KDIR=${KDIR} ==="
# -C, not cd: DKMS runs make with the staged directory as its working
# directory, so $(CURDIR) must be the staged tree and not this one. If it
# were this one, the build would quietly pick up the repository's abi/
# headers and prove nothing about the staged copies.
make -C "${STAGE}" -j"$(nproc 2>/dev/null || echo 2)" \
     KERNELRELEASE="${KVER}" KDIR="${KDIR}"

if [ ! -f "${STAGE}/vwifi.ko" ]; then
    echo "ERROR: build reported success but produced no vwifi.ko" >&2
    exit 1
fi

echo ""
echo "=== OK: staged tree builds, vwifi.ko produced ==="
