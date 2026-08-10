#!/usr/bin/env bash
#
# vwifi-pseudohost — run the whole test suite.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# No dependencies beyond a Python 3 interpreter: crypto known-answer
# vectors, frame round-trips, the four-way handshake against an
# in-process authenticator, the netstack/DHCP, and a full end-to-end
# connect against an in-process mock hub+AP.
#
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"
echo "== vwifi-pseudohost tests (using $($PY --version 2>&1)) =="
"$PY" -m unittest discover -s tests -p 'test_*.py' -v
