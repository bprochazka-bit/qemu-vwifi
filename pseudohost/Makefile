# pseudohost — virtual hosts on the vwifi medium
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Pure Python, stdlib only. There is nothing to build; this Makefile is
# here for the test target and so the directory matches the rest of the
# tree's `make test` convention.

PYTHON ?= python3

.PHONY: test help
help:
	@echo "pseudohost — virtual hosts on the vwifi medium (no VM)"
	@echo
	@echo "  make test        run the full suite (stdlib unittest)"
	@echo
	@echo "  ./pseudohost --sock /tmp/vwifi.sock --essid <ESSID> \\"
	@echo "               [--passphrase <psk>] [--profile linux|windows|printer|nas]"

test:
	$(PYTHON) -m unittest discover -s tests -p 'test_*.py'
