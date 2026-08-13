# pseudohost — virtual hosts on the vwifi medium
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Pure Python, stdlib only. There is nothing to build; this Makefile is
# here for the test target and so the directory matches the rest of the
# tree's `make test` convention.

PYTHON ?= python3

# Install layout. The launchers import the vwifi_pseudohost package by
# adding their own directory to sys.path, so the package and the launcher
# scripts must sit together. We install the whole tree under LIBDIR and
# drop thin wrappers in BINDIR that exec the real launcher — a wrapper is
# used rather than a symlink because the launcher resolves its package dir
# from __file__, which a symlink would point back at BINDIR.
PREFIX  ?= /usr/local
BINDIR  ?= $(PREFIX)/bin
LIBDIR  ?= $(PREFIX)/lib/vwifi-pseudohost
INSTALL ?= install

# name in BINDIR -> launcher in LIBDIR
BIN_HOST := vwifi-pseudohost
BIN_AP   := vwifi-pseudoap

.PHONY: test help install uninstall
help:
	@echo "pseudohost — virtual hosts on the vwifi medium (no VM)"
	@echo
	@echo "  make test        run the full suite (stdlib unittest)"
	@echo "  make install     install $(BIN_HOST) + $(BIN_AP) into $(BINDIR)"
	@echo "  make uninstall   remove them again"
	@echo
	@echo "  join a network as a host:"
	@echo "  ./pseudohost --sock /tmp/vwifi.sock --essid <ESSID> \\"
	@echo "               [--passphrase <psk>] [--profile linux|printer|bambu|...]"
	@echo
	@echo "  beacon a network as an AP:"
	@echo "  ./pseudoap   --sock /tmp/vwifi.sock --essid <ESSID> --channel 6 \\"
	@echo "               [--encryption wpa2 --passphrase <psk>] [--services lpd,nas]"
	@echo
	@echo "  discover a device type's config (for tooling / a UI):"
	@echo "  ./pseudohost --dump-config     # or: ./pseudoap --dump-config"

test:
	$(PYTHON) -m unittest discover -s tests -p 'test_*.py'

install:
	$(INSTALL) -d $(DESTDIR)$(LIBDIR)/vwifi_pseudohost
	# The package (including its profiles/ subpackage) and both launchers.
	# Copy .py sources only; skip __pycache__ so we don't ship stale bytecode.
	cd vwifi_pseudohost && find . -name '*.py' -exec $(INSTALL) -D -m 0644 \
		'{}' '$(DESTDIR)$(LIBDIR)/vwifi_pseudohost/{}' ';'
	$(INSTALL) -m 0755 pseudohost $(DESTDIR)$(LIBDIR)/pseudohost
	$(INSTALL) -m 0755 pseudoap   $(DESTDIR)$(LIBDIR)/pseudoap
	$(INSTALL) -d $(DESTDIR)$(BINDIR)
	# Thin wrappers so the launchers keep finding their package via __file__.
	printf '#!/bin/sh\nexec %s "%s/pseudohost" "$$@"\n' \
		"$(PYTHON)" "$(LIBDIR)" > $(DESTDIR)$(BINDIR)/$(BIN_HOST)
	printf '#!/bin/sh\nexec %s "%s/pseudoap" "$$@"\n' \
		"$(PYTHON)" "$(LIBDIR)" > $(DESTDIR)$(BINDIR)/$(BIN_AP)
	chmod 0755 $(DESTDIR)$(BINDIR)/$(BIN_HOST) $(DESTDIR)$(BINDIR)/$(BIN_AP)

uninstall:
	rm -f $(DESTDIR)$(BINDIR)/$(BIN_HOST) $(DESTDIR)$(BINDIR)/$(BIN_AP)
	rm -rf $(DESTDIR)$(LIBDIR)
