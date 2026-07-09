# QEMU Virtual Atheros AR9285 – Project Makefile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# This Makefile provides convenience targets for the out-of-tree
# development workflow.  The actual compilation is done by QEMU's
# Meson build system; this Makefile orchestrates the integration,
# build, and test steps.
#
# Required:
#   QEMU_SRC  – path to a QEMU source tree (git clone of qemu/qemu)
#
# Typical workflow:
#   make integrate QEMU_SRC=/path/to/qemu
#   make build     QEMU_SRC=/path/to/qemu
#   make test      QEMU_SRC=/path/to/qemu
#
# Or all at once:
#   make all QEMU_SRC=/path/to/qemu

# ---- Configuration ------------------------------------------------

QEMU_SRC       ?=
QEMU_BUILD_DIR  = $(QEMU_SRC)/build
QEMU_BINARY     = $(QEMU_BUILD_DIR)/qemu-system-x86_64
GUEST_IMAGE    ?=
NPROC          := $(shell nproc 2>/dev/null || echo 4)

# Where the built QEMU binary gets installed.  Overridable, e.g.
#   make install QEMU_SRC=/path/to/qemu INSTALL_PREFIX=$HOME/.local
INSTALL_PREFIX ?= /usr/local
INSTALLED_BINARY = $(INSTALL_PREFIX)/bin/qemu-system-x86_64

# ---- Validation ---------------------------------------------------

.PHONY: check-qemu-src
check-qemu-src:
ifndef QEMU_SRC
	$(error QEMU_SRC is not set. Usage: make <target> QEMU_SRC=/path/to/qemu)
endif
	@test -f "$(QEMU_SRC)/meson.build" || \
		{ echo "ERROR: $(QEMU_SRC)/meson.build not found"; exit 1; }

# ---- Targets ------------------------------------------------------

.PHONY: all integrate build test install upgrade clean help

help:
	@echo ""
	@echo "Virtual ath9k – QEMU Device Build System"
	@echo "========================================="
	@echo ""
	@echo "Targets:"
	@echo "  integrate  Copy sources into the QEMU tree and patch build files"
	@echo "  configure  Run QEMU's configure step"
	@echo "  build      Build QEMU with the ath9k device"
	@echo "  test       Run the Phase 1 test suite"
	@echo "  install    Install the built QEMU binary if not already installed"
	@echo "  upgrade    Reinstall the built QEMU binary, overwriting any existing install"
	@echo "  clean      Remove the build directory"
	@echo "  all        integrate + configure + build + test + install"
	@echo ""
	@echo "Required variables:"
	@echo "  QEMU_SRC=/path/to/qemu    Path to a QEMU source tree"
	@echo ""
	@echo "Optional variables:"
	@echo "  GUEST_IMAGE=/path/to.qcow2 Linux guest image for Level 3 tests"
	@echo "  INSTALL_PREFIX=/usr/local  Install location for 'make install'"
	@echo ""

all: integrate configure build test install

integrate: check-qemu-src
	@echo "=== Integrating ath9k sources into QEMU ==="
	./scripts/integrate.sh "$(QEMU_SRC)"

configure: check-qemu-src
	@echo "=== Configuring QEMU build ==="
	mkdir -p "$(QEMU_BUILD_DIR)"
	cd "$(QEMU_BUILD_DIR)" && \
		"$(QEMU_SRC)/configure" \
			--target-list=x86_64-softmmu \
			--prefix="$(INSTALL_PREFIX)" \
			--enable-debug

build: check-qemu-src
	@echo "=== Building QEMU ==="
	@test -d "$(QEMU_BUILD_DIR)" || \
		{ echo "ERROR: Build directory not found. Run 'make configure' first."; exit 1; }
	$(MAKE) -C "$(QEMU_BUILD_DIR)" -j$(NPROC)

test: check-qemu-src
	@echo "=== Running Phase 1 tests ==="
	@test -x "$(QEMU_BINARY)" || \
		{ echo "ERROR: $(QEMU_BINARY) not found. Run 'make build' first."; exit 1; }
ifdef GUEST_IMAGE
	./tests/test_probe.sh "$(QEMU_BINARY)" "$(GUEST_IMAGE)"
else
	./tests/test_probe.sh "$(QEMU_BINARY)"
endif

install: check-qemu-src
	@echo "=== Installing QEMU ==="
	@if [ -x "$(INSTALLED_BINARY)" ]; then \
		echo "   $(INSTALLED_BINARY) already exists – skipping install"; \
	else \
		test -x "$(QEMU_BINARY)" || \
			{ echo "ERROR: $(QEMU_BINARY) not found. Run 'make build' first."; exit 1; }; \
		echo "   Installing into $(INSTALL_PREFIX) ..."; \
		$(MAKE) -C "$(QEMU_BUILD_DIR)" install; \
		echo "   Installed $(INSTALLED_BINARY)"; \
	fi

upgrade: check-qemu-src
	@echo "=== Upgrading QEMU install ==="
	@test -x "$(QEMU_BINARY)" || \
		{ echo "ERROR: $(QEMU_BINARY) not found. Run 'make build' first."; exit 1; }
	@echo "   Reinstalling into $(INSTALL_PREFIX) (overwriting existing) ..."
	$(MAKE) -C "$(QEMU_BUILD_DIR)" install
	@echo "   Installed $(INSTALLED_BINARY)"

clean: check-qemu-src
	@echo "=== Cleaning QEMU build ==="
	@test -d "$(QEMU_BUILD_DIR)" && \
		$(MAKE) -C "$(QEMU_BUILD_DIR)" clean || true

# ---- CCMP crypto unit test (host build, requires OpenSSL) ---------

.PHONY: test-crypto
test-crypto:
	@echo "=== Building and running CCMP crypto unit test ==="
	@command -v cc >/dev/null 2>&1 || { echo "cc not found"; exit 1; }
	cc -O2 -Wall -Wno-deprecated-declarations -Isrc \
		-o tests/test_ccmp tests/test_ccmp.c -lcrypto
	./tests/test_ccmp

.PHONY: test-wep
test-wep:
	@echo "=== Building and running WEP crypto unit test ==="
	@command -v cc >/dev/null 2>&1 || { echo "cc not found"; exit 1; }
	cc -O2 -Wall -Isrc \
		-o tests/test_wep tests/test_wep.c -lz
	./tests/test_wep

.PHONY: test-tkip
test-tkip:
	@echo "=== Building and running TKIP crypto unit test ==="
	@command -v cc >/dev/null 2>&1 || { echo "cc not found"; exit 1; }
	cc -O2 -Wall -Isrc \
		-o tests/test_tkip tests/test_tkip.c
	./tests/test_tkip

# ---- Static analysis (optional, requires cppcheck) ----------------

.PHONY: lint
lint:
	@echo "=== Running static analysis on vwifi-ath9k sources ==="
	@command -v cppcheck >/dev/null 2>&1 || \
		{ echo "cppcheck not found – skipping"; exit 0; }
	cppcheck --enable=all --suppress=missingInclude \
		--suppress=unusedFunction \
		-I src/ src/vwifi_ath9k_pci.c
