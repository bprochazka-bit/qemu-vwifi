# vwifi-virt – QEMU device Makefile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The device model itself is compiled by QEMU's Meson build; this
# Makefile orchestrates integration into a QEMU tree, and runs the
# portable core's unit tests, which need neither QEMU nor a guest.
#
# Unit tests (no QEMU_SRC needed):
#   make test
#
# Integration workflow:
#   make integrate QEMU_SRC=/path/to/qemu
#   make build     QEMU_SRC=/path/to/qemu

QEMU_SRC       ?=
QEMU_ARCH      ?= x86_64
QEMU_BUILD_DIR  = $(QEMU_SRC)/build
QEMU_BINARY     = $(QEMU_BUILD_DIR)/qemu-system-$(QEMU_ARCH)
NPROC          := $(shell nproc 2>/dev/null || echo 4)
INSTALL_PREFIX ?= /usr/local
INSTALLED_BINARY = $(INSTALL_PREFIX)/bin/qemu-system-$(QEMU_ARCH)

.PHONY: help
help:
	@echo ""
	@echo "vwifi-virt – QEMU device"
	@echo "========================"
	@echo ""
	@echo "Targets:"
	@echo "  test       Run the device unit tests (no QEMU, no guest, no WDK)"
	@echo "  integrate  Copy sources into a QEMU tree and patch its build files"
	@echo "  configure  Run QEMU's configure step"
	@echo "  build      Build QEMU with the device"
	@echo "  install    Install the built QEMU, but not over an existing install"
	@echo "  upgrade    Install, overwriting an existing install"
	@echo "  clean      Remove test binaries"
	@echo ""
	@echo "Required for integrate/configure/build:"
	@echo "  QEMU_SRC=/path/to/qemu"
	@echo ""

.PHONY: check-qemu-src
check-qemu-src:
ifndef QEMU_SRC
	$(error QEMU_SRC is not set. Usage: make <target> QEMU_SRC=/path/to/qemu)
endif
	@test -f "$(QEMU_SRC)/meson.build" || \
		{ echo "ERROR: $(QEMU_SRC)/meson.build not found"; exit 1; }

# ---- Unit tests ---------------------------------------------------
#
# The device logic is deliberately free of QEMU headers and reaches its
# host only through a backend vtable, so the whole state machine — scan,
# connect, WPA2, monitor, SoftAP — runs against a mock backend here.

.PHONY: test
test:
	@echo "=== Running vwifi-virt device tests ==="
	./tests/run_tests.sh

# ---- QEMU integration ---------------------------------------------

.PHONY: integrate configure build install upgrade clean
integrate: check-qemu-src
	@echo "=== Integrating vwifi-virt sources into QEMU ==="
	./scripts/integrate.sh "$(QEMU_SRC)"

configure: check-qemu-src
	@echo "=== Configuring QEMU build ==="
	mkdir -p "$(QEMU_BUILD_DIR)"
	cd "$(QEMU_BUILD_DIR)" && \
		"$(QEMU_SRC)/configure" \
			--target-list=$(QEMU_ARCH)-softmmu \
			--prefix="$(INSTALL_PREFIX)" \
			--enable-debug

build: check-qemu-src
	@echo "=== Building QEMU ==="
	@test -d "$(QEMU_BUILD_DIR)" || \
		{ echo "ERROR: Build directory not found. Run 'make configure' first."; exit 1; }
	$(MAKE) -C "$(QEMU_BUILD_DIR)" -j$(NPROC)

# install refuses to overwrite an existing qemu-system binary; upgrade
# is how you opt in to overwriting one. The default prefix is
# /usr/local, where a distro or an earlier build may already have put a
# QEMU that other things depend on, so clobbering it is never a silent
# side effect of a build.
install: check-qemu-src
	@echo "=== Installing QEMU ==="
	@if [ -x "$(INSTALLED_BINARY)" ]; then \
		echo "   $(INSTALLED_BINARY) already exists - skipping install"; \
		echo "   Run 'make upgrade' to overwrite it."; \
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

clean:
	rm -rf tests/build
