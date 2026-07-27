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
QEMU_BUILD_DIR  = $(QEMU_SRC)/build
QEMU_BINARY     = $(QEMU_BUILD_DIR)/qemu-system-x86_64
NPROC          := $(shell nproc 2>/dev/null || echo 4)
INSTALL_PREFIX ?= /usr/local

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

.PHONY: integrate configure build clean
integrate: check-qemu-src
	@echo "=== Integrating vwifi-virt sources into QEMU ==="
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

clean:
	rm -rf tests/build
