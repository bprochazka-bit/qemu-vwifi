# Makefile for qemu-vwifi — top level
#
# The repository holds four kinds of thing (see README.md):
#   abi/      shared contracts, header-only
#   medium/   the hub and the tools that attach to it  -> userspace binaries
#   host/     the host's own mac80211 radio            -> kernel module + tools
#   devices/  QEMU device models                       -> built inside a QEMU tree
#   drivers/  guest drivers                            -> built with each OS's SDK
#
# This Makefile builds everything a plain host toolchain can build, and
# drives a QEMU source tree to build the device models. The guest drivers
# need their own SDKs and are not built from here.
#
# Host-side targets:
#   make                     — userspace binaries (no kernel headers needed)
#   make driver              — the vwifi-virt Linux guest driver
#   sudo make dkms           — ... installed via DKMS (survives kernel upgrades)
#   sudo make dkms-remove    — ... unregistered again
#   make dkms-check          — ... built the way DKMS builds it, no root needed
#   make module              — host kernel module against the running kernel
#   make module KDIR=/path   — ... against a specific kernel source tree
#   make install             — install the kernel module (requires root)
#   make install SKIP_SIGN=1 — install without the module-signing step, for
#                              distros that ship no signing key
#   make install-userspace   — install userspace binaries (honors PREFIX, DESTDIR)
#   make uninstall-userspace — remove them again
#   make test                — every test suite that runs on the host
#   make legacy              — build the superseded ath9k_medium hubs
#   make clean               — remove host-side build artifacts
#
# QEMU targets (all need QEMU_SRC=/path/to/qemu):
#   make qemu                — integrate + configure + build, both devices
#   make qemu DEVICES=ath9k  — ... with only vwifi-ath9k
#   make qemu DEVICES=vwifi  — ... with only vwifi-virt
#   make qemu-install        — install the built QEMU, but NOT over an
#                              existing install (see "install vs upgrade")
#   make qemu-upgrade        — install, overwriting an existing install
#   make qemu-test           — run the ath9k probe tests against the build
#   make qemu-reconfigure    — force a fresh configure
#   make qemu-clean          — clean the QEMU build directory
#   make qemu-help           — the same list, with the variables
#
# See "QEMU device builds" below for the full variable list.

TOPDIR := $(CURDIR)
BUILD  ?= $(TOPDIR)/build
ABI    := $(TOPDIR)/abi

CC       ?= gcc
CFLAGS   ?= -Wall -Wextra -O2
CPPFLAGS += -I$(ABI)

# ---------- Userspace ----------

MEDIUM_BINS := \
	$(BUILD)/vwifi-medium \
	$(BUILD)/vwifi-phys-bridge \
	$(BUILD)/vwifi-linkbench

HOST_BINS := \
	$(BUILD)/vwifi-host-relay \
	$(BUILD)/vwifi-ctl

USERSPACE_BINS := $(MEDIUM_BINS) $(HOST_BINS)

.PHONY: all userspace medium host-tools
all: userspace
userspace: $(USERSPACE_BINS)
medium: $(MEDIUM_BINS)
host-tools: $(HOST_BINS)

$(BUILD):
	@mkdir -p $(BUILD)

# The hub is the only binary that needs libm (the propagation model).
$(BUILD)/vwifi-medium: medium/src/vwifi_medium.c $(ABI)/vwifi.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $< -lm

$(BUILD)/vwifi-phys-bridge: medium/tools/vwifi_phys_bridge.c $(ABI)/vwifi.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $<

$(BUILD)/vwifi-linkbench: medium/tools/vwifi_linkbench.c | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $<

$(BUILD)/vwifi-host-relay: host/tools/vwifi_host_relay.c $(ABI)/vwifi.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $<

$(BUILD)/vwifi-ctl: host/tools/vwifi_ctl.c $(ABI)/vwifi.h $(ABI)/vwifi_host_ioctl.h | $(BUILD)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $<

# ---------- Legacy medium hubs ----------
# Superseded by vwifi-medium; see medium/legacy/README.md.

LEGACY_BINS := $(BUILD)/ath9k_medium_hub $(BUILD)/ath9k_medium_hub_scalable

.PHONY: legacy
legacy: $(LEGACY_BINS)

$(BUILD)/ath9k_medium_hub: medium/legacy/ath9k_medium_hub.c | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $<

$(BUILD)/ath9k_medium_hub_scalable: medium/legacy/ath9k_medium_hub_scalable.c | $(BUILD)
	$(CC) $(CFLAGS) -o $@ $<

# ---------- Guest driver (vwifi-virt) ----------
#
# The Linux driver for the vwifi-virt device. It builds like any
# out-of-tree module, but DKMS is what you want in a guest whose kernel
# gets upgraded -- a plain module is built for one kernel version and
# stops loading after the next upgrade.

.PHONY: driver dkms dkms-remove dkms-check
driver:
	$(MAKE) -C drivers/vwifi/linux KDIR=$(KDIR)

dkms:
	$(MAKE) -C drivers/vwifi/linux dkms

dkms-remove:
	$(MAKE) -C drivers/vwifi/linux dkms-remove

# Build the staged tree the way DKMS does, without dkms or root. A
# plain "make driver" does not exercise that path and has twice stayed
# green while "make dkms" was broken.
dkms-check:
	$(MAKE) -C drivers/vwifi/linux dkms-check

# ---------- Host kernel module ----------

KDIR ?= /lib/modules/$(shell uname -r)/build

.PHONY: module install
module:
	$(MAKE) -C host KDIR=$(KDIR)

install:
	$(MAKE) -C host install KDIR=$(KDIR) SKIP_SIGN=$(SKIP_SIGN)

# ---------- Install ----------

PREFIX  ?= /usr/local
BINDIR  ?= $(PREFIX)/bin
INSTALL ?= install

.PHONY: install-userspace uninstall-userspace
install-userspace: $(USERSPACE_BINS)
	$(INSTALL) -d $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0755 $(USERSPACE_BINS) $(DESTDIR)$(BINDIR)

uninstall-userspace:
	rm -f $(addprefix $(DESTDIR)$(BINDIR)/,$(notdir $(USERSPACE_BINS)))

# ---------- QEMU device builds ----------
#
# Both devices go into ONE QEMU tree and ONE build directory: a single
# qemu-system binary that carries whichever devices you asked for. They
# do not conflict — each lands in its own hw/net/<device>/ subdirectory
# with its own Kconfig symbol, and both can be present at once. Pick per
# VM at runtime with -device vwifi-ath9k or -device vwifi-virt.
#
# Variables:
#   QEMU_SRC              path to a QEMU source tree (required)
#   DEVICES               all (default) | ath9k | vwifi | "ath9k vwifi"
#   QEMU_ARCH             guest architecture (default x86_64)
#   QEMU_TARGET_LIST      meson target list (default $(QEMU_ARCH)-softmmu)
#   QEMU_CONFIGURE_FLAGS  extra configure flags (default --enable-debug)
#   INSTALL_PREFIX        install prefix (default /usr/local)
#   GUEST_IMAGE           qcow2 for the level-3 probe test (optional)
#
# install vs upgrade
# ------------------
# `qemu-install` refuses to overwrite an existing qemu-system binary at
# the install prefix and says so. `qemu-upgrade` overwrites it. The
# distinction matters because the default prefix is /usr/local, where a
# distro or an earlier build may already have put a QEMU that other
# things on the machine depend on — clobbering it as a side effect of a
# build would be a nasty surprise. So installing over one is opt-in, and
# `qemu-upgrade` is how you opt in. The per-device Makefiles have had
# the same pair for the same reason; this is that behaviour, hoisted to
# cover a build carrying both devices.

QEMU_SRC             ?=
DEVICES              ?= all
QEMU_ARCH            ?= x86_64
QEMU_TARGET_LIST     ?= $(QEMU_ARCH)-softmmu
QEMU_CONFIGURE_FLAGS ?= --enable-debug
INSTALL_PREFIX       ?= /usr/local
GUEST_IMAGE          ?=
NPROC                := $(shell nproc 2>/dev/null || echo 4)

QEMU_BUILD_DIR    = $(QEMU_SRC)/build
QEMU_BINARY       = $(QEMU_BUILD_DIR)/qemu-system-$(QEMU_ARCH)
INSTALLED_BINARY  = $(INSTALL_PREFIX)/bin/qemu-system-$(QEMU_ARCH)

ALL_DEVICES := ath9k vwifi
comma       := ,
ifeq ($(strip $(DEVICES)),all)
DEVICE_LIST := $(ALL_DEVICES)
else
# Accept both DEVICES="ath9k vwifi" and DEVICES=ath9k,vwifi.
DEVICE_LIST := $(strip $(subst $(comma), ,$(DEVICES)))
endif

# Reject typos loudly rather than silently building nothing.
BAD_DEVICES := $(filter-out $(ALL_DEVICES),$(DEVICE_LIST))

# Everything that, if changed, makes the existing build directory wrong.
# Not just the device set: changing QEMU_CONFIGURE_FLAGS to turn features
# on is exactly the case where silently reusing an old configure would
# hand you a QEMU without the features you just asked for.
QEMU_CONFIG_KEY = $(DEVICE_LIST) | $(QEMU_TARGET_LIST) | $(INSTALL_PREFIX) | $(QEMU_CONFIGURE_FLAGS)

# Records which devices the build directory was configured for, so that
# changing DEVICES between runs forces a reconfigure instead of quietly
# producing a QEMU without the device you just asked for.
DEVICE_STAMP = $(QEMU_BUILD_DIR)/.vwifi-devices

.PHONY: qemu-help
qemu-help:
	@echo ""
	@echo "QEMU device builds — all targets need QEMU_SRC=/path/to/qemu"
	@echo "==========================================================="
	@echo ""
	@echo "  qemu               integrate + configure + build"
	@echo "  qemu-integrate     copy device sources into the QEMU tree"
	@echo "  qemu-configure     configure the QEMU build directory"
	@echo "  qemu-reconfigure   force a fresh configure"
	@echo "  qemu-build         build QEMU"
	@echo "  qemu-install       install, but NOT over an existing install"
	@echo "  qemu-upgrade       install, overwriting an existing install"
	@echo "  qemu-test          run the ath9k probe tests against the build"
	@echo "  qemu-clean         clean the QEMU build directory"
	@echo ""
	@echo "  DEVICES=all        both devices (default)"
	@echo "  DEVICES=ath9k      vwifi-ath9k only  (AR9285, stock Linux driver)"
	@echo "  DEVICES=vwifi      vwifi-virt only   (Windows guests)"
	@echo '  DEVICES="ath9k vwifi"  explicit list'
	@echo ""
	@echo "  QEMU_ARCH=x86_64            guest architecture"
	@echo "  QEMU_TARGET_LIST=...        meson target list"
	@echo "  QEMU_CONFIGURE_FLAGS=...    extra configure flags"
	@echo "  INSTALL_PREFIX=/usr/local   install prefix"
	@echo "  GUEST_IMAGE=/path/to.qcow2  for the level-3 probe test"
	@echo ""
	@echo "Currently selected: DEVICES=$(DEVICE_LIST)"
	@echo ""

.PHONY: check-qemu-src
check-qemu-src:
ifndef QEMU_SRC
	$(error QEMU_SRC is not set. Usage: make <target> QEMU_SRC=/path/to/qemu)
endif
	@test -f "$(QEMU_SRC)/meson.build" || \
		{ echo "ERROR: $(QEMU_SRC)/meson.build not found."; \
		  echo "       Is $(QEMU_SRC) a QEMU source tree?"; exit 1; }
ifneq ($(BAD_DEVICES),)
	$(error Unknown device(s) in DEVICES: $(BAD_DEVICES). Valid: $(ALL_DEVICES) all)
endif

.PHONY: qemu qemu-integrate qemu-configure qemu-reconfigure qemu-build
.PHONY: qemu-install qemu-upgrade qemu-test qemu-clean

qemu: qemu-build

qemu-integrate: check-qemu-src
	@echo "=== Integrating device(s) into QEMU: $(DEVICE_LIST) ==="
	@for d in $(DEVICE_LIST); do \
		$(MAKE) --no-print-directory -C devices/$$d integrate \
			QEMU_SRC="$(QEMU_SRC)" || exit 1; \
	done

# Skips the (slow) configure when the build directory is already set up
# for exactly this device set. Changing DEVICES invalidates it.
qemu-configure: qemu-integrate
	@echo "=== Configuring QEMU build ==="
	@mkdir -p "$(QEMU_BUILD_DIR)"
	@if [ -f "$(QEMU_BUILD_DIR)/build.ninja" ] && \
	   [ -f "$(DEVICE_STAMP)" ] && \
	   [ "$$(cat '$(DEVICE_STAMP)')" = "$(QEMU_CONFIG_KEY)" ]; then \
		echo "   already configured for: $(DEVICE_LIST) – skipping"; \
		echo "   (use 'make qemu-reconfigure' to force)"; \
	else \
		echo "   configuring for: $(DEVICE_LIST)"; \
		cd "$(QEMU_BUILD_DIR)" && \
			"$(QEMU_SRC)/configure" \
				--target-list=$(QEMU_TARGET_LIST) \
				--prefix="$(INSTALL_PREFIX)" \
				$(QEMU_CONFIGURE_FLAGS) || exit 1; \
		echo "$(QEMU_CONFIG_KEY)" > "$(DEVICE_STAMP)"; \
	fi

qemu-reconfigure: check-qemu-src
	@rm -f "$(DEVICE_STAMP)"
	@$(MAKE) --no-print-directory qemu-configure

qemu-build: qemu-configure
	@echo "=== Building QEMU with: $(DEVICE_LIST) ==="
	$(MAKE) -C "$(QEMU_BUILD_DIR)" -j$(NPROC)
	@echo ""
	@echo "   Built $(QEMU_BINARY)"
	@echo "   Devices available: $(foreach d,$(DEVICE_LIST),$(if $(filter ath9k,$(d)),vwifi-ath9k,vwifi-virt))"
	@echo ""

# Deliberately non-destructive — see "install vs upgrade" above.
qemu-install: check-qemu-src
	@echo "=== Installing QEMU ==="
	@if [ -x "$(INSTALLED_BINARY)" ]; then \
		echo "   $(INSTALLED_BINARY) already exists – skipping install"; \
		echo "   Run 'make qemu-upgrade' to overwrite it."; \
	else \
		test -x "$(QEMU_BINARY)" || \
			{ echo "ERROR: $(QEMU_BINARY) not found. Run 'make qemu' first."; exit 1; }; \
		echo "   Installing into $(INSTALL_PREFIX) ..."; \
		$(MAKE) -C "$(QEMU_BUILD_DIR)" install; \
		echo "   Installed $(INSTALLED_BINARY)"; \
	fi

qemu-upgrade: check-qemu-src
	@echo "=== Upgrading QEMU install ==="
	@test -x "$(QEMU_BINARY)" || \
		{ echo "ERROR: $(QEMU_BINARY) not found. Run 'make qemu' first."; exit 1; }
	@echo "   Reinstalling into $(INSTALL_PREFIX) (overwriting existing) ..."
	$(MAKE) -C "$(QEMU_BUILD_DIR)" install
	@echo "   Installed $(INSTALLED_BINARY)"

# The probe suite belongs to the ath9k device: it boots a guest and checks
# the stock Linux ath9k driver binds. vwifi-virt has no equivalent, because
# its guest driver is the part that still needs hardware in the loop; its
# device logic is covered by 'make test-devices' instead.
qemu-test: check-qemu-src
	@test -x "$(QEMU_BINARY)" || \
		{ echo "ERROR: $(QEMU_BINARY) not found. Run 'make qemu' first."; exit 1; }
ifeq ($(filter ath9k,$(DEVICE_LIST)),)
	@echo "   DEVICES=$(DEVICE_LIST) does not include ath9k – nothing to run."
	@echo "   The guest probe suite is ath9k-only; see 'make test-devices'."
else
	$(MAKE) -C devices/ath9k test QEMU_SRC="$(QEMU_SRC)" \
		GUEST_IMAGE="$(GUEST_IMAGE)"
endif

qemu-clean: check-qemu-src
	@echo "=== Cleaning QEMU build ==="
	@rm -f "$(DEVICE_STAMP)"
	@test -d "$(QEMU_BUILD_DIR)" && \
		$(MAKE) -C "$(QEMU_BUILD_DIR)" clean || true

# ---------- Tests ----------
#
# Everything here runs on a plain host toolchain: no QEMU, no guest, no
# kernel headers. Both QEMU device models keep their logic in a portable
# core that the tests drive through a mock backend.

.PHONY: test test-medium test-devices
test: test-medium test-devices

test-medium: $(BUILD)/vwifi-medium
	VWIFI_MEDIUM=$(BUILD)/vwifi-medium python3 medium/tests/harness.py

test-devices:
	$(MAKE) -C devices/vwifi test
	$(MAKE) -C devices/ath9k test-crypto test-wep test-tkip test-ampdu

# ---------- Clean ----------

.PHONY: clean
clean:
	rm -rf $(BUILD)
	$(MAKE) -C host clean KDIR=$(KDIR)
	$(MAKE) -C drivers/vwifi/linux clean KDIR=$(KDIR)
	$(MAKE) -C devices/vwifi clean
	$(MAKE) -C devices/ath9k clean-tests
