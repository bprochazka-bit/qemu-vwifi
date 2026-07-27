# Makefile for qemu-vwifi — top level
#
# The repository holds four kinds of thing (see README.md):
#   abi/      shared contracts, header-only
#   medium/   the hub and the tools that attach to it  -> userspace binaries
#   host/     the host's own mac80211 radio            -> kernel module + tools
#   devices/  QEMU device models                       -> built inside a QEMU tree
#   drivers/  guest drivers                            -> built with each OS's SDK
#
# This Makefile builds everything a plain host toolchain can build. The
# QEMU devices need a QEMU source tree and the guest drivers need their
# own SDKs, so they have their own entry points:
#
#   make -C devices/ath9k integrate QEMU_SRC=/path/to/qemu
#   make -C devices/vwifi integrate QEMU_SRC=/path/to/qemu
#
# Targets:
#   make                     — userspace binaries (no kernel headers needed)
#   make module              — host kernel module against the running kernel
#   make module KDIR=/path   — ... against a specific kernel source tree
#   make install             — install the kernel module (requires root)
#   make install SKIP_SIGN=1 — install without the module-signing step, for
#                              distros that ship no signing key
#   make install-userspace   — install userspace binaries (honors PREFIX, DESTDIR)
#   make uninstall-userspace — remove them again
#   make test                — every test suite that runs on the host
#   make legacy              — build the superseded ath9k_medium hubs
#   make clean               — remove all build artifacts

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
	$(MAKE) -C devices/vwifi clean
	$(MAKE) -C devices/ath9k clean-tests
