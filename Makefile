# Makefile for qemu-vwifi
#
# Targets:
#   make                    — build kernel module against running kernel
#   make KDIR=/path/to/src  — build module against specific kernel source
#   make install            — install module (requires root)
#   make userspace          — build vwifi-medium / vwifi-host-relay /
#                             vwifi-phys-bridge userspace binaries
#   make test               — run tests/harness.py (requires userspace built)
#   make clean              — remove all build artifacts (kernel + userspace)

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

obj-m += vwifi_host.o
ccflags-y += -DDEBUG

# ---------- Kernel module ----------
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

vwifi_host.o: vwifi.h

# ---------- Userspace utilities ----------
# These don't require kernel headers, so they can be built standalone
# (e.g. in CI environments without a matching kernel-headers package).
CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -O2

USERSPACE_BINS := vwifi-medium vwifi-host-relay vwifi-phys-bridge

vwifi-medium: vwifi_medium.c vwifi.h
	$(CC) $(CFLAGS) -o $@ $< -lm

vwifi-host-relay: vwifi_host_relay.c vwifi.h
	$(CC) $(CFLAGS) -o $@ $<

vwifi-phys-bridge: vwifi_phys_bridge.c vwifi.h
	$(CC) $(CFLAGS) -o $@ $<

userspace: $(USERSPACE_BINS)

# ---------- Tests ----------
test: vwifi-medium
	python3 tests/harness.py

# ---------- Clean ----------
clean:
	-$(MAKE) -C $(KDIR) M=$(PWD) clean 2>/dev/null || true
	rm -f $(USERSPACE_BINS)

.PHONY: all install userspace test clean
