# Makefile for qemu-vwifi
#
# Targets:
#   make                    — build kernel module against running kernel
#   make KDIR=/path/to/src  — build module against specific kernel source
#   make install            — install module (requires root)
#   make userspace          — build userspace hub / relay / bridge binaries
#   make test               — run tests/harness.py (requires userspace built)
#   make clean              — remove all build artifacts (kernel + userspace)

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

obj-m += ath9k_medium_host.o
ccflags-y += -DDEBUG

# ---------- Kernel module ----------
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

ath9k_medium_host.o: ath9k_medium.h

# ---------- Userspace utilities ----------
# These don't require kernel headers, so they can be built standalone
# (e.g. in CI environments without a matching kernel-headers package).
CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -O2

USERSPACE_BINS := ath9k_hub ath9k_host_relay ath9k_phys_bridge

ath9k_hub: ath9k_medium_hub_scalable.c ath9k_medium.h
	$(CC) $(CFLAGS) -o $@ $< -lm

ath9k_host_relay: ath9k_host_relay.c ath9k_medium.h
	$(CC) $(CFLAGS) -o $@ $<

ath9k_phys_bridge: ath9k_phys_bridge.c ath9k_medium.h
	$(CC) $(CFLAGS) -o $@ $<

userspace: $(USERSPACE_BINS)

# ---------- Tests ----------
test: ath9k_hub
	python3 tests/harness.py

# ---------- Clean ----------
clean:
	-$(MAKE) -C $(KDIR) M=$(PWD) clean 2>/dev/null || true
	rm -f $(USERSPACE_BINS)

.PHONY: all install userspace test clean
