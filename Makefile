# Makefile for qemu-vwifi
#
# Targets:
#   make                    — build kernel module against running kernel
#   make KDIR=/path/to/src  — build module against specific kernel source
#   make install            — install module (requires root)
#   make install SKIP_SIGN=1 — install without the (failing) module-signing
#                             step on distros that ship no signing key
#   make userspace          — build vwifi-medium / vwifi-host-relay /
#                             vwifi-phys-bridge userspace binaries
#   make install-userspace  — install userspace binaries only, no kernel
#                             module (honors PREFIX, DESTDIR)
#   make uninstall-userspace — remove installed userspace binaries
#   make test               — run tests/harness.py (requires userspace built)
#   make clean              — remove all build artifacts (kernel + userspace)

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

obj-m += vwifi_host.o
ccflags-y += -DDEBUG

# ---------- Kernel module ----------
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

# Module signing: stock Debian/Ubuntu kernel-headers packages do not ship
# the private signing key, so Kbuild's post-install sign-file step fails
# harmlessly (the module is simply left unsigned, which is fine unless
# Secure Boot is enabled). Pass SKIP_SIGN=1 to skip signing and silence
# that noisy OpenSSL "no such file" error.
SKIP_SIGN ?=
ifeq ($(SKIP_SIGN),1)
MODINST_ARGS := mod_sign_cmd=true
endif

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install $(MODINST_ARGS)
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

# Install destination (DESTDIR for staged/packaged installs).
PREFIX  ?= /usr/local
BINDIR  ?= $(PREFIX)/bin
INSTALL ?= install

install-userspace: $(USERSPACE_BINS)
	$(INSTALL) -d $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0755 $(USERSPACE_BINS) $(DESTDIR)$(BINDIR)

uninstall-userspace:
	rm -f $(addprefix $(DESTDIR)$(BINDIR)/,$(USERSPACE_BINS))

# ---------- Tests ----------
test: vwifi-medium
	python3 tests/harness.py

# ---------- Clean ----------
clean:
	-$(MAKE) -C $(KDIR) M=$(PWD) clean 2>/dev/null || true
	rm -f $(USERSPACE_BINS)

.PHONY: all install userspace install-userspace uninstall-userspace test clean
