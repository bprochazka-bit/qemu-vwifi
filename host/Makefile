# vwifi host kernel module
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Builds vwifi_host.ko, the mac80211 driver that gives the host machine
# its own radios on the virtual medium. The userspace pieces that go with
# it (vwifi-host-relay, vwifi-ctl) are built from the top-level Makefile,
# which has no kernel-headers dependency.
#
#   make                     — build against the running kernel
#   make KDIR=/path/to/src   — build against a specific kernel tree
#   make install             — install and depmod (requires root)
#   make install SKIP_SIGN=1 — skip the module-signing step
#   make clean

ifneq ($(KERNELRELEASE),)

# ---- kbuild pass: included by the kernel build system ----

obj-m += vwifi_host.o

# The wire protocol is shared with the hub and the QEMU devices; there is
# one copy of it, in abi/, and nobody forks it.
ccflags-y += -I$(src)/../abi -DDEBUG

else

# ---- normal pass ----

KDIR ?= /lib/modules/$(shell uname -r)/build

# Module signing: stock Debian/Ubuntu kernel-headers packages do not ship
# the private signing key, so Kbuild's post-install sign-file step fails
# harmlessly (the module is simply left unsigned, which is fine unless
# Secure Boot is enabled). Pass SKIP_SIGN=1 to skip signing and silence
# that noisy OpenSSL "no such file" error.
SKIP_SIGN ?=
ifeq ($(SKIP_SIGN),1)
MODINST_ARGS := mod_sign_cmd=true
endif

.PHONY: all install clean
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install $(MODINST_ARGS)
	depmod -a

clean:
	-$(MAKE) -C $(KDIR) M=$(CURDIR) clean 2>/dev/null || true

endif
