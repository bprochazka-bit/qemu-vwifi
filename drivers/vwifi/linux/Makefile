# vwifi — Linux full-MAC driver for the vwifi-virt QEMU device
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
#   make                     — build against the running kernel
#   make KDIR=/path/to/src   — build against a specific kernel tree
#   make install             — install and depmod (requires root)
#   sudo make dkms           — install via DKMS: rebuilds on kernel upgrade
#   sudo make dkms-remove    — unregister from DKMS
#   make dkms-check          — build the way DKMS does, without dkms or root
#   make clean
#
# Deliberately NOT wrapped in "ifneq ($(KERNELRELEASE),)".
#
# That idiom assumes KERNELRELEASE is set only when kbuild is driving
# the build. DKMS breaks the assumption: it runs
#
#     make -jN KERNELRELEASE=<version> KDIR=<tree>
#
# in the staged directory, which selects the kbuild half of a split
# Makefile with no kbuild there to run it. make then finds nothing but
# variable assignments and reports "No targets.  Stop." -- an error that
# says nothing about the real problem, from a tree that builds perfectly
# when invoked by hand.
#
# The flat form below works under either caller. kbuild always names an
# explicit goal (modules / clean / modules_install), so the "all" target
# here is never the one it picks, and obj-m is inert when make reads
# this file directly.

obj-m += vwifi.o
vwifi-y := vwifi_main.o vwifi_cfg80211.o vwifi_net.o vwifi_monitor.o

# vwifi_abi.h is reached from one of two places, and the build has to
# work either way:
#
#   in-repo    the one authoritative copy, three levels up in abi/
#   DKMS       a staged copy beside these sources, because DKMS copies
#              exactly one directory into /usr/src and the shared header
#              is not in it (see dkms-install.sh)
#
# The quoted #include finds a sibling copy on its own, so the DKMS case
# needs no -I at all; wildcard keeps us from passing an -I for a
# directory that does not exist there. $(src) is set by kbuild and empty
# otherwise, which makes this line inert outside a kbuild pass.
ccflags-y += $(addprefix -I,$(wildcard $(src)/../../../abi))

# Recursively expanded on purpose: the shell runs only if something
# actually expands KDIR, never during a kbuild pass over this file.
KDIR ?= /lib/modules/$(shell uname -r)/build

.PHONY: all install clean dkms dkms-remove dkms-check

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	depmod -a

clean:
	-$(MAKE) -C $(KDIR) M=$(CURDIR) clean 2>/dev/null || true

# DKMS: build against every installed kernel and rebuild on upgrade.
# This is what you want in a guest whose kernel gets updated; a plain
# "make install" module disappears on the next boot after an upgrade.
dkms:
	./dkms-install.sh install

dkms-remove:
	./dkms-install.sh remove

# Stage and build exactly as DKMS would, in a temp directory, needing
# neither dkms nor root. "make dkms" has now failed twice for reasons a
# plain "make" could not see -- both times in the gap between this
# Makefile and the staged tree, which nothing else exercises.
dkms-check:
	./dkms-selftest.sh
