# vwifi — Linux full-MAC driver for the vwifi-virt QEMU device
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
#   make                     — build against the running kernel
#   make KDIR=/path/to/src   — build against a specific kernel tree
#   make install             — install and depmod (requires root)
#   sudo make dkms           — install via DKMS: rebuilds on kernel upgrade
#   sudo make dkms-remove    — unregister from DKMS
#   make clean

ifneq ($(KERNELRELEASE),)

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
# directory that does not exist there.
ccflags-y += $(addprefix -I,$(wildcard $(src)/../../../abi))

else

KDIR ?= /lib/modules/$(shell uname -r)/build

.PHONY: all install clean dkms dkms-remove
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

endif
