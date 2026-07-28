# vwifi — Linux full-MAC driver for the vwifi-virt QEMU device
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
#   make                     — build against the running kernel
#   make KDIR=/path/to/src   — build against a specific kernel tree
#   make install             — install and depmod (requires root)
#   make clean

ifneq ($(KERNELRELEASE),)

obj-m += vwifi.o
vwifi-y := vwifi_main.o vwifi_cfg80211.o vwifi_net.o

# The device<->driver contract has one copy, at the top of the repo.
ccflags-y += -I$(src)/../../../abi

else

KDIR ?= /lib/modules/$(shell uname -r)/build

.PHONY: all install clean
all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules_install
	depmod -a

clean:
	-$(MAKE) -C $(KDIR) M=$(CURDIR) clean 2>/dev/null || true

endif
