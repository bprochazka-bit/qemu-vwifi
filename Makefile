# Makefile for ath9k_medium_host kernel module
#
# Usage:
#   make                    — build against running kernel
#   make KDIR=/path/to/src  — build against specific kernel source
#   make clean              — remove build artifacts
#   make install            — install module (requires root)

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

obj-m += ath9k_medium_host.o

# Extra compiler flags
ccflags-y += -DDEBUG

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install:
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	depmod -a

# Copy the shared header so the module can find it
# (it's already in the same directory)
ath9k_medium_host.o: ath9k_medium.h

.PHONY: all clean install
