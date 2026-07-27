# vwifi-virt – Meson build integration
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# This file is meant to be placed at hw/net/vwifi-virt/meson.build inside
# the QEMU source tree.  The parent hw/net/meson.build must include it
# via: subdir('vwifi-virt')
#
# scripts/integrate.sh copies the shared headers (vwifi_abi.h, the device
# <-> guest-driver contract, and vwifi.h, the medium wire protocol) out of
# the repository's abi/ directory into this one, so nothing here needs an
# include path pointing outside the QEMU tree.  abi/ stays authoritative:
# never edit the copies inside a QEMU tree, re-run integrate.sh instead.

system_ss.add(when: 'CONFIG_VWIFI_VIRT', if_true: files(
  'vwifi_qemu.c',      # QEMU backend: PCIDevice, MSI-X, chardev, timer
  'vwifi_device.c',    # portable device logic
  'vwifi_crypto.c',    # AES-128 + CCMP-128
))
