# QEMU Virtual Atheros AR9285 – Meson build integration
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# This file is meant to be placed at hw/net/vwifi-ath9k/meson.build inside
# the QEMU source tree.  The parent hw/net/meson.build must include it
# via: subdir('vwifi-ath9k')
#
# The vwifi-medium hub is a standalone tool built separately.

system_ss.add(when: 'CONFIG_VWIFI_ATH9K', if_true: files(
  'vwifi_ath9k_pci.c',
))
