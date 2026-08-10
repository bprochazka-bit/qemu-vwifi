#
# vwifi-pseudohost — virtual hosts on the vwifi medium
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A pseudo-host is a peer on the vwifi medium that behaves like a whole
# machine — it scans, associates (open or WPA2-PSK), pulls a DHCP lease
# and answers on the network — without a VM, a kernel, or a driver.  The
# PseudoHost base class carries the 802.11 station and a small userspace
# TCP/IP stack; device profiles (workstations, printers, NAS, ...)
# subclass it and add the services that make each look like what it
# claims to be.
#
__all__ = ["crypto", "ieee80211", "medium", "supplicant", "station",
           "netstack", "dhcp", "services", "host"]

__version__ = "0.1.0"
