#
# vwifi-pseudohost — NAS / network storage profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A network-attached storage appliance.  The recognisable surface is
# file-sharing and management: SMB/445, NFS/2049, AFP/548, plus an HTTP
# admin UI and mDNS/SSDP presence.  As with the printer, the port list
# is advertised now and the protocol implementations are later phases.
#
from ..host import PseudoHost
from ..services import HostnameBeaconService, Service


class NASService(Service):
    name = "nas"
    udp_ports = ()
    tcp_ports = (139, 445, 2049, 548, 80, 443)   # SMB, NFS, AFP, admin UI


class NAS(PseudoHost):
    persona = "nas"
    os_ttl = 64                            # typically a Linux-based appliance
    hostname = "nas-ph01"
    mac_oui = bytes([0x02, 0x11, 0x32])
    icmp = True
    # A NAS advertises itself for discovery; the beacon stands in until
    # mDNS/SSDP land.
    services = [NASService, lambda: HostnameBeaconService(period=20.0)]
