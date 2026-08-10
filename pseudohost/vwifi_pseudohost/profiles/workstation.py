#
# vwifi-pseudohost — workstation profiles
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Linux and Windows client machines.  For phase 1 the observable
# difference between them is the fingerprint — TTL and MAC OUI and
# hostname style — plus which discovery services they run.  As the TCP
# layer and richer services arrive these grow the protocols each OS
# actually speaks (SMB/NBNS on Windows, mDNS/SSH on Linux); the class
# stays the seam where that goes.
#
from ..host import PseudoHost
from ..services import HostnameBeaconService, UDPEchoService


class LinuxWorkstation(PseudoHost):
    persona = "linux-workstation"
    os_ttl = 64                            # Linux default IPv4 TTL
    hostname = "ws-linux"
    # A commonly-seen locally-administered range is fine for a lab; a
    # real deployment would seed this from the vendor being impersonated.
    mac_oui = bytes([0x02, 0x1A, 0x11])
    icmp = True
    services = [UDPEchoService]


class WindowsWorkstation(PseudoHost):
    persona = "windows-workstation"
    os_ttl = 128                           # Windows default IPv4 TTL
    hostname = "DESKTOP-PH01"
    mac_oui = bytes([0x02, 0x1C, 0x42])
    icmp = True
    # NBNS/LLMNR-style name announcement stands in for the Windows
    # discovery chatter until the real protocols are implemented.
    services = [lambda: HostnameBeaconService(port=5355, period=15.0)]
