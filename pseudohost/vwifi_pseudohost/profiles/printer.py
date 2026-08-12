#
# vwifi-pseudohost — network printer profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A network-attached printer.  Embedded print-server stacks are a
# recognisable fingerprint on their own: a high TTL, a vendor MAC, and a
# small, fixed set of open ports (IPP/631, JetDirect/9100, LPD/515,
# SNMP/161).  The tcp_ports on the service below advertise that surface
# now; the actual IPP/JetDirect responders are the next phase, and slot
# in as UDPService/TCP handlers without touching this profile.
#
from ..host import PseudoHost
from ..services import Service
from ..printing import JetDirectService
from ..ipp import IPPService
from ..snmp import SNMPAgent
from ..wsd import WSDiscoveryService, WSDHttpService


class PrinterService(Service):
    """The TCP port surface of a print server (SNMP/IPP/9100 own theirs)."""

    name = "printer"
    tcp_ports = (515,)                     # LPD (advertisement)


class NetworkPrinter(PseudoHost):
    persona = "network-printer"
    os_ttl = 255                           # embedded stacks often ship 255
    hostname = "HPLJ-PH01"
    mac_oui = bytes([0x02, 0x60, 0xB0])    # HP-like locally-administered OUI
    icmp = True
    wsd_manufacturer = "HP"
    wsd_model = "HP LaserJet"
    wsd_model_number = "PH01"
    # IPP (631) is the modern driverless add+print path (mDNS _ipp._tcp);
    # WSD makes it discoverable by older Windows; SNMP lets the "Standard
    # TCP/IP Port" wizard identify it; JetDirect/9100 always prints raw.
    services = [PrinterService, IPPService, JetDirectService, SNMPAgent,
                WSDiscoveryService, WSDHttpService]
