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
from ..mdns import Advert, MDNSResponder
from ..services import Service
from ..printing import JetDirectService
from ..ipp import IPPService
from ..snmp import SNMPAgent
from ..wsd import WSDiscoveryService, WSDHttpService, uuid_from_mac


class PrinterService(Service):
    """The TCP port surface of a print server (SNMP/IPP/9100 own theirs)."""

    name = "printer"
    tcp_ports = (515,)                     # LPD (advertisement)


def _printer_adverts(host):
    """mDNS/Bonjour adverts for a plain network printer: IPP (the modern
    driverless path), LPD, raw JetDirect, and the embedded web page. This is
    what makes the printer show up in a Bonjour browse / macOS 'Add Printer'
    / Windows-with-Bonjour, the same way the multifunction profile does."""
    inst = host.hostname
    model = getattr(host, "wsd_model", "Network Printer")
    ty = ("ty=" + model).encode()
    uuid = uuid_from_mac(host.mac).split(":")[-1].encode()
    ipp_txt = [
        b"txtvers=1", b"qtotal=1",
        b"rp=ipp/print",
        ty,
        ("product=(%s)" % model).encode(),
        b"pdl=application/pdf,image/urf,image/pwg-raster,image/jpeg",
        b"URF=CP1,PQ4-5,RS300-600,SRGB24,W8,V1.4,DM1",
        b"UUID=" + uuid,
        b"adminurl=http://" + host.hostname.encode() + b".local./",
        b"priority=50",
        b"Color=F", b"Duplex=T",
    ]
    return [
        Advert("_ipp._tcp.local", inst, 631, ipp_txt),
        Advert("_printer._tcp.local", inst, 515, [ty]),
        Advert("_pdl-datastream._tcp.local", inst, 9100, [ty]),
        Advert("_http._tcp.local", inst, 80, [b"path=/"]),
    ]


def _printer_mdns():
    return MDNSResponder(advert_factory=_printer_adverts, announce_period=30.0)


class NetworkPrinter(PseudoHost):
    persona = "network-printer"
    os_ttl = 255                           # embedded stacks often ship 255
    hostname = "HPLJ-PH01"
    mac_oui = bytes([0x02, 0x60, 0xB0])    # HP-like locally-administered OUI
    icmp = True
    wsd_manufacturer = "HP"
    wsd_model = "HP LaserJet"
    wsd_model_number = "PH01"
    # IPP (631) is the modern driverless add+print path, advertised over
    # mDNS (_ipp._tcp) so Bonjour/macOS/Windows-with-Bonjour discover it;
    # WSD makes it discoverable by stock Windows; SNMP lets the "Standard
    # TCP/IP Port" wizard identify it; JetDirect/9100 always prints raw.
    services = [PrinterService, _printer_mdns, IPPService, JetDirectService,
                SNMPAgent, WSDiscoveryService, WSDHttpService]
