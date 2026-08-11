#
# vwifi-pseudohost — HP printer / scanner / fax multifunction profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# An HP "OfficeJet"-class all-in-one: print, scan, and fax in one box.
# It is the NetworkPrinter's surface plus the scanner and web-services
# halves that make it a multifunction:
#
#   print   IPP/631, raw JetDirect/9100, LPD/515, SNMP/161
#   scan    eSCL / AirScan (_uscan._tcp, over HTTP), WSD scan
#   fax     modelled at the discovery layer (no media path in phase 1)
#   mgmt    embedded web server on 80/443, SNMP inventory
#   disc    mDNS for _ipp/_printer/_pdl-datastream/_uscan/_http, WSD 3702
#
# The mDNS responder is live, so the device is discoverable as a printer
# *and* a scanner (avahi-browse _uscan._tcp / _ipp._tcp), which is the
# thing that distinguishes an MFP from a plain print server.  The actual
# IPP/eSCL/SNMP protocol handlers are later phases; the ports and the
# advertisements are correct now.
#
from ..host import PseudoHost
from ..mdns import Advert, MDNSResponder
from ..services import Service
from ..printing import JetDirectService
from ..wsd import WSDiscoveryService, WSDHttpService

# A plausible model string; TXT records below mirror what an HP MFP
# publishes so a scan of the mDNS records reads like the real thing.
HP_MODEL = "HP OfficeJet Pro 9015"


class MFPPortsService(Service):
    """The port surface of the multifunction; SNMP probes are logged.

    WS-Discovery (UDP 3702) and its metadata HTTP (TCP 5357) are owned by
    the WSD services, not advertised here, so there's no handler clash.
    """
    name = "hp-mfp"
    udp_ports = (161,)                          # SNMP
    tcp_ports = (80, 443, 515, 631, 9100, 8080, 8290)  # web, LPD, IPP, scan

    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        if dst_port == 161:
            self.log("SNMP probe from %d.%d.%d.%d" % tuple(src_ip))


def _mfp_adverts(host):
    """IPP + scanner + web mDNS adverts for the multifunction."""
    inst = host.hostname                        # e.g. "HP-OfficeJet-PH01"
    common_ty = ("ty=" + HP_MODEL).encode()
    ipp_txt = [
        b"rp=ipp/print",
        common_ty,
        b"pdl=application/pdf,image/urf,image/jpeg",
        b"URF=CP1,PQ4-5,RS300-600,SRGB24,V1.4,W8,DM1",
        b"Color=T", b"Duplex=T", b"Scan=T", b"Fax=T",
        b"note=Front Office",
    ]
    scan_txt = [
        b"rs=eSCL",
        common_ty,
        b"vers=2.63",
        b"representation=/eSCL/ScannerIcon.png",
        b"cs=color,grayscale,binary",
        b"is=platen,adf",
        b"duplex=T",
    ]
    return [
        Advert("_ipp._tcp.local", inst, 631, ipp_txt),
        Advert("_printer._tcp.local", inst, 515, [common_ty]),
        Advert("_pdl-datastream._tcp.local", inst, 9100, [common_ty]),
        Advert("_uscan._tcp.local", inst, 8080, scan_txt),
        Advert("_scanner._tcp.local", inst, 8290, [common_ty]),
        Advert("_http._tcp.local", inst, 80, [b"path=/"]),
    ]


def _mfp_mdns():
    return MDNSResponder(advert_factory=_mfp_adverts, announce_period=30.0)


class HPMultifunction(PseudoHost):
    persona = "hp-mfp"
    os_ttl = 255                               # embedded print/scan stack
    hostname = "HP-OfficeJet-PH01"
    mac_oui = bytes([0x02, 0x60, 0xB0])        # HP-like locally-administered
    icmp = True
    # What the WSD metadata Get reports to Windows (the name it shows).
    wsd_manufacturer = "HP"
    wsd_model = HP_MODEL
    wsd_model_number = "9015"
    # WSD is what stock Windows uses for "Network" and "Add a printer";
    # mDNS covers macOS / IPP-Everywhere clients; JetDirect/9100 is the
    # raw path that always prints.
    services = [MFPPortsService, _mfp_mdns, JetDirectService,
                WSDiscoveryService, WSDHttpService]
