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
from ..wsd import WSDiscoveryService, WSDHttpService


class PrinterService(Service):
    """The port surface and (soon) protocols of a print server."""

    name = "printer"
    udp_ports = (161,)                     # SNMP — status/inventory queries
    tcp_ports = (515, 631, 9100)           # LPD, IPP, raw JetDirect

    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        # SNMP GET handling is a later phase; for now just note the probe
        # so a scan of the pseudo-printer is at least visible in the log.
        self.log("SNMP probe from %d.%d.%d.%d" % tuple(src_ip))


class NetworkPrinter(PseudoHost):
    persona = "network-printer"
    os_ttl = 255                           # embedded stacks often ship 255
    hostname = "HPLJ-PH01"
    mac_oui = bytes([0x02, 0x60, 0xB0])    # HP-like locally-administered OUI
    icmp = True
    wsd_manufacturer = "HP"
    wsd_model = "HP LaserJet"
    wsd_model_number = "PH01"
    # WSD makes it discoverable by stock Windows (Network / Add a printer);
    # JetDirect/9100 is the raw path that always prints.
    services = [PrinterService, JetDirectService,
                WSDiscoveryService, WSDHttpService]
