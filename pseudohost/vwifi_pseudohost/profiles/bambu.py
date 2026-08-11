#
# vwifi-pseudohost — Bambu Lab 3D printer profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A Bambu Lab network 3D printer (X1/P1/A1 class).  Its signature is a
# vendor SSDP advertisement on 239.255.255.250:1990 — that is how Bambu
# Studio and the Handy app find a printer on the LAN — carrying the
# DevModel/DevName/DevSignal/DevConnect headers.  Control is MQTT over
# TLS (8883), file transfer is FTPS (990), and the camera is an RTSP-over-
# TLS stream (322).  The stack is embedded Linux: TTL 64.
#
# The SSDP responder is live: it answers the M-SEARCH Bambu Studio sends
# and periodically announces ssdp:alive, so the pseudo-printer shows up
# in the slicer's device list. The MQTT/FTPS/RTSP ports are advertised;
# their (TLS) protocols land with the TLS/TCP phase.
#
from ..host import PseudoHost
from ..services import Service
from ..ssdp import SSDPResponder, Target, uuid_from_mac

BAMBU_SSDP_PORT = 1990
BAMBU_NT = "urn:bambulab-com:device:3dprinter:1"
BAMBU_MODEL = "C11"                             # P1S-class model code
BAMBU_MODEL_NAME = "Bambu Lab P1S"


class BambuPortsService(Service):
    """The control/media surface of the printer (serviced with TLS/TCP)."""
    name = "bambu"
    tcp_ports = (322, 990, 8883)                # RTSPS camera, FTPS, MQTTS


def _bambu_targets(host):
    serial = "01P00A" + host.mac.hex().upper()[:12]
    usn = uuid_from_mac(host.mac)
    extra = [
        ("Server", "Bambu Lab"),
        ("DevModel.bambu.com", BAMBU_MODEL),
        ("DevName.bambu.com", host.hostname),
        ("DevSignal.bambu.com", "-44"),
        ("DevConnect.bambu.com", "lan"),
        ("DevBind.bambu.com", "free"),
        ("DevVersion.bambu.com", "01.05.00.00"),
        ("DevSerial.bambu.com", serial),
    ]
    # Bambu's LOCATION is the bare printer IP (substituted at send time).
    return [Target(BAMBU_NT, usn, location="{ip}", server="Bambu Lab",
                   extra=extra)]


def _bambu_ssdp():
    return SSDPResponder(target_factory=_bambu_targets, port=BAMBU_SSDP_PORT,
                         server="Bambu Lab", announce_period=10.0)


class BambuLabPrinter(PseudoHost):
    persona = "bambu-3dprinter"
    os_ttl = 64
    hostname = "3DP-Bambu-PH01"
    mac_oui = bytes([0x02, 0x83, 0x8C])         # Bambu-like locally-administered
    icmp = True
    services = [BambuPortsService, _bambu_ssdp]
