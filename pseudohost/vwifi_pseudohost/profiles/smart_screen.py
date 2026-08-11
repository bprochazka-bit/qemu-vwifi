#
# vwifi-pseudohost — smart screen / smart display profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A wall/desk smart display (a smart-home hub screen, a DLNA/cast-capable
# panel).  Its network signature is UPnP: it advertises itself over SSDP
# on 239.255.255.250:1900 as a rootdevice and a MediaRenderer (DLNA), and
# a DIAL service so phones can launch apps onto the screen.  It runs a
# small embedded web server for the device description and setup UI.  The
# stack is Android/embedded-Linux: TTL 64.
#
# SSDP is live (answers M-SEARCH, announces alive) so the screen appears
# to any UPnP/DLNA controller or DIAL client, and it also advertises an
# HTTP UI over mDNS.  The rendering/control HTTP endpoints the LOCATION
# points at land with the TCP phase.
#
from ..host import PseudoHost
from ..mdns import Advert, MDNSResponder
from ..services import Service
from ..ssdp import SSDPResponder, Target, uuid_from_mac

_SERVER = "Linux/5.10 UPnP/1.0 SmartScreen/2.4"
_LOCATION = "http://{ip}:8080/description.xml"


class ScreenPortsService(Service):
    """The web / cast surface of the display (serviced with TCP)."""
    name = "smart-screen"
    tcp_ports = (8008, 8009, 8080, 8443)        # DIAL/cast setup + HTTP(S)


def _screen_targets(host):
    base = uuid_from_mac(host.mac)
    nts = [
        "upnp:rootdevice",
        "urn:schemas-upnp-org:device:MediaRenderer:1",
        "urn:dial-multiscreen-org:service:dial:1",
    ]
    return [Target(nt, "%s::%s" % (base, nt), location=_LOCATION,
                   server=_SERVER) for nt in nts]


def _screen_ssdp():
    return SSDPResponder(target_factory=_screen_targets, server=_SERVER,
                         announce_period=20.0)


def _screen_mdns():
    return MDNSResponder(
        advert_factory=lambda h: [Advert("_http._tcp.local", h.hostname, 8080,
                                         [b"path=/setup"])],
        announce_period=30.0)


class SmartScreen(PseudoHost):
    persona = "smart-screen"
    os_ttl = 64
    hostname = "SmartDisplay-Kitchen"
    mac_oui = bytes([0x02, 0x71, 0x47])
    icmp = True
    services = [ScreenPortsService, _screen_ssdp, _screen_mdns]
