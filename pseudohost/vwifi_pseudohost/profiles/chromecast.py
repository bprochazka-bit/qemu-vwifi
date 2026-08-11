#
# vwifi-pseudohost — Chromecast / Google Cast profile
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A Google Cast receiver.  What identifies a Chromecast on a network is
# its mDNS advertisement of _googlecast._tcp with a TXT record carrying
# the friendly name, model and device id, plus the Cast ports (8008 HTTP
# setup, 8009 TLS Cast v2, 8443).  The stack is embedded Linux: TTL 64.
#
# The mDNS responder is live: browse _googlecast._tcp.local (avahi-browse,
# dns-sd, the Cast SDK) and the pseudo-Chromecast answers with PTR/SRV/
# TXT/A, resolving to its DHCP address.  The Cast control channel on 8009
# is a TLS/protobuf protocol and a later phase; the port is advertised.
#
import os

from ..host import PseudoHost
from ..mdns import Advert, MDNSResponder
from ..services import Service


class CastPortsService(Service):
    """The TCP surface of a Cast receiver (serviced when TCP lands)."""
    name = "cast"
    tcp_ports = (8008, 8009, 8443, 9000)


def _cast_adverts(host):
    """The Chromecast's mDNS adverts, derived from the host's identity.

    Resolved at bind time (MDNSResponder advert_factory) so the friendly
    name and device id come from the running host's hostname and MAC.
    """
    dev_id = host.mac.hex()                      # 12 hex chars, id-like
    instance = "Chromecast-%s" % dev_id
    txt = [
        b"id=" + dev_id.encode(),
        b"md=Chromecast",
        b"fn=" + host.hostname.encode(),
        b"ve=05",
        b"ca=4101",
        b"st=0",
        b"rs=",
    ]
    return [Advert("_googlecast._tcp.local", instance, 8009, txt)]


def _cast_mdns():
    return MDNSResponder(advert_factory=_cast_adverts, announce_period=30.0)


class Chromecast(PseudoHost):
    persona = "chromecast"
    os_ttl = 64
    hostname = "Chromecast-Living-Room"
    # Google-registered OUI 6C:AD:F8 (Azurewave, used by Chromecast),
    # marked locally-administered here since these are virtual devices.
    mac_oui = bytes([0x02, 0xAD, 0xF8])
    icmp = True
    services = [CastPortsService, _cast_mdns]
