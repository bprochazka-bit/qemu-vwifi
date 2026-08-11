#
# vwifi-pseudohost — point-of-sale terminal profile (Square-style)
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# A countertop point-of-sale terminal like a Square Register/Terminal: an
# Android-based device that talks to the payment cloud over HTTPS and, on
# the local network, discovers and drives a receipt printer.  Its
# observable shape is therefore a locked-down Android host (TTL 64) that
# advertises a small local pairing/status web UI over mDNS and keeps a
# tight port surface — most of its interesting traffic is outbound TLS to
# the acquirer, not a listener.
#
# Modelled here as: mDNS presence for an HTTP setup/status endpoint and a
# vendor pairing service, ICMP, and the HTTPS/admin ports advertised.
# Deliberately quiet: a PoS device that answered a broad set of probes
# would be the wrong fingerprint — payment terminals are hardened.
#
from ..host import PseudoHost
from ..mdns import Advert, MDNSResponder
from ..services import Service


class POSPortsService(Service):
    """The (small) local surface of the terminal.

    A payment terminal mostly talks *out* over TLS; locally it exposes a
    setup/status endpoint and a pairing port. Kept intentionally minimal.
    """
    name = "pos"
    tcp_ports = (443, 8080)                     # cloud/admin TLS, local status


def _pos_mdns():
    return MDNSResponder(
        advert_factory=lambda h: [
            # A local status/pairing web UI, discoverable on the LAN.
            Advert("_http._tcp.local", h.hostname, 8080, [b"path=/status"]),
            # A vendor pairing service stands in for the terminal's
            # device-to-dock discovery (name kept generic on purpose).
            Advert("_pos-pairing._tcp.local", h.hostname, 8080,
                   [b"vendor=square-like", b"model=Register",
                    b"id=" + h.mac.hex().encode()]),
        ],
        announce_period=30.0)


class POSTerminal(PseudoHost):
    persona = "pos-terminal"
    os_ttl = 64                                # Android-based terminal
    hostname = "Square-Register-PH01"
    mac_oui = bytes([0x02, 0x51, 0x71])
    icmp = True
    services = [POSPortsService, _pos_mdns]
