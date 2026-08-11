#
# vwifi-pseudohost — a small SSDP / UPnP-discovery responder
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# SSDP is HTTP-over-UDP: a controller multicasts an `M-SEARCH`, devices
# reply with a `200 OK`, and devices periodically multicast `NOTIFY
# ssdp:alive`.  It is how a whole class of devices are found — UPnP/DLNA
# media renderers and DIAL screens on 239.255.255.250:1900, and Bambu Lab
# 3D printers on their own 239.255.255.250:1990 with vendor headers.
#
# This responder does both halves for an arbitrary set of "targets"
# (each an advertised NT/USN with its own headers), on a configurable
# port and multicast group, so one class serves the UPnP devices and the
# Bambu printer alike — only the target list differs.
#
# Deliberately partial: it answers M-SEARCH and announces alive/does not
# implement the eventing (GENA) or the description-document HTTP server
# those advertisements point at; the LOCATION is advertised and the HTTP
# side lands with the TCP phase.
#
import time

from .netstack import ip_str
from .services import Service

SSDP_MCAST = "239.255.255.250"
SSDP_PORT = 1900


class Target:
    """One advertised SSDP device/service.

    nt        : notification / search target, e.g.
                "urn:schemas-upnp-org:device:MediaRenderer:1"
    usn       : unique service name, e.g. "uuid:...::<nt>"
    location  : LOCATION header; may contain "{ip}", substituted at send
                time (None -> the bare host IP, as Bambu uses)
    server    : SERVER header override for this target (else the
                responder's default)
    extra     : list of (header, value) pairs appended verbatim — this is
                where vendor headers go (DevModel.bambu.com, ...)
    """

    def __init__(self, nt, usn, location=None, server=None, extra=None):
        self.nt = nt
        self.usn = usn
        self.location = location
        self.server = server
        self.extra = list(extra or [])


class SSDPResponder(Service):
    name = "ssdp"

    def __init__(self, targets=(), target_factory=None, port=SSDP_PORT,
                 mcast=SSDP_MCAST, server="PseudoHost/1.0 UPnP/1.0",
                 announce_period=0.0):
        super().__init__()
        self.targets = list(targets)
        self._target_factory = target_factory
        self.port = port
        self.mcast = mcast
        self.server = server
        self.announce_period = announce_period
        self.udp_ports = (port,)
        self._next_announce = 0.0

    def on_start(self):
        if self._target_factory is not None:
            self.targets = list(self._target_factory(self.host))

    # -- header assembly ---------------------------------------------------
    def _location(self, t):
        ip = ip_str(self.stack.ip) if self.stack.ip else "0.0.0.0"
        if t.location is None:
            return ip
        if "{ip}" in t.location:
            return t.location.format(ip=ip)
        return t.location

    def _headers(self, t):
        return ["SERVER: %s" % (t.server or self.server),
                "USN: %s" % t.usn] + ["%s: %s" % kv for kv in t.extra]

    def _response(self, t):
        lines = ["HTTP/1.1 200 OK",
                 "CACHE-CONTROL: max-age=1800",
                 "EXT:",
                 "LOCATION: %s" % self._location(t),
                 "ST: %s" % t.nt] + self._headers(t) + ["", ""]
        return "\r\n".join(lines).encode("utf-8")

    def _notify(self, t):
        lines = ["NOTIFY * HTTP/1.1",
                 "HOST: %s:%d" % (self.mcast, self.port),
                 "CACHE-CONTROL: max-age=1800",
                 "LOCATION: %s" % self._location(t),
                 "NT: %s" % t.nt,
                 "NTS: ssdp:alive"] + self._headers(t) + ["", ""]
        return "\r\n".join(lines).encode("utf-8")

    # -- query handling ----------------------------------------------------
    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        if not payload[:8].upper().startswith(b"M-SEARCH"):
            return
        st = self._header(payload, b"st") or "ssdp:all"
        for t in self.targets:
            if self._matches(st, t):
                self.reply_udp(src_ip, src_port, self._response(t),
                               src_port=dst_port)

    @staticmethod
    def _matches(st, t):
        if st in ("ssdp:all", t.nt):
            return True
        if st == "upnp:rootdevice" and "rootdevice" in t.nt:
            return True
        return False

    @staticmethod
    def _header(payload, name):
        name = name.lower()
        for line in payload.split(b"\r\n")[1:]:
            if b":" not in line:
                continue
            k, v = line.split(b":", 1)
            if k.strip().lower() == name:
                return v.strip().strip(b'"').decode("utf-8", "replace")
        return None

    # -- announcement ------------------------------------------------------
    def tick(self):
        if self.announce_period <= 0 or self.stack.ip is None:
            return
        now = time.monotonic()
        if now < self._next_announce:
            return
        self._next_announce = now + self.announce_period
        for t in self.targets:
            self.stack.send_udp(self.mcast, self.port, self._notify(t),
                                src_port=self.port)


def uuid_from_mac(mac):
    """A stable, device-like UUID seeded from the MAC.

    Not a real RFC 4122 UUID — just a stable, well-formed-looking id so a
    device's USN is consistent across announcements, which is all SSDP
    needs it to be.
    """
    return "uuid:00000000-0000-1000-8000-%s" % mac.hex()
