#
# vwifi-pseudohost — a small multicast-DNS / DNS-SD responder
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Enough of mDNS (RFC 6762) and DNS-SD (RFC 6763) for a pseudo-host to be
# discoverable the way the real device is: it answers PTR browse queries
# for the service types it advertises, and the SRV/TXT/A records that go
# with them.  A Chromecast is defined by its _googlecast._tcp
# advertisement; a modern printer/scanner by _ipp._tcp / _uscan._tcp.
# Without this they would be on the network but invisible to the tools
# that look for them (avahi-browse, dns-sd, the Cast SDK).
#
# Scope, deliberately: unicast/multicast query in, a multicast response
# out to 224.0.0.251:5353, no name compression on the way out (valid,
# just larger), no probing/conflict resolution.  It is a responder for a
# virtual device on a virtual medium, not a general resolver.
#
import struct
import time

from .services import Service

MDNS_ADDR = "224.0.0.251"
MDNS_PORT = 5353

# record types
T_A = 1
T_PTR = 12
T_TXT = 16
T_SRV = 33
T_ANY = 255

CLASS_IN = 1
CACHE_FLUSH = 0x8000            # top bit of the class field in responses

# The DNS-SD meta-query every browser uses to enumerate service types.
SERVICE_ENUM = "_services._dns-sd._udp.local"


def encode_name(name):
    """Encode a dotted DNS name as length-prefixed labels + a root zero."""
    out = bytearray()
    for label in name.split("."):
        if not label:
            continue
        b = label.encode("utf-8")[:63]
        out.append(len(b))
        out += b
    out.append(0)
    return bytes(out)


def _rr(name, rtype, rdata, ttl=120, cache_flush=True):
    cls = CLASS_IN | (CACHE_FLUSH if cache_flush else 0)
    return (encode_name(name) + struct.pack(">HHIH", rtype, cls, ttl,
                                            len(rdata)) + rdata)


def encode_txt(items):
    """TXT rdata: a sequence of length-prefixed strings (never empty)."""
    if not items:
        return b"\x00"
    out = bytearray()
    for it in items:
        if isinstance(it, str):
            it = it.encode("utf-8")
        out.append(min(len(it), 255))
        out += it[:255]
    return bytes(out)


def _read_name(buf, off):
    """Parse a (possibly compressed) DNS name; return (name, next_off)."""
    labels = []
    jumped = False
    orig = off
    steps = 0
    while True:
        if off >= len(buf) or steps > 128:
            break
        ln = buf[off]
        if ln == 0:
            off += 1
            break
        if (ln & 0xC0) == 0xC0:                    # compression pointer
            ptr = ((ln & 0x3F) << 8) | buf[off + 1]
            if not jumped:
                orig = off + 2
            off = ptr
            jumped = True
            steps += 1
            continue
        labels.append(buf[off + 1 : off + 1 + ln].decode("utf-8", "replace"))
        off += 1 + ln
    return ".".join(labels), (orig if jumped else off)


def parse_questions(payload):
    """Return [(qname_lower, qtype), ...] from an mDNS query, or []."""
    if len(payload) < 12:
        return []
    (_id, _flags, qd, _an, _ns, _ar) = struct.unpack_from(">HHHHHH", payload, 0)
    off = 12
    out = []
    for _ in range(qd):
        name, off = _read_name(payload, off)
        if off + 4 > len(payload):
            break
        qtype, _qclass = struct.unpack_from(">HH", payload, off)
        off += 4
        out.append((name.lower(), qtype))
    return out


def build_response(answers, additionals=()):
    """Assemble an authoritative mDNS response from prebuilt RRs."""
    hdr = struct.pack(">HHHHHH", 0, 0x8400, 0, len(answers), 0,
                      len(additionals))
    return hdr + b"".join(answers) + b"".join(additionals)


class Advert:
    """One advertised DNS-SD service.

    stype    : service type, e.g. "_googlecast._tcp.local"
    instance : the instance label, e.g. "Chromecast-1a2b"
    port     : TCP/UDP port the service listens on
    txt      : list of TXT strings/bytes (key=value)
    """

    def __init__(self, stype, instance, port, txt=()):
        self.stype = stype
        self.instance = instance
        self.port = port
        self.txt = list(txt)

    @property
    def fqdn(self):
        return "%s.%s" % (self.instance, self.stype)


class MDNSResponder(Service):
    """Answers browse queries for a set of advertised services.

    Chromecast and the HP MFP both hand this a list of Adverts; the
    behaviour is identical, only the advertised set differs.
    """

    name = "mdns"
    udp_ports = (MDNS_PORT,)

    def __init__(self, adverts=(), announce_period=0.0, advert_factory=None):
        """
        adverts         : fixed list of Advert, or
        advert_factory  : callable(host) -> [Advert], resolved at bind time
                          (use this when an advert depends on the host's
                          identity — its MAC-derived id, its hostname).
        """
        super().__init__()
        self.adverts = list(adverts)
        self.announce_period = announce_period
        self._advert_factory = advert_factory
        self.host_name = "pseudohost.local"
        self._next_announce = 0.0

    def on_start(self):
        # host label from the profile's hostname (DNS labels: no spaces).
        label = self.host.hostname.replace(" ", "-")
        self.host_name = "%s.local" % label
        if self._advert_factory is not None:
            self.adverts = list(self._advert_factory(self.host))

    # -- record assembly ---------------------------------------------------
    def _srv_and_txt(self, a):
        srv = _rr(a.fqdn, T_SRV,
                  struct.pack(">HHH", 0, 0, a.port) + encode_name(self.host_name))
        txt = _rr(a.fqdn, T_TXT, encode_txt(a.txt))
        return srv, txt

    def _a_record(self):
        if self.stack.ip is None:
            return None
        return _rr(self.host_name, T_A, bytes(self.stack.ip))

    def _answer_for_type(self, a):
        """The full PTR + SRV + TXT + A bundle for a browse of a.stype."""
        ptr = _rr(a.stype, T_PTR, encode_name(a.fqdn), cache_flush=False)
        srv, txt = self._srv_and_txt(a)
        extra = [srv, txt]
        arec = self._a_record()
        if arec:
            extra.append(arec)
        return ptr, extra

    # -- query handling ----------------------------------------------------
    def on_udp(self, src_ip, src_port, dst_ip, dst_port, payload):
        questions = parse_questions(payload)
        if not questions:
            return
        answers = []
        additionals = []
        for qname, qtype in questions:
            self._match(qname, qtype, answers, additionals)
        if answers:
            resp = build_response(answers, additionals)
            self.stack.send_udp(MDNS_ADDR, MDNS_PORT, resp,
                                src_port=MDNS_PORT)

    def _match(self, qname, qtype, answers, additionals):
        # Service enumeration: list the types we offer.
        if qname == SERVICE_ENUM and qtype in (T_PTR, T_ANY):
            for a in self.adverts:
                answers.append(_rr(SERVICE_ENUM, T_PTR, encode_name(a.stype),
                                   cache_flush=False))
            return
        for a in self.adverts:
            if qname == a.stype.lower() and qtype in (T_PTR, T_ANY):
                ptr, extra = self._answer_for_type(a)
                answers.append(ptr)
                additionals.extend(extra)
            elif qname == a.fqdn.lower() and qtype in (T_SRV, T_TXT, T_ANY):
                srv, txt = self._srv_and_txt(a)
                answers.extend([srv, txt])
                arec = self._a_record()
                if arec:
                    additionals.append(arec)
        if qname == self.host_name.lower() and qtype in (T_A, T_ANY):
            arec = self._a_record()
            if arec:
                answers.append(arec)

    # -- unsolicited announcement -----------------------------------------
    def tick(self):
        if self.announce_period <= 0 or self.stack.ip is None:
            return
        now = time.monotonic()
        if now < self._next_announce:
            return
        self._next_announce = now + self.announce_period
        answers = []
        additionals = []
        for a in self.adverts:
            ptr, extra = self._answer_for_type(a)
            answers.append(ptr)
            additionals.extend(extra)
        if answers:
            self.stack.send_udp(MDNS_ADDR, MDNS_PORT,
                                build_response(answers, additionals),
                                src_port=MDNS_PORT)
