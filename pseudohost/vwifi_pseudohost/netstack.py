#
# vwifi-pseudohost — a small userspace TCP/IP stack
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Everything above the radio.  The station hands this module raw
# Ethernet SDUs (it has already stripped 802.11 and LLC/SNAP and
# decrypted); this module runs ARP, IPv4, ICMP and UDP demultiplexing,
# and hands datagrams up to registered protocol handlers (DHCP, and the
# service framework).
#
# It is intentionally minimal and host-shaped: one IPv4 address, one
# gateway, an ARP cache, an ICMP-echo responder.  It does not route or
# fragment.  A pseudo-host is an endpoint, not a router.
#
import struct
import time

from . import ieee80211 as dot11

ETH_P_IP = dot11.ETH_P_IP
ETH_P_ARP = dot11.ETH_P_ARP

IPPROTO_ICMP = 1
IPPROTO_UDP = 17
IPPROTO_TCP = 6

BROADCAST_MAC = b"\xff\xff\xff\xff\xff\xff"


def _cksum(data):
    """The Internet checksum (RFC 1071)."""
    if len(data) % 2:
        data += b"\x00"
    s = 0
    for i in range(0, len(data), 2):
        s += (data[i] << 8) | data[i + 1]
    s = (s & 0xFFFF) + (s >> 16)
    s = (s & 0xFFFF) + (s >> 16)
    return (~s) & 0xFFFF


def ip_str(b):
    return ".".join(str(x) for x in b)


def ip_bytes(s):
    if isinstance(s, (bytes, bytearray)):
        return bytes(s)
    return bytes(int(x) for x in s.split("."))


def ip_to_int(b):
    return struct.unpack(">I", ip_bytes(b))[0]


def int_to_ip(n):
    return struct.pack(">I", n & 0xFFFFFFFF)


def parse_cidr(cidr):
    """Turn a CIDR like "10.10.10.0/24" into an addressing plan.

    Returns a dict of dotted-quad strings: network, netmask, gateway,
    pool_start, pool_end.  The gateway is the address's host part when
    one is given ("10.10.10.1/24") else the first usable address; the
    pool defaults to the .100-.200 window when it fits inside the subnet
    and otherwise spans every usable address (skipping the gateway).
    """
    import ipaddress

    iface = ipaddress.ip_interface(str(cidr).strip())
    net = iface.network
    if net.version != 4:
        raise ValueError("only IPv4 CIDR is supported")
    network = int(net.network_address)
    bcast = int(net.broadcast_address)

    gw = int(iface.ip) if int(iface.ip) != network else network + 1

    lo, hi = network + 2, bcast - 1          # usable minus net and gateway
    if hi < lo:                              # /31, /32 — degenerate
        lo, hi = network, bcast
    ps, pe = network + 100, network + 200
    if not (lo <= ps <= hi and lo <= pe <= hi):
        ps, pe = lo, hi

    q = lambda n: ip_str(int_to_ip(n))       # noqa: E731
    return {
        "network": q(network),
        "netmask": str(net.netmask),
        "gateway": q(gw),
        "pool_start": q(ps),
        "pool_end": q(pe),
    }


class NetStack:
    """One host's L2/L3.  Bound to a Station for egress."""

    def __init__(self, station, hostname="pseudohost", log=None):
        self.station = station
        self.hostname = hostname
        self.log = log or (lambda *a: None)
        self.mac = station.mac
        self.ip = None                         # bytes(4) once leased
        self.netmask = None
        self.gateway = None
        self.dns = None
        self.arp_cache = {}                    # ip_bytes -> (mac, expiry)
        self._udp_handlers = {}                # dst_port -> callback
        self._tcp = None                       # optional TCPStack
        self._icmp_enabled = True
        # Default IPv4 TTL — a cheap but effective OS fingerprint.  A
        # device profile overrides it (Linux 64, Windows 128, many
        # embedded stacks 255) so the persona is consistent to a scanner.
        self.ttl = 64
        self._pending_tx = []                  # frames awaiting ARP resolve
        station.on_eth_rx = self._on_eth

    # ---- configuration ---------------------------------------------------
    def configure(self, ip, netmask, gateway=None, dns=None):
        self.ip = ip_bytes(ip)
        self.netmask = ip_bytes(netmask)
        self.gateway = ip_bytes(gateway) if gateway else None
        self.dns = ip_bytes(dns) if dns else None
        self.log("ip: %s/%s gw=%s" % (
            ip_str(self.ip), ip_str(self.netmask),
            ip_str(self.gateway) if self.gateway else "-"))

    def set_icmp_enabled(self, on):
        self._icmp_enabled = on

    def register_udp(self, port, handler):
        """handler(src_ip, src_port, dst_ip, dst_port, payload)."""
        self._udp_handlers[port] = handler

    def attach_tcp(self, tcp_stack):
        """Attach a TCPStack; TCP segments for us are handed to it."""
        self._tcp = tcp_stack

    # ---- egress ----------------------------------------------------------
    def send_eth(self, dst_mac, ethertype, payload):
        self.station.send_eth(dst_mac, ethertype, payload)

    def send_ip(self, dst_ip, proto, payload, src_ip=None):
        dst_ip = ip_bytes(dst_ip)
        src = src_ip if src_ip is not None else (self.ip or bytes(4))
        pkt = self._build_ip(src, dst_ip, proto, payload)
        # Next hop: on-link dst, else gateway.
        nexthop = dst_ip
        if self.netmask and not self._same_subnet(dst_ip):
            nexthop = self.gateway or dst_ip
        if dst_ip == b"\xff\xff\xff\xff" or (dst_ip[0] & 0xF0) == 0xE0:
            self.send_eth(BROADCAST_MAC, ETH_P_IP, pkt)   # bcast/mcast
            return
        mac = self._arp_lookup(nexthop)
        if mac is None:
            self._pending_tx.append((nexthop, pkt))
            self._send_arp_request(nexthop)
            return
        self.send_eth(mac, ETH_P_IP, pkt)

    def send_udp(self, dst_ip, dst_port, payload, src_port=0, src_ip=None,
                 dst_mac=None):
        dst_ip_b = ip_bytes(dst_ip)
        src = src_ip if src_ip is not None else (self.ip or bytes(4))
        udp = self._build_udp(src, dst_ip_b, src_port, dst_port, payload)
        if dst_mac is not None:
            # Caller pinned L2 (DHCP before we have an IP/ARP).
            pkt = self._build_ip(src, dst_ip_b, IPPROTO_UDP, udp)
            self.send_eth(dst_mac, ETH_P_IP, pkt)
            return
        self.send_ip(dst_ip_b, IPPROTO_UDP, udp, src_ip=src)

    # ---- ingress ---------------------------------------------------------
    def _on_eth(self, src_mac, dst_mac, ethertype, sdu):
        if ethertype == ETH_P_ARP:
            self._on_arp(sdu)
        elif ethertype == ETH_P_IP:
            self._on_ip(src_mac, sdu)

    # ---- ARP -------------------------------------------------------------
    def _on_arp(self, p):
        if len(p) < 28:
            return
        oper = (p[6] << 8) | p[7]
        sha = bytes(p[8:14])
        spa = bytes(p[14:18])
        tpa = bytes(p[24:28])
        if spa != bytes(4):
            self.arp_cache[spa] = (sha, time.time() + 120)
            self._flush_pending(spa)
        if oper == 1 and self.ip and tpa == self.ip:      # request for us
            self._send_arp_reply(sha, spa)

    def _send_arp_request(self, tpa):
        if self.ip is None:
            return
        self.send_eth(BROADCAST_MAC, ETH_P_ARP,
                      self._arp(1, self.mac, self.ip, bytes(6), tpa))

    def _send_arp_reply(self, tha, tpa):
        self.send_eth(tha, ETH_P_ARP,
                      self._arp(2, self.mac, self.ip, tha, tpa))

    @staticmethod
    def _arp(oper, sha, spa, tha, tpa):
        return struct.pack(">HHBBH", 1, ETH_P_IP, 6, 4, oper) + \
            sha + spa + tha + tpa

    def _arp_lookup(self, ip):
        e = self.arp_cache.get(ip)
        if e and e[1] > time.time():
            return e[0]
        return None

    def _flush_pending(self, ip):
        if not self._pending_tx:
            return
        still = []
        mac = self._arp_lookup(ip)
        for nexthop, pkt in self._pending_tx:
            if nexthop == ip and mac is not None:
                self.send_eth(mac, ETH_P_IP, pkt)
            else:
                still.append((nexthop, pkt))
        self._pending_tx = still

    # ---- IPv4 ------------------------------------------------------------
    def _same_subnet(self, ip):
        if not self.ip or not self.netmask:
            return False
        return all((a & m) == (b & m)
                   for a, b, m in zip(self.ip, ip, self.netmask))

    def _build_ip(self, src, dst, proto, payload, ttl=None):
        if ttl is None:
            ttl = self.ttl
        total = 20 + len(payload)
        hdr = bytearray(struct.pack(">BBHHHBBH4s4s",
                                    0x45, 0x00, total, 0, 0x4000,
                                    ttl, proto, 0, src, dst))
        c = _cksum(bytes(hdr))
        hdr[10] = c >> 8
        hdr[11] = c & 0xFF
        return bytes(hdr) + payload

    def _on_ip(self, src_mac, p):
        if len(p) < 20 or (p[0] >> 4) != 4:
            return
        ihl = (p[0] & 0x0F) * 4
        proto = p[9]
        src = bytes(p[12:16])
        dst = bytes(p[16:20])
        # Learn the sender's MAC opportunistically (helps replies).
        if src != bytes(4):
            self.arp_cache[src] = (src_mac, time.time() + 120)
        body = p[ihl:]
        if self.ip and dst != self.ip and dst != b"\xff\xff\xff\xff" \
                and (dst[0] & 0xF0) != 0xE0:
            return                                     # not for us
        if proto == IPPROTO_ICMP:
            self._on_icmp(src, dst, body)
        elif proto == IPPROTO_UDP:
            self._on_udp(src, dst, body)
        elif proto == IPPROTO_TCP and self._tcp is not None:
            self._tcp.on_segment(src, dst, body)

    # ---- ICMP ------------------------------------------------------------
    def _on_icmp(self, src, dst, body):
        if len(body) < 4 or not self._icmp_enabled:
            return
        if body[0] == 8:                               # echo request
            reply = bytearray(body)
            reply[0] = 0                               # echo reply
            reply[2] = reply[3] = 0
            c = _cksum(bytes(reply))
            reply[2] = c >> 8
            reply[3] = c & 0xFF
            self.log("icmp: echo request from %s -> reply" % ip_str(src))
            self.send_ip(src, IPPROTO_ICMP, bytes(reply))

    def ping(self, dst_ip, ident=0x4242, seq=1, payload=b"pseudohost"):
        body = bytearray(struct.pack(">BBHHH", 8, 0, 0, ident, seq)) + payload
        c = _cksum(bytes(body))
        body[2] = c >> 8
        body[3] = c & 0xFF
        self.send_ip(dst_ip, IPPROTO_ICMP, bytes(body))

    # ---- UDP -------------------------------------------------------------
    def _build_udp(self, src, dst, sport, dport, payload):
        length = 8 + len(payload)
        hdr = struct.pack(">HHHH", sport, dport, length, 0)
        pseudo = src + dst + bytes([0, IPPROTO_UDP]) + struct.pack(">H", length)
        c = _cksum(pseudo + hdr + payload)
        if c == 0:
            c = 0xFFFF
        return struct.pack(">HHHH", sport, dport, length, c) + payload

    def _on_udp(self, src, dst, body):
        if len(body) < 8:
            return
        sport = (body[0] << 8) | body[1]
        dport = (body[2] << 8) | body[3]
        payload = body[8:]
        h = self._udp_handlers.get(dport)
        if h:
            h(src, sport, dst, dport, payload)
