#
# vwifi-pseudohost — a tiny client-side TCP for driving server tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The package's tcp.py is server-side only.  To test it we need a client;
# this is a minimal one that talks to a server NetStack+TCPStack by
# injecting segments through the server's ingress and reading the
# server's replies out of its captured egress.  It is a test double, not
# part of the product.
#
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import netstack               # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11      # noqa: E402
from vwifi_pseudohost import tcp                     # noqa: E402


class ServerHost:
    """A netstack + TCP stack with a captured L2, standing in for a host."""

    def __init__(self, ip="192.168.9.1", mac="02:00:00:00:00:01"):
        self.mac = dot11.mac_bytes(mac)
        self.hostname = "srv"
        self.station = _FakeStation(self.mac)
        self.stack = netstack.NetStack(self.station, hostname="srv")
        self.stack.configure(ip, "255.255.255.0")
        self.tcp = tcp.TCPStack(self.stack)

    def log(self, *a):
        pass


class _FakeStation:
    def __init__(self, mac):
        self.mac = mac
        self.on_eth_rx = None
        self.sent = []

    def send_eth(self, dst, ethertype, sdu):
        self.sent.append((bytes(dst), ethertype, bytes(sdu)))


class ClientSim:
    def __init__(self, server, client_ip="192.168.9.100",
                 client_mac="02:00:00:00:00:aa", sport=40000, dport=515):
        self.server = server
        self.cip = netstack.ip_bytes(client_ip)
        self.cmac = dot11.mac_bytes(client_mac)
        self.sip = server.stack.ip
        self.sport = sport
        self.dport = dport
        self.iss = struct.unpack(">I", os.urandom(4))[0]
        self.snd_nxt = self.iss
        self.rcv_nxt = 0
        self._seen = 0

    # -- move a client segment into the server -----------------------------
    def _to_server(self, flags, payload=b""):
        seg = tcp._build_segment(self.sport, self.dport, self.snd_nxt,
                                 self.rcv_nxt, flags, payload, self.cip,
                                 self.sip)
        ip = _ip(self.cip, self.sip, netstack.IPPROTO_TCP, seg)
        self.server.station.on_eth_rx(self.cmac, self.server.mac,
                                      dot11.ETH_P_IP, ip)

    # -- read new server segments back -------------------------------------
    def _from_server(self):
        out = []
        frames = self.server.station.sent[self._seen:]
        self._seen = len(self.server.station.sent)
        for dst_mac, et, sdu in frames:
            if et != dot11.ETH_P_IP or sdu[9] != netstack.IPPROTO_TCP:
                continue
            ihl = (sdu[0] & 0x0F) * 4
            seg = sdu[ihl:]
            sport, dport, seq, ack, off_flags = struct.unpack_from(
                ">HHIIH", seg, 0)
            # The egress list is shared across all ClientSims on this
            # server; only take segments addressed to *our* connection.
            if dport != self.sport or sport != self.dport:
                continue
            data_off = (off_flags >> 12) * 4
            flags = off_flags & 0x3F
            payload = seg[data_off:]
            out.append((flags, seq, ack, payload))
        return out

    # -- API ---------------------------------------------------------------
    def connect(self):
        self._to_server(tcp.SYN)
        for flags, seq, ack, _p in self._from_server():
            if (flags & tcp.SYN) and (flags & tcp.ACK):
                self.rcv_nxt = tcp._u32(seq + 1)
                self.snd_nxt = tcp._u32(self.snd_nxt + 1)
                self._to_server(tcp.ACK)
                return True
        return False

    def send(self, data):
        self._to_server(tcp.PSH | tcp.ACK, data)
        self.snd_nxt = tcp._u32(self.snd_nxt + len(data))

    def recv(self):
        """Return concatenated application bytes the server has sent."""
        buf = b""
        for flags, seq, ack, payload in self._from_server():
            if payload:
                self.rcv_nxt = tcp._u32(self.rcv_nxt + len(payload))
                buf += payload
        return buf

    def close(self):
        self._to_server(tcp.FIN | tcp.ACK)
        self.snd_nxt = tcp._u32(self.snd_nxt + 1)
        # consume server's ACK/FIN
        self.recv()


def _ip(src, dst, proto, payload, ttl=64):
    total = 20 + len(payload)
    hdr = bytearray(struct.pack(">BBHHHBBH4s4s", 0x45, 0, total, 0, 0,
                                ttl, proto, 0, src, dst))
    c = netstack._cksum(bytes(hdr))
    hdr[10], hdr[11] = c >> 8, c & 0xFF
    return bytes(hdr) + payload
