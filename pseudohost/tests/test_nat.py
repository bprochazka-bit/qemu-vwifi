#
# vwifi-pseudohost — userspace NAT tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# The NAT masquerades a station's off-subnet traffic onto the real host.
# These tests drive it against real loopback servers: a fake NetStack feeds
# it crafted station packets and captures what it injects back, while the
# "real world" is an ordinary 127.0.0.1 socket.  No medium, no radio.
#
import os
import select
import socket
import struct
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import netstack, tcp                 # noqa: E402
from vwifi_pseudohost.nat import NAT, resolve_bind_ip      # noqa: E402

STA_IP = "192.168.4.100"
GW_IP = "192.168.4.1"

SYN = 0x02
ACK = 0x10
PSH = 0x08
FIN = 0x01


class FakeStack:
    """Just the slice of NetStack the NAT touches; records egress."""

    def __init__(self, ip=GW_IP, netmask="255.255.255.0"):
        self.ip = netstack.ip_bytes(ip)
        self.netmask = netstack.ip_bytes(netmask)
        self.sent = []              # ("ip"|"udp", ...)
        self.forward = None

    def set_forward_handler(self, fn):
        self.forward = fn

    def same_subnet(self, dst):
        d = netstack.ip_bytes(dst)
        return all((a & m) == (b & m)
                   for a, b, m in zip(self.ip, d, self.netmask))

    def send_ip(self, dst, proto, payload, src_ip=None):
        self.sent.append(("ip", netstack.ip_bytes(dst), proto,
                          bytes(payload), src_ip))

    def send_udp(self, dst_ip, dst_port, payload, src_port=0, src_ip=None,
                 dst_mac=None):
        self.sent.append(("udp", netstack.ip_bytes(dst_ip), dst_port,
                          bytes(payload), src_port, src_ip))


def _sta_tcp(sport, dport, seq, ack, flags, payload=b"", dip="127.0.0.1"):
    return tcp._build_segment(sport, dport, seq, ack, flags, payload,
                              netstack.ip_bytes(STA_IP),
                              netstack.ip_bytes(dip))


def _parse_tcp(seg):
    sport, dport, seq, ack, off_flags = struct.unpack_from(">HHIIH", seg, 0)
    off = (off_flags >> 12) * 4
    return sport, dport, seq, ack, off_flags & 0x3F, seg[off:]


def _pump_until(nat, cond, timeout=3.0):
    end = time.time() + timeout
    while not cond() and time.time() < end:
        r, w, _ = select.select(nat.rlist(), nat.wlist(), [], 0.05)
        for s in w:
            nat.handle_writable(s)
        for s in r:
            nat.handle_readable(s)
    return cond()


def _tcp_echo_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def serve():
        try:
            c, _ = srv.accept()
            data = c.recv(1024)
            c.sendall(b"echo:" + data)
            c.close()
        except OSError:
            pass
        finally:
            srv.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    return port, t


def _udp_echo_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    srv.bind(("127.0.0.1", 0))
    port = srv.getsockname()[1]

    def serve():
        try:
            data, addr = srv.recvfrom(1024)
            srv.sendto(b"echo:" + data, addr)
        except OSError:
            pass
        finally:
            srv.close()

    t = threading.Thread(target=serve, daemon=True)
    t.start()
    return port, t


class TestNATTCP(unittest.TestCase):
    def test_tcp_round_trip(self):
        port, t = _tcp_echo_server()
        stack = FakeStack()
        nat = NAT(stack, log=lambda m: None)

        # Station opens the connection (SYN).
        nat.forward(netstack.ip_bytes(STA_IP), netstack.ip_bytes("127.0.0.1"),
                    netstack.IPPROTO_TCP,
                    _sta_tcp(40000, port, 1000, 0, SYN))
        # NAT connects out and answers SYN|ACK.
        self.assertTrue(_pump_until(nat, lambda: len(stack.sent) >= 1),
                        "no SYN|ACK from the NAT")
        _, _, _, seg, _ = stack.sent[0]
        sport, dport, srv_seq, srv_ack, flags, _ = _parse_tcp(seg)
        self.assertTrue(flags & SYN and flags & ACK)
        self.assertEqual(dport, 40000)               # back to the station port
        self.assertEqual(srv_ack, 1001)              # acks our SYN

        # Station completes the handshake and sends a request.
        nat.forward(netstack.ip_bytes(STA_IP), netstack.ip_bytes("127.0.0.1"),
                    netstack.IPPROTO_TCP,
                    _sta_tcp(40000, port, 1001, srv_seq + 1, ACK))
        nat.forward(netstack.ip_bytes(STA_IP), netstack.ip_bytes("127.0.0.1"),
                    netstack.IPPROTO_TCP,
                    _sta_tcp(40000, port, 1001, srv_seq + 1, PSH | ACK,
                             b"hello"))

        # The echoed reply must come back to the station as TCP data.
        def got_echo():
            for entry in stack.sent[1:]:
                if entry[0] != "ip":
                    continue
                seg = entry[3]
                _, _, _, _, fl, pl = _parse_tcp(seg)
                if b"echo:hello" in pl:
                    return True
            return False

        self.assertTrue(_pump_until(nat, got_echo),
                        "echoed payload never returned to the station")
        nat.close()
        t.join(timeout=2.0)

    def test_connect_refused_resets_station(self):
        # Nothing listening on this port -> connect fails -> RST to station.
        stack = FakeStack()
        nat = NAT(stack, log=lambda m: None)
        # Grab a definitely-closed port.
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()

        nat.forward(netstack.ip_bytes(STA_IP), netstack.ip_bytes("127.0.0.1"),
                    netstack.IPPROTO_TCP,
                    _sta_tcp(41000, port, 5, 0, SYN))

        def got_rst():
            for entry in stack.sent:
                if entry[0] != "ip":
                    continue
                _, _, _, _, fl, _ = _parse_tcp(entry[3])
                if fl & 0x04:                        # RST
                    return True
            return False

        self.assertTrue(_pump_until(nat, got_rst),
                        "connect failure did not RST the station")
        nat.close()


class TestNATUDP(unittest.TestCase):
    def test_udp_round_trip(self):
        port, t = _udp_echo_server()
        stack = FakeStack()
        nat = NAT(stack, log=lambda m: None)

        udp = netstack.NetStack._build_udp(
            None, netstack.ip_bytes(STA_IP), netstack.ip_bytes("127.0.0.1"),
            50000, port, b"ping")
        nat.forward(netstack.ip_bytes(STA_IP), netstack.ip_bytes("127.0.0.1"),
                    netstack.IPPROTO_UDP, udp)

        self.assertTrue(
            _pump_until(nat, lambda: any(e[0] == "udp" for e in stack.sent)),
            "no UDP reply mapped back to the station")
        reply = [e for e in stack.sent if e[0] == "udp"][0]
        # ("udp", dst_ip, dst_port, payload, src_port, src_ip)
        self.assertEqual(reply[2], 50000)             # back to station's port
        self.assertEqual(reply[4], port)              # from the server's port
        self.assertEqual(reply[3], b"echo:ping")
        nat.close()
        t.join(timeout=2.0)


class TestNATGuards(unittest.TestCase):
    def test_on_subnet_is_not_natted(self):
        stack = FakeStack()
        nat = NAT(stack, log=lambda m: None)
        # A destination on the AP's own subnet must never open a host socket.
        nat.forward(netstack.ip_bytes(STA_IP),
                    netstack.ip_bytes("192.168.4.50"),
                    netstack.IPPROTO_TCP,
                    _sta_tcp(40000, 80, 1, 0, SYN, dip="192.168.4.50"))
        self.assertEqual(nat.rlist(), [])
        self.assertEqual(nat.wlist(), [])
        self.assertEqual(stack.sent, [])

    def test_ap_address_is_not_natted(self):
        stack = FakeStack()
        nat = NAT(stack, log=lambda m: None)
        nat.forward(netstack.ip_bytes(STA_IP), stack.ip,
                    netstack.IPPROTO_UDP,
                    struct.pack(">HHHH", 5, 6, 8, 0))
        self.assertEqual(stack.sent, [])


class TestAPWiring(unittest.TestCase):
    """The AP turns NAT on by default and hooks it into its netstack."""

    def _ap(self, **kw):
        from vwifi_pseudohost.accesspoint import PseudoAP
        return PseudoAP("/nonexistent.sock", "Lab", log=lambda m: None, **kw)

    def test_nat_on_by_default(self):
        ap = self._ap()
        self.assertIsNotNone(ap.nat)
        # The netstack routes off-subnet packets into the NAT.
        self.assertEqual(ap.stack._forward, ap.nat.forward)

    def test_no_nat(self):
        ap = self._ap(nat=False)
        self.assertIsNone(ap.nat)
        self.assertIsNone(ap.stack._forward)

    def test_bind_ip_passed_through(self):
        ap = self._ap(nat_bind_ip="10.0.0.9")
        self.assertEqual(ap.nat.bind_ip, "10.0.0.9")

    def test_off_subnet_packet_reaches_nat(self):
        # An off-subnet IP delivered to the AP's stack must land in the NAT
        # (here: a UDP datagram bound for a public address).
        ap = self._ap()
        seen = []
        ap.nat.forward = lambda *a: seen.append(a)
        ap.stack.set_forward_handler(ap.nat.forward)
        pkt = ap.stack._build_ip(netstack.ip_bytes("192.168.4.100"),
                                 netstack.ip_bytes("8.8.8.8"),
                                 netstack.IPPROTO_UDP,
                                 struct.pack(">HHHH", 5, 53, 8, 0))
        ap.stack._on_ip(b"\x02\x00\x00\x00\x00\x01", pkt)
        self.assertEqual(len(seen), 1)
        self.assertEqual(seen[0][1], netstack.ip_bytes("8.8.8.8"))


class TestBindResolve(unittest.TestCase):
    def test_ip_literal_passthrough(self):
        self.assertEqual(resolve_bind_ip("10.20.30.40"), "10.20.30.40")
        self.assertIsNone(resolve_bind_ip(None))

    def test_bad_interface_raises(self):
        with self.assertRaises(ValueError):
            resolve_bind_ip("definitely-not-an-iface-xyz")


if __name__ == "__main__":
    unittest.main()
