#
# vwifi-pseudohost — CIDR scope + DHCP-server allocation tests
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import dhcp, netstack             # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11          # noqa: E402
from vwifi_pseudohost.dhcp_server import DHCPServer       # noqa: E402


class TestParseCIDR(unittest.TestCase):
    def test_slash24_network_form(self):
        p = netstack.parse_cidr("10.10.10.0/24")
        self.assertEqual(p["netmask"], "255.255.255.0")
        self.assertEqual(p["gateway"], "10.10.10.1")
        self.assertEqual(p["pool_start"], "10.10.10.100")
        self.assertEqual(p["pool_end"], "10.10.10.200")

    def test_explicit_gateway_host_bits(self):
        p = netstack.parse_cidr("192.168.50.254/24")
        self.assertEqual(p["gateway"], "192.168.50.254")
        self.assertEqual(p["netmask"], "255.255.255.0")

    def test_small_subnet_pool_fits_inside(self):
        p = netstack.parse_cidr("172.16.5.0/29")     # .0-.7, usable .1-.6
        self.assertEqual(p["netmask"], "255.255.255.248")
        self.assertEqual(p["gateway"], "172.16.5.1")
        # .100/.200 don't fit -> pool spans the usable range past gateway
        self.assertEqual(p["pool_start"], "172.16.5.2")
        self.assertEqual(p["pool_end"], "172.16.5.6")

    def test_slash16_spans_octets(self):
        p = netstack.parse_cidr("10.0.0.0/16")
        self.assertEqual(p["netmask"], "255.255.0.0")
        self.assertEqual(p["pool_start"], "10.0.0.100")
        self.assertEqual(p["pool_end"], "10.0.0.200")

    def test_rejects_ipv6(self):
        with self.assertRaises(ValueError):
            netstack.parse_cidr("fd00::/64")


class _FakeStation:
    def __init__(self, mac):
        self.mac = mac
        self.on_eth_rx = None
        self.sent = []

    def send_eth(self, dst, ethertype, sdu):
        self.sent.append((bytes(dst), ethertype, bytes(sdu)))


def _discover(mac, xid=0x11223344):
    import struct
    hdr = struct.pack(">BBBBIHH4s4s4s4s16s64s128s", 1, 1, 6, 0, xid, 0, 0x8000,
                      bytes(4), bytes(4), bytes(4), bytes(4),
                      bytes(mac) + bytes(10), bytes(64), bytes(128))
    opts = dhcp.MAGIC_COOKIE + bytes([dhcp.OPT_MSGTYPE, 1, dhcp.DISCOVER,
                                      dhcp.OPT_END])
    return hdr + opts


class TestDHCPServerRange(unittest.TestCase):
    def _server(self, **kw):
        st = _FakeStation(dot11.mac_bytes("02:00:00:00:00:01"))
        ns = netstack.NetStack(st, "ap")
        plan = netstack.parse_cidr(kw.pop("cidr"))
        ns.configure(plan["gateway"], plan["netmask"], plan["gateway"])
        srv = DHCPServer(ns, plan["gateway"], netmask=plan["netmask"],
                         gateway=plan["gateway"], pool_start=plan["pool_start"],
                         pool_end=plan["pool_end"], **kw)
        return srv

    def test_leases_within_scope(self):
        srv = self._server(cidr="10.10.10.0/24")
        ip1 = srv._allocate(dot11.mac_bytes("02:00:00:00:aa:01"))
        ip2 = srv._allocate(dot11.mac_bytes("02:00:00:00:aa:02"))
        self.assertEqual(netstack.ip_str(ip1), "10.10.10.100")
        self.assertEqual(netstack.ip_str(ip2), "10.10.10.101")
        # a repeat request for the same MAC keeps its lease
        self.assertEqual(srv._allocate(dot11.mac_bytes("02:00:00:00:aa:01")),
                         ip1)

    def test_never_hands_out_the_gateway(self):
        # A tight scope where the gateway sits inside the pool range.
        srv = self._server(cidr="172.16.5.0/29")     # gw .1, pool .2-.6
        handed = set()
        for i in range(5):
            ip = srv._allocate(dot11.mac_bytes(bytes([2, 0, 0, 0, 0, i])))
            self.assertIsNotNone(ip)
            handed.add(netstack.ip_str(ip))
        self.assertNotIn("172.16.5.1", handed)       # gateway excluded
        self.assertNotIn("172.16.5.0", handed)       # network never in pool

    def test_pool_exhaustion_returns_none(self):
        srv = self._server(cidr="172.16.5.0/29")     # only .2-.6 = 5 addrs
        got = [srv._allocate(bytes([2, 0, 0, 0, 0, i])) for i in range(8)]
        self.assertEqual(sum(1 for x in got if x is not None), 5)
        self.assertIsNone(got[-1])


if __name__ == "__main__":
    unittest.main()
