#
# vwifi-pseudohost — netstack + DHCP tests (no live medium)
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vwifi_pseudohost import dhcp, netstack           # noqa: E402
from vwifi_pseudohost import ieee80211 as dot11        # noqa: E402


class FakeStation:
    """Stands in for a Station: captures egress, lets tests inject RX."""

    def __init__(self, mac):
        self.mac = mac
        self.on_eth_rx = None
        self.sent = []                     # (dst_mac, ethertype, sdu)

    def send_eth(self, dst, ethertype, sdu):
        self.sent.append((bytes(dst), ethertype, bytes(sdu)))

    # test-side injection
    def deliver(self, src_mac, dst_mac, ethertype, sdu):
        self.on_eth_rx(src_mac, dst_mac, ethertype, sdu)


def make_ip(src, dst, proto, payload):
    total = 20 + len(payload)
    hdr = bytearray(struct.pack(">BBHHHBBH4s4s", 0x45, 0, total, 0, 0,
                                64, proto, 0, netstack.ip_bytes(src),
                                netstack.ip_bytes(dst)))
    c = netstack._cksum(bytes(hdr))
    hdr[10], hdr[11] = c >> 8, c & 0xFF
    return bytes(hdr) + payload


class TestICMP(unittest.TestCase):
    def test_echo_reply(self):
        st = FakeStation(dot11.mac_bytes("02:00:00:00:00:01"))
        ns = netstack.NetStack(st, "unit")
        ns.configure("192.168.1.50", "255.255.255.0", "192.168.1.1")
        peer_mac = dot11.mac_bytes("02:00:00:00:00:99")
        ns.arp_cache[netstack.ip_bytes("192.168.1.9")] = (peer_mac, 1e18)

        echo = bytearray(struct.pack(">BBHHH", 8, 0, 0, 0x1234, 1)) + b"ping!!"
        c = netstack._cksum(bytes(echo))
        echo[2], echo[3] = c >> 8, c & 0xFF
        ip = make_ip("192.168.1.9", "192.168.1.50", netstack.IPPROTO_ICMP,
                     bytes(echo))
        st.deliver(peer_mac, st.mac, dot11.ETH_P_IP, ip)

        self.assertTrue(st.sent, "no reply emitted")
        dst_mac, et, sdu = st.sent[-1]
        self.assertEqual(et, dot11.ETH_P_IP)
        self.assertEqual(dst_mac, peer_mac)
        ihl = (sdu[0] & 0x0F) * 4
        icmp = sdu[ihl:]
        self.assertEqual(icmp[0], 0)                       # echo reply
        self.assertEqual(netstack._cksum(icmp), 0)         # valid checksum
        self.assertEqual(icmp[8:], b"ping!!")


class TestARP(unittest.TestCase):
    def test_reply_to_request(self):
        st = FakeStation(dot11.mac_bytes("02:00:00:00:00:02"))
        ns = netstack.NetStack(st, "unit")
        ns.configure("10.0.0.5", "255.255.255.0")
        who = dot11.mac_bytes("02:00:00:00:00:aa")
        arp = netstack.NetStack._arp(1, who, netstack.ip_bytes("10.0.0.1"),
                                     bytes(6), netstack.ip_bytes("10.0.0.5"))
        st.deliver(who, netstack.BROADCAST_MAC, dot11.ETH_P_ARP, arp)
        self.assertTrue(st.sent)
        dst_mac, et, sdu = st.sent[-1]
        self.assertEqual(et, dot11.ETH_P_ARP)
        oper = (sdu[6] << 8) | sdu[7]
        self.assertEqual(oper, 2)                          # reply
        self.assertEqual(bytes(sdu[8:14]), st.mac)         # our MAC
        self.assertEqual(bytes(sdu[14:18]), netstack.ip_bytes("10.0.0.5"))


class DHCPServer:
    """A one-lease DHCP server, just enough to answer the client."""

    def __init__(self, station, server_ip="192.168.7.1", lease_ip="192.168.7.100"):
        self.st = station
        self.server_ip = netstack.ip_bytes(server_ip)
        self.lease_ip = netstack.ip_bytes(lease_ip)

    def pump(self):
        """Consume any DISCOVER/REQUEST the client sent and answer it.

        Snapshot and clear first: answering an OFFER makes the client emit
        a REQUEST synchronously, and that must survive for the next pump.
        """
        pending = list(self.st.sent)
        self.st.sent.clear()
        for dst_mac, et, sdu in pending:
            if et != dot11.ETH_P_IP:
                continue
            ihl = (sdu[0] & 0x0F) * 4
            if sdu[9] != netstack.IPPROTO_UDP:
                continue
            udp = sdu[ihl:]
            dport = (udp[2] << 8) | udp[3]
            if dport != dhcp.DHCP_SERVER_PORT:
                continue
            self._answer(udp[8:])

    def _answer(self, req):
        xid = req[4:8]
        opts = dhcp.DHCPClient._parse_opts(req[240:])
        mtype = opts.get(dhcp.OPT_MSGTYPE, b"\x00")[0]
        if mtype == dhcp.DISCOVER:
            self._reply(xid, dhcp.OFFER)
        elif mtype == dhcp.REQUEST:
            self._reply(xid, dhcp.ACK)

    def _reply(self, xid, mtype):
        chaddr = bytes(self.st.mac) + bytes(10)
        hdr = struct.pack(">BBBBIHH4s4s4s4s16s64s128s",
                          2, 1, 6, 0, struct.unpack(">I", xid)[0], 0, 0,
                          bytes(4), self.lease_ip, self.server_ip, bytes(4),
                          chaddr, bytes(64), bytes(128))
        opts = bytearray(dhcp.MAGIC_COOKIE)
        opts += bytes([dhcp.OPT_MSGTYPE, 1, mtype])
        opts += bytes([dhcp.OPT_SERVER_ID, 4]) + self.server_ip
        opts += bytes([dhcp.OPT_SUBNET, 4, 255, 255, 255, 0])
        opts += bytes([dhcp.OPT_ROUTER, 4]) + self.server_ip
        opts += bytes([dhcp.OPT_DNS, 4]) + self.server_ip
        opts += bytes([dhcp.OPT_LEASE, 4, 0, 0, 0x0e, 0x10])
        opts += bytes([dhcp.OPT_END])
        bootp = hdr + bytes(opts)
        udp = struct.pack(">HHHH", dhcp.DHCP_SERVER_PORT,
                          dhcp.DHCP_CLIENT_PORT, 8 + len(bootp), 0) + bootp
        ip = make_ip(netstack.ip_str(self.server_ip), "255.255.255.255",
                     netstack.IPPROTO_UDP, udp)
        self.st.deliver(self.st.mac, netstack.BROADCAST_MAC,
                        dot11.ETH_P_IP, ip)


class TestDHCP(unittest.TestCase):
    def test_full_lease(self):
        st = FakeStation(dot11.mac_bytes("02:00:00:00:00:07"))
        ns = netstack.NetStack(st, "unit")
        server = DHCPServer(st)
        client = dhcp.DHCPClient(ns, "unit-host")
        client.start()                     # DISCOVER
        server.pump()                      # -> OFFER, client -> REQUEST
        server.pump()                      # -> ACK
        self.assertTrue(client.bound())
        self.assertEqual(ns.ip, netstack.ip_bytes("192.168.7.100"))
        self.assertEqual(ns.gateway, netstack.ip_bytes("192.168.7.1"))
        self.assertEqual(client.lease["lease_secs"], 3600)


if __name__ == "__main__":
    unittest.main()
